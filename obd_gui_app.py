import customtkinter as ctk
import tkinter as tk
import tkinter.messagebox as messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time
import csv
import obd
import random
from datetime import datetime
import os
import subprocess
import glob
import webbrowser
import winsound
import queue
import sys
import json
import re
import warnings

warnings.filterwarnings("ignore", category=UserWarning, message="Tight layout not applied.*")

# --- CONSTANTS FOR INTERNAL LOGIC ---
class VehicleType:
    ICE = "ICE"
    HEV = "HEV"
    EV = "EV"

class FuelType:
    PETROL = "PETROL"
    DIESEL = "DIESEL"

# --- VIRTUAL CONNECTION START ---
class MockValue:
    def __init__(self, magnitude):
        self.magnitude = magnitude

class MockResponse:
    def __init__(self, value, is_dtc=False):
        self.value = value
        self.is_dtc = is_dtc

class MockInterface:
    def send_and_receive(self, cmd):
        # Symulacja odpowiedzi UDS + TP2.0 dla VAG
        cmd_str = cmd.decode().upper() if isinstance(cmd, bytes) else str(cmd).upper()
        
        if cmd_str.startswith("2E"): # Write DID
            return b"6E" + cmd_str[2:].encode()
        elif cmd_str.startswith("22"): # Read DID
            did = cmd_str[2:6]
            return b"62" + did.encode() + b"01" # Zwracamy '01' jako symulowaną wartość
        elif cmd_str.startswith("27"): # Security Access
            return b"67" + cmd_str[2:].encode()
        elif cmd_str.startswith("10"): # Session Control
            return b"50" + cmd_str[2:].encode()
        # --- TP2.0 Simulation ---
        elif "200" in cmd_str: # Channel Setup Request (broadcast)
            return b"A1 0F F6 03 00 00 10" # Simulated CS_Response with dynamic RX=0x300, TX=0x301
        elif cmd_str.startswith("A0"): # Channel Setup ACK
            return b"A1 00" # Acknowledged
        elif cmd_str.startswith("B0"): # Data Transfer
            return b"B1 00" # Data ACK
        elif cmd_str.startswith("A8"): # Channel close
            return b"A9 00"
            
        return b"OK (SIMULATED)"

class VirtualConnection:
    """Moduł testowy symulujący zaawansowane parametry pracy silnika i diagnotyki."""
    def __init__(self):
        self._connected = True
        self.start_time = time.time()
        self.dtc_cleared = False
        self.interface = MockInterface()

    def is_connected(self):
        return self._connected

    def query(self, cmd):
        time.sleep(0.04) # lekka asynchroniczność komunikacji CAN
        
        # Obsługa Błędów "Check Engine"
        if cmd == obd.commands.GET_DTC:
            if self.dtc_cleared:
                return MockResponse([])
            return MockResponse([
                ("P0104", "Mass or Volume Air Flow Circuit Intermittent"), 
                ("P0420", "Catalyst System Efficiency Below Threshold (Bank 1)"),
                ("P0300", "Random/Multiple Cylinder Misfire Detected")
            ])
        elif cmd == obd.commands.CLEAR_DTC:
            self.dtc_cleared = True
            time.sleep(0.5)
            return MockResponse("OK")
            
        # Parametry Robocze (Praca Silnika)
        elif cmd == obd.commands.RPM:
            el = time.time() - self.start_time
            return MockResponse(MockValue((800 + (el * 80) % 2500) + random.randint(-40, 40)))
        elif cmd == obd.commands.SPEED:
            el = time.time() - self.start_time
            return MockResponse(MockValue((el * 3) % 140))
        elif cmd == obd.commands.COOLANT_TEMP:
            el = time.time() - self.start_time
            return MockResponse(MockValue(min(90.0, 20.0 + el * 0.3)))
        elif cmd == obd.commands.MAF: # Przepływka
            el = time.time() - self.start_time
            return MockResponse(MockValue(2.0 + (el * 1.5) % 15.0))
        elif cmd == obd.commands.ENGINE_LOAD:
            el = time.time() - self.start_time
            return MockResponse(MockValue(15.0 + (el * 6) % 85.0))
        elif cmd == obd.commands.THROTTLE_POS: # Przepustnica
            el = time.time() - self.start_time
            return MockResponse(MockValue(5.0 + (el * 7) % 95.0))
        elif cmd == obd.commands.CONTROL_MODULE_VOLTAGE:
            return MockResponse(MockValue(13.8 + random.uniform(-0.3, 0.4)))
            
        # Niestandardowe parametry dla HEV i Skrzyni (symulowane)
        elif cmd == "TRANS_TEMP":
            el = time.time() - self.start_time
            return MockResponse(MockValue(min(80.0, 40.0 + el * 0.2)))
        elif cmd == "HEV_SOC":
            el = time.time() - self.start_time
            return MockResponse(MockValue(60.0 - (el * 0.1) % 40.0))
        elif cmd == "HEV_VOLTS":
            el = time.time() - self.start_time
            return MockResponse(MockValue(205.0 + random.uniform(-1.5, 1.5)))
        elif cmd == "HEV_POWER":
            el = time.time() - self.start_time
            return MockResponse(MockValue(15.0 + (el * 2.5) % 45.0 + random.uniform(-2, 2)))

        return MockResponse(None)

    def close(self):
        self._connected = False
# --- VIRTUAL CONNECTION END ---


# --- CAN TP2.0 TRANSPORT LAYER ---
class CANTP20Session:
    """
    VAG-proprietary CAN TP2.0 transport protocol session over ELM327.
    Used for older VAG vehicles (PQ35/PQ46 platform, ~2004-2012).

    Protocol Steps:
      1. Channel Setup (CS): Send broadcast frame to 0x200 with logical address of target module.
      2. CS_Response: Module replies with dynamic TX/RX CAN IDs for this session.
      3. CS_ACK: We confirm the channel.
      4. Data Transfer: Sequenced packets (max 6 bytes each) with flow control.
      5. Keepalive: Ping every ~1s to keep session alive.
      6. Channel Close: Terminate session when done.
    """
    # Module address map: logical_id -> (CS_CAN_ID, physical_name)
    MODULE_MAP = {
        "01": ("0x200", "Engine (Silnik)"),
        "03": ("0x200", "ABS/ESP"),
        "08": ("0x200", "Klimatyzacja"),
        "09": ("0x200", "Centrala (BCM)"),
        "15": ("0x200", "Poduszki powietrzne"),
        "17": ("0x200", "Zestaw Wskaźników"),
        "19": ("0x200", "Diagnostyka (Gateway)"),
        "37": ("0x200", "Nawigacja"),
        "46": ("0x200", "Komfort Centralny"),
        "52": ("0x200", "Drzwi (prawy bok)"),
        "55": ("0x200", "Akumulator (BMS)"),
        "5F": ("0x200", "Radio / Infotainment"),
    }

    def __init__(self, interface, module_id: str):
        self.interface = interface  # MockInterface or real ELM327 serial wrapper
        self.module_id = module_id.upper()
        self.rx_id: int = 0x300  # Assigned dynamically after CS (default fallback)
        self.tx_id: int = 0x301  # Assigned dynamically after CS (default fallback)
        self._seq = 0         # Transmit sequence number (0-15, wraps)
        self._open = False
        self._log = []       # Session log for display

    def open(self) -> tuple[bool, str]:
        """Initiate TP2.0 channel setup. Returns (success, message)."""
        try:
            # 1. Switch ELM327 to raw CAN mode (no auto-formatting)
            self.interface.send_and_receive(b"AT D")
            self.interface.send_and_receive(b"AT L0")
            self.interface.send_and_receive(b"AT H1")  # Show headers
            self.interface.send_and_receive(b"AT SP 6")  # ISO 15765-4 CAN
            self.interface.send_and_receive(b"AT CF 200")  # CAN filter for responses

            # 2. Channel Setup Request to broadcast address 0x200
            # Frame: [destination_addr, 0x00, 0x10, 0x00, 0x03, 0xB8, 0x10]
            dest = int(self.module_id, 16)
            cs_frame = f"200 07 {dest:02X} 00 10 00 03 B8 10"
            self._log.append(f"[TP2.0] CS → {cs_frame}")
            resp = self.interface.send_and_receive(cs_frame.encode())
            resp_str = resp.decode(errors="ignore") if isinstance(resp, bytes) else str(resp)
            self._log.append(f"[TP2.0] CS_RESP ← {resp_str}")

            # 3. Parse CS_Response for dynamic RX/TX CAN IDs
            # Expected: A1 0F F6 <RX_LO> <RX_HI> <TX_LO> <TX_HI>
            parts = resp_str.replace("\n", "").split()
            if len(parts) >= 7:
                self.rx_id = int(parts[3] + parts[4], 16) if len(parts) > 4 else 0x300
                self.tx_id = int(parts[5] + parts[6], 16) if len(parts) > 6 else 0x301
            else:
                # Simulation fallback
                self.rx_id = 0x300
                self.tx_id = 0x301

            # 4. Set ELM327 to use the dynamic TX ID as header
            self.interface.send_and_receive(f"AT SH {self.tx_id:03X}".encode())

            # 5. Channel Setup ACK
            ack_frame = f"A0 0F {self.rx_id & 0xFF:02X} {(self.rx_id >> 8) & 0xFF:02X}"
            resp = self.interface.send_and_receive(ack_frame.encode())
            self._log.append(f"[TP2.0] CS_ACK → {ack_frame}")

            self._open = True
            self._seq = 0
            return True, f"Kanał TP2.0 otwarty (moduł {self.module_id}, TX=0x{self.tx_id:03X} RX=0x{self.rx_id:03X})"
        except Exception as e:
            return False, f"Błąd otwierania kanału TP2.0: {e}"

    def send_data(self, data_hex: str) -> tuple[bool, str]:
        """
        Send a data payload (as hex string, e.g. '2E060D01') over TP2.0.
        Handles multi-packet chunking automatically.
        """
        if not self._open:
            return False, "Kanał TP2.0 nie jest otwarty."
        try:
            raw = bytes.fromhex(data_hex.replace(" ", ""))
            chunks = [raw[i:i+6] for i in range(0, len(raw), 6)]
            responses = []
            num_chunks = len(chunks)

            for i, chunk in enumerate(chunks):
                is_last = (i == num_chunks - 1)
                # TP2.0 packet header byte: high nibble=opcode, low nibble=seq
                # 0xF = more follows, 0x1 = last packet (no more)
                pkt_type = 0x10 if is_last else 0xF0
                seq_nibble = self._seq & 0x0F
                header_byte = pkt_type | seq_nibble
                self._seq = (self._seq + 1) % 16

                pkt_data = bytes([header_byte]) + chunk
                pkt_hex = " ".join(f"{b:02X}" for b in pkt_data)
                self._log.append(f"[TP2.0] DATA[{i}] → {pkt_hex}")

                resp = self.interface.send_and_receive(pkt_hex.encode())
                resp_str = resp.decode(errors="ignore") if isinstance(resp, bytes) else str(resp)
                self._log.append(f"[TP2.0] ACK[{i}] ← {resp_str}")
                responses.append(resp_str)

            return True, " | ".join(responses)
        except Exception as e:
            return False, f"Błąd transmisji TP2.0: {e}"

    def close(self):
        """Close TP2.0 channel."""
        if self._open:
            try:
                self.interface.send_and_receive(b"A8 00")
            except:
                pass
            self._open = False
        return self._log
# --- CAN TP2.0 TRANSPORT LAYER END ---


class OBDBackend:
    """Zarządca połączenia, pamięci podręcznej odczytów i zapisu do logów CSV."""
    def __init__(self, use_virtual=False):
        self.use_virtual = use_virtual
        self.connection = None
        self.is_connected_loop = False
        self.is_logging = False
        self.is_replay = False
        self.replay_data = []
        self.replay_index = 0
        
        # --- EXTREME PRO ---
        self.csv_interval = 0.2
        self.last_csv_write = 0
        self.use_imperial = False
        self.fuel_type: str | None = None  # Wymuś wybór użytkownika
        self.vehicle_type: str | None = None
        self.is_replay_paused = False  # pauza podczas Replay
        self.is_scrubbing = False # Nowe: Flaga blokująca nadpisywanie danych podczas przewijania
        self.replay_paused_duration = 0.0 # Skumulowany czas wstrzymania (dla synchronizacji)
        self.replay_start_real = 0.0
        self.replay_start_log = 0.0
        self.replay_alarm_indices = [] # Pre-calculated indices for alarms
        # -------------------
        
        self.x_data = []
        
        self.start_time = 0
        self.data_csv = ""
        self.last_error = ""  # opis ostatniego błędu połączenia ELM327
        self.current_vehicle_id = None        
        self.current = {
            "rpm": 0, "speed": 0, "temp": 0,
            "maf": 0.0, "load": 0.0, "throttle": 0.0,
            "trans_temp": 0.0, "hev_soc": 0.0, "hev_volts": 0.0, "hev_power": 0.0,
            "mpg": 0.0, # Spalanie wyliczane L/100km
            "voltage_12v": 0.0,
            "replayed_graphs": "" # Dla synchronizacji UI podczas Replay
        }
        
        self.active_graphs_list: list[str] = [] # List of currently graphed keys (for logging)
        
        self.history = {k: [] for k in self.current.keys()}
        self.data_lock = threading.Lock()

    def connect(self, port_name="AUTO", baudrate=None):
        self.last_error = ""
        if self.use_virtual:
            self.connection = VirtualConnection()
            success = self.connection.is_connected()
        else:
            try:
                if port_name == "AUTO":
                    self.connection = obd.OBD()
                else:
                    baud = int(baudrate) if baudrate and baudrate != "AUTO" else 38400
                    self.connection = obd.OBD(port_name, baudrate=baud)
                success = self.connection.is_connected()
                if not success:
                    self.last_error = "err_elm_no_veh"
            except Exception as e:
                success = False
                err_str = str(e).lower()
                if "could not open port" in err_str or "access is denied" in err_str:
                    self.last_error = "err_port_busy"
                elif "permissionerror" in err_str:
                    self.last_error = "err_port_denied"
                elif "timeout" in err_str or "timed out" in err_str:
                    self.last_error = "err_conn_timeout"
                elif "no such file" in err_str or "filenotfounderror" in err_str:
                    self.last_error = "err_port_missing"
                else:
                    self.last_error = f"ERROR: {e}"

        if success:
            self.is_connected_loop = True
            self.is_replay = False
            self.start_time = time.time()
            threading.Thread(target=self._loop, daemon=True).start()
        return success

    def disconnect(self):
        self.is_connected_loop = False
        self.is_logging = False
        self.current_vehicle_id = None
        if self.connection and self.connection.is_connected():
            self.connection.close()
        
        # Resetowanie danych (powrót do zer/pustych list)
        for k in self.current.keys():
            if k == "replayed_graphs": self.current[k] = ""
            else: self.current[k] = 0.0
        
        with self.data_lock:
            self.x_data.clear()
            for v in self.history.values():
                v.clear()

    def read_errors(self):
        if not self.connection or not self.connection.is_connected():
            return None
        res = self.connection.query(obd.commands.GET_DTC)
        return res.value if res and res.value else []

    def clear_errors(self):
        if not self.connection or not self.connection.is_connected():
            return False
        res = self.connection.query(obd.commands.CLEAR_DTC)
        return res is not None

    def start_logging(self, vehicle_id=""):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        vtype = self.vehicle_type                        # ICE / HEV / EV
        fuel  = self.fuel_type if vtype != "EV" else ""  # Benzyna / Diesel / ""
        
        # Nazwa pliku zawiera identyfikator, typ pojazdu i paliwo
        vid_part = f"[{vehicle_id}]_" if vehicle_id else ""
        fuel_part = f"_{fuel}" if fuel else ""
        self.data_csv = f"{vid_part}log_obd_{vtype}{fuel_part}_{stamp}.csv"
        
        # Universal headers containing all possible graphable parameters
        self._csv_headers = [
            'Sys_Time', 'Rel_Time_s', 'Speed_kmh', 'Coolant_Temp_C', 'Voltage_12V_V',
            'RPM', 'MAF_gs', 'EngineLoad_pct', 'Throttle_pct', 'TransTemp_C', 
            'HEV_Battery_pct', 'HEV_Voltage_V', 'HEV_Power_kW', 'Consumption_L100km',
            'ActiveGraphs'
        ]

        self.last_csv_write = 0
        with open(self.data_csv, mode='w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(self._csv_headers)
            
        self.is_logging = True

    def stop_logging(self):
        self.is_logging = False # Ustaw flagę natychmiast, aby przerwać pętlę zapisu w _loop
        if not self.data_csv:
            return
        
        try:
            el = time.time() - self.start_time
            vtype = self.vehicle_type
            fuel  = self.fuel_type if vtype != "EV" else ""
            data_row = [
                datetime.now().isoformat(), round(el, 1),
                round(self.current["speed"], 1), round(self.current["temp"], 1),
                round(self.current["voltage_12v"], 1), round(self.current["rpm"], 1),
                round(self.current["maf"], 1), round(self.current["load"], 1),
                round(self.current["throttle"], 1), round(self.current["trans_temp"], 1),
                round(self.current["hev_soc"], 1), round(self.current["hev_volts"], 1),
                round(self.current["hev_power"], 1), round(self.current["mpg"], 2),
                ",".join(self.active_graphs_list)
            ]
            with open(self.data_csv, mode='a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow(data_row)
        except: pass
        
        self.is_logging = False

    def start_replay(self, filepath):
        self.replay_data = []
        self.replay_index = 0
        try:
            import csv
            with open(filepath, mode='r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row_dict in reader:
                    row = {}
                    for k, v in row_dict.items():
                        if not k: continue
                        kl = k.lower()
                        if "time_s" in kl: row["time"] = float(v)
                        elif "sys_time" in kl: pass
                        elif "rpm" in kl: row["rpm"] = v
                        elif "speed" in kl: row["speed"] = v
                        elif "coolant" in kl: row["temp"] = v
                        elif "maf" in kl: row["maf"] = v
                        elif "load" in kl: row["load"] = v
                        elif "throttle" in kl: row["throttle"] = v
                        elif "transtemp" in kl: row["trans_temp"] = v
                        elif "soc" in kl: row["hev_soc"] = v
                        elif "hev_voltage" in kl or "traction" in kl: row["hev_volts"] = v
                        elif "power" in kl: row["hev_power"] = v
                        elif "12v" in kl: row["voltage_12v"] = v
                        elif "fuelcons" in kl or "mpg" in kl: row["mpg"] = v
                        elif "activegraphs" in kl: row["active_graphs"] = v
                    self.replay_data.append(row)
            
            fname = os.path.basename(filepath)
            # Format: [ID]_log_obd_VTYPE_FUEL_STAMP.csv lub log_obd_VTYPE_STAMP.csv
            match = re.search(r"log_obd_([^_]+)(?:_([^_]+))?_", fname)
            if match:
                vt = match.group(1)
                ft = match.group(2)
                if vt in [VehicleType.ICE, VehicleType.HEV, VehicleType.EV]:
                    self.vehicle_type = vt
                if ft in [FuelType.PETROL, FuelType.DIESEL]:
                    self.fuel_type = ft

        except Exception as e:
            return False
            
        self.is_replay = True
        self.replay_index = 0
        self.is_replay_paused = False
        self.start_time = time.time()
        self.replay_alarm_indices = []

        # Konwersja danych na liczby (raz przy ładowaniu, by uniknąć float() w pętlach UI)
        if self.replay_data:
            bt = 0
            try: bt = float(self.replay_data[0].get("time", 0))
            except: pass
            
            numeric_keys = ["rpm", "speed", "temp", "maf", "load", "throttle", 
                           "trans_temp", "hev_soc", "hev_volts", "hev_power", 
                           "voltage_12v", "mpg"]
            for row in self.replay_data:
                try: row["time"] = float(row.get("time", 0)) - bt
                except: row["time"] = 0.0
                for k in numeric_keys:
                    try: row[k] = float(row.get(k, 0))
                    except: row[k] = 0.0
        
        # Inicjalne ustawienie parametrów z pierwszego wiersza (by UI nie czekało 100ms)
        if self.replay_data:
            row0 = self.replay_data[0]
            self.current["replayed_graphs"] = row0.get("active_graphs", "")
            
        # Jeśli pętla nie działa, uruchom ją
        if not self.is_connected_loop:
            self.is_connected_loop = True
            threading.Thread(target=self._loop, daemon=True).start()
        
        return True

    def _loop(self):
        try:
            while self.is_connected_loop:
                if self.is_replay:
                    # Jeśli to początek odtwarzania, daj bufor na stabilizację UI
                    if self.replay_index == 0:
                        time.sleep(0.5)
                        self.replay_start_log = float(self.replay_data[0].get("time", 0)) if self.replay_data else 0
                        
                    # Jeśli pauza lub przewijanie, śpij i nie procesuj dalej
                    if self.is_replay_paused or self.is_scrubbing:
                        time.sleep(0.1)
                        if self.is_replay_paused: self.replay_paused_duration += 0.1
                        continue
                        
                    if self.replay_index < len(self.replay_data):
                        row = self.replay_data[self.replay_index]
                        
                        # Precyzyjna synchronizacja czasu z uwzględnieniem pauz
                        now_log = float(row.get("time", 0))
                        target_real = self.replay_start_real + (now_log - self.replay_start_log) + self.replay_paused_duration
                        wait_time = target_real - time.time()
                        if wait_time > 0:
                            # Interruptible sleep
                            sleep_end = time.time() + min(wait_time, 5.0) # Max 5s limit for safety
                            while time.time() < sleep_end and self.is_connected_loop and not self.is_replay_paused:
                                time.sleep(0.05)
                        
                        try:
                            self.current["rpm"] = float(row.get("rpm", 0))
                            self.current["speed"] = float(row.get("speed", 0))
                            self.current["temp"] = float(row.get("temp", 0))
                            self.current["maf"] = float(row.get("maf", 0))
                            self.current["load"] = float(row.get("load", 0))
                            self.current["throttle"] = float(row.get("throttle", 0))
                            self.current["trans_temp"] = float(row.get("trans_temp", 0))
                            self.current["hev_soc"] = float(row.get("hev_soc", 0))
                            self.current["hev_volts"] = float(row.get("hev_volts", 0))
                            self.current["hev_power"] = float(row.get("hev_power", 0))
                            self.current["voltage_12v"] = float(row.get("voltage_12v", 0))
                            self.current["mpg"] = float(row.get("mpg", 0))
                            self.current["replayed_graphs"] = row.get("active_graphs", "")
                            
                            now_v = float(row.get("time", 0))
                            with self.data_lock:
                                self.x_data.append(now_v)
                                for k in self.history.keys():
                                    self.history[k].append(self.current[k])
                                if len(self.x_data) > 60:
                                    self.x_data.pop(0)
                                    for k in self.history.keys():
                                        self.history[k].pop(0)
                        except: pass
                        self.replay_index += 1
                    else:
                        self.is_replay_paused = True # Zamiast kończyć, po prostu włącz pauzę na końcu
                    continue
                    
                if not self.connection or not self.connection.is_connected():
                    time.sleep(1)
                    continue
                    
                now = time.time() - self.start_time
                
                # Lista komend do wysłania
                cmds = [
                    obd.commands.RPM, obd.commands.SPEED, obd.commands.COOLANT_TEMP,
                    obd.commands.MAF, obd.commands.ENGINE_LOAD, obd.commands.THROTTLE_POS
                ]
                
                vals = []
                for c in cmds:
                    req = self.connection.query(c)
                    vals.append(req.value.magnitude if req.value is not None else 0)
                # --- Nowa implementacja odczytów (z użyciem lokalnych buforów dla bezpieczeństwa wątków) ---
                if not self.is_connected_loop: break
                
                v_rpm = self.connection.query(obd.commands.RPM).value
                v_speed = self.connection.query(obd.commands.SPEED).value
                v_t = self.connection.query(obd.commands.COOLANT_TEMP).value
                v_maf = self.connection.query(obd.commands.MAF).value
                v_load = self.connection.query(obd.commands.ENGINE_LOAD).value
                v_thr = self.connection.query(obd.commands.THROTTLE_POS).value
                v_12v = self.connection.query(obd.commands.CONTROL_MODULE_VOLTAGE).value
                
                if self.use_virtual:
                    v_trans = self.connection.query("TRANS_TEMP").value.magnitude
                    v_hev_soc = self.connection.query("HEV_SOC").value.magnitude
                    v_hev_v = self.connection.query("HEV_VOLTS").value.magnitude
                    v_hev_kw = self.connection.query("HEV_POWER").value.magnitude
                else:
                    v_trans, v_hev_soc, v_hev_v, v_hev_kw = 0.0, 0.0, 0.0, 0.0
                
                # --- WERYFIKACJA: Czy nadal jesteśmy połączeni przed zapisem do słownika? ---
                if not self.is_connected_loop: break
                
                # Zapis do słownika 'current' następuje tylko, gdy flaga jest aktywna
                self.current["rpm"] = int(v_rpm.magnitude) if v_rpm else 0
                self.current["speed"] = int(v_speed.magnitude) if v_speed else 0
                self.current["temp"] = round(v_t.magnitude) if v_t else 0
                self.current["maf"] = round(v_maf.magnitude, 1) if v_maf else 0.0
                self.current["load"] = round(v_load.magnitude, 1) if v_load else 0.0
                self.current["throttle"] = round(v_thr.magnitude, 1) if v_thr else 0.0
                self.current["voltage_12v"] = round(v_12v.magnitude, 1) if v_12v else 0.0
                self.current["trans_temp"] = round(v_trans, 1)
                self.current["hev_soc"] = round(v_hev_soc, 1)
                self.current["hev_volts"] = round(v_hev_v, 1)
                self.current["hev_power"] = round(v_hev_kw, 2)
                
                # --- EXTREME PRO: Kalkulator Spalania ---
                maf_val = self.current["maf"]
                speed_val = self.current["speed"]
                afr = 14.7 if self.fuel_type == FuelType.PETROL else 14.5
                f_dens = 745.0 if self.fuel_type == FuelType.PETROL else 832.0 
                if maf_val > 0:
                    l_h = (maf_val / afr / f_dens) * 3600.0
                    if speed_val > 2.0:
                        self.current["mpg"] = min((l_h / speed_val) * 100.0, 99.9)
                    else:
                        self.current["mpg"] = l_h # na postoju: l/h
                else:
                    self.current["mpg"] = 0.0
                
                if not self.is_connected_loop: break
                
                with self.data_lock:
                    self.x_data.append(time.time() - self.start_time)
                    for k in self.history.keys():
                        self.history[k].append(self.current[k])
                    
                    if len(self.x_data) > 60:
                        self.x_data.pop(0)
                        for k in self.history.keys():
                            self.history[k].pop(0)
                        
                if self.is_logging:
                    if time.time() - self.last_csv_write >= self.csv_interval:
                        el = time.time() - self.start_time
                        data_row = [
                            datetime.now().isoformat(), round(el, 1),
                            round(self.current["speed"], 1),
                            round(self.current["temp"], 1),
                            round(self.current["voltage_12v"], 1),
                            round(self.current["rpm"], 1),
                            round(self.current["maf"], 1),
                            round(self.current["load"], 1),
                            round(self.current["throttle"], 1),
                            round(self.current["trans_temp"], 1),
                            round(self.current["hev_soc"], 1),
                            round(self.current["hev_volts"], 1),
                            round(self.current["hev_power"], 1),
                            round(self.current["mpg"], 2),
                            ",".join(self.active_graphs_list)
                        ]
                        with open(self.data_csv, mode='a', newline='', encoding='utf-8') as f:
                            csv.writer(f).writerow(data_row)
                        self.last_csv_write = time.time()
                
                time.sleep(0.1)
        finally:
            # GWARANTOWANE ZEROWANIE przy zakończeniu wątku
            # To zapobiega "losowemu" zostawaniu wartości po rozłączeniu
            for k in self.current.keys():
                if k == "replayed_graphs": self.current[k] = ""
                else: self.current[k] = 0.0

    def execute_vag_command(self, module_id, commands, security_code=None, is_restore=False):
        """Wysyła sekwencję surowych komend HEX do konkretnego modułu VAG."""
        if not self.connection or not self.connection.is_connected():
            return False, "Brak połączenia z interfejsem."
            
        # Pobieramy interfejs (ELM327 lub Mock)
        interface = getattr(self.connection, 'interface', None)
        if not interface:
            return False, "Interfejs nie obsługuje surowych komend."

        # Blokujemy pętlę odczytu na czas kodowania
        was_looping = self.is_connected_loop
        self.is_connected_loop = False
        time.sleep(0.4) 
        
        try:
            # 1. Ustawienie protokołu CAN (ISO 15765-4 11/500)
            interface.send_and_receive(b"AT SP 6")
            
            # 2. Ustawienie nagłówka (Header) dla danego modułu
            header_map = {"17": "714", "09": "710", "01": "7E0", "19": "712", "5F": "773", "52": "71D"}
            header = header_map.get(module_id, module_id)
            interface.send_and_receive(f"AT SH {header}".encode())
            
            # 3. Sesja diagnostyczna (Extended Diagnostic Session)
            interface.send_and_receive(b"10 03")
            
            # 4. Security Access (opcjonalnie)
            if security_code:
                interface.send_and_receive(b"27 01")
                interface.send_and_receive(f"27 02 {security_code}".encode())

            # 5. Wykonanie komend właściwych z automatycznym backupem
            responses = []
            for cmd in commands:
                clean_cmd = cmd.replace(" ", "")
                
                # Backup: jeśli to zapis (2E), najpierw odczytaj (22)
                if not is_restore and clean_cmd.startswith("2E") and len(clean_cmd) >= 6:
                    did = clean_cmd[2:6]
                    try:
                        read_resp = interface.send_and_receive(f"22{did}".encode())
                        if read_resp and read_resp.startswith(b"62"):
                            self._save_vag_backup(module_id, did, read_resp.decode() if isinstance(read_resp, bytes) else str(read_resp))
                    except:
                        pass
                
                resp = interface.send_and_receive(clean_cmd.encode())
                responses.append(resp.decode() if isinstance(resp, bytes) else str(resp))
                time.sleep(0.1)
                
            return True, " | ".join(responses)
        except Exception as e:
            return False, str(e)
        finally:
            if was_looping:
                self.is_connected_loop = True

    def get_vag_backup(self, module_id, did):
        """Pobiera oryginalną wartość dla danego modułu i DID z pliku backupu."""
        if not os.path.exists("vag_backups.json"):
            return None
        try:
            with open("vag_backups.json", "r", encoding="utf-8") as f:
                data = json.load(f)
                entry = data.get(module_id, {}).get(did)
                if isinstance(entry, dict):
                    return entry.get("original_value")
                return entry # Fallback dla starych backupów
        except:
            return None

    def execute_vag_restore(self, module_id, did, security_code=None):
        """Przywraca oryginalne kodowanie dla konkretnego modułu i DID z pliku backupu."""
        original_value = self.get_vag_backup(module_id, did)
        if not original_value:
            return False, "Brak kopii zapasowej."
            
        restore_cmd = f"2E{did}{original_value}"
        return self.execute_vag_command(module_id, ["1003", restore_cmd], security_code, is_restore=True)

    def _save_vag_backup(self, module_id, did, raw_response):
        """Zapisuje oryginalną wartość sterownika do pliku przed zmianą."""
        try:
            backup_file = "vag_backups.json"
            data = {}
            if os.path.exists(backup_file):
                with open(backup_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
            
            timestamp = datetime.now().isoformat()
            if module_id not in data: data[module_id] = {}
            
            # Wycinamy samą wartość (pomijamy 62 + DID)
            # Odpowiedź 62 DID VALUE...
            value = raw_response[6:].strip() if len(raw_response) > 6 else raw_response
            
            data[module_id][did] = {
                "original_value": value,
                "timestamp": timestamp,
                "raw_response": raw_response
            }
            
            with open(backup_file, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=4)
        except:
            pass

    def execute_vag_tp20_command(self, module_id: str, commands: list, security_code=None) -> tuple:
        """Wysyła komendy do sterownika VAG przez protokół CAN TP2.0 (starsze auta PQ35/PQ46)."""
        if not self.connection or not self.connection.is_connected():
            return False, "Brak połączenia z interfejsem."
        interface = getattr(self.connection, 'interface', None)
        if not interface:
            return False, "Interfejs nie obsługuje surowych komend CAN."

        # Wstrzymaj pętlę OBD na czas komunikacji
        was_looping = self.is_connected_loop
        self.is_connected_loop = False
        time.sleep(0.4)

        session = CANTP20Session(interface, module_id)
        try:
            # Otwórz kanał TP2.0
            ok, msg = session.open()
            if not ok:
                return False, f"TP2.0 Channel Setup failed: {msg}"

            # Security Access (jeśli wymagany)
            if security_code:
                session.send_data("2701")          # Request seed
                session.send_data(f"2702{security_code}")  # Send key

            # Wykonaj komendy
            all_responses = []
            for cmd in commands:
                clean = cmd.replace(" ", "")
                ok_data, resp = session.send_data(clean)
                all_responses.append(resp)
                if not ok_data:
                    return False, f"Błąd transmisji TP2.0 dla komendy {cmd}: {resp}"
                time.sleep(0.1)

            return True, " | ".join(all_responses)
        except Exception as e:
            return False, str(e)
        finally:
            session.close()
            if was_looping:
                self.is_connected_loop = True

    def execute_uds_custom(self, target_header: str, commands: list, security_code: str = None, is_restore: bool = False) -> tuple[bool, str]:
        """Wykonuje listę komend UDS z funkcjonalnością backupu i niestandardowymi docelowymi headerami CAN."""
        if not self.connection or not self.connection.is_connected():
            return False, "Brak połączenia z interfejsem."
        
        interface = getattr(self.connection, 'interface', None)
        if not interface:
            return False, "Interfejs nie obsługuje surowych komend CAN."

        was_looping = self.is_connected_loop
        self.is_connected_loop = False
        time.sleep(0.4)
        try:
            # Ustawienie nagłówka specyficzne dla Stellantis / UDS
            header = "714"
            if len(target_header) == 3 and target_header.isalnum():
                header = target_header
            elif "ECM" in target_header: header = "7E0"
            elif "BCM" in target_header or "BSI" in target_header: header = "7A6"
            elif "IPC" in target_header: header = "7A0"
            else: header = target_header

            interface.send_and_receive(f"AT SH {header}".encode()) 
            interface.send_and_receive(b"1003")
            
            if security_code:
                interface.send_and_receive(b"2701")
                interface.send_and_receive(f"2702{security_code}".encode())

            results = []
            for cmd in commands:
                clean_cmd = cmd.replace(" ", "")
                
                # Auto-backup dla Stellantis przy komendach 2E (Write Data By Identifier)
                if not is_restore and clean_cmd.startswith("2E") and len(clean_cmd) >= 6:
                    did = clean_cmd[2:6]
                    try:
                        read_resp = interface.send_and_receive(f"22{did}".encode())
                        if read_resp and read_resp.startswith(b"62"):
                            self._save_vag_backup(header, did, read_resp.decode() if isinstance(read_resp, bytes) else str(read_resp))
                    except:
                        pass
                
                resp = interface.send_and_receive(clean_cmd.encode())
                results.append(resp.decode() if isinstance(resp, bytes) else str(resp))
                time.sleep(0.1)

            return True, " | ".join(results)
            for cmd in commands:
                resp = interface.send_and_receive(cmd.encode())
                results.append(resp.decode() if isinstance(resp, bytes) else str(resp))
                time.sleep(0.05)
            
            return True, " | ".join(results)
        except Exception as e:
            return False, str(e)
        finally:
            if was_looping:
                self.is_connected_loop = True

    def load_zdc_file(self, filepath: str) -> tuple:
        """Parsuje plik .ZDC i zwraca listę komend HEX do wykonania.

        Format pliku ZDC:
          - Linie zaczynające się od '#' lub ';' → komentarze (ignorowane)
          - Puste linie → ignorowane
          - Każda inna linia → komenda HEX (np. '2E 06 0D 01')
        Returns: (success, list_of_hex_strings | error_message)
        """
        try:
            commands = []
            with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#") or line.startswith(";"):
                        continue
                    # Accept both space-separated and continuous hex
                    hex_candidate = line.replace(" ", "")
                    if all(c in "0123456789ABCDEFabcdef" for c in hex_candidate):
                        commands.append(hex_candidate.upper())
            if not commands:
                return False, "Plik ZDC nie zawiera żadnych poprawnych komend HEX."
            return True, commands
        except FileNotFoundError:
            return False, f"Plik nie istnieje: {filepath}"
        except Exception as e:
            return False, f"Błąd odczytu pliku ZDC: {e}"


import codecs

LOCALIZATION = {}
def load_localizations():
    global LOCALIZATION
    loc_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "i18n")
    if not os.path.exists(loc_dir):
        return
    for lang in ["pl", "en", "de"]:
        path = os.path.join(loc_dir, f"{lang}.json")
        if os.path.exists(path):
            with codecs.open(path, "r", "utf-8") as f:
                LOCALIZATION[lang] = json.load(f)

load_localizations()


class OBDApp(ctk.CTk):
    """Główna klasa GUI bazująca na systemie ZAKĹADEK (Tabview)."""
    def __init__(self):
        super().__init__()
        self.title("OBD_MASTER_PRO_V7")
        self.geometry("1280x820")
        self.minsize(1280, 820)
        
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.current_lang = "pl"
        self.backend = OBDBackend(use_virtual=True) 
        self.gui_refresh_rate = 300 # Zwiększone z 100ms, aby odciążyć procesor i uniknąć lagów GUI
        self.show_anim = True
        
        # Opcje Extreme PRO - Zaawansowane Alarmy
        self.alarm_enabled = True
        self.alarm_configs = {
            "rpm": {"limit": 4500, "active": True, "label": "Engine Revs (RPM)", "mode": "max", "freq": 1000},
            "speed": {"limit": 140, "active": False, "label": "Vehicle Speed (km/h)", "mode": "max", "freq": 800},
            "temp": {"limit": 105, "active": True, "label": "Coolant Temp (Â°C)", "mode": "max", "freq": 600},
            "maf": {"limit": 150, "active": False, "label": "MAF Flow (g/s)", "mode": "max", "freq": 500},
            "load": {"limit": 95, "active": False, "label": "Engine Load (%)", "mode": "max", "freq": 400},
            "throttle": {"limit": 90, "active": False, "label": "Throttle Pos (%)", "mode": "max", "freq": 400},
            "trans_temp": {"limit": 110, "active": False, "label": "Trans/EV Temp (Â°C)", "mode": "max", "freq": 600},
            "hev_soc": {"limit": 15, "active": False, "label": "Low Battery SOC (%)", "mode": "min", "freq": 450},
            "hev_volts": {"limit": 250, "active": False, "label": "HV Battery Volts (V)", "mode": "max", "freq": 700},
            "hev_power": {"limit": 80, "active": False, "label": "EV Motor Power (kW)", "mode": "max", "freq": 800},
            "voltage_12v": {"limit": 11.8, "active": False, "label": "Low 12V Volts (V)", "mode": "min", "freq": 300},
            "mpg": {"limit": 20, "active": False, "label": "High Consumption (L/100)", "mode": "max", "freq": 350}
        }
        
        self.VAG_TWEAKS_CONFIG = {
            "dash": [
                {"id": "needle", "name_key": "vag_tweak_needle", "module": "17", "commands": ["1003", "2E060D01"]},
                {"id": "laptimer", "name_key": "vag_tweak_laptimer", "module": "17", "commands": ["1003", "2E0AF201"]},
                {"id": "refuel", "name_key": "vag_tweak_refuel", "module": "17", "commands": ["1003", "2E019201"]},
                {"id": "oiltemp", "name_key": "vag_tweak_oiltemp", "module": "17", "commands": ["1003", "2E023401"]},
                {"id": "carbon", "name_key": "vag_tweak_carbon", "module": "17", "commands": ["1003", "2E045A01"]},
                {"id": "seatbelt", "name_key": "vag_tweak_seatbelt", "module": "17", "commands": ["1003", "2E012500"]}
            ],
            "elec": [
                {"id": "heartbeat", "name_key": "vag_tweak_heartbeat", "module": "09", "security": "31347", "commands": ["1003", "2E020C01"]},
                {"id": "comfort", "name_key": "vag_tweak_comfort", "module": "09", "security": "31347", "commands": ["1003", "2E054105"]},
                {"id": "drlmenu", "name_key": "vag_tweak_drlmenu", "module": "09", "security": "31347", "commands": ["1003", "2E080401"]},
                {"id": "cornering", "name_key": "vag_tweak_cornering", "module": "09", "security": "31347", "commands": ["1003", "2E049101"]},
                {"id": "chlh", "name_key": "vag_tweak_chlh", "module": "09", "security": "31347", "commands": ["1003", "2E049201"]},
                {"id": "drltail", "name_key": "vag_tweak_drltail", "module": "09", "security": "31347", "commands": ["1003", "2E049301"]},
                {"id": "emergstop", "name_key": "vag_tweak_emergstop", "module": "09", "security": "31347", "commands": ["1003", "2E049401"]}
            ],
            "maintenance": [
                {"id": "oil", "name_key": "vag_tweak_oil", "module": "17", "commands": ["1003", "2E02C601"]},
                {"id": "insp", "name_key": "vag_tweak_insp", "module": "17", "commands": ["1003", "2E02C701"]},
                {"id": "startstop", "name_key": "vag_tweak_startstop", "module": "19", "commands": ["1003", "2E010279"]},
                {"id": "throttle", "name_key": "vag_tweak_throttle", "module": "01", "commands": ["1003", "2E04B40C"]},
                {"id": "xds", "name_key": "vag_tweak_xds", "module": "03", "commands": ["1003", "2E012302"]}
            ],
            "comfort": [
                {"id": "mirrordip", "name_key": "vag_tweak_mirrordip", "module": "52", "commands": ["1003", "2E04B40C"]},
                {"id": "acoustic", "name_key": "vag_tweak_acoustic", "module": "09", "security": "31347", "commands": ["1003", "2E02D501"]},
                {"id": "rainclose", "name_key": "vag_tweak_rainclose", "module": "09", "security": "31347", "commands": ["1003", "2E049101"]},
                {"id": "autolock", "name_key": "vag_tweak_autolock", "module": "09", "security": "31347", "commands": ["1003", "2E012301"]},
                {"id": "autounlock", "name_key": "vag_tweak_autounlock", "module": "09", "security": "31347", "commands": ["1003", "2E012401"]},
                {"id": "easyentry", "name_key": "vag_tweak_easyentry", "module": "36", "commands": ["1003", "2E012501"]},
                {"id": "remotewindows", "name_key": "vag_tweak_remotewindows", "module": "09", "security": "31347", "commands": ["1003", "2E012601"]},
                {"id": "puddlelights", "name_key": "vag_tweak_puddlelights", "module": "09", "security": "31347", "commands": ["1003", "2E012701"]}
            ]
        }

        self.STELLANTIS_TWEAKS_CONFIG = {
            "Opel": {
                "Astra J / Insignia A": [
                    {"id": "opel_bc_unlock", "name_key": "stl_op_bc_unlock", "module": "IPC (0x244)", "commands": ["1003", "3B010101"]},
                    {"id": "opel_oil_reset_uds", "name_key": "stl_op_oil_reset", "module": "ECM", "commands": ["1003", "2E02C601"]},
                    {"id": "opel_tpms_learn", "name_key": "stl_op_tpms", "module": "BCM", "commands": ["1003", "31010001"]}
                ],
                "Astra K / Insignia B": [
                    {"id": "opel_needle_uds", "name_key": "stl_op_needle", "module": "IPC", "commands": ["1003", "2E060D01"]},
                    {"id": "opel_eco_off", "name_key": "stl_op_eco", "module": "BCM", "commands": ["1003", "2E010200"]},
                    {"id": "opel_theme", "name_key": "stl_opel_theme", "module": "IPC", "commands": ["1003", "2E08A201"]}
                ],
                "Corsa E": [
                    {"id": "opel_rev_camera", "name_key": "stl_op_camera", "module": "DISP", "commands": ["1003", "2E098201"]}
                ]
            },
            "Fiat": {
                "Tipo / 500X": [
                    {"id": "fiat_srv_reset", "name_key": "stl_fi_srv_reset", "module": "BCM", "commands": ["1003", "31020004"]},
                    {"id": "fiat_dpf_regen", "name_key": "stl_fi_dpf", "module": "ECM", "commands": ["1003", "31010005"]},
                    {"id": "fiat_learn_proxy", "name_key": "stl_fi_proxy", "module": "BCM", "commands": ["1080", "1081", "31010203"]},
                    {"id": "fiat_corner", "name_key": "stl_fiat_cornering", "module": "BCM", "commands": ["1003", "2E054201"]}
                ]
            },
            "PSA (Peugeot/Citroen)": {
                "308 / 508 / C4": [
                    {"id": "psa_mirror_fold", "name_key": "stl_psa_mirror", "module": "BSI", "commands": ["1003", "2E013401"]},
                    {"id": "psa_drl_menu", "name_key": "stl_psa_drl", "module": "BSI", "commands": ["1003", "2E054101"]},
                    {"id": "psa_vim", "name_key": "stl_psa_video", "module": "NAC", "commands": ["1003", "2E065201"]},
                    {"id": "psa_diag_screen", "name_key": "stl_psa_diag", "module": "SMEG", "commands": ["1003", "2E091101"]},
                    {"id": "psa_startstop", "name_key": "stl_psa_startstop", "module": "BSI", "commands": ["1003", "2E010200"]}
                ]
            },
            "Alfa Romeo": {
                "Giulia / Stelvio": [
                    {"id": "alfa_sgw", "name_key": "stl_alfa_sgw", "module": "SGW", "commands": ["1003", "31010201"]},
                    {"id": "alfa_race", "name_key": "stl_alfa_dyn", "module": "BCM", "commands": ["1003", "2E054501"]},
                    {"id": "alfa_seatbelt", "name_key": "stl_alfa_seatbelt", "module": "IPC", "commands": ["1003", "2E033100"]},
                    {"id": "alfa_logo", "name_key": "stl_alfa_logo", "module": "ETM", "commands": ["1003", "2E098402"]}
                ],
                "MiTo": [
                    {"id": "alfa_mito_needle", "name_key": "stl_alfa_mito_needle", "module": "IPC", "commands": ["1003", "2E060D01"]},
                    {"id": "alfa_mito_fog", "name_key": "stl_alfa_mito_fog", "module": "BCM", "commands": ["1003", "2E054201"]}
                ]
            },
            "Jeep": {
                "Renegade / Compass": [
                    {"id": "jeep_drl", "name_key": "stl_jeep_drl", "module": "BCM", "commands": ["1003", "2E023101"]},
                    {"id": "jeep_horn", "name_key": "stl_jeep_horn", "module": "BCM", "commands": ["1003", "2E012401"]},
                    {"id": "jeep_offroad", "name_key": "stl_jeep_offroad", "module": "RADIO", "commands": ["1003", "2E0CC201"]}
                ]
            },
            "Dodge": {
                "Challenger / Charger": [
                    {"id": "dg_srt_pages", "name_key": "stl_dg_srt", "module": "7E0", "commands": ["1003", "2E010A01"]},
                    {"id": "dg_drl_turn", "name_key": "stl_dg_drl", "module": "7A6", "commands": ["1003", "2E023402"]},
                    {"id": "dg_belt_chime", "name_key": "stl_dg_belt", "module": "7A0", "commands": ["1003", "2E033100"]}
                ]
            },
            "RAM": {
                "1500 / 2500": [
                    {"id": "ram_trailer", "name_key": "stl_ram_trailer", "module": "7A6", "commands": ["1003", "2E054401"]},
                    {"id": "ram_cargo", "name_key": "stl_ram_cargo", "module": "7E0", "commands": ["1003", "2E0CC401"]},
                    {"id": "ram_fold", "name_key": "stl_ram_fold", "module": "7A6", "commands": ["1003", "2E012801"]}
                ]
            },
            "DS Automobiles": {
                "DS 7 / DS 3": [
                    {"id": "ds_ambient", "name_key": "stl_ds_ambient", "module": "BSI", "commands": ["1003", "2E014502"]},
                    {"id": "ds_matrix", "name_key": "stl_ds_matrix", "module": "BSI", "commands": ["1003", "2E067101"]}
                ]
            },
            "Maserati": {
                "Ghibli / Levante": [
                    {"id": "mas_valves", "name_key": "stl_mas_valves", "module": "7E0", "commands": ["1003", "2E010F01"]},
                    {"id": "mas_stopstart", "name_key": "stl_mas_stopstart", "module": "7A6", "commands": ["1003", "2E011200"]}
                ]
            },
            "Lancia": {
                "Ypsilon / Delta": [
                    {"id": "lan_city", "name_key": "stl_lan_city", "module": "7A6", "commands": ["1003", "2E054801"]},
                    {"id": "lan_srv", "name_key": "stl_lan_srv", "module": "7A6", "commands": ["1003", "31020004"]}
                ]
            }
        }
        
        self.alarm_queue = queue.Queue()
        self.hud_mode = False
        self._alarm_ui_elements = {} # Do dynamicznej aktualizacji
        self.active_graphs = ["rpm", "speed", "temp"]
        self._is_scrubbing = False
        self._scrubbing_after_id = None
        self.vag_restore_buttons = {} # Dynamiczna aktualizacja po kodowaniu
        
        # Wątek dźwiękowy
        threading.Thread(target=self._alarm_sound_worker, daemon=True).start()
        
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        
        self.load_settings() # Wczytaj przed budową GUI
        
        self._build_sidebar()
        self._build_main_area()
        self.update_ui_text() 
        self._current_override = None
        self.update_digital_indicators()
        self._sync_dashboard_indicators() # Synchronizuj przyciski ze stanem backendu
        
        # Uruchomienie animacji na końcu, by nie zawieszać innych komponentów Tkinter
        if hasattr(self, 'fig'):
            # Interwał 300ms dla płynniejszego działania na słabszych systemach (zmniejszenie obciążenia wątku GUI)
            self.ani = animation.FuncAnimation(self.fig, self.update_graphs, interval=300, cache_frame_data=False)
        
        # Obsługa zamykania
        self._indicators_after_id = None
        self._replay_bar_after_id = None
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def save_settings(self):
        """Zapisuje bieżące ustawienia do pliku JSON."""
        settings = {
            "lang": self.current_lang,
            "appearance": ctk.get_appearance_mode(),
            "refresh_rate": self.gui_refresh_rate,
            "show_anim": self.show_anim,
            "units": "imperial" if self.backend.use_imperial else "metric",
            "alarm_enabled": self.alarm_enabled,
            "csv_interval": self.backend.csv_interval,
            "hud_mode": self.hud_mode,
            "vehicle_type": self.backend.vehicle_type,
            "fuel_type": self.backend.fuel_type,
            "com_port": self.port_var.get() if hasattr(self, 'port_var') else "AUTO",
            "baudrate": self.baudrate_var.get() if hasattr(self, 'baudrate_var') else "38400",
            "alarms": {k: {"active": v["active"], "limit": v["limit"]} for k, v in self.alarm_configs.items()}
        }
        try:
            with open("settings_obd.json", "w", encoding="utf-8") as f:
                json.dump(settings, f, indent=4)
        except Exception as e:
            print(f"Error saving settings: {e}")

    def reset_settings(self):
        """Usuwa plik ustawień i resetuje do domyślnych."""
        texts = LOCALIZATION[self.current_lang]
        if not messagebox.askyesno(texts["reset_settings_title"], texts["reset_settings_confirm"]):
            return
        try:
            if os.path.exists("settings_obd.json"):
                os.remove("settings_obd.json")
        except Exception as e:
            messagebox.showerror(texts["err_title"], str(e))
            return
        messagebox.showinfo(texts["reset_settings_title"], texts["reset_settings_done"])

    def load_settings(self):
        """Wczytuje ustawienia z pliku JSON."""
        if not os.path.exists("settings_obd.json"):
            return
        try:
            with open("settings_obd.json", "r", encoding="utf-8") as f:
                s = json.load(f)
                self.current_lang = s.get("lang", self.current_lang)
                ctk.set_appearance_mode(s.get("appearance", "dark"))
                self.gui_refresh_rate = s.get("refresh_rate", self.gui_refresh_rate)
                self.show_anim = s.get("show_anim", self.show_anim)
                self.backend.use_imperial = (s.get("units") == "imperial")
                self.alarm_enabled = s.get("alarm_enabled", self.alarm_enabled)
                self.backend.csv_interval = s.get("csv_interval", self.backend.csv_interval)
                self.hud_mode = s.get("hud_mode", self.hud_mode)
                self.backend.vehicle_type = s.get("vehicle_type", self.backend.vehicle_type)
                self.backend.fuel_type = s.get("fuel_type", self.backend.fuel_type)
                if "com_port" in s and hasattr(self, 'port_var'):
                    self.port_var.set(s["com_port"])
                if "baudrate" in s and hasattr(self, 'baudrate_var'):
                    self.baudrate_var.set(s["baudrate"])
                if "alarms" in s:
                    for k, v in s["alarms"].items():
                        if k in self.alarm_configs:
                            self.alarm_configs[k]["active"] = v["active"]
                            self.alarm_configs[k]["limit"] = v["limit"]
        except Exception as e:
            print(f"Error loading settings: {e}")

    def _build_sidebar(self):
        texts = LOCALIZATION[self.current_lang]
        # 1. Panel Boczny (Zawsze widoczny pasek boczny)
        self.sidebar = ctk.CTkFrame(self, width=280, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.sidebar.grid_rowconfigure(8, weight=1)
        
        self.logo_label = ctk.CTkLabel(self.sidebar, text="OBD_MASTER_PRO_V7", font=ctk.CTkFont(size=24, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(30, 20))
        
        # Wybór Języka (Flag)
        self.lang_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        self.lang_frame.grid(row=1, column=0, padx=20, pady=(0, 20), sticky="ew")
        
        self.lang_pl_btn = ctk.CTkButton(self.lang_frame, text="Polski", width=75, height=32, command=lambda: self.change_lang("pl"))
        self.lang_pl_btn.pack(side="left", padx=2)
        
        self.lang_en_btn = ctk.CTkButton(self.lang_frame, text="English", width=75, height=32, command=lambda: self.change_lang("en"))
        self.lang_en_btn.pack(side="left", padx=2)
        
        self.lang_de_btn = ctk.CTkButton(self.lang_frame, text="Deutsch", width=75, height=32, command=lambda: self.change_lang("de"))
        self.lang_de_btn.pack(side="left", padx=2)

        # Przełącznik Wirtualny/Prawdziwy
        self.switch_var = ctk.StringVar(value="virtual" if self.backend.use_virtual else "real")
        self.virtual_switch = ctk.CTkSwitch(self.sidebar, text=texts["sim_switch"], variable=self.switch_var, onvalue="virtual", offvalue="real", command=self.toggle_virtual)
        self.virtual_switch.grid(row=2, column=0, padx=20, pady=(0, 20), sticky="w")
        if self.backend.use_virtual: self.virtual_switch.select()
        else: self.virtual_switch.deselect()
        self.toggle_virtual() 
        
        self.connect_btn = ctk.CTkButton(self.sidebar, text="Connect to OBD Device", height=45, font=ctk.CTkFont(weight="bold", size=14), command=self.connect_obd)
        self.connect_btn.grid(row=3, column=0, padx=20, pady=(10, 10), sticky="ew")
        
        self.logging_btn = ctk.CTkButton(self.sidebar, text="Start Data Logger", height=45, state="disabled", font=ctk.CTkFont(size=14), command=self.toggle_logging)
        self.logging_btn.grid(row=4, column=0, padx=20, pady=10, sticky="ew")

        # Status Ĺączności
        self.statusFrame = ctk.CTkFrame(self.sidebar, corner_radius=10)
        self.statusFrame.grid(row=5, column=0, padx=20, pady=20, sticky="ew")
        
        self.status_header_lbl = ctk.CTkLabel(self.statusFrame, text="Interface Status:", font=ctk.CTkFont(size=12))
        self.status_header_lbl.pack(anchor="center", pady=(10,0))
        self.status_label = ctk.CTkLabel(self.statusFrame, text="DISCONNECTED", text_color="#FF4C4C", font=ctk.CTkFont(size=18, weight="bold"))
        self.status_label.pack(anchor="center", pady=(5, 2))
        # Mały label na szczegóły błędu â€“ przycięty i zawijany, nie przesuwa layoutu
        self.error_detail_lbl = ctk.CTkLabel(
            self.statusFrame, text="",
            font=ctk.CTkFont(size=10), text_color="#FF8888",
            wraplength=230, justify="center"
        )
        self.error_detail_lbl.pack(anchor="center", pady=(0, 8))

    def update_status_labels(self):
        """Aktualizuje oba wskaźniki statusu (Sidebar + Dashboard) zgodnie z logiką zapisaną w lokalizacji."""
        texts = LOCALIZATION[self.current_lang]
        
        # Logika priorytetów stanów
        if self.backend.is_replay:
            if self.backend.is_replay_paused:
                status_text = texts.get("replay_paused", "PAUSE")
                color = ("#B28700", "#FFD700")
            else:
                status_text = texts["state_replay"]
                color = ("#D35400", "#ff8c00")
        elif self.backend.is_logging:
            status_text = texts["state_logging"]
            color = ("#D8000C", "#FF4C4C")
        elif self.backend.is_connected_loop:
            status_text = texts["state_live"]
            color = ("#007700", "#4CFF4C")
        else:
            status_text = texts["state_idle"]
            color = ("#444444", "#555555") if self.current_lang != "en" else ("#555555", "gray") # Subtle for idle
            
        self.status_label.configure(text=status_text, text_color=color)

        # Stopka
        self.signature = ctk.CTkLabel(self.sidebar, text="OBD_MASTER_PRO_V7", font=ctk.CTkFont(size=11, slant="italic"), text_color=("#333333", "gray"))
        self.signature.grid(row=9, column=0, padx=20, pady=20, sticky="s")

    def change_lang(self, lang_code):
        self.current_lang = lang_code
        self.update_ui_text()
        self.save_settings()

    def update_ui_text(self):
        texts = LOCALIZATION[self.current_lang]
        self.title(texts["title"])
        
        # Sidebar
        self.virtual_switch.configure(text=texts["sim_switch"])
        if not self.backend.is_connected_loop:
            self.connect_btn.configure(text=texts["connect"])
            self.status_label.configure(text=texts["disconnected"])
        else:
            self.connect_btn.configure(text=texts["disconnect"])
            self.status_label.configure(text=texts["connected"])
            
        if not self.backend.is_logging:
            self.logging_btn.configure(text=texts["start_logger"])
        else:
            self.logging_btn.configure(text=texts["stop_logger"])
            
        self.status_header_lbl.configure(text=texts["status_header"])
        self.update_status_labels()
        
        if hasattr(self, "del_all_btn"):
            self.del_all_btn.configure(text=texts.get("del_all_btn", "Delete All"))
            
        self.signature.configure(text="OBD_MASTER_PRO_V7")
        
        if hasattr(self, "logo_label"):
            self.logo_label.configure(text=texts.get("sidebar_logo", "OBD_MASTER_PRO_V7"))
        
        # Tab titles (Robust Dynamic localization)
        try:
            titles = [
                texts["tab_live"], texts["tab_dtc"], texts["tab_vag"], 
                texts.get("tab_stl", "Stellantis"), texts["tab_logs"], texts["tab_settings"]
            ]
            # Accessing the button widgets directly to change their visible text
            tab_buttons = list(self.tabview._segmented_button._buttons_dict.values())
            for i, btn in enumerate(tab_buttons):
                if i < len(titles):
                    btn.configure(text=titles[i])
        except Exception as e:
            print(f"Error updating tab titles: {e}")
        
        # Diagnostics
        self.btn_scan_dtc.configure(text=texts["scan_dtc_btn"])
        self.btn_clear_dtc.configure(text=texts["clear_dtc_btn"])
        # Update text box content if it has the default message
        self.dtc_text.configure(state="normal")
        # Check if current content matches any of the headers
        current_content = self.dtc_text.get("0.0", "end")
        for l in LOCALIZATION.values():
            if l["dtc_header"] in current_content:
                self.dtc_text.delete("0.0", "end")
                self.dtc_text.insert("0.0", texts["dtc_header"])
                break
        self.dtc_text.configure(state="disabled")

        # Dashboard labels
        self.drivetrain_lbl.configure(text=texts["drivetrain"])
        self.fuel_type_lbl.configure(text=texts["fuel_type"])
        
        # Dashboard Buttons (ICE/HEV/EV and Petrol/Diesel)
        for code, (btn, af, ifo) in self._vtype_btns.items():
            if code == VehicleType.ICE: btn.configure(text=texts["ice"])
            elif code == VehicleType.HEV: btn.configure(text=texts["hev"])
            elif code == VehicleType.EV: btn.configure(text=texts["ev"])
            
        for lbl_key, btn in self._fuel_btns.items():
            btn.configure(text=texts[lbl_key])

        # Card Labels - Force update even if not connected
        is_ev = self.backend.vehicle_type == VehicleType.EV
        is_imp = self.backend.use_imperial
        
        if is_ev:
            self.lbl_rpm.configure(text=texts["ev_power_lbl"])
            self.lbl_soc.configure(text=texts["bat_state_lbl"])
            self.lbl_maf.configure(text=texts["bat_volt_lbl"])
            if is_imp:
                self.lbl_temp.configure(text=texts["temp_bat_f"])
                self.lbl_speed.configure(text=texts["speed_f"])
                self.lbl_trans.configure(text=texts["temp_ev_f"])
            else:
                self.lbl_temp.configure(text=texts["temp_bat_c"])
                self.lbl_speed.configure(text=texts["speed_c"])
                self.lbl_trans.configure(text=texts["temp_ev_c"])
        else:
            self.lbl_rpm.configure(text=texts["rpm_lbl"])
            self.lbl_maf.configure(text=texts["maf_lbl"])
            if is_imp:
                self.lbl_temp.configure(text=texts["temp_f"])
                self.lbl_speed.configure(text=texts["speed_f"])
                self.lbl_trans.configure(text=texts["temp_trans_f"])
                self.lbl_soc.configure(text=texts["cons_f"])
            else:
                self.lbl_temp.configure(text=texts["temp_c"])
                self.lbl_speed.configure(text=texts["speed_c"])
                self.lbl_trans.configure(text=texts["temp_trans_c"])
                self.lbl_soc.configure(text=texts["cons_c"])

        self.lbl_load.configure(text=texts["engine_load"])
        self.lbl_throttle.configure(text=texts["throttle_pos"])
        self.lbl_hev_v.configure(text=texts["hev_voltage"])
        self.lbl_hev_kw.configure(text=texts["ev_power"])
        self.lbl_12v.configure(text=texts["v12_voltage"])
        self.lbl_drive_mode.configure(text=texts["drive_mode"])

        # Update Session tab
        self.session_header_lbl.configure(text=texts["session_history"])
        self.refresh_btn.configure(text=texts["refresh_btn"])
        if hasattr(self, 'reset_settings_btn'):
            self.reset_settings_btn.configure(text=texts["reset_settings_btn"])
        self.refresh_sessions() # Refresh logs list to apply new language

        # Update Alarms in Settings
        for key, elements in self._alarm_ui_elements.items():
            cfg = self.alarm_configs[key]
            elements["label"].configure(text=f"{texts['alarm_' + key]}:")
            m_text = texts["alarm_above"] if cfg["mode"] == "max" else texts["alarm_below"]
            elements["mode"].configure(text=m_text)

        # Settings labels
        self.settings_info_lbl.configure(text=texts["settings_info"])
        self.appearance_lbl.configure(text=texts["appearance"])
        self.refresh_rate_lbl.configure(text=texts["refresh_label"])
        self.com_port_lbl.configure(text=texts["com_port"])
        self.com_hint_lbl.configure(text=texts["com_hint"])
        self.units_lbl.configure(text=texts["units_label"])
        self.alarm_config_lbl.configure(text=texts["alarm_config"])
        self.csv_rate_lbl.configure(text=texts["csv_rate"])
        self.theme_seg.configure(values=[texts["theme_dark"], texts["theme_light"], texts["theme_system"]])
        mode = ctk.get_appearance_mode().lower()
        self.theme_seg.set(texts.get("theme_" + mode, texts["theme_dark"]))

        self.refresh_seg.configure(values=[texts["r_fast"], texts["r_normal"], texts["r_eco"]])
        if self.gui_refresh_rate == 200: self.refresh_seg.set(texts["r_fast"])
        elif self.gui_refresh_rate == 500: self.refresh_seg.set(texts["r_normal"])
        else: self.refresh_seg.set(texts["r_eco"])

        self.unit_seg.configure(values=[texts["u_metric"], texts["u_imperial"]])
        self.unit_seg.set(texts["u_imperial"] if self.backend.use_imperial else texts["u_metric"])

        self.csv_seg.configure(values=[texts["csv_freq"], texts["csv_std"], texts["csv_eco"]])
        if self.backend.csv_interval == 0.5: self.csv_seg.set(texts["csv_freq"])
        elif self.backend.csv_interval == 2.0: self.csv_seg.set(texts["csv_std"])
        else: self.csv_seg.set(texts["csv_eco"])

        self.alarm_switch.configure(text=texts["master_switch"])

        # --- VAG Specialist tab live translations ---
        for key, widget in getattr(self, "_vag_i18n_widgets", {}).items():
            try:
                widget.configure(text=texts.get(key, key))
            except Exception:
                pass
        # Update per-tweak buttons and labels
        for tweak_id, refs in getattr(self, "_vag_tweak_widgets", {}).items():
            try:
                refs["name_lbl"].configure(text=texts.get(refs["name_key"], tweak_id))
                refs["apply_btn"].configure(text=texts.get("vag_btn_apply", "Apply"))
                refs["restore_btn"].configure(text=texts.get("vag_btn_restore", "Restore"))
            except Exception:
                pass
        # Update category headers
        for cat_key, lbl in getattr(self, "_vag_cat_labels", {}).items():
            try:
                lbl.configure(text=texts.get(f"vag_category_{cat_key}", cat_key).upper())
            except Exception:
                pass

        # --- Stellantis Specialist tab live translations ---
        for key, widget in getattr(self, "_stl_i18n_widgets", {}).items():
            try:
                widget.configure(text=texts.get(key, key))
            except Exception: pass
        # Update per-tweak buttons and labels
        for tweak_id, refs in getattr(self, "_stl_tweak_widgets", {}).items():
            try:
                refs["name_lbl"].configure(text=texts.get(refs["name_key"], tweak_id))
                refs["apply_btn"].configure(text=texts.get("stl_btn_apply", "Apply"))
                refs["restore_btn"].configure(text=texts.get("stl_btn_restore", "Restore"))
            except Exception: pass
        # Refresh Stellantis tweak list to update states if needed
        try:
            self._on_stl_model_changed(self.stl_model_menu.get())
        except Exception: pass

        # Card Labels
        self.update_digital_indicators()


    def _build_main_area(self):
        texts = LOCALIZATION[self.current_lang]
        self.tabview = ctk.CTkTabview(self, width=250)
        self.tabview.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        
        self.tabview.add("Dashboard")
        self.tabview.add("Diagnostics")
        self.tabview.add("VAG Specialist")
        self.tabview.add("Stellantis Specialist")
        self.tabview.add("Sessions")
        self.tabview.add("Settings")
        
        self.tabview.set("Dashboard")
        
        self._build_dashboard_tab()
        self._build_diagnostics_tab()
        self._build_vag_tab()
        self._build_stellantis_tab()
        self._build_sessions_tab()
        self._build_settings_tab()


    def _build_dashboard_tab(self):
        texts = LOCALIZATION[self.current_lang]
        tab = self.tabview.tab("Dashboard")
        # row 0  = karty wskaźników     (nie rośnie)
        # row 1  = pasek napęd/paliwo    (nie rośnie)
        # row 2  = animacja auta          (nie rośnie)
        # row 3  = wykresy               (rośnie)
        tab.grid_rowconfigure(0, weight=0)
        tab.grid_rowconfigure(1, weight=0)
        tab.grid_rowconfigure(2, weight=0)  # graph selector
        tab.grid_rowconfigure(3, weight=1)  # graphs
        tab.grid_rowconfigure(4, weight=0)  # replay bar
        tab.grid_columnconfigure(0, weight=1)
        
        # --- BLOK WSKAĹąNIĂ“KĂ“W (Karty) ---
        self.cards_frame = ctk.CTkFrame(tab, fg_color="transparent")
        self.cards_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        # Update to 6 columns with uniform width to prevent overlaps
        for i in range(6):
            self.cards_frame.grid_columnconfigure(i, weight=1, uniform="equal_cols")
            
        self.graph_vars = {}
        self.graph_checkboxes = {}
        def on_graph_toggle(key):
            # Pobieramy stan z checkboxa, który został kliknięty
            new_state = self.graph_vars[key].get()
            
            if new_state: # Zaznaczono nowy wykres
                if key not in self.active_graphs:
                    self.active_graphs.append(key)
            else: # Odznaczono wykres
                if key in self.active_graphs:
                    self.active_graphs.remove(key)
            
            # Wymuszenie odświeżenia layoutu wykresów
            self._needs_layout = True
            if hasattr(self, '_on_resize_ref'):
                self._on_resize_ref()
            
            # Aktualizacja listy w backendzie dla logowania
            self.backend.active_graphs_list = list(self.active_graphs)
                
        def create_card(parent, title, u_color, r, c, graph_key=None):
            # Fixed height to prevent "clashing" or "jumping" when labels wrap in different languages
            frame = ctk.CTkFrame(parent, corner_radius=15, border_width=1, border_color="#45456b", fg_color="#1a1a2e", height=82)
            frame.grid(row=r, column=c, padx=3, pady=5, sticky="nsew")
            frame.pack_propagate(False) # CRITICAL: stops frame from expanding with content
            parent.grid_rowconfigure(r, weight=1)
            
            # Using smaller fonts for a clean, unified look
            val_lbl = ctk.CTkLabel(frame, text="0", font=ctk.CTkFont(family="Consolas", size=20, weight="bold"), text_color=u_color)
            val_lbl.pack(pady=(8, 0), expand=True)
            title_lbl = ctk.CTkLabel(frame, text=title, font=ctk.CTkFont(size=9, weight="bold"), text_color=("#333333", "gray"), wraplength=110, justify="center")
            title_lbl.pack(pady=(0, 10), side="bottom")
            
            if graph_key:
                var = ctk.BooleanVar(value=(graph_key in self.active_graphs))
                self.graph_vars[graph_key] = var
                cb = ctk.CTkCheckBox(
                    frame, text="", variable=var, 
                    command=lambda k=graph_key: on_graph_toggle(k), 
                    width=15, height=15, checkbox_width=18, checkbox_height=18,
                    corner_radius=4, border_width=2, fg_color="#3a7ebf", hover_color="#325882"
                )
                cb.place(relx=0.92, rely=0.08, anchor="ne")  # Floating top-right corner
                self.graph_checkboxes[graph_key] = cb
                
            return val_lbl, title_lbl

        # Rząd 1: 6 cards (Podstawowe + Skrzynia + MAF + Load)
        self.val_rpm, self.lbl_rpm = create_card(self.cards_frame, texts["engine_revs"], ("#007700", "#00FF00"), 0, 0, "rpm")
        self.val_speed, self.lbl_speed = create_card(self.cards_frame, texts["vehicle_speed"], ("#007700", "#00FF00"), 0, 1, "speed")
        self.val_temp, self.lbl_temp = create_card(self.cards_frame, texts["coolant_temp"], ("#007700", "#00FF00"), 0, 2, "temp")
        self.val_trans, self.lbl_trans = create_card(self.cards_frame, texts["trans_temp"], ("#007700", "#00FF00"), 0, 3, "trans_temp")
        self.val_maf, self.lbl_maf = create_card(self.cards_frame, texts["maf_flow"], ("#007700", "#00FF00"), 0, 4, "maf")
        self.val_load, self.lbl_load = create_card(self.cards_frame, texts["engine_load"], ("#007700", "#00FF00"), 0, 5, "load")
        
        # Rząd 2: 6 cards (Przepustnica + Hybryda + 12V + Drive Mode)
        self.val_throttle, self.lbl_throttle = create_card(self.cards_frame, texts["throttle_pos"], ("#007700", "#00FF00"), 1, 0, "throttle")
        self.val_soc, self.lbl_soc = create_card(self.cards_frame, texts["battery_soc"], ("#007700", "#00FF00"), 1, 1, "hev_soc")
        self.val_hev_v, self.lbl_hev_v = create_card(self.cards_frame, texts["hev_voltage"], ("#007700", "#00FF00"), 1, 2, "hev_volts")
        self.val_hev_kw, self.lbl_hev_kw = create_card(self.cards_frame, texts["ev_power"], ("#007700", "#00FF00"), 1, 3, "hev_power")
        self.val_12v, self.lbl_12v = create_card(self.cards_frame, texts["v12_voltage"], ("#007700", "#00FF00"), 1, 4, "voltage_12v")
        self.val_drive_mode, self.lbl_drive_mode = create_card(self.cards_frame, texts["drive_mode"], ("#007700", "#00FF00"), 1, 5, None)
        self.val_drive_mode.configure(text="ECO") # Dummy
        
        # --- SELEKTOR POJAZDU NA DASHBOARDZIE ---
        vtype_bar = ctk.CTkFrame(tab, fg_color=("#E8E8EB", "#161623"), corner_radius=15, height=54, border_width=1, border_color=("#C8C8CC", "#2d2d44"))
        vtype_bar.grid(row=1, column=0, sticky="ew", padx=10, pady=(2, 2))
        vtype_bar.pack_propagate(False)
        vtype_bar.grid_propagate(False)

        # Typ napędu â€“ prosta etykieta + 3 przyciski
        self.drivetrain_lbl = ctk.CTkLabel(vtype_bar, text=texts["drivetrain"], font=ctk.CTkFont(size=13, weight="bold"), text_color="#888888")
        self.drivetrain_lbl.pack(side="left", padx=(14, 6))

        self._vtype_btns = {}
        _VTYPE_OPTIONS = [
            (texts["ice"],       "ICE",  "#C1440E", "#8B3A1E"),
            (texts["hev"],    "HEV",  "#27AE60", "#1A5C2E"),
            (texts["ev"],  "EV",   "#3498DB", "#1A2E5C"),
        ]
        for label, code, active_fg, inactive_fg in _VTYPE_OPTIONS:
            b = ctk.CTkButton(
                vtype_bar, text=label, width=110, height=32,
                corner_radius=8, font=ctk.CTkFont(size=12, weight="bold"),
                fg_color=inactive_fg, hover_color=active_fg,
                command=lambda c=code, af=active_fg, ifo=inactive_fg: self._select_vtype(c, af, ifo)
            )
            b.pack(side="left", padx=4, pady=8)
            self._vtype_btns[code] = (b, active_fg, inactive_fg)

        # Separator
        ctk.CTkFrame(vtype_bar, width=2, height=30, fg_color="#333344").pack(side="left", padx=8)

        # Paliwo
        self.fuel_type_lbl = ctk.CTkLabel(vtype_bar, text=texts["fuel_type"], font=ctk.CTkFont(size=13, weight="bold"), text_color="#888888")
        self.fuel_type_lbl.pack(side="left", padx=(4, 6))

        self._fuel_active = texts["petrol"]
        self._fuel_btns = {}
        for flabel_key in ["petrol", "diesel"]:
            fb = ctk.CTkButton(
                vtype_bar, text=texts[flabel_key], width=100, height=32,
                corner_radius=8, font=ctk.CTkFont(size=12, weight="bold"),
                fg_color="#2a3a50", hover_color="#3a7ebf",
                command=lambda k=flabel_key: self._select_fuel_by_key(k)
            )
            fb.pack(side="left", padx=4, pady=8)
            self._fuel_btns[flabel_key] = fb
        self.fuel_seg = None  # kept for compatibility

        # self._sync_dashboard_indicators()  # Usunięte - funkcja nie istnieje, powodowała crash



        # --- WYKRESY (Matplotlib) W ZAKĹADCE ---

        # --- WYKRESY (Matplotlib) W ZAKĹADCE ---

        self.graph_frame = ctk.CTkScrollableFrame(tab, corner_radius=10, fg_color="transparent")
        self.graph_frame.grid(row=3, column=0, sticky="nsew", padx=10, pady=5)
        
        bg_color = "#F0F0F0" if ctk.get_appearance_mode() == "Light" else "#2b2b2b"
        self.fig = Figure(facecolor=bg_color)
        self.fig.subplots_adjust(left=0.06, right=0.98, top=0.96, bottom=0.06, hspace=0.3)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.configure(borderwidth=0, highlightthickness=0)
        self.canvas_widget.pack(fill="x", expand=False, pady=0)
        
        # Responsywność: przelicz layout po każdej zmianie rozmiaru okna
        def _on_resize(event=None):
            try:
                # Mierzymy szerokość z zewnętrznego taba, bo CTkScrollableFrame.winfo_width()
                # zwraca nieprawidłową wartość wewnątrz scrollable frame
                w = tab.winfo_width() - 55  # margines na scrollbar + paddingi
                active_count = max(1, len(self.active_graphs))
                
                # Każdy wykres ma 250px wysokości, minimum 350px
                h = max(350, active_count * 250)
                
                if w > 50:
                    dpi = self.fig.get_dpi()
                    self.fig.set_size_inches(w / dpi, h / dpi, forward=True)
                    # Ustaw rozmiar canvas widgetu zgodnie z figurą
                    try:
                        self.canvas_widget.configure(width=w, height=h)
                    except: pass
                    self.canvas.draw_idle()
            except Exception:
                pass
        
        # Bind do tab (zewnętrzny kontener) - on dostaje poprawne wymiary po resize okna
        tab.bind("<Configure>", _on_resize, add="+")
        self._on_resize_ref = _on_resize
        
        # FuncAnimation przeniesione na koniec __init__ by nie blokowało UI
        self.update_digital_indicators()

        # --- PASEK ODTWARZANIA REPLAY (ukryty domyślnie) ---
        self.replay_bar = ctk.CTkFrame(tab, fg_color="transparent", height=72)
        # Rezerwujemy miejsce na stałe, by nie przesuwało wykresów (doliczamy do minsize)
        self.replay_bar.grid(row=4, column=0, sticky="ew", padx=10, pady=(2, 6))
        self.replay_bar.grid_columnconfigure(1, weight=1)

        # Przycisk Stop
        self.replay_stop_btn = ctk.CTkButton(
            self.replay_bar, text="◼", width=44, height=36,
            font=ctk.CTkFont(size=20), fg_color="#B22222", hover_color="#8B0000",
            command=self._replay_stop
        )
        self.replay_stop_btn.grid(row=0, column=0, padx=(10, 4), pady=9)

        # Przycisk Play/Pause (toggle)
        self.replay_pause_btn = ctk.CTkButton(
            self.replay_bar, text="▶", width=44, height=36,
            font=ctk.CTkFont(size=20), fg_color="#1f538d", hover_color="#3a7ebf",
            command=self._replay_toggle_pause
        )
        self.replay_pause_btn.grid(row=0, column=2, padx=(4, 10), pady=9)

        # Timeline (slider przeciągalny + etykieta czasu)
        self._replay_timeline_frame = ctk.CTkFrame(self.replay_bar, fg_color="transparent")
        self._replay_timeline_frame.grid(row=0, column=1, sticky="ew", padx=6)
        self._replay_timeline_frame.grid_columnconfigure(0, weight=1)

        self.replay_progress = ctk.CTkSlider(
            self._replay_timeline_frame,
            from_=0, to=1, number_of_steps=200,
            height=16,
            command=self._replay_seek
        )
        self.replay_progress.set(0)
        self.replay_progress.bind("<Button-1>", self._on_slider_press)
        self.replay_progress.bind("<ButtonRelease-1>", self._on_slider_release)
        self.replay_progress.grid(row=0, column=0, sticky="ew", pady=(10, 2))

        # Pasek markerów alarmów pod sliderem
        self.replay_marker_canvas = tk.Canvas(
            self._replay_timeline_frame,
            height=10, bg="#0d0d1a", highlightthickness=0
        )
        self.replay_marker_canvas.grid(row=1, column=0, sticky="ew", pady=(0, 2))
        self.replay_marker_canvas.bind(
            "<Configure>",
            lambda e: self._draw_replay_markers()
        )

        self.replay_time_lbl = ctk.CTkLabel(
            self._replay_timeline_frame, text="0:00 / 0:00",
            font=ctk.CTkFont(size=13, weight="bold"), text_color="#aaaaaa"
        )
        self.replay_time_lbl.grid(row=2, column=0, pady=(0, 6))
        
        # Ukrywamy dzieci na start (miejsce paska zostaje zarezerwowane przez height=72 i grid)
        for child in self.replay_bar.winfo_children():
            child.grid_remove()

    def _build_diagnostics_tab(self):
        texts = LOCALIZATION[self.current_lang]
        tab = self.tabview.tab("Diagnostics")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)
        
        # Pasek narzędzi
        toolbar = ctk.CTkFrame(tab, fg_color="transparent")
        toolbar.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 0))
        
        self.btn_scan_dtc = ctk.CTkButton(toolbar, text=texts["scan_dtc_btn"], width=180, height=40, font=ctk.CTkFont(weight="bold"), command=self.action_scan_dtc)
        self.btn_scan_dtc.pack(side="left", padx=5)
        
        self.btn_export_dtc = ctk.CTkButton(toolbar, text="💾 Export CSV", width=120, height=40, font=ctk.CTkFont(weight="bold"), command=self.action_export_dtc, fg_color="#1f538d", hover_color="#3a7ebf")
        self.btn_export_dtc.pack(side="left", padx=5)
        
        self.btn_clear_dtc = ctk.CTkButton(toolbar, text=texts["clear_dtc_btn"], fg_color="#B22222", hover_color="#8B0000", width=220, height=40, font=ctk.CTkFont(weight="bold"), command=self.action_clear_dtc)
        self.btn_clear_dtc.pack(side="right", padx=5)

        # Okno wyników
        self.dtc_text = ctk.CTkTextbox(tab)
        self.dtc_text.grid(row=1, column=0, sticky="nsew", padx=20, pady=20)
        self.dtc_text.insert("0.0", texts["dtc_header"])
        self.dtc_text.configure(state="disabled")

    def _build_vag_tab(self):
        """Buduje zakładkę VAG Specialist."""
        texts = LOCALIZATION[self.current_lang]
        tab = self.tabview.tab("VAG Specialist")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)

        self._vag_i18n_widgets = {}
        self._vag_tweak_widgets = {}
        self._vag_cat_labels = {}

        warn_frame = ctk.CTkFrame(tab, fg_color=("#FFE4C4", "#3d2b1f"), border_width=1, border_color=("#FF9900", "#ff8c00"))
        warn_frame.grid(row=0, column=0, sticky="ew", padx=20, pady=10)
        ctk.CTkLabel(warn_frame, text="⚠️ VAG SPECIALIST - EXPERT MODE", font=ctk.CTkFont(weight="bold"), text_color="#ff8c00").pack(pady=(5, 0))
        vag_warn_lbl = ctk.CTkLabel(warn_frame, text=texts["vag_warning_text"], font=ctk.CTkFont(size=11), wraplength=800)
        vag_warn_lbl.pack(pady=10, padx=20)
        self._vag_i18n_widgets["vag_warning_text"] = vag_warn_lbl

        scroll = ctk.CTkScrollableFrame(tab, fg_color="transparent")
        scroll.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        scroll.grid_columnconfigure(0, weight=1)

        row_idx = 0
        for cat_key, tweaks in self.VAG_TWEAKS_CONFIG.items():
            cat_lbl = ctk.CTkLabel(scroll, text=texts.get(f"vag_category_{cat_key}", cat_key).upper(), font=ctk.CTkFont(size=14, weight="bold"), text_color="#3a7ebf")
            cat_lbl.grid(row=row_idx, column=0, sticky="w", padx=15, pady=(15, 5))
            self._vag_cat_labels[cat_key] = cat_lbl
            row_idx += 1
            for t in tweaks:
                t_frame = ctk.CTkFrame(scroll, fg_color=("#F2F2F5", "#1a1a2e"), border_width=1, border_color=("#D0D0D5", "#2b2b3d"))
                t_frame.grid(row=row_idx, column=0, sticky="ew", padx=10, pady=3)
                t_frame.grid_columnconfigure(0, weight=1)
                name = texts.get(t["name_key"], t["id"])
                lbl = ctk.CTkLabel(t_frame, text=name, font=ctk.CTkFont(weight="bold"))
                lbl.grid(row=0, column=0, sticky="w", padx=15, pady=10)
                # Restore button first (left)
                last_cmd = str(t["commands"][-1]) if t["commands"] else ""
                did = last_cmd[2:6] if last_cmd.startswith("2E") else "0000"
                has_backup = self.backend.get_vag_backup(t["module"], did) is not None
                btn_restore = ctk.CTkButton(t_frame, text=texts["vag_btn_restore"], width=140, fg_color="#5F1E1E", hover_color="#8B1E1E", state="normal" if has_backup else "disabled", command=lambda tweak=t: self.action_restore_vag_tweak(tweak))
                btn_restore.grid(row=0, column=1, padx=10, pady=10)
                
                # Apply button last (right)
                btn = ctk.CTkButton(t_frame, text=texts["vag_btn_apply"], width=140, fg_color="#1E3A5F", hover_color="#2a4a75", command=lambda tweak=t: self.action_execute_vag_tweak(tweak))
                btn.grid(row=0, column=2, padx=10, pady=10)
                self.vag_restore_buttons[t["id"]] = btn_restore
                self._vag_tweak_widgets[t["id"]] = {"name_lbl": lbl, "apply_btn": btn, "restore_btn": btn_restore, "name_key": t["name_key"]}
                row_idx += 1

        expert_frame = ctk.CTkFrame(tab, height=120)
        expert_frame.grid(row=2, column=0, sticky="ew", padx=20, pady=10)
        expert_desc_lbl = ctk.CTkLabel(expert_frame, text=texts["vag_expert_desc"], font=ctk.CTkFont(weight="bold"))
        expert_desc_lbl.grid(row=0, column=0, columnspan=4, pady=5)
        self._vag_i18n_widgets["vag_expert_desc"] = expert_desc_lbl
        expert_hdr_lbl = ctk.CTkLabel(expert_frame, text=texts["vag_header_lbl"])
        expert_hdr_lbl.grid(row=1, column=0, padx=5)
        self._vag_i18n_widgets["vag_header_lbl"] = expert_hdr_lbl
        self.vag_expert_header = ctk.CTkEntry(expert_frame, width=80, placeholder_text="714")
        self.vag_expert_header.grid(row=1, column=1, padx=5)
        expert_cmd_lbl = ctk.CTkLabel(expert_frame, text=texts["vag_command_lbl"])
        expert_cmd_lbl.grid(row=1, column=2, padx=5)
        self._vag_i18n_widgets["vag_command_lbl"] = expert_cmd_lbl
        self.vag_expert_cmd = ctk.CTkEntry(expert_frame, width=250, placeholder_text="2E 06 0D 01")
        self.vag_expert_cmd.grid(row=1, column=3, padx=5)
        self._vag_expert_send_btn = ctk.CTkButton(expert_frame, text=texts["vag_btn_send"], width=100, command=self.action_vag_send_hex)
        self._vag_expert_send_btn.grid(row=1, column=4, padx=10, pady=10)
        self._vag_i18n_widgets["vag_btn_send"] = self._vag_expert_send_btn

        tab.grid_rowconfigure(3, weight=0)
        proto_frame = ctk.CTkFrame(tab, fg_color=("#E8E8EB", "#161623"), border_width=1, border_color=("#C8C8CC", "#2d2d44"), corner_radius=12)
        proto_frame.grid(row=3, column=0, sticky="ew", padx=20, pady=(4, 4))
        proto_hdr_lbl = ctk.CTkLabel(proto_frame, text=texts.get("vag_protocol_header", "🔧 VAG Protocol"), font=ctk.CTkFont(size=13, weight="bold"), text_color="#3a7ebf")
        proto_hdr_lbl.pack(side="left", padx=(16, 10), pady=10)
        self._vag_i18n_widgets["vag_protocol_header"] = proto_hdr_lbl
        self._vag_protocol_var = ctk.StringVar(value="UDS")
        self._proto_uds_btn = ctk.CTkRadioButton(proto_frame, text=texts.get("vag_proto_uds", "CAN UDS"), variable=self._vag_protocol_var, value="UDS", font=ctk.CTkFont(size=12))
        self._proto_uds_btn.pack(side="left", padx=10, pady=10)
        self._vag_i18n_widgets["vag_proto_uds"] = self._proto_uds_btn
        self._proto_tp2_btn = ctk.CTkRadioButton(proto_frame, text=texts.get("vag_proto_tp20", "CAN TP2.0"), variable=self._vag_protocol_var, value="TP20", font=ctk.CTkFont(size=12), text_color="#ff9900")
        self._proto_tp2_btn.pack(side="left", padx=10, pady=10)
        self._vag_i18n_widgets["vag_proto_tp20"] = self._proto_tp2_btn

        tab.grid_rowconfigure(4, weight=0)
        zdc_frame = ctk.CTkFrame(tab, fg_color=("#F2F2F5", "#1a1a2e"), border_width=1, border_color=("#FF9900", "#ff8c00"), corner_radius=12)
        zdc_frame.grid(row=4, column=0, sticky="ew", padx=20, pady=(4, 12))
        zdc_hdr_lbl = ctk.CTkLabel(zdc_frame, text=texts.get("zdc_section_header", "📁 Dataset Loader"), font=ctk.CTkFont(size=13, weight="bold"), text_color="#ff8c00")
        zdc_hdr_lbl.grid(row=0, column=0, sticky="w", padx=15, pady=(10, 4), columnspan=5)
        self._vag_i18n_widgets["zdc_section_header"] = zdc_hdr_lbl
        zdc_mod_lbl = ctk.CTkLabel(zdc_frame, text=texts.get("zdc_module_label", "Module:"))
        zdc_mod_lbl.grid(row=1, column=0, padx=(15, 5), pady=8, sticky="w")
        self._vag_i18n_widgets["zdc_module_label"] = zdc_mod_lbl
        self.zdc_module_entry = ctk.CTkEntry(zdc_frame, width=60, placeholder_text="01")
        self.zdc_module_entry.grid(row=1, column=1, padx=5, pady=8)
        self._zdc_filepath = ctk.StringVar(value="")
        self.zdc_file_lbl = ctk.CTkLabel(zdc_frame, textvariable=self._zdc_filepath, text_color="#aaaaaa", width=320, anchor="w")
        self.zdc_file_lbl.grid(row=1, column=2, padx=10, pady=8, sticky="ew")
        self._zdc_browse_btn = ctk.CTkButton(zdc_frame, text=texts.get("zdc_choose_file", "📁 Choose ZDC"), width=150, fg_color="#2a3a50", hover_color="#3a7ebf", command=self._action_zdc_browse)
        self._zdc_browse_btn.grid(row=1, column=3, padx=8, pady=8)
        self._vag_i18n_widgets["zdc_choose_file"] = self._zdc_browse_btn
        self._zdc_load_btn = ctk.CTkButton(zdc_frame, text=texts.get("zdc_upload", "⬆️ Upload Dataset"), width=140, fg_color="#8B3A1E", hover_color="#5C1F0A", font=ctk.CTkFont(weight="bold"), command=self._action_zdc_load)
        self._zdc_load_btn.grid(row=1, column=4, padx=(4, 15), pady=8)
        self._vag_i18n_widgets["zdc_upload"] = self._zdc_load_btn

    def _build_stellantis_tab(self):
        """Buduje zakładkę Stellantis Specialist."""
        texts = LOCALIZATION[self.current_lang]
        tab = self.tabview.tab("Stellantis Specialist")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(2, weight=1)

        self._stl_i18n_widgets = {}
        self._stl_tweak_widgets = {}
        self.stl_restore_buttons = {}

        # 1. Nagłówek i Selektory
        header_frame = ctk.CTkFrame(tab, fg_color="transparent")
        header_frame.grid(row=0, column=0, sticky="ew", padx=20, pady=10)

        brand_lbl = ctk.CTkLabel(header_frame, text=texts.get("stl_brand_lbl", "Marka:"), font=ctk.CTkFont(weight="bold"))
        brand_lbl.pack(side="left", padx=5)
        self._stl_i18n_widgets["stl_brand_lbl"] = brand_lbl

        self.stl_brand_menu = ctk.CTkOptionMenu(header_frame, values=list(self.STELLANTIS_TWEAKS_CONFIG.keys()), command=self._on_stl_brand_changed)
        self.stl_brand_menu.pack(side="left", padx=10)

        model_lbl = ctk.CTkLabel(header_frame, text=texts.get("stl_model_lbl", "Model:"), font=ctk.CTkFont(weight="bold"))
        model_lbl.pack(side="left", padx=5)
        self._stl_i18n_widgets["stl_model_lbl"] = model_lbl

        self.stl_model_menu = ctk.CTkOptionMenu(header_frame, values=["-"], command=self._on_stl_model_changed)
        self.stl_model_menu.pack(side="left", padx=10)

        # 2. Ostrzeżenie
        warn_frame = ctk.CTkFrame(tab, fg_color=("#FFE4C4", "#3d2b1f"), border_width=1, border_color=("#FF9900", "#ff8c00"))
        warn_frame.grid(row=1, column=0, sticky="ew", padx=20, pady=5)
        self.stl_warn_lbl = ctk.CTkLabel(warn_frame, text=texts.get("stl_warning", "UWAGA: Tryb Ekspert Stellantis. Tylko dla aut z szyną CAN."), font=ctk.CTkFont(size=11), wraplength=800)
        self.stl_warn_lbl.pack(pady=10, padx=20)
        self._stl_i18n_widgets["stl_warning"] = self.stl_warn_lbl

        # 3. Lista Tweaków (Scrollable)
        self.stl_tweak_scroll = ctk.CTkScrollableFrame(tab, fg_color="transparent")
        self.stl_tweak_scroll.grid(row=2, column=0, sticky="nsew", padx=10, pady=5)
        
        # --- Stellantis Expert Mode Frame ---
        expert_frame = ctk.CTkFrame(tab, fg_color=("#F2F2F5", "#1a1a2e"), border_width=1, border_color="#1E3A5F", corner_radius=12)
        expert_frame.grid(row=3, column=0, sticky="ew", padx=20, pady=(12, 4))
        
        expert_desc = ctk.CTkLabel(expert_frame, text=texts.get("stl_expert_desc", "TRYB EKSPERCKI (STELLANTIS)"), font=ctk.CTkFont(size=12, weight="bold"))
        expert_desc.grid(row=0, column=0, columnspan=5, padx=15, pady=(10, 5), sticky="w")
        self._stl_i18n_widgets["stl_expert_desc"] = expert_desc

        ctk.CTkLabel(expert_frame, text=texts["vag_header_lbl"]).grid(row=1, column=0, padx=(15, 5), pady=8, sticky="w")
        self.stl_expert_header = ctk.CTkEntry(expert_frame, width=80, placeholder_text="714")
        self.stl_expert_header.grid(row=1, column=1, padx=5, pady=8)

        ctk.CTkLabel(expert_frame, text=texts["vag_command_lbl"]).grid(row=1, column=2, padx=(15, 5), pady=8, sticky="w")
        self.stl_expert_cmd = ctk.CTkEntry(expert_frame, width=300, placeholder_text="2E060D01")
        self.stl_expert_cmd.grid(row=1, column=3, padx=5, pady=8)

        self._stl_expert_send_btn = ctk.CTkButton(expert_frame, text=texts["vag_btn_send"], width=100, command=self.action_stl_send_hex)
        self._stl_expert_send_btn.grid(row=1, column=4, padx=(5, 15), pady=8)
        self._stl_i18n_widgets["vag_btn_send"] = self._stl_expert_send_btn

        # --- Stellantis Dataset Loader Frame ---
        zdc_frame = ctk.CTkFrame(tab, fg_color=("#F2F2F5", "#1a1a2e"), border_width=1, border_color="#5C1F0A", corner_radius=12)
        zdc_frame.grid(row=4, column=0, sticky="ew", padx=20, pady=(4, 12))
        
        zdc_hdr = ctk.CTkLabel(zdc_frame, text=texts.get("zdc_section_header", "Dataset Loader"), font=ctk.CTkFont(size=12, weight="bold"), text_color="#ff8c00")
        zdc_hdr.grid(row=0, column=0, columnspan=5, padx=15, pady=(10, 5), sticky="w")
        self._stl_i18n_widgets["zdc_section_header"] = zdc_hdr

        ctk.CTkLabel(zdc_frame, text=texts["zdc_module_label"]).grid(row=1, column=0, padx=(15, 5), pady=8, sticky="w")
        self.stl_zdc_module_entry = ctk.CTkEntry(zdc_frame, width=60, placeholder_text="01")
        self.stl_zdc_module_entry.grid(row=1, column=1, padx=5, pady=8)

        self._stl_zdc_filepath = ctk.StringVar(value="")
        self.stl_zdc_file_lbl = ctk.CTkLabel(zdc_frame, textvariable=self._stl_zdc_filepath, text_color="#aaaaaa", width=320, anchor="w")
        self.stl_zdc_file_lbl.grid(row=1, column=2, padx=10, pady=8, sticky="ew")

        self._stl_zdc_browse_btn = ctk.CTkButton(zdc_frame, text=texts.get("zdc_choose_file", "📁 Wybierz"), width=130, fg_color="#2a3a50", command=self._action_stl_zdc_browse)
        self._stl_zdc_browse_btn.grid(row=1, column=3, padx=5, pady=8)
        self._stl_i18n_widgets["zdc_choose_file"] = self._stl_zdc_browse_btn

        self._stl_zdc_load_btn = ctk.CTkButton(zdc_frame, text=texts.get("zdc_upload", "⬆️ Wgraj"), width=130, fg_color="#8B3A1E", command=self._action_stl_zdc_load)
        self._stl_zdc_load_btn.grid(row=1, column=4, padx=(5, 15), pady=8)
        self._stl_i18n_widgets["zdc_upload"] = self._stl_zdc_load_btn

        # Inicjalizacja listy modeli
        # Wait, self.stl_brand_menu needs to be assigned before _on_stl_brand_changed
        self._on_stl_brand_changed(self.stl_brand_menu.get())

    def action_stl_send_hex(self):
        texts = LOCALIZATION[self.current_lang]
        if not self.backend.connection or not self.backend.connection.is_connected():
            messagebox.showwarning(texts["err_no_conn"], texts["err_no_conn_msg"])
            return
        header = self.stl_expert_header.get().strip() or "714"
        cmd = self.stl_expert_cmd.get().strip()
        if not cmd: return
        ok, res = self.backend.execute_uds_custom(header, [cmd])
        if ok: messagebox.showinfo(texts.get("success", "Sukces"), res)
        else: messagebox.showerror(texts.get("err_title", "Błąd"), res)

    def _action_stl_zdc_browse(self):
        texts = LOCALIZATION[self.current_lang]
        import tkinter.filedialog as filedialog
        path = filedialog.askopenfilename(title=texts["zdc_choose_file"], filetypes=[("Dataset", "*.zdc *.hex *.txt"), ("All", "*.*")])
        if path: self._stl_zdc_filepath.set(path)

    def _action_stl_zdc_load(self):
        texts = LOCALIZATION[self.current_lang]
        path = self._stl_zdc_filepath.get()
        mod = self.stl_zdc_module_entry.get().strip()
        if not path or not mod:
            messagebox.showwarning(texts.get("err_title", "Błąd"), texts.get("zdc_no_file", "Wybierz plik i moduł!"))
            return
        try:
            with open(path, 'r') as f: lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]
            if messagebox.askyesno(texts.get("zdc_confirm_title", "Wgrać?"), texts.get("zdc_confirm_msg", "Wgrać {} komend?").format(len(lines))):
                ok, res = self.backend.execute_uds_custom(mod, lines)
                if ok: messagebox.showinfo(texts.get("success", "Sukces"), texts.get("zdc_success", "Wgrano pomyślnie!"))
                else: messagebox.showerror(texts.get("err_title", "Błąd"), res)
        except Exception as e:
            messagebox.showerror(texts.get("err_title", "Błąd"), str(e))

    def _on_stl_brand_changed(self, brand):
        models = list(self.STELLANTIS_TWEAKS_CONFIG.get(brand, {}).keys())
        if not models: models = ["-"]
        self.stl_model_menu.configure(values=models)
        self.stl_model_menu.set(models[0])
        self._on_stl_model_changed(models[0])

    def _on_stl_model_changed(self, model):
        for child in self.stl_tweak_scroll.winfo_children():
            child.destroy()
        brand = self.stl_brand_menu.get()
        tweaks = self.STELLANTIS_TWEAKS_CONFIG.get(brand, {}).get(model, [])
        texts = LOCALIZATION[self.current_lang]
        if not tweaks:
            ctk.CTkLabel(self.stl_tweak_scroll, text=texts.get("stl_no_tweaks", "Brak dostępnych tweaków dla tego modelu.")).pack(pady=20)
            return
        for tweak in tweaks:
            f = ctk.CTkFrame(self.stl_tweak_scroll, corner_radius=10, border_width=1, border_color="#45456b", fg_color="#1a1a2e")
            f.pack(fill="x", padx=10, pady=5)
            lbl_name = ctk.CTkLabel(f, text=texts.get(tweak["name_key"], tweak["id"]), font=ctk.CTkFont(weight="bold"))
            lbl_name.pack(side="left", padx=15, pady=12)
            lbl_mod = ctk.CTkLabel(f, text=f"({tweak['module']})", font=ctk.CTkFont(size=10, slant="italic"), text_color="gray")
            lbl_mod.pack(side="left", padx=2)
            
            # Check for backup to set restore button state
            did_to_check = ""
            for cmd_raw in reversed(tweak.get("commands", [])):
                cmd_str = str(cmd_raw).replace(" ","")
                if cmd_str.startswith("2E") and len(cmd_str) >= 6:
                    did_to_check = cmd_str[2:6]; break
            
            has_backup = False
            if did_to_check:
                # Use UDS header mapping for search
                hdr = "714"
                m_str = str(tweak.get("module", ""))
                if "ECM" in m_str: hdr = "7E0"
                elif "BCM" in m_str or "BSI" in m_str: hdr = "7A6"
                elif "IPC" in m_str: hdr = "7A0"
                elif len(m_str) == 3 and m_str.isalnum(): hdr = m_str
                has_backup = self.backend.get_vag_backup(hdr, did_to_check) is not None
            
            btn_apply = ctk.CTkButton(f, text=texts.get("stl_btn_apply", "Wykonaj"), width=100, fg_color="#1E3A5F", command=lambda t=tweak: self._execute_stl_tweak(t))
            btn_apply.pack(side="right", padx=10, pady=10)
            
            btn_restore = ctk.CTkButton(f, text=texts.get("stl_btn_restore", "Przywróć"), width=100, fg_color="#5F1E1E", state="normal" if has_backup else "disabled", command=lambda t=tweak: self._restore_stl_tweak(t))
            btn_restore.pack(side="right", padx=5, pady=10)
            
            self.stl_restore_buttons[tweak["id"]] = btn_restore
            self._stl_tweak_widgets[tweak["id"]] = {"name_lbl": lbl_name, "apply_btn": btn_apply, "restore_btn": btn_restore, "name_key": tweak["name_key"]}

    def _execute_stl_tweak(self, tweak):
        texts = LOCALIZATION[self.current_lang]
        if messagebox.askyesno(texts.get("stl_confirm_title", "Potwierdzenie"), texts.get("stl_confirm_msg", "Czy wykonać ten tweak?")):
            ok, res = self.backend.execute_uds_custom(tweak["module"], tweak["commands"])
            if ok:
                messagebox.showinfo(texts.get("success", "Sukces"), texts.get("stl_tweak_applied", "Tweak zastosowany!"))
                # Enable restore button if it was disabled
                if tweak["id"] in self.stl_restore_buttons:
                    self.stl_restore_buttons[tweak["id"]].configure(state="normal")
            else:
                messagebox.showerror(texts.get("err_title", "Błąd"), f"{texts.get('stl_tweak_error', 'Błąd')}: {res}")

    def _restore_stl_tweak(self, tweak):
        texts = LOCALIZATION[self.current_lang]
        if messagebox.askyesno(texts.get("stl_confirm_title", "Potwierdzenie"), texts.get("stl_restore_confirm_msg", "Przywrócić oryginał?")):
            # Find DID
            did_to_restore = ""
            for cmd_raw in reversed(tweak.get("commands", [])):
                cmd_str = str(cmd_raw).replace(" ","")
                if cmd_str.startswith("2E") and len(cmd_str) >= 6:
                    did_to_restore = cmd_str[2:6]; break
            
            if not did_to_restore:
                messagebox.showerror(texts.get("err_title", "Błąd"), "Nie można określić identyfikatora DID dla przywracania.")
                return

            # Resolve header
            hdr = "714"
            m_str = str(tweak.get("module", ""))
            if "ECM" in m_str: hdr = "7E0"
            elif "BCM" in m_str or "BSI" in m_str: hdr = "7A6"
            elif "IPC" in m_str: hdr = "7A0"
            elif len(m_str) == 3 and m_str.isalnum(): hdr = m_str

            orig_val = self.backend.get_vag_backup(hdr, did_to_restore)
            if not orig_val:
                messagebox.showerror(texts.get("err_title", "Błąd"), texts.get("stl_no_backup", "Brak kopii zapasowej dla tego modułu!"))
                return

            restore_cmd = f"2E{did_to_restore}{orig_val}"
            ok, res = self.backend.execute_uds_custom(hdr, ["1003", restore_cmd], is_restore=True)
            if ok:
                messagebox.showinfo(texts.get("success", "Sukces"), texts.get("stl_tweak_restored", "Przywrócono!"))
            else:
                messagebox.showerror(texts.get("err_title", "Błąd"), res)

    def action_execute_vag_tweak(self, tweak):
        texts = LOCALIZATION[self.current_lang]
        if not self.backend.connection or not self.backend.connection.is_connected():
            messagebox.showwarning(texts["err_no_conn"], texts["err_no_conn_msg"])
            return
        
        name = texts.get(tweak["name_key"], tweak["id"])
        if not messagebox.askyesno(texts.get("vag_tweak_confirm_title", "Potwierdzenie"), texts.get("vag_tweak_confirm_msg", "Czy na pewno chcesz wykonać ten tweak?") + f"\n\n{name}"):
            return
            
        # Wykonaj komendę (TP2.0 dla większości z listy)
        ok, res = self.backend.execute_vag_tp20_command(tweak["module"], tweak["commands"], tweak.get("security"))
        
        if ok:
            messagebox.showinfo(texts.get("success", "Sukces"), f"{name}: {texts.get('vag_tweak_applied', 'Tweak zastosowany!')}\n{res}")
            # Aktywuj przycisk przywracania
            if tweak["id"] in self.vag_restore_buttons:
                self.vag_restore_buttons[tweak["id"]].configure(state="normal")
        else:
            messagebox.showerror(texts.get("err_title", "Błąd"), f"{name}: {texts.get('vag_tweak_error', 'Błąd podczas kodowania.')}\n{res}")

    def action_restore_vag_tweak(self, tweak):
        texts = LOCALIZATION[self.current_lang]
        if not self.backend.connection or not self.backend.connection.is_connected():
            messagebox.showwarning(texts["err_no_conn"], texts["err_no_conn_msg"])
            return
            
        did = tweak["commands"][-1][2:6]
        name = texts.get(tweak["name_key"], tweak["id"])
        
        if not messagebox.askyesno(texts.get("vag_tweak_confirm_title", "Potwierdzenie"), texts.get("vag_restore_confirm_msg", "Czy przywrócić ustawienia fabryczne?") + f"\n\n{name}"):
            return

        ok, res = self.backend.execute_vag_restore(tweak["module"], did, tweak.get("security"))
        if ok:
            messagebox.showinfo(texts.get("success", "Sukces"), f"{name}: {texts.get('vag_tweak_restored', 'Przywrócono oryginał!')}\n{res}")
        else:
            messagebox.showerror(texts.get("err_title", "Błąd"), f"{name}: {res}")

    def action_vag_send_hex(self):
        texts = LOCALIZATION[self.current_lang]
        if not self.backend.connection or not self.backend.connection.is_connected():
            messagebox.showwarning(texts["err_no_conn"], texts["err_no_conn_msg"])
            return
            
        header = self.vag_expert_header.get().strip() or "714"
        cmd = self.vag_expert_cmd.get().strip()
        if not cmd: return
        
        proto = self._vag_protocol_var.get()
        if proto == "TP20":
            ok, res = self.backend.execute_vag_tp20_command(header, [cmd])
        else:
            # UDS custom handle
            ok, res = self.backend.execute_uds_custom(header, [cmd])
            
        if ok:
            messagebox.showinfo(texts.get("success", "Sukces"), res)
        else:
            messagebox.showerror(texts.get("err_title", "Błąd"), res)

    def _action_zdc_browse(self):
        """Open file dialog to select a .ZDC dataset file."""
        texts = LOCALIZATION[self.current_lang]
        import tkinter.filedialog as filedialog
        path = filedialog.askopenfilename(
            title=texts["zdc_choose_file"],
            filetypes=[("ZDC Dataset", "*.zdc *.ZDC *.hex *.HEX *.txt"), ("All files", "*.*")]
        )
        if path:
            self._zdc_filepath.set(path)

    def _action_zdc_load(self):
        """Parse and upload a ZDC dataset file to the selected module."""
        texts = LOCALIZATION[self.current_lang]
        filepath = self._zdc_filepath.get()
        module_id = self.zdc_module_entry.get().strip().upper()

        if not filepath:
            messagebox.showwarning(texts.get("zdc_section_header", "Dataset Loader"), texts["zdc_no_file"])
            return
        if not module_id:
            messagebox.showwarning(texts.get("zdc_section_header", "Dataset Loader"), texts["zdc_no_module"])
            return
        if not self.backend.connection or not self.backend.connection.is_connected():
            messagebox.showwarning(texts["err_title"], texts["err_no_conn_msg"])
            return

        ok, result = self.backend.load_zdc_file(filepath)
        if not ok:
            messagebox.showerror(texts.get("zdc_section_header", "Dataset Loader"), f"{texts['zdc_parse_error']}\n{result}")
            return

        commands = result  # list of hex strings
        if not messagebox.askyesno(
            texts["zdc_confirm_title"],
            texts["zdc_confirm_msg"].replace("{n}", str(len(commands))).replace("{m}", module_id)
        ):
            return

        proto = getattr(self, "_vag_protocol_var", None)
        use_tp20 = proto and proto.get() == "TP20"
        if use_tp20:
            success, results = self.backend.execute_vag_tp20_command(module_id, commands)
        else:
            success, results = self.backend.execute_vag_command(module_id, commands)

        if success:
            messagebox.showinfo(texts.get("zdc_section_header", "Dataset Loader"), texts["zdc_success"].replace("{m}", module_id) + f"\n\nOdpowiedzi: {results[:200]}")
        else:
            messagebox.showerror(texts.get("zdc_section_header", "Dataset Loader"), texts["zdc_error"] + f"\n{results}")

    def _build_sessions_tab(self):
        texts = LOCALIZATION[self.current_lang]
        tab = self.tabview.tab("Sessions")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)
        
        # Pasek górny
        top = ctk.CTkFrame(tab, fg_color="transparent")
        top.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
        self.session_header_lbl = ctk.CTkLabel(top, text=texts["session_history"], font=ctk.CTkFont(size=20, weight="bold"))
        self.session_header_lbl.pack(side="left")
        self.refresh_btn = ctk.CTkButton(top, text=texts["refresh_btn"], command=self.refresh_sessions)
        self.refresh_btn.pack(side="right")
        
        self.del_all_btn = ctk.CTkButton(top, text=texts.get("del_all_btn", "Delete All"), fg_color="#B22222", hover_color="#8B0000", width=120, command=self.delete_all_sessions)
        self.del_all_btn.pack(side="right", padx=(0, 10))
        
        # Przewijana przestrzeń dla plików
        self.sessions_scroll = ctk.CTkScrollableFrame(tab)
        self.sessions_scroll.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
        
        self.refresh_sessions()

    def _build_settings_tab(self):
        texts = LOCALIZATION[self.current_lang]
        tab = self.tabview.tab("Settings")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(0, weight=1)
        
        # MAIN SCROLL (Responsiveness Fix)
        self.settings_scroll = ctk.CTkScrollableFrame(tab, fg_color="transparent")
        self.settings_scroll.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        self.settings_scroll.grid_columnconfigure(0, weight=1)

        # --- PASEK GĂ“RNY: RESET ---
        top_bar = ctk.CTkFrame(self.settings_scroll, fg_color="transparent")
        top_bar.pack(fill="x", padx=30, pady=(10, 0))
        self.reset_settings_btn = ctk.CTkButton(
            top_bar, text=texts["reset_settings_btn"],
            fg_color=("#A54B2A", "#8B3A1E"), hover_color=("#7A2F15", "#5C1F0A"),
            height=36, font=ctk.CTkFont(size=13, weight="bold"),
            command=self.reset_settings
        )
        self.reset_settings_btn.pack(side="right")
        
        # Info Section
        sg = ctk.CTkFrame(self.settings_scroll, corner_radius=10)
        sg.pack(fill="x", padx=30, pady=20)
        self.settings_info_lbl = ctk.CTkLabel(sg, text=texts["settings_info"], font=ctk.CTkFont(size=14))
        self.settings_info_lbl.pack(anchor="w", padx=20, pady=20)
        
        # Theme
        vg = ctk.CTkFrame(self.settings_scroll, corner_radius=10)
        vg.pack(fill="x", padx=30, pady=10)
        self.appearance_lbl = ctk.CTkLabel(vg, text=texts["appearance"], font=ctk.CTkFont(size=18, weight="bold"))
        self.appearance_lbl.pack(anchor="w", padx=20, pady=(20, 10))
        
        def _apply_theme(v):
            texts_now = LOCALIZATION[self.current_lang]
            mode_map = {
                texts_now["theme_dark"]: "dark",
                texts_now["theme_light"]: "light",
                texts_now["theme_system"]: "system"
            }
            ctk.set_appearance_mode(mode_map.get(v, "dark"))
            self.save_settings()

        self.theme_seg = ctk.CTkSegmentedButton(vg, values=[texts["theme_dark"], texts["theme_light"], texts["theme_system"]], command=_apply_theme)
        self.theme_seg.set(texts["theme_dark"])
        self.theme_seg.pack(anchor="w", padx=20, pady=(0, 20))
        
        # GUI Refresh
        rg = ctk.CTkFrame(self.settings_scroll, corner_radius=10)
        rg.pack(fill="x", padx=30, pady=10)
        self.refresh_rate_lbl = ctk.CTkLabel(rg, text=texts["refresh_label"], font=ctk.CTkFont(size=18, weight="bold"))
        self.refresh_rate_lbl.pack(anchor="w", padx=20, pady=(20, 10))
        self.refresh_seg = ctk.CTkSegmentedButton(rg, values=[texts["r_fast"], texts["r_normal"], texts["r_eco"]], command=self.change_refresh_rate)
        self.refresh_seg.set(texts["r_fast"])
        self.refresh_seg.pack(anchor="w", padx=20, pady=(0, 20))
        
        # COM Port
        pg = ctk.CTkFrame(self.settings_scroll, corner_radius=10)
        pg.pack(fill="x", padx=30, pady=10)
        self.com_port_lbl = ctk.CTkLabel(pg, text=texts["com_port"], font=ctk.CTkFont(size=18, weight="bold"))
        self.com_port_lbl.pack(anchor="w", padx=20, pady=(10, 5))
        
        com_row = ctk.CTkFrame(pg, fg_color="transparent")
        com_row.pack(anchor="w", padx=20, pady=5)
        
        self.port_var = ctk.StringVar(value="AUTO")
        self.port_entry = ctk.CTkEntry(com_row, textvariable=self.port_var, width=150)
        self.port_entry.pack(side="left", padx=(0, 12))
        
        ctk.CTkLabel(com_row, text="Baudrate:", font=ctk.CTkFont(size=13), text_color=("#333333", "gray")).pack(side="left", padx=(0, 6))
        self.baudrate_var = ctk.StringVar(value="38400")
        self.baudrate_menu = ctk.CTkOptionMenu(
            com_row,
            variable=self.baudrate_var,
            values=["AUTO", "9600", "38400", "115200"],
            width=110,
            command=lambda _: self.save_settings()
        )
        self.baudrate_menu.pack(side="left")
        
        self.com_hint_lbl = ctk.CTkLabel(pg, text=texts["com_hint"], font=ctk.CTkFont(size=12), text_color=("#333333", "gray"))
        self.com_hint_lbl.pack(anchor="w", padx=20, pady=(0, 10))

        # Units
        sg2 = ctk.CTkFrame(self.settings_scroll, corner_radius=10)
        sg2.pack(fill="x", padx=30, pady=10)
        self.units_lbl = ctk.CTkLabel(sg2, text=texts["units_label"], font=ctk.CTkFont(size=18, weight="bold"))
        self.units_lbl.pack(anchor="w", padx=20, pady=(10, 5))
        self.unit_seg = ctk.CTkSegmentedButton(sg2, values=[texts["u_metric"], texts["u_imperial"]], command=self.change_units)
        self.unit_seg.set(texts["u_metric"])
        self.unit_seg.pack(anchor="w", padx=20, pady=(0, 10))
        
        # Alarms Config
        ag2 = ctk.CTkFrame(self.settings_scroll, corner_radius=10)
        ag2.pack(fill="x", padx=30, pady=10)
        ag2.grid_columnconfigure(0, weight=1)
        
        top_a = ctk.CTkFrame(ag2, fg_color="transparent")
        top_a.pack(fill="x", padx=10, pady=10)
        self.alarm_config_lbl = ctk.CTkLabel(top_a, text=texts["alarm_config"], font=ctk.CTkFont(size=18, weight="bold"))
        self.alarm_config_lbl.pack(side="left")
        
        self.alarm_switch = ctk.CTkSwitch(top_a, text=texts["master_switch"], command=self.toggle_alarms)
        if self.alarm_enabled: self.alarm_switch.select()
        else: self.alarm_switch.deselect()
        self.alarm_switch.pack(side="right")
        
        self.alarm_list_frame = ctk.CTkFrame(ag2, fg_color="transparent")
        self.alarm_list_frame.pack(fill="x", padx=10, pady=(0, 10))
        
        self._alarm_vars = {} 
        
        for key, cfg in self.alarm_configs.items():
            row = ctk.CTkFrame(self.alarm_list_frame, fg_color="transparent")
            row.pack(fill="x", pady=2)
            
            a_var = tk.BooleanVar(value=cfg["active"])
            cb = ctk.CTkCheckBox(row, text="", variable=a_var, width=20, command=self.update_alarms_list)
            cb.pack(side="left", padx=5)
            
            lbl_text = f"{texts['alarm_' + key]}:"
            lbl = ctk.CTkLabel(row, text=lbl_text, width=150, anchor="w")
            lbl.pack(side="left")
            
            m_text = texts["alarm_above"] if cfg["mode"] == "max" else texts["alarm_below"]
            m_lbl = ctk.CTkLabel(row, text=m_text, font=ctk.CTkFont(size=10), text_color=("#333333", "gray"), width=60)
            m_lbl.pack(side="left")
            
            l_var = ctk.StringVar(value=str(cfg["limit"]))
            entry = ctk.CTkEntry(row, textvariable=l_var, width=70)
            entry.pack(side="left", padx=5)
            l_var.trace_add("write", self.update_alarms_list)
            
            self._alarm_vars[key] = (a_var, l_var)
            self._alarm_ui_elements[key] = {"label": lbl, "mode": m_lbl}
        
        # CSV Logging
        cg = ctk.CTkFrame(self.settings_scroll, corner_radius=10)
        cg.pack(fill="x", padx=30, pady=10)
        self.csv_rate_lbl = ctk.CTkLabel(cg, text=texts["csv_rate"], font=ctk.CTkFont(size=18, weight="bold"))
        self.csv_rate_lbl.pack(anchor="w", padx=20, pady=(10, 5))
        self.csv_seg = ctk.CTkSegmentedButton(cg, values=[texts["csv_freq"], texts["csv_std"], texts["csv_eco"]], command=self.change_csv_rate)
        self.csv_seg.set(texts["csv_freq"])
        self.csv_seg.pack(anchor="w", padx=20, pady=(0, 10))
        
    def _sync_dashboard_indicators(self):
        """Aktualizuje przyciski typu pojazdu i paliwa zgodnie ze stanem backendu."""
        vt = self.backend.vehicle_type
        if vt is None:
            # Nie zaznaczaj nic, jeśli typ nie jest ustawiony
            for c, (btn, af, ifo) in self._vtype_btns.items():
                btn.configure(fg_color=ifo, border_width=0)
            for k, btn in self._fuel_btns.items():
                btn.configure(fg_color="#2a3a50", border_width=0)
            return

        if vt == VehicleType.EV:
            self._select_vtype(VehicleType.EV, "#3498DB", "#1A2E5C")
        elif vt == VehicleType.HEV:
            self._select_vtype(VehicleType.HEV, "#27AE60", "#1A5C2E")
        else:
            self._select_vtype(VehicleType.ICE, "#C1440E", "#8B3A1E")
            
        fk = "petrol" if self.backend.fuel_type == FuelType.PETROL else ("diesel" if self.backend.fuel_type == FuelType.DIESEL else None)
        if fk:
            self._select_fuel_by_key(fk)
        else:
            for k, btn in self._fuel_btns.items():
                btn.configure(fg_color="#2a3a50", border_width=0)
        
    # ---- EVENTY USTAWIEN PRO ----
    def _select_vtype(self, code, active_fg, inactive_fg):
        """Highlights the active tile and updates the backend."""
        for c, (btn, af, ifo) in self._vtype_btns.items():
            if c == code:
                btn.configure(fg_color=af, border_width=2, border_color=("#000000", "white"))
            else:
                btn.configure(fg_color=ifo, border_width=0)
        self.change_vtype(code)

    def _select_fuel(self, val):
        """Deprecated but kept for safety. Use _select_fuel_by_key."""
        pass

    def _select_fuel_by_key(self, key):
        texts = LOCALIZATION[self.current_lang]
        val = texts[key]
        self._fuel_active = val
        for k, btn in self._fuel_btns.items():
            btn.configure(text=texts[k]) # Update text to current lang
            if k == key:
                btn.configure(fg_color="#3a7ebf", border_width=2, border_color=("#000000", "white"))
            else:
                btn.configure(fg_color="#2a3a50", border_width=0)
        self.backend.fuel_type = FuelType.DIESEL if key == "diesel" else FuelType.PETROL
        self.save_settings()
        if self.backend.is_logging:
            # Warn user: log file won't reflect the new fuel type until logging is restarted
            import tkinter.messagebox as _mb
            _mb.showwarning(
                "Log â€“ zmiana paliwa" if self.current_lang == "pl" else ("Log â€“ Kraftstofftausch" if self.current_lang == "de" else "Log â€“ Fuel Change"),
                "Typ paliwa zmieniony. Zatrzymaj i wznów logowanie, aby nowy plik CSV odzwierciedlał zmianę."
                if self.current_lang == "pl" else
                ("Kraftstofftyp geĂ¤ndert. Logging neu starten, damit die neue CSV-Datei die Ă„nderung widerspiegelt."
                if self.current_lang == "de" else
                "Fuel type changed. Stop and restart logging so the new CSV file reflects the change.")
            )

    def change_units(self, val):
        texts = LOCALIZATION[self.current_lang]
        self.backend.use_imperial = (val == texts["u_imperial"])
        self.update_digital_indicators()
        self.save_settings()
        
    def change_vtype(self, val):
        """Updates vehicle type. Accepts ICE/HEV/EV codes."""
        if val == VehicleType.EV:
            self.backend.vehicle_type = VehicleType.EV
            # Disable fuel buttons if available
            for btn in getattr(self, '_fuel_btns', {}).values():
                btn.configure(state="disabled")
        elif val == VehicleType.HEV:
            self.backend.vehicle_type = VehicleType.HEV
            for btn in getattr(self, '_fuel_btns', {}).values():
                btn.configure(state="normal")
        else:
            self.backend.vehicle_type = VehicleType.ICE
            for btn in getattr(self, '_fuel_btns', {}).values():
                btn.configure(state="normal")
        self.update_digital_indicators()
        self.save_settings()
        if self.backend.is_logging:
            # Warn user: log file won't reflect the new vehicle type until logging is restarted
            import tkinter.messagebox as _mb
            _mb.showwarning(
                "Log â€“ zmiana napędu" if self.current_lang == "pl" else ("Log â€“ Antriebswechsel" if self.current_lang == "de" else "Log â€“ Drive Type Change"),
                "Typ napędu zmieniony. Zatrzymaj i wznów logowanie, aby nowy plik CSV odzwierciedlał zmianę."
                if self.current_lang == "pl" else
                ("Antriebstyp geĂ¤ndert. Logging neu starten, damit die neue CSV-Datei die Ă„nderung widerspiegelt."
                if self.current_lang == "de" else
                "Drive type changed. Stop and restart logging so the new CSV file reflects the change.")
            )
        
    def change_fuel(self, val):
        texts = LOCALIZATION[self.current_lang]
        self.backend.fuel_type = FuelType.DIESEL if texts["diesel"] == val else FuelType.PETROL
        
    def toggle_alarms(self):
        self.alarm_enabled = self.alarm_switch.get() == 1
        self.save_settings()

    def update_alarms_list(self, *args):
        for key, (a_var, l_var) in self._alarm_vars.items():
            self.alarm_configs[key]["active"] = a_var.get()
            try:
                self.alarm_configs[key]["limit"] = float(l_var.get())
            except: pass
        self.save_settings()

    def _alarm_sound_worker(self):
        """Wątek zajmujący się odtwarzaniem dźwięków po kolei."""
        while True:
            try:
                # Pobierz częstotliwość z kolejki
                freq = self.alarm_queue.get()
                if self.alarm_enabled:
                    winsound.Beep(freq, 200) # 200ms dźwięku
                    time.sleep(0.1)          # Przerwa między dźwiękami
                self.alarm_queue.task_done()
            except:
                time.sleep(1)

    def change_csv_rate(self, val):
        texts = LOCALIZATION[self.current_lang]
        if texts["csv_freq"] == val: self.backend.csv_interval = 0.5
        elif texts["csv_std"] == val: self.backend.csv_interval = 2.0
        else: self.backend.csv_interval = 10.0
        self.save_settings()

    # ---- FUNKCJE LOGIKI APLIKACJI ----

    def toggle_virtual(self):
        new_use_virtual = self.switch_var.get() == "virtual"
        
        # Jeśli jest aktywne połączenie i zmieniamy tryb â€“ rozłącz najpierw
        if self.backend.is_connected_loop and hasattr(self, 'connect_btn'):
            if self.backend.is_replay:
                self._replay_stop()
            else:
                self.backend.disconnect()
                texts = LOCALIZATION[self.current_lang]
                self.logging_btn.configure(state="disabled", text=texts["start_logger"])

        self.backend.use_virtual = new_use_virtual

        # Jeśli nie trwa aktywne połączenie, przywróć przycisk do stanu bazowego
        if not self.backend.is_connected_loop and hasattr(self, 'connect_btn'):
            texts = LOCALIZATION[self.current_lang]
            self.connect_btn.configure(
                state="normal",
                text=texts["connect"],
                fg_color=['#3a7ebf', '#1f538d'],
                hover_color=['#325882', '#14375e']
            )
            self.update_status_labels()
            if hasattr(self, 'error_detail_lbl'):
                self.error_detail_lbl.configure(text="")

    def connect_obd(self):
        texts = LOCALIZATION[self.current_lang]
        if self.backend.is_connected_loop:
            if self.backend.is_replay:
                self._replay_stop()
                # Nie wracaj â€“ kontynuuj do połączenia live poniżej
            else:
                self.backend.disconnect()
                self.connect_btn.configure(
                    text=texts["connect"],
                    fg_color=['#3a7ebf', '#1f538d'],
                    hover_color=['#325882', '#14375e']
                )
                self.update_status_labels()
                self.logging_btn.configure(state="disabled", text=texts["start_logger"])
                
                # Wymuś odświeżenie UI po rozłączeniu (wyczyszczenie zegarów i wykresów)
                self.update_digital_indicators()
                if hasattr(self, 'canvas'):
                    self.update_graphs(None)
                    self.canvas.draw_idle()
                return
        else:
            # Weryfikacja czy wybrano typ pojazdu i paliwo
            if self.backend.vehicle_type is None:
                messagebox.showwarning(texts.get("err_title", "Błąd"), texts.get("err_select_vtype", "Please select drivetrain"))
                return
            
            if self.backend.vehicle_type != VehicleType.EV and self.backend.fuel_type is None:
                messagebox.showwarning(texts.get("err_title", "Błąd"), texts.get("err_select_fuel", "Please select fuel type"))
                return

            self.status_label.configure(text=texts["connecting"], text_color="#FFC100")
            self.connect_btn.configure(state="disabled")
            if hasattr(self, 'error_detail_lbl'):
                self.error_detail_lbl.configure(text="")
            self.update_idletasks()
            
            # --- TOKEN: każda próba ma unikalny ID; spóźniony wynik starej próby jest ignorowany ---
            import time as _time
            self._conn_attempt_id = getattr(self, '_conn_attempt_id', 0) + 1
            attempt_id = self._conn_attempt_id
            self._conn_start_time = _time.time()
            self._conn_timeout = 20  # sekundy
            
            # Animowany licznik sekundowy w labelu statusu
            def _tick():
                if getattr(self, '_conn_attempt_id', 0) != attempt_id:
                    return  # próba już zakończona lub anulowana
                elapsed = int(_time.time() - self._conn_start_time)
                self.status_label.configure(
                    text=texts['connecting'],
                    text_color="#FFC100"
                )
                if hasattr(self, 'error_detail_lbl'):
                    self.error_detail_lbl.configure(
                        text=f"âŹł {elapsed}s / {self._conn_timeout}s",
                        text_color="#FFC100"
                    )
                if elapsed >= self._conn_timeout:
                    # Timeout â€“ pokaż błąd i odblokuj UI (wątek OBD może nadal działać w tle)
                    self._conn_attempt_id += 1  # unieważnij bieżącą próbę
                    self.connect_btn.configure(
                        state="normal",
                        text=texts["connect"],
                        fg_color=['#3a7ebf', '#1f538d'],
                        hover_color=['#325882', '#14375e']
                    )
                    self.status_label.configure(text=texts["disconnected"], text_color="#FF4C4C")
                    if hasattr(self, 'error_detail_lbl'):
                        timeout_msg = texts.get("err_conn_timeout", "Timeout").replace("{s}", str(self._conn_timeout))
                        self.error_detail_lbl.configure(text=timeout_msg)
                else:
                    self.after(1000, _tick)
            
            self.after(1000, _tick)
            
            # WÄ„TEK W TLE: żeby `obd.OBD()` nie zamrażało ekranu
            def try_connect():
                try:
                    port = getattr(self, "port_var", None)
                    p_val = port.get() if port else "AUTO"
                    baud = getattr(self, "baudrate_var", None)
                    b_val = baud.get() if baud else "38400"
                    success = self.backend.connect(p_val, baudrate=b_val)
                    self.after(0, self._finish_connection, success, attempt_id)
                except Exception as e:
                    self.after(0, self._finish_connection_error, str(e), attempt_id)
                    
            threading.Thread(target=try_connect, daemon=True).start()

    def _finish_connection(self, success, attempt_id=None):
        # Jeśli próba została unieważniona (timeout) â€“ ignoruj wynik
        if attempt_id is not None and getattr(self, '_conn_attempt_id', 0) != attempt_id:
            return
        # Unieważnij timer tiku
        self._conn_attempt_id = getattr(self, '_conn_attempt_id', 0) + 1
        texts = LOCALIZATION[self.current_lang]
        # Zawsze odblokuj i przywróć przycisk do stanu bazowego
        self.connect_btn.configure(
            state="normal",
            fg_color=['#3a7ebf', '#1f538d'],
            hover_color=['#325882', '#14375e']
        )
        if success:
            self.connect_btn.configure(text=texts["disconnect"], fg_color="#B22222", hover_color="#8B0000")
            self.update_status_labels()
            if hasattr(self, 'error_detail_lbl'):
                self.error_detail_lbl.configure(text="")
            self.logging_btn.configure(state="normal")
        else:
            self.connect_btn.configure(text=texts["connect"])
            err_detail = self.backend.last_error
            self.update_status_labels()
            if hasattr(self, 'error_detail_lbl'):
                # Sprawdź czy to klucz tłumaczenia
                port_val = getattr(self, "port_var", None)
                p_str = port_val.get() if port_val else "AUTO"
                localized_err = texts.get(err_detail, err_detail).replace("{p}", p_str)
                self.error_detail_lbl.configure(text=localized_err)
            
    def _finish_connection_error(self, message, attempt_id=None):
        if attempt_id is not None and getattr(self, '_conn_attempt_id', 0) != attempt_id:
            return
        self._conn_attempt_id = getattr(self, '_conn_attempt_id', 0) + 1
        texts = LOCALIZATION[self.current_lang]
        self.connect_btn.configure(
            state="normal",
            text=texts["connect"],
            fg_color=['#3a7ebf', '#1f538d'],
            hover_color=['#325882', '#14375e']
        )
        self.update_status_labels()
        if hasattr(self, 'error_detail_lbl'):
            self.error_detail_lbl.configure(text=message)
        messagebox.showerror(texts.get("err_no_conn", "Error"), message)

    def create_modal(self, title, width=400, height=250):
        # Usunięto overlay (CTkToplevel z alpha), bo na Windows generował duże opóźnienia przy odświeżaniu Matplotlib
        mx = self.winfo_rootx()
        my = self.winfo_rooty()
        mw = self.winfo_width()
        mh = self.winfo_height()
        
        dialog = ctk.CTkToplevel(self)
        dialog.title(title)
        dialog.transient(self)
        dialog.resizable(False, False)
        
        dx = mx + (mw // 2) - (width // 2)
        dy = my + (mh // 2) - (height // 2)
        dialog.geometry(f"{width}x{height}+{dx}+{dy}")
        
        dialog.grab_set()
        dialog.focus_force()
        
        def close_all():
            try: dialog.grab_release()
            except: pass
            dialog.destroy()
            
        dialog.protocol("WM_DELETE_WINDOW", close_all)
        return dialog, close_all

    def toggle_logging(self):
        texts = LOCALIZATION[self.current_lang]
        if self.backend.is_logging:
            self.backend.stop_logging()
            self.logging_btn.configure(text=texts["start_logger"], fg_color=['#3a7ebf', '#1f538d'], hover_color=['#325882', '#14375e'])
            self.update_status_labels()
        else:
            # Jeśli ID pojazdu jest już znane, startuj od razu
            if getattr(self.backend, "current_vehicle_id", None):
                self.backend.start_logging(self.backend.current_vehicle_id)
                self.logging_btn.configure(text=texts["stop_logger"], fg_color="#188040", hover_color="#10572b")
                self.update_status_labels()
                return

            self.update_idletasks() # Wymuś przetworzenie zdarzeń przed otwarciem okna
            dialog, close_all_default = self.create_modal(texts.get("log_name_title", "Vehicle ID"), 350, 230)
            
            lbl = ctk.CTkLabel(dialog, text=texts.get("log_name_prompt", "Name (3-10 chars):"), font=ctk.CTkFont(size=14))
            lbl.pack(pady=(20, 10))
            
            entry_var = ctk.StringVar()
            def limit_char(*args):
                val = entry_var.get()
                if len(val) > 10:
                    entry_var.set(val[:10])
            entry_var.trace_add('write', limit_char)
            
            entry = ctk.CTkEntry(dialog, textvariable=entry_var, width=220)
            entry.pack(pady=5)
            entry.focus()
            
            err_lbl = ctk.CTkLabel(dialog, text="", text_color="#FF4C4C", font=ctk.CTkFont(size=12))
            err_lbl.pack(pady=(0, 5))
            
            def submit_name(event=None):
                veh_name = entry.get()
                if not veh_name or len(veh_name.strip()) < 3:
                    err_lbl.configure(text=texts.get("log_name_err", "Too short!"))
                    return
                close_all_default()
                veh_name = veh_name.strip()[:10].upper()
                self.backend.current_vehicle_id = veh_name
                # FAKTYCZNY START LOGOWANIA DOPIERO TUTAJ:
                self.backend.start_logging(veh_name)
                self.logging_btn.configure(text=texts["stop_logger"], fg_color="#188040", hover_color="#10572b")
                self.update_status_labels()
                
            def cancel_name():
                close_all_default()
            
            dialog.protocol("WM_DELETE_WINDOW", cancel_name)
            entry.bind("<Return>", submit_name)
            
            btn_frame = ctk.CTkFrame(dialog, fg_color="transparent")
            btn_frame.pack(pady=10)
            
            ctk.CTkButton(btn_frame, text="OK", command=submit_name, width=100).pack(side="left", padx=10)
            ctk.CTkButton(btn_frame, text=texts.get("cancel", "Cancel"), fg_color="#B22222", hover_color="#8B0000", command=cancel_name, width=100).pack(side="left", padx=10)

    def action_export_dtc(self):
        import tkinter.filedialog as filedialog
        import csv
        from datetime import datetime
        
        content = self.dtc_text.get("1.0", "end-1c")
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_name = f"dtc_report_{stamp}.csv"
        
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=default_name,
            title="Export DTC to CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if filepath:
            try:
                with open(filepath, 'w', newline='', encoding='utf-8-sig') as f:
                    writer = csv.writer(f)
                    writer.writerow(["DTC Report"])
                    writer.writerow([f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"])
                    writer.writerow([])
                    for line in content.split('\n'):
                        writer.writerow([line])
                import tkinter.messagebox as messagebox
                messagebox.showinfo("Export Successful", f"DTC saved to\n{filepath}")
            except Exception as e:
                import tkinter.messagebox as messagebox
                messagebox.showerror("Export Failed", str(e))

    def action_scan_dtc(self):
        texts = LOCALIZATION[self.current_lang]
        if not self.backend.connection or not self.backend.connection.is_connected():
            messagebox.showwarning(texts["err_no_conn"], texts["err_no_conn_msg"])
            return
            
        self.dtc_text.configure(state="normal")
        self.dtc_text.delete("0.0", tk.END)
        self.dtc_text.insert("0.0", texts["dtc_scanning"])
        self.update_idletasks()
        
        codes = self.backend.read_errors()
        
        self.dtc_text.delete("0.0", tk.END)
        if not codes:
            self.dtc_text.insert("end", texts["dtc_none"])
        else:
            self.dtc_text.insert("end", texts["dtc_found_prefix"].replace("{n}", str(len(codes))) + "\n" + ("="*55) + "\n\n")
            
            ff_msg = texts["dtc_no_ff"]
            if self.backend.use_virtual:
                ff_msg = texts["dtc_ff_sim"]
                
            for kod, opis in codes:
                self.dtc_text.insert("end", f"▶ {kod}: {opis}\n")
                self.dtc_text.insert("end", f"   {ff_msg}\n\n")
            self.dtc_text.insert("end", ("="*55) + "\n" + texts["dtc_suggestion"])
            
        self.dtc_text.configure(state="disabled")

    def action_clear_dtc(self):
        texts = LOCALIZATION[self.current_lang]
        if not self.backend.connection or not self.backend.connection.is_connected():
            messagebox.showwarning(texts["err_no_conn"], texts["err_clear_no_conn"])
            return
            
        resp = messagebox.askyesno(texts["clear_dtc_title"], texts["clear_dtc_confirm"])
        if resp:
            # Kasuje
            if self.backend.clear_errors():
                messagebox.showinfo(texts["clear_success_title"], texts["clear_success_msg"])
                self.action_scan_dtc() # automatyczne poskanowanie na dowód czystości
            else:
                messagebox.showerror(texts["err_title"], texts["clear_err_msg"])

    # --- ZARZÄ„DZANIE SESJAMI I PLIKAMI ---

    def refresh_sessions(self):
        texts = LOCALIZATION[self.current_lang]
        # Czyszczenie widoku GUI
        for child in self.sessions_scroll.winfo_children():
            child.destroy()
            
        files = glob.glob(os.path.join(os.getcwd(), "*log_obd_*.csv"))
        file_data = []
        for f in files:
            try:
                file_data.append((f, os.path.getctime(f)))
            except: pass
        file_data.sort(key=lambda x: x[1], reverse=True)
        
        if not file_data:
            ctk.CTkLabel(self.sessions_scroll, text=texts["no_logs"], font=ctk.CTkFont(slant="italic"), text_color=("#333333", "gray")).pack(pady=40)
            return

        now = datetime.now()
        today_date = now.date()
        yesterday_date = today_date - __import__("datetime").timedelta(days=1)
        current_group = None
            
        for f, ctimestamp in file_data:
            cdate = datetime.fromtimestamp(ctimestamp).date()
            if cdate == today_date: group_name = texts.get("today", "Today")
            elif cdate == yesterday_date: group_name = texts.get("yesterday", "Yesterday")
            else: group_name = cdate.strftime('%Y-%m-%d')
            
            if current_group != group_name:
                current_group = group_name
                # Dodaj separator / nagłówek daty
                lbl_group = ctk.CTkLabel(self.sessions_scroll, text=group_name, font=ctk.CTkFont(size=14, weight="bold"), text_color="#aaaaaa")
                lbl_group.pack(anchor="w", padx=15, pady=(15, 2))

            try:
                size_kb = os.path.getsize(f) / 1024.0
                ctime = datetime.fromtimestamp(ctimestamp).strftime('%H:%M:%S')
            except Exception:
                size_kb = 0
                ctime = texts["unknown"]
                
            fname = os.path.basename(f)
            
            vid_display = ""
            import re
            m = re.match(r"^\[(.*?)\]_log_obd_", fname)
            if m: vid_display = f"🚗 {m.group(1)} | "

            row = ctk.CTkFrame(self.sessions_scroll, corner_radius=8, border_width=1, border_color="#333333")
            row.pack(fill="x", padx=10, pady=5)
            row.grid_columnconfigure(0, weight=1) # Przestrzeń dla napisu
            row.grid_columnconfigure(1, weight=0) # Przestrzeń dla przycisków
            
            # KRĂ“TKI OPIS
            info_text = f"{vid_display}" + texts["log_info_short"].replace("{t}", ctime).replace("{s}", f"{size_kb:.1f}")
            lbl = ctk.CTkLabel(row, text=info_text, font=ctk.CTkFont(size=12, weight="bold"))
            lbl.grid(row=0, column=0, sticky="w", padx=10, pady=15)
            
            # Ramka na przyciski (prawa strona)
            btn_frame = ctk.CTkFrame(row, fg_color="transparent")
            btn_frame.grid(row=0, column=1, sticky="e", padx=10)
            
            # Przyciski kontrolne (bardzo kompaktowe szerokości, żeby zmieścić ich 5)
            ctk.CTkButton(btn_frame, text=texts["del_log_btn"], width=60, fg_color="#B22222", hover_color="#8B0000", command=lambda x=f: self.delete_session(x)).pack(side="right", padx=3, pady=10)
            ctk.CTkButton(btn_frame, text=texts.get("share_btn", "Share"), width=80, fg_color="#28a745", hover_color="#218838", command=lambda x=f: self.share_log(x)).pack(side="right", padx=3, pady=10)
            ctk.CTkButton(btn_frame, text=texts["replay_btn"], width=80, fg_color="#ff8c00", hover_color="#c86d00", command=lambda x=f: self.action_replay(x)).pack(side="right", padx=3, pady=10)
            ctk.CTkButton(btn_frame, text=texts["folder_btn"], width=80, fg_color="#3a7ebf", hover_color="#1f538d", command=lambda x=f: self.open_folder(x)).pack(side="right", padx=3, pady=10)
            ctk.CTkButton(btn_frame, text=texts["analyze_btn"], width=90, fg_color="#6a0dad", hover_color="#4b0082", command=lambda x=f: self.analyze_with_ai(x)).pack(side="right", padx=3, pady=10)

    def analyze_with_ai(self, filepath):
        """Wczytuje log CSV, buduje profesjonalny prompt inżynierski i wysyła go do Gemini przez przeglądarkę."""
        texts = LOCALIZATION[self.current_lang]
        try:
            import csv as csvmod
            rows = []
            with open(filepath, newline="", encoding="utf-8") as f:
                reader = csvmod.reader(f)
                header = next(reader, None)
                for i, row in enumerate(reader):
                    rows.append(row)
                    if i >= 199: # Limit 200 wierszy, by nie przeciążyć promptu
                        break

            fname = os.path.basename(filepath)
            
            # Ekstrakcja typu pojazdu z nazwy pliku
            vtype_from_file = texts["unknown"]
            if f"_{VehicleType.ICE}" in str(fname): vtype_from_file = texts["ice"]
            elif f"_{VehicleType.HEV}" in str(fname): vtype_from_file = texts["hev"]
            elif f"_{VehicleType.EV}" in str(fname): vtype_from_file = texts["ev"]
            else: vtype_from_file = self.backend.vehicle_type

            # Szukanie paliwa w nazwie
            fuel_info = ""
            if f"_{FuelType.PETROL}" in str(fname): fuel_info = texts["fuel_type"] + ": " + texts["petrol"]
            elif f"_{FuelType.DIESEL}" in str(fname): fuel_info = texts["fuel_type"] + ": " + texts["diesel"]

            num_rows = len(rows)
            csv_preview = "\n".join([",".join([str(x) for x in (header or [])])] + [",".join([str(x) for x in r]) for r in rows[:50]])

            # Statystyki z logów - dynamiczne szukanie kolumn
            speeds, temps, rpms = [], [], []
            try:
                # Mapowanie nazw kolumn (obsługa różnych wariantów językowych)
                idx_rpm = None
                idx_spd = None
                idx_tmp = None
                
                if header:
                    for i, h in enumerate(header):
                        h_low = str(h).lower()
                        if "rpm" in h_low: idx_rpm = i
                        if "prędkość" in h_low or "speed" in h_low or "speed" in h_low: idx_spd = i
                        if "temp" in h_low: idx_tmp = i

                for r in rows:
                    try:
                        if idx_rpm is not None and len(r) > idx_rpm and r[idx_rpm]: rpms.append(float(r[idx_rpm]))
                        if idx_spd is not None and len(r) > idx_spd and r[idx_spd]: speeds.append(float(r[idx_spd]))
                        if idx_tmp is not None and len(r) > idx_tmp and r[idx_tmp]: temps.append(float(r[idx_tmp]))
                    except: continue
            except Exception:
                pass

            stats = ""
            if rpms: stats += f"- RPM: min={min(rpms):.0f}, max={max(rpms):.0f}, śr={sum(rpms)/len(rpms):.0f}\n"
            if speeds: stats += f"- Speed: max={max(speeds):.1f}, śr={sum(speeds)/len(speeds):.1f}\n"
            if temps: stats += f"- Temp: min={min(temps):.1f}, max={max(temps):.1f}\n"

            prompt = texts["ai_prompt_format"].replace("{vt}", vtype_from_file).replace("{fi}", fuel_info).replace("{fn}", fname).replace("{nr}", str(num_rows)).replace("{st}", stats).replace("{cp}", csv_preview)

            # Kopiuj do schowka
            self.clipboard_clear()
            self.clipboard_append(prompt)
            self.update()

            # Otwórz Gemini w przeglądarce
            webbrowser.open("https://gemini.google.com/app")

            messagebox.showinfo(
                texts["ai_ready_title"],
                texts["ai_ready_msg"].replace("{f}", fname)
            )

        except Exception as e:
            messagebox.showerror(texts["err_title"], f"{texts['err_module']}: {e}")

    def action_replay(self, filepath):
        """Uruchamia odtwarzanie logu z pliku CSV z 1-sekundowym buforem na załadowanie widoku."""
        texts = LOCALIZATION[self.current_lang]
        
        # 1. Blokada aktywnych sesji - z bezpiecznym pobraniem tytułu błędu
        if self.backend.is_connected_loop or self.backend.is_logging:
            msg = texts.get("err_replay_active", "Stop OBD/Logging first.")
            title = texts.get("err_title", "Warning")
            messagebox.showwarning(title, msg)
            return

        # 2. Przełączenie tabu (zawsze używamy wewnętrznej nazwy "Dashboard")
        self.tabview.set("Dashboard")
        self._current_override = "loading"
        self.update_mode_status(override="loading")
        
        # Zapisz stan przed replay, by _replay_stop mógł go przywrócić
        self._pre_replay_state = {
            "active_graphs": list(self.active_graphs),
            "vehicle_type": self.backend.vehicle_type,
            "fuel_type": self.backend.fuel_type
        }
        
        self.update_idletasks()
        
        # 3. Wczytanie logu do backendu
        if self.backend.start_replay(filepath):
            # Rozpoczynamy zapauzowani, by UI zdążyło się "narysować"
            self.backend.is_replay_paused = True
            
            # --- PRZYWRĂ“CONE: Inicjalizacja UI Replay ---
            self._set_dashboard_interactive(False)
            self.connect_btn.configure(text=texts["stop_replay"], fg_color="#B22222", hover_color="#8B0000")
            self.logging_btn.configure(state="disabled")
            self._sync_dashboard_indicators()
            
            # Pokazujemy dzieci paska (miejsce paska juz jest zarezerwowane)
            self.replay_stop_btn.grid(row=0, column=0, padx=(10, 4), pady=9)
            self._replay_timeline_frame.grid(row=0, column=1, sticky="ew", padx=6)
            self.replay_pause_btn.grid(row=0, column=2, padx=(4, 10), pady=0) # mniejszy pady by sie zmiescilo
            self.replay_pause_btn.configure(text="▶")
            # --------------------------------------------
            
            # 4. Synchronizacja stanu GUI z logiem (1:1) + natychmiastowe zaladowanie danych
            if self.backend.replay_data:
                row0 = self.backend.replay_data[0]
                
                # Zaladuj wartosci pierwszej klatki natychmiast do backend.current i history
                # dzieki temu wskazniki i wykresy nie czekaja na watek replay
                try:
                    num_keys = [
                        "rpm", "speed", "temp", "maf", "load", "throttle",
                        "trans_temp", "hev_soc", "hev_volts", "hev_power",
                        "voltage_12v", "mpg"
                    ]
                    # Update current values
                    self.backend.current.update({
                        k: float(row0.get(k, 0) or 0) for k in num_keys
                    })
                    self.backend.current["replayed_graphs"] = row0.get("active_graphs", "")
                    
                    # Fill history so graphs have data immediately
                    with self.backend.data_lock:
                        self.backend.x_data.clear()
                        for v in self.backend.history.values():
                            v.clear()
                        
                        self.backend.x_data.append(float(row0.get("time", 0)))
                        for k in num_keys:
                            self.backend.history[k].append(self.backend.current[k])
                except Exception:
                    pass
                
                # Synchronizacja aktywnych wykresow z pierwszej klatki
                rec_graphs_val = row0.get("active_graphs", "")
                if rec_graphs_val and isinstance(rec_graphs_val, str):
                    rec_list = [g.strip() for g in rec_graphs_val.split(",") if g.strip()]
                    if rec_list:
                        self.active_graphs = rec_list
                        for k, var in self.graph_vars.items():
                            var.set(k in self.active_graphs)
                        if hasattr(self, '_on_resize_ref'):
                            self._on_resize_ref()
            
            # 5. Przygotowanie osi czasu i markerow
            self._precompute_replay_alarms(self.alarm_configs, self.alarm_enabled)
            self._draw_replay_markers()
            
            # 6. Pokaz pierwsza klatke danych natychmiast
            self._needs_layout = True
            self.update_digital_indicators()
            if hasattr(self, 'canvas'):
                self.update_graphs(None)
                self.canvas.draw_idle()
            self.update_idletasks()
            
            # Po 1000ms puszczamy odtwarzanie (w tym czasie widać "Ĺadowanie...")
            self.after(1000, self._finalize_replay_start)
        else:
            messagebox.showerror(texts["err_title"], texts["err_replay_parse"])

    def _finalize_replay_start(self):
        """Metoda wywoływana po 'ładowaniu', puszcza playback."""
        if not self.backend.is_replay:
            return
            
        # Zsynchronizuj bazę czasu przed startem, by uniknąć fly-through (skoku do przodu)
        self._replay_sync_time_base()
        self.backend.is_replay_paused = False
        
        # Ustaw ikone przycisku na pause (bo replay teraz gra)
        self.replay_pause_btn.configure(text="⏸")
        
        self._current_override = None
        self.update_mode_status()
        self._update_replay_bar()

    def _precompute_replay_alarms(self, alarm_configs, alarm_enabled):
        """Pre-calculates indices of rows with alarms to avoid O(N) processing during UI updates."""
        if not self.backend.is_replay or not self.backend.replay_data:
            return
            
        indices = []
        fuel = self.backend.fuel_type
        for i, row in enumerate(self.backend.replay_data):
            is_danger = False
            
            # 1. Manual Alarms
            for key, val_str in row.items():
                if key == "time": continue
                try:
                    val = row.get(key, 0.0) # Już skonwertowane na float w start_replay
                    cfg = alarm_configs.get(key)
                    if alarm_enabled and cfg and cfg["active"]:
                        if cfg["mode"] == "max" and val >= cfg["limit"]: is_danger = True
                        if cfg["mode"] == "min" and val <= cfg["limit"]: is_danger = True
                except: pass
                if is_danger: break
            
            # 2. Smart Alarms
            if not is_danger:
                try:
                    rpm = row.get("rpm", 0.0)
                    temp = row.get("temp", 0.0)
                    v12 = row.get("voltage_12v", 14.0)
                    soc = row.get("hev_soc", 50.0)
                    trans = row.get("trans_temp", 0.0)
                    
                    if rpm >= (4500 if fuel == FuelType.DIESEL else 6500): is_danger = True
                    elif temp >= 105: is_danger = True
                    elif v12 <= 11.5 and row.get("voltage_12v"): is_danger = True
                    elif soc <= 15 and row.get("hev_soc"): is_danger = True
                    elif trans >= 110: is_danger = True
                except: pass
                
            if is_danger:
                indices.append(i)
        
        self.backend.replay_alarm_indices = indices

    def _draw_replay_markers(self):
        """Uses pre-calculated alarm indices to draw markers efficiently."""
        self.replay_marker_canvas.delete("all")
        if not self.backend.is_replay or not self.backend.replay_data:
            return
            
        w = self.replay_marker_canvas.winfo_width()
        h = self.replay_marker_canvas.winfo_height()
        if w <= 1: return
            
        total_rows = len(self.backend.replay_data)
        if total_rows < 2: return
            
        max_time = float(self.backend.replay_data[-1].get("time", 1))
        if max_time <= 0: max_time = 1
            
        # Optimization: if too many markers, we might want to bin them, but for now 
        # let's just use the precomputed list.
        for idx in self.backend.replay_alarm_indices:
            row = self.backend.replay_data[idx]
            r_time = float(row.get("time", 0))
            px = (r_time / max_time) * w
            
            self.replay_marker_canvas.create_line(
                px, 2, px, h, fill="#FF4C4C", width=1
            )

    def _replay_stop(self):
        """Zatrzymuje odtwarzanie i wraca do stanu rozłączonego polaczenia, przywracając ustawienia manualne."""
        self.backend.disconnect() # Zastępuje ręczne is_connected_loop=False i resetuje dane
        self.backend.is_replay = False
        self.backend.is_replay_paused = False
        
        # Przywróć interaktywność dashboardu
        self._set_dashboard_interactive(True)
        texts = LOCALIZATION[self.current_lang]
        self.connect_btn.configure(
            text=texts["connect"],
            fg_color=['#3a7ebf', '#1f538d'],
            hover_color=['#325882', '#14375e']
        )
        self.update_status_labels()
        
        # Wymuś odświeżenie UI po zatrzymaniu replay (wyczyszczenie wskaźników)
        self.update_digital_indicators()
        if hasattr(self, 'canvas'):
            self.update_graphs(None)
            self.canvas.draw_idle()
            
        self.logging_btn.configure(state="disabled")
        # Nie usuwamy z grida, tylko czyścimy widoczność (rezerwacja miejsca)
        for child in self.replay_bar.winfo_children():
            child.grid_remove() 
        self._current_override = None

        # --- ZERUJ wszystkie wskaźniki i wykresy PRZED odświeżeniem UI ---
        for k in list(self.backend.current.keys()):
            if isinstance(self.backend.current[k], (int, float)):
                self.backend.current[k] = 0
        self.backend.current["replayed_graphs"] = ""
        with self.backend.data_lock:
            self.backend.x_data.clear()
            for v in self.backend.history.values():
                v.clear()

        # Wymuś odświeżenie UI, aby pokazać zera
        self.update_digital_indicators()
        self.update_graphs(None)

        # Przywróć stan sprzed odtwarzania
        if hasattr(self, "_pre_replay_state"):
            s = self._pre_replay_state
            self.active_graphs = list(s["active_graphs"])
            self.backend.vehicle_type = s["vehicle_type"]
            self.backend.fuel_type = s["fuel_type"]

            # Synchronizacja checkboxów w UI
            for gk, b_var in getattr(self, 'graph_vars', {}).items():
                b_var.set(gk in self.active_graphs)

            # Synchronizacja przycisków Dashboardu
            self._sync_dashboard_indicators()

            # Wymuszenie odświeżenia layoutu wykresów i finalny redraw
            self._needs_layout = True
            if hasattr(self, '_on_resize_ref'):
                self._on_resize_ref()
            
            self.update_graphs(None)
            if hasattr(self, 'canvas'):
                self.canvas.draw_idle()

            # Wymuszenie odświeżenia UI po przywróceniu stanu
            self.update()


    def _set_dashboard_interactive(self, enabled=True):
        """Włącza/wyłącza przyciski i checkbox'y na dashboardzie (używane podczas Replay)."""
        state = "normal" if enabled else "disabled"
        # Checkboxy
        for cb in getattr(self, "graph_checkboxes", {}).values():
            cb.configure(state=state)
        # Typy napędu
        for data in getattr(self, "_vtype_btns", {}).values():
            if isinstance(data, (list, tuple)) and len(data) > 0:
                data[0].configure(state=state)
        # Typy paliwa
        for btn in getattr(self, "_fuel_btns", {}).values():
            btn.configure(state=state)
        # Przycisk nagrywania (zawsze wyłączony podczas Replay)
        if hasattr(self, "logging_btn"):
            self.logging_btn.configure(state="disabled" if not enabled else "disabled")
            if enabled:
                self.logging_btn.configure(state="normal" if self.backend.is_connected_loop and not self.backend.is_replay else "disabled")
            else:
                self.logging_btn.configure(state="disabled")

    def _replay_sync_time_base(self):
        """Synchronizuje czas z logiem po pauzie lub seek."""
        self.backend.replay_paused_duration = 0.0
        self.backend.replay_start_real = __import__("time").time()
        idx = self.backend.replay_index
        if self.backend.replay_data and idx < len(self.backend.replay_data):
            self.backend.replay_start_log = float(self.backend.replay_data[idx].get("time", 0))

    def _replay_toggle_pause(self):
        """Przelacza pauze/wznowienie replay."""
        if self.backend.is_replay_paused and self.backend.replay_index >= len(self.backend.replay_data):
            self.backend.replay_index = 0
            with self.backend.data_lock:
                self.backend.x_data.clear()
                for v in self.backend.history.values():
                    v.clear()
            self.update_graphs(None)

        is_pausing = not self.backend.is_replay_paused

        if not is_pausing:
            self._replay_sync_time_base()
            self.replay_pause_btn.configure(text="⏸")  # gra: pokaz ikone pauzy
        else:
            self.replay_pause_btn.configure(text="▶")  # spauzowane: pokaz play

        self.backend.is_replay_paused = is_pausing
        self.update_status_labels()

    def _on_slider_press(self, event):
        """Zapamietuje stan przed scrubbing."""
        self._was_playing_before_scrub = not self.backend.is_replay_paused
        self.backend.is_scrubbing = True
        self._last_seek_label_update = 0

    def _on_slider_release(self, event):
        """Konczy scrubbing i wznawia odtwarzanie."""
        self._finish_scrubbing()
        self.backend.is_replay_paused = False
        self.replay_pause_btn.configure(text="⏸")
        self._replay_sync_time_base()
        self.update_status_labels()

    def _replay_seek(self, value):
        """Ultra-szybka aktualizacja suwaka."""
        total = len(self.backend.replay_data)
        if total > 0:
            new_index = int(float(value) * (total - 1))
            self.backend.replay_index = max(0, min(new_index, total - 1))
            row = self.backend.replay_data[self.backend.replay_index]
            try:
                self.backend.current.update({
                    k: row.get(k, 0.0) for k in ["rpm", "speed", "temp", "maf", "load", "throttle",
                                               "trans_temp", "hev_soc", "hev_volts", "hev_power",
                                               "voltage_12v", "mpg"]
                })
                self.backend.current["replayed_graphs"] = row.get("active_graphs", "")
            except: pass
            import time as _t
            now = _t.time()
            if now - getattr(self, "_last_seek_label_update", 0) > 0.03:
                self._last_seek_label_update = now
                self.update_digital_indicators()
            with self.backend.data_lock:
                self.backend.x_data.clear()
                for v in self.backend.history.values():
                    v.clear()

    def _finish_scrubbing(self):
        """Konczy scrubbing i wymusza redraw."""
        self.backend.is_scrubbing = False
        self.update_digital_indicators()
        self.update_graphs(None)
        self._replay_sync_time_base()

    def _update_replay_bar(self):
        """Cyklicznie aktualizuje pasek replay."""
        if self._replay_bar_after_id:
            self.after_cancel(self._replay_bar_after_id)
            self._replay_bar_after_id = None
        if self.backend.is_scrubbing:
            self._replay_bar_after_id = self.after(200, self._update_replay_bar)
            return
        if not self.backend.is_replay:
            return
        total = len(self.backend.replay_data)
        current = self.backend.replay_index
        if total > 0:
            progress = min(current / total, 1.0)
            self.replay_progress.set(progress)
            row_now = self.backend.replay_data[min(current, total - 1)]
            row_end = self.backend.replay_data[-1]
            cur_s = int(float(row_now.get("time", 0)))
            tot_s = int(float(row_end.get("time", 0)))
            self.replay_time_lbl.configure(
                text=f"{cur_s // 60}:{cur_s % 60:02d} / {tot_s // 60}:{tot_s % 60:02d}"
            )
            # Po dotarciu do końca — pauzujemy i pokazujemy ▶ (kliknięcie wznowi od początku)
            if current >= total:
                if not self.backend.is_replay_paused:
                    self.backend.is_replay_paused = True
                    self.update_status_labels()
                self.replay_pause_btn.configure(text="▶")

        self._replay_bar_after_id = self.after(500, self._update_replay_bar)



    def delete_session(self, filepath):
        texts = LOCALIZATION[self.current_lang]
        if messagebox.askyesno(texts["del_log_title"], texts["del_log_confirm"].replace("{f}", os.path.basename(filepath))):
            try:
                os.remove(filepath)
                self.refresh_sessions()
            except Exception as e:
                messagebox.showerror(texts["err_title"], f"{e}")

    def delete_all_sessions(self):
        texts = LOCALIZATION[self.current_lang]
        if messagebox.askyesno(texts.get("del_all_title", "Delete All"), texts.get("del_all_confirm", "Delete all?")):
            files = glob.glob(os.path.join(os.getcwd(), "*log_obd_*.csv"))
            for f in files:
                try: os.remove(f)
                except: pass
            self.refresh_sessions()

    def share_log(self, filepath):
        texts = LOCALIZATION[self.current_lang]
        dialog, close_all = self.create_modal(texts.get("share_title", "Share"), 350, 200)
        
        def on_email():
            close_all()
            try:
                import urllib.parse
                subj = urllib.parse.quote("OBD Master Pro - Log Data")
                body = urllib.parse.quote(f"Pamiętaj żeby dodać do załącznika plik: {os.path.basename(filepath)}")
                webbrowser.open(f"mailto:?subject={subj}&body={body}")
                self.open_folder(filepath)
            except: pass
            
        def on_drive():
            close_all()
            try:
                webbrowser.open("https://drive.google.com/drive/my-drive")
                self.open_folder(filepath)
            except: pass
            
        ctk.CTkLabel(dialog, text=texts.get("share_msg", "Where?"), font=ctk.CTkFont(size=14)).pack(pady=20)
        ctk.CTkButton(dialog, text=texts.get("share_email", "Email"), command=on_email).pack(pady=5)
        ctk.CTkButton(dialog, text=texts.get("share_drive", "Drive"), command=on_drive).pack(pady=5)

    def open_folder(self, filepath):
        try:
            path = os.path.dirname(os.path.abspath(filepath))
            os.startfile(path)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_closing(self):
        texts = LOCALIZATION[self.current_lang]
        # Fixed: use exit_confirm instead of non-existent exit_msg
        if messagebox.askyesno(texts["exit_title"], texts["exit_confirm"]):
            try:
                self.backend.disconnect()
            except:
                pass
            self.destroy()
            os._exit(0) # Wymuszenie zamknięcia procesu i wszystkich wątków w Windows

    # --- ZARZÄ„DZANIE WIDOKAMI I USTAWIENIAMI ---

    def change_refresh_rate(self, val):
        if "200" in val: self.gui_refresh_rate = 200
        elif "500" in val: self.gui_refresh_rate = 500
        else:
            self.gui_refresh_rate = 100 if self.backend.is_replay else 1000

    def update_digital_indicators(self):
        """Cykliczna aktualizacja wskaźników tekstowych i statusów."""
        if getattr(self, "_indicators_after_id", None):
            self.after_cancel(self._indicators_after_id)
            self._indicators_after_id = None

        texts = LOCALIZATION[self.current_lang]
        # UNITS CONVERSION
        t_val = self.backend.current["temp"]
        s_val = self.backend.current["speed"]
        tr_val = self.backend.current["trans_temp"]
        
        if self.backend.use_imperial:
            t_val = (t_val * 1.8) + 32
            s_val = s_val * 0.621371
            tr_val = (tr_val * 1.8) + 32

        # DYNAMIC UI BASED ON VEHICLE TYPE
        if self.backend.vehicle_type == VehicleType.EV:
            rpm_val = self.backend.current["hev_power"]
            if self.backend.use_imperial:
                self.lbl_temp.configure(text=texts["temp_bat_f"])
                self.lbl_speed.configure(text=texts["speed_f"])
                self.lbl_trans.configure(text=texts["temp_ev_f"])
            else:
                self.lbl_temp.configure(text=texts["temp_bat_c"])
                self.lbl_speed.configure(text=texts["speed_c"])
                self.lbl_trans.configure(text=texts["temp_ev_c"])
                
            self.lbl_rpm.configure(text=texts["ev_power_lbl"])
            self.lbl_soc.configure(text=texts["bat_state_lbl"])
            self.lbl_maf.configure(text=texts["bat_volt_lbl"])
            
            self.val_rpm.configure(text=f"{rpm_val:.1f}")
            self.val_soc.configure(text=f"{self.backend.current['hev_soc']:.1f}")
            self.val_maf.configure(text=f"{self.backend.current['hev_volts']:.1f}")
            
            self.val_load.configure(text="-")
            self.val_hev_v.configure(text="-")
            self.val_hev_kw.configure(text="-")
            
        else:
            rpm_val = self.backend.current["rpm"]
            mpg_val = self.backend.current["mpg"]
            
            if self.backend.use_imperial:
                self.lbl_temp.configure(text=texts["temp_f"])
                self.lbl_speed.configure(text=texts["speed_f"])
                self.lbl_trans.configure(text=texts["temp_trans_f"])
                self.lbl_soc.configure(text=texts["cons_f"])
            else:
                self.lbl_temp.configure(text=texts["temp_c"])
                self.lbl_speed.configure(text=texts["speed_c"])
                self.lbl_trans.configure(text=texts["temp_trans_c"])
                self.lbl_soc.configure(text=texts["cons_c"])
                
            self.lbl_rpm.configure(text=texts["rpm_lbl"])
            self.lbl_maf.configure(text=texts["maf_lbl"])
            
            self.val_rpm.configure(text=f"{int(rpm_val)}")
            self.val_soc.configure(text=f"{mpg_val:.1f}")
            self.val_maf.configure(text=f"{self.backend.current['maf']}")
            self.val_load.configure(text=str(self.backend.current["load"]))
            
            if self.backend.vehicle_type == VehicleType.HEV:
                self.val_hev_v.configure(text=f"{self.backend.current['hev_volts']:.1f}")
                self.val_hev_kw.configure(text=f"{self.backend.current['hev_power']:.1f}")
            else:
                self.val_hev_v.configure(text="-")
                self.val_hev_kw.configure(text="-")
        
        # Baza dla wskaźników (reset kolorów)
        all_indicators = {
            "rpm": self.val_rpm, "speed": self.val_speed, "temp": self.val_temp,
            "maf": self.val_maf, "load": self.val_load, "throttle": self.val_throttle,
            "trans_temp": self.val_trans, "hev_soc": self.val_soc, "hev_volts": self.val_hev_v,
            "hev_power": self.val_hev_kw, "voltage_12v": self.val_12v, "mpg": self.val_soc # mpg też używa val_soc
        }
        
        # --- SMART COLORING & ALARMS ---
        if not hasattr(self, "_active_alerts"): self._active_alerts = set()
        
        # Słownik z aktualnymi wartościami do sprawdzenia
        current_vals = {
            "rpm": rpm_val, "speed": s_val, "temp": t_val,
            "maf": self.backend.current["maf"], "load": self.backend.current["load"],
            "throttle": self.backend.current["throttle"], "trans_temp": tr_val,
            "hev_soc": self.backend.current["hev_soc"], "hev_volts": self.backend.current["hev_volts"],
            "hev_power": self.backend.current["hev_power"], "voltage_12v": self.backend.current["voltage_12v"],
            "mpg": mpg_val if self.backend.vehicle_type != VehicleType.EV else 0
        }

        for key, widget in all_indicators.items():
            if not widget: continue
            val = current_vals.get(key, 0)
            cfg = self.alarm_configs.get(key)
            
            # 1. Sprawdź ALARM RÄCZNY (ma priorytet nad inteligentnym kolorowaniem)
            manual_triggered = False
            if self.alarm_enabled and cfg and cfg["active"]:
                if cfg["mode"] == "max" and val >= cfg["limit"]: manual_triggered = True
                if cfg["mode"] == "min" and val <= cfg["limit"]: manual_triggered = True
            
            # 2. Sprawdź ALARM INTELIGENTNY (automatyczne progi)
            smart_triggered = False
            if not manual_triggered:
                vtype = self.backend.vehicle_type
                fuel = self.backend.fuel_type
                if key == "rpm":
                    limit = 4500 if fuel == FuelType.DIESEL else 6500
                    if val >= limit: smart_triggered = True
                elif key == "temp":
                    if val >= 105: smart_triggered = True # Przegrzanie
                elif key == "voltage_12v":
                    if val <= 11.5: smart_triggered = True # Napięcie
                elif key == "hev_soc":
                    if val <= 15: smart_triggered = True # Rozładowany
                elif key == "trans_temp":
                    if val >= 110: smart_triggered = True
            
            is_danger = manual_triggered or smart_triggered
            
            # Zastosuj kolor
            if is_danger:
                widget.configure(text_color="#FF4C4C") # RED
                # Dźwięk tylko gdy ręczny alarm jest włączony i aktywny
                if manual_triggered:
                    if key not in self._active_alerts:
                        self.alarm_queue.put(cfg["freq"])
                        self._active_alerts.add(key)
            else:
                # Powrót do ZIELONEGO (Normalny stan)
                # W trybie HUD wymuszamy tylko zielone dla RPM/Speed, ale tutaj wszystko może być zielone dla spójności
                widget.configure(text_color="#00FF00")
                if key in self._active_alerts:
                    self._active_alerts.remove(key)

        self.val_speed.configure(text=f"{int(s_val)}")
        self.val_temp.configure(text=f"{int(t_val)}")
        self.val_trans.configure(text=f"{int(tr_val)}")
        self.val_throttle.configure(text=str(self.backend.current["throttle"]))
        self.val_12v.configure(text=f"{self.backend.current['voltage_12v']:.1f}")
        
        # --- Synchronizacja wykresów podczas Replay (1:1) ---
        if self.backend.is_replay:
            rep_g = self.backend.current.get("replayed_graphs", "")
            if rep_g and isinstance(rep_g, str):
                new_graphs = [g.strip() for g in rep_g.split(",") if g.strip()]
                if new_graphs and new_graphs != self.active_graphs:
                    self.active_graphs = new_graphs
                    # Aktualizacja checkboxów w UI
                    for gk, b_var in getattr(self, 'graph_vars', {}).items():
                        b_var.set(gk in self.active_graphs)
                    # Wymuszenie layoutu
                    self._needs_layout = True
                    if hasattr(self, '_on_resize_ref'):
                        self._on_resize_ref()
                    # Natychmiastowe odświeżenie wykresu, by uniknąć "dziury" w widoku
                    self.update_graphs(None)
                    if hasattr(self, 'canvas'):
                        self.canvas.draw_idle()

        # Zawsze aktualizujemy status widoku (Tryb na żywo / Replay / Idle)
        self.update_mode_status()
        refresh = 100 if self.backend.is_replay else getattr(self, 'gui_refresh_rate', 300)
        self._indicators_after_id = self.after(refresh, self.update_digital_indicators)

    def update_graphs(self, frame):
        if getattr(self.backend, 'is_scrubbing', False):
            return
            
        # Throttling dla trybu LIVE
        now = time.time()
        if not self.backend.is_replay:
            last = getattr(self, '_last_graph_draw', 0)
            if now - last < 0.25: return
        self._last_graph_draw = now

        texts = LOCALIZATION.get(self.current_lang, {})
        with self.backend.data_lock:
            x = list(self.backend.x_data)
            # Capture all history keys to support dynamic redirection (e.g. SOC -> MPG)
            hist = {k: list(v) for k, v in self.backend.history.items()}

        active_count = len(self.active_graphs)
        if active_count == 0:
            if not hasattr(self, '_last_active_graphs') or len(self._last_active_graphs) > 0:
                self.fig.clf()
                bg_color = "#F0F0F0" if ctk.get_appearance_mode() == "Light" else "#2b2b2b"
                self.fig.patch.set_facecolor(bg_color)
                self.canvas.draw_idle()
                self._last_active_graphs = []
            return

        # Re-create axes only if selection changed
        if not hasattr(self, '_last_active_graphs') or self._last_active_graphs != self.active_graphs:
            self.fig.clf()
            is_light = ctk.get_appearance_mode() == "Light"
            bg_color = "#F0F0F0" if is_light else "#2b2b2b"
            fg_color = "#333333" if is_light else "gray"
            grid_color = "#CCCCCC" if is_light else "#444444"
            
            self.fig.patch.set_facecolor(bg_color)
            self._axes = []
            self._lines = {} # Store line objects for set_data
            for i in range(active_count):
                ax = self.fig.add_subplot(active_count, 1, i+1)
                ax.set_facecolor(bg_color)
                ax.tick_params(colors=fg_color, labelsize=8)
                ax.grid(color=grid_color, linestyle='dashed', linewidth=0.5, alpha=0.5)
                self._axes.append(ax)
                
                key = self.active_graphs[i]
                colors = ["#FF4C4C", "#4CA6FF", "#4CFF4C", "#FFD700", "#FF8C00", "#FF69B4", "#8A2BE2", "#00FFFF", "#FF1493", "#32CD32", "#FF00FF", "#1E90FF"]
                color = colors[i % len(colors)]
                line, = ax.plot([], [], color=color, linewidth=2.5, antialiased=True)
                self._lines[key] = line
                
            self._last_active_graphs = list(self.active_graphs)
            self._needs_layout = True
            
        axes = getattr(self, '_axes', [])
        
        is_ev = self.backend.vehicle_type == "EV"
        is_imp = self.backend.use_imperial
        
        labels = {
            "rpm": texts.get("ev_power_lbl" if is_ev else "rpm_lbl", texts.get("engine_revs", "RPM")),
            "speed": texts.get("speed_f" if is_imp else "speed_c", texts.get("vehicle_speed", "Speed")),
            "temp": texts.get("temp_bat_f" if is_imp else "temp_bat_c", texts.get("coolant_temp", "Temp")) if is_ev else texts.get("temp_f" if is_imp else "temp_c", texts.get("coolant_temp", "Temp")),
            "maf": texts.get("bat_volt_lbl" if is_ev else "maf_lbl", texts.get("maf_flow", "MAF")),
            "load": texts.get("engine_load", "Load"),
            "throttle": texts.get("throttle_pos", "Throttle"),
            "trans_temp": texts.get("temp_ev_f" if is_imp else "temp_ev_c", texts.get("trans_temp", "Trans Temp")) if is_ev else texts.get("temp_trans_f" if is_imp else "temp_trans_c", texts.get("trans_temp", "Trans Temp")),
            "hev_soc": texts.get("bat_state_lbl" if is_ev else "batt_soc", texts.get("battery_soc", "SOC")),
            "hev_volts": texts.get("hev_voltage", "HV Volts"),
            "hev_power": texts.get("ev_power", "EV Power"),
            "voltage_12v": texts.get("v12_voltage", "12V"),
            "mpg": texts.get("cons_f" if is_imp else "cons_c", "Consumption")
        }

        for i, key in enumerate(self.active_graphs):
            if i >= len(axes): break
            ax = axes[i]
            
            # Resolve logical key to actual data key (e.g., hev_soc -> mpg for Non-EVs)
            data_key = key
            if key == "hev_soc" and self.backend.vehicle_type != VehicleType.EV:
                data_key = "mpg"
                
            y = hist[data_key]
            line = self._lines.get(key)
            if line and x and y:
                m_len = min(len(x), len(y))
                curr_x = x[:m_len]
                curr_y = y[:m_len]
                line.set_data(curr_x, curr_y)
                ax.set_xlim(min(x), max(x))

                # Gradient-like fill (Premium Look)
                if hasattr(ax, '_fill_poly'):
                    ax._fill_poly.remove()
                color = line.get_color()
                ax._fill_poly = ax.fill_between(curr_x, curr_y, color=color, alpha=0.15, edgecolor='none', linewidth=0)

                # --- SMART Y-AXIS scaling ---
                # "Stable minimum" approach: don't look flat, but don't jump on every small change
                MIN_RANGES = {
                    "rpm":          4000,
                    "speed":        120,
                    "temp":         130, # Steady
                    "maf":          100,
                    "load":         105, # Steady
                    "throttle":     105, # Steady
                    "trans_temp":   130, 
                    "hev_soc":      100, 
                    "hev_volts":    450,
                    "hev_power":    100,
                    "voltage_12v":  16,
                    "mpg":          25,
                }
                
                y_max_data = max(curr_y) if curr_y else 0
                y_min_data = min(curr_y) if curr_y else 0
                
                limit_max = MIN_RANGES.get(data_key, 100)
                if y_max_data > limit_max * 0.9:
                    limit_max = y_max_data * 1.15 # Expand if nearing top
                
                # Specyficzne miny dla napięcia 12V
                limit_min = 0
                if data_key == "voltage_12v":
                    limit_min = 9
                    if y_min_data < 10: limit_min = 0 # Expand down if battery is dying
                
                ax.set_ylim(limit_min, limit_max)
                
                # Używamy data_key do tytułu, aby odzwierciedlić faktyczne dane (np. Spalanie zamiast SOC)
                ax.set_title(labels.get(data_key, data_key.upper()), color='gray', fontsize=8, loc='left', pad=2)
            else:
                # Jeśli brak danych (np. po rozłączeniu), czyścimy wykresy
                if line:
                    line.set_data([], [])
                if hasattr(ax, '_fill_poly'):
                    try: ax._fill_poly.remove()
                    except: pass
                    delattr(ax, '_fill_poly')
                ax.set_title(labels.get(data_key, data_key.upper()), color='gray', fontsize=8, loc='left', pad=2)

        if getattr(self, '_needs_layout', False):
            try:
                self.fig.tight_layout(pad=1.5)
            except: pass
            self._needs_layout = False
            if hasattr(self, '_on_resize_ref'):
                self.after(50, self._on_resize_ref)
        
        self.canvas.draw_idle()

    def update_mode_status(self, override=None):
        """Aktualizuje pasek stanu nad wykresami (Tryb na żywo, Logowanie, Replay, Oczekiwanie)."""
        if not hasattr(self, 'mode_status_lbl'):
            return
            
        texts = LOCALIZATION[self.current_lang]
        actual_override = override or getattr(self, '_current_override', None)
        
        if actual_override == "loading":
            self.mode_status_lbl.configure(text=texts.get("state_loading", "đźźˇ Ĺadowanie..."), text_color="#FFC100")
        elif self.backend.is_replay:
            if self.backend.is_replay_paused:
                self.mode_status_lbl.configure(text=texts.get("replay_paused", "âŹ¸ď¸Ź PAUZA"), text_color="#FF4C4C")
            else:
                self.mode_status_lbl.configure(text=texts.get("state_replay", "đźź  Tryb Odtwarzania"), text_color="#ff8c00")
        elif self.backend.is_logging:
            self.mode_status_lbl.configure(text=texts.get("state_logging", "đź”´ Tryb Logowania"), text_color="#FF4C4C")
        elif self.backend.is_connected_loop:
            self.mode_status_lbl.configure(text=texts.get("state_live", "đźź˘ Tryb Odczytu (Na Ĺ»ywo)"), text_color="#00FF00")
        else:
            self.mode_status_lbl.configure(text=texts.get("state_idle", "âšŞ Oczekiwanie (Rozłączono)"), text_color="#555555")

if __name__ == "__main__":
    app = OBDApp()
    app.mainloop()

