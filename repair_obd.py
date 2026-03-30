import sys
import os
import json

TARGET_FILE = r'c:\Users\piotr_7toppme\Searches\obd_gui_app.py'
PL_LOC = r'c:\Users\piotr_7toppme\Searches\i18n\pl.json'
EN_LOC = r'c:\Users\piotr_7toppme\Searches\i18n\en.json'

# --- 1. REPAIR obd_gui_app.py ---
if os.path.exists(TARGET_FILE):
    with open(TARGET_FILE, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()

    start_marker = '    def _build_vag_tab(self):'
    end_marker = '    def _action_zdc_browse(self):'

    idx_start = content.find(start_marker)
    idx_end = content.find(end_marker)

    if idx_start != -1 and idx_end != -1:
        new_block = """    def _build_vag_tab(self):
        \"\"\"Buduje zakładkę VAG Specialist.\"\"\"
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
                btn = ctk.CTkButton(t_frame, text=texts["vag_btn_apply"], width=140, fg_color=("#14375E", "#1f538d"), hover_color=("#1F538D", "#3a7ebf"), command=lambda tweak=t: self.action_execute_vag_tweak(tweak))
                btn.grid(row=0, column=1, padx=10, pady=10)
                has_backup = self.backend.get_vag_backup(t["module"], t["commands"][-1][2:6]) is not None
                btn_restore = ctk.CTkButton(t_frame, text=texts["vag_btn_restore"], width=140, fg_color=("#D3D3D3", "#3d3d3d"), hover_color=("#A9A9A9", "#4d4d4d"), state="normal" if has_backup else "disabled", command=lambda tweak=t: self.action_restore_vag_tweak(tweak))
                btn_restore.grid(row=0, column=2, padx=10, pady=10)
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
        \"\"\"Buduje zakładkę Stellantis Specialist.\"\"\"
        texts = LOCALIZATION[self.current_lang]
        tab = self.tabview.tab("Stellantis Specialist")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(2, weight=1)
        header_frame = ctk.CTkFrame(tab, fg_color="transparent")
        header_frame.grid(row=0, column=0, sticky="ew", padx=20, pady=10)
        ctk.CTkLabel(header_frame, text=texts.get("stl_brand_lbl", "Marka:"), font=ctk.CTkFont(weight="bold")).pack(side="left", padx=5)
        self.stl_brand_menu = ctk.CTkOptionMenu(header_frame, values=list(self.STELLANTIS_TWEAKS_CONFIG.keys()), command=self._on_stl_brand_changed)
        self.stl_brand_menu.pack(side="left", padx=10)
        ctk.CTkLabel(header_frame, text=texts.get("stl_model_lbl", "Model:"), font=ctk.CTkFont(weight="bold")).pack(side="left", padx=5)
        self.stl_model_menu = ctk.CTkOptionMenu(header_frame, values=["-"], command=self._on_stl_model_changed)
        self.stl_model_menu.pack(side="left", padx=10)
        warn_frame = ctk.CTkFrame(tab, fg_color=("#FFE4C4", "#3d2b1f"), border_width=1, border_color=("#FF9900", "#ff8c00"))
        warn_frame.grid(row=1, column=0, sticky="ew", padx=20, pady=5)
        self.stl_warn_lbl = ctk.CTkLabel(warn_frame, text=texts.get("stl_warning", "UWAGA: Tryb Ekspert Stellantis. Tylko dla aut z szyną CAN."), font=ctk.CTkFont(size=11), wraplength=800)
        self.stl_warn_lbl.pack(pady=10, padx=20)
        self.stl_tweak_scroll = ctk.CTkScrollableFrame(tab, fg_color="transparent")
        self.stl_tweak_scroll.grid(row=2, column=0, sticky="nsew", padx=10, pady=5)
        self._on_stl_brand_changed(self.stl_brand_menu.get())

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
            btn = ctk.CTkButton(f, text=texts.get("stl_exec_btn", "Wykonaj"), width=100, command=lambda t=tweak: self._execute_stl_tweak(t))
            btn.pack(side="right", padx=15, pady=10)

    def _execute_stl_tweak(self, tweak):
        texts = LOCALIZATION[self.current_lang]
        if not self.backend.connection or not self.backend.connection.is_connected():
            messagebox.showwarning(texts["err_no_conn"], texts["err_no_conn_msg"])
            return
        name = texts.get(tweak["name_key"], tweak["id"])
        if not messagebox.askyesno(texts.get("stl_confirm_title", "Potwierdzenie"), texts.get("stl_confirm_msg", "Czy na pewno chcesz wykonać ten tweak?") + f"\\n\\n{name}"):
            return
        ok, res = self.backend.execute_uds_custom(tweak["commands"], tweak.get("security"))
        if ok:
            messagebox.showinfo("Sukces", f"{name}: WYKONANO\\n{res}")
        else:
            messagebox.showerror("Błąd", f"{name}: NIEPOWODZENIE\\n{res}")

"""
        new_content = content[:idx_start] + new_block + content[idx_end:]
        with open(TARGET_FILE, 'w', encoding='utf-8') as f:
            f.write(new_content)
        print('SUCCESS: obd_gui_app.py repaired.')

# --- 2. UPDATE LOCALIZATIONS ---
STL_STRINGS = {
    "pl": {
        "tab_stl": "Stellantis Specialist",
        "stl_brand_lbl": "Marka:",
        "stl_model_lbl": "Model:",
        "stl_exec_btn": "Wykonaj",
        "stl_confirm_title": "Potwierdzenie",
        "stl_confirm_msg": "Czy na pewno chcesz wykonać ten tweak?",
        "stl_warning": "UWAGA: Tryb Ekspert Stellantis. Tylko dla aut z szyną CAN.",
        "stl_no_tweaks": "Brak dostępnych tweaków dla tego modelu.",
        "stl_op_bc_unlock": "Odblokuj Statystyki BC (V, Olej, Temp)",
        "stl_op_oil_reset": "Kasowanie Inspekcji Oleju (UDS)",
        "stl_op_tpms": "Tryb Nauki TPMS",
        "stl_op_needle": "Test Wskazówek (Inscenizacja)",
        "stl_op_eco": "Wyłącz System ECO (Stop-Start)",
        "stl_op_camera": "Aktywacja Kamery Cofania",
        "stl_fi_srv_reset": "Reset Serwisowy (UDS)",
        "stl_fi_dpf": "Wymuszenie Regeneracji DPF",
        "stl_fi_proxy": "Procedura Proxi Alignment",
        "stl_psa_mirror": "Składanie Lusterek przy Zamknięciu",
        "stl_psa_drl": "Menu Świateł Dziennych (DRL) w Radiu"
    },
    "en": {
        "tab_stl": "Stellantis Specialist",
        "stl_brand_lbl": "Brand:",
        "stl_model_lbl": "Model:",
        "stl_exec_btn": "Execute",
        "stl_confirm_title": "Confirm",
        "stl_confirm_msg": "Are you sure you want to execute this tweak?",
        "stl_warning": "CAUTION: Stellantis Expert Mode. CAN-bus models only.",
        "stl_no_tweaks": "No tweaks available for this model.",
        "stl_op_bc_unlock": "Unlock BC Statistics (V, Oil, Temp)",
        "stl_op_oil_reset": "Oil Life Reset (UDS)",
        "stl_op_tpms": "TPMS Learn Mode",
        "stl_op_needle": "Needle Sweep (Staging)",
        "stl_op_eco": "Disable ECO (Stop-Start)",
        "stl_op_camera": "Rear Camera Activation",
        "stl_fi_srv_reset": "Service Reset (UDS)",
        "stl_fi_dpf": "Force DPF Regeneration",
        "stl_fi_proxy": "Proxi Alignment Procedure",
        "stl_psa_mirror": "Mirror Fold on Lock",
        "stl_psa_drl": "DRL Menu in Radio"
    }
}

for lang, loc_file in [('pl', PL_LOC), ('en', EN_LOC)]:
    if os.path.exists(loc_file):
        with open(loc_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
        data.update(STL_STRINGS[lang])
        with open(loc_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        print(f'SUCCESS: Updated {lang} localization.')
