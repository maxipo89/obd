$bytes = [System.IO.File]::ReadAllBytes('obd_gui_app.py')
$i = 0
foreach($b in $bytes){
    if($b -eq 0xC5 -or $b -eq 0xC3 -or $b -eq 0xC4){
        $start = [Math]::Max(0,$i-5)
        $end = [Math]::Min($bytes.Length-1, $i+30)
        $chunk = $bytes[$start..$end]
        $hex = ($chunk | ForEach-Object { $_.ToString('X2') }) -join ' '
        Write-Output "at byte offset $i : $hex"
        break
    }
    $i++
}
