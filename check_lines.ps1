$bytes = [System.IO.File]::ReadAllBytes('obd_gui_app.py')
# Find all lines with multi-byte sequences like C3, C4, C5, E2
$i = 0
$lineNo = 1
$lineStart = 0
$hits = 0
foreach($b in $bytes){
    if($b -eq 13){ 
        # end of line
        if($hits -gt 0){
            $chunk = $bytes[$lineStart..($i-1)]
            $hex = ($chunk | ForEach-Object { $_.ToString('X2') }) -join ' '
            Write-Output "LINE $lineNo : $hex"
        }
        $lineStart = $i + 2
        $lineNo++
        $hits = 0
    }
    if($b -eq 0xC5 -or $b -eq 0xC3 -or $b -eq 0xC4){
        $hits++
    }
    $i++
    if($lineNo -gt 100){ break }
}
