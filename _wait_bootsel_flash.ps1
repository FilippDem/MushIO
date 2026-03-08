$uf2 = "C:\Users\filip\OneDrive\Desktop\MushIV1p0\firmware_c\build\mushio_c.uf2"
Write-Host "[FLASH] Waiting for RPI-RP2 / RP2350 BOOTSEL drive (hold BOOTSEL then plug USB)..."
$deadline = [DateTime]::Now.AddSeconds(60)
while ([DateTime]::Now -lt $deadline) {
    foreach ($letter in [char[]]"DEFGHIJKLMNOPQRSTUVWXYZ") {
        $path = "${letter}:\"
        if (Test-Path $path) {
            try {
                $vol = (Get-WmiObject Win32_LogicalDisk -Filter "DeviceID='${letter}:'" -ErrorAction Stop).VolumeName
                if ($vol -match "RPI-RP2|RP2350") {
                    Write-Host "[FLASH] Drive found: $path ($vol)" -ForegroundColor Green
                    Copy-Item $uf2 $path -Force
                    Write-Host "[FLASH] UF2 copied -- Pico rebooting..." -ForegroundColor Green
                    exit 0
                }
            } catch {}
        }
    }
    Write-Host "." -NoNewline
    Start-Sleep -Milliseconds 500
}
Write-Host ""
Write-Host "[FLASH] Timed out -- BOOTSEL drive never appeared." -ForegroundColor Red
exit 1
