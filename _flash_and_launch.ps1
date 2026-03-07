$ErrorActionPreference = "Continue"
$uf2 = "C:\Users\filip\OneDrive\Desktop\MushIV1p0\firmware_c\build\mushio_c.uf2"
$gui = "C:\Users\filip\OneDrive\Desktop\MushIV1p0\host\gui.py"
$ota = "C:\Users\filip\OneDrive\Desktop\MushIV1p0\host\ota_client.py"

# ---- 1. Find BOOTSEL drive -----------------------------------------------
$bootsel = $null
foreach ($letter in [char[]]"DEFGHIJKLMNOPQRSTUVWXYZ") {
    $path = "${letter}:\"
    if (Test-Path $path) {
        try {
            $vol = (Get-WmiObject Win32_LogicalDisk -Filter "DeviceID='${letter}:'" -ErrorAction Stop).VolumeName
            if ($vol -match 'RPI-RP2|RP2350') {
                $bootsel = $path
                Write-Host "[FLASH] BOOTSEL drive found: $path ($vol)" -ForegroundColor Green
                break
            }
        } catch {}
    }
}

# ---- 2. Flash ------------------------------------------------------------
if ($bootsel) {
    Write-Host "[FLASH] Copying UF2 to $bootsel ..." -ForegroundColor Cyan
    Copy-Item $uf2 $bootsel -Force
    Write-Host "[FLASH] Done. Waiting 5 s for reboot..." -ForegroundColor Cyan
    Start-Sleep 5
} else {
    Write-Host "[FLASH] No BOOTSEL drive - trying OTA (Pico must be running firmware)..." -ForegroundColor Yellow
    $host_ip = ""
    # Check for active connection on port 9000 to guess Pico IP
    $conn = netstat -an 2>$null | Select-String "9000\s+ESTABLISHED"
    if ($conn) {
        Write-Host "[INFO] Found active port-9000 connection: $conn"
    }
    # Run ota_client - it will handle drive detection itself
    python $ota $uf2
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[WARN] OTA flash failed or Pico not reachable." -ForegroundColor Yellow
        Write-Host "       Hold BOOTSEL on the Pico, plug USB, then re-run this script." -ForegroundColor Yellow
    }
}

# ---- 3. Launch GUI -------------------------------------------------------
Write-Host ""
Write-Host "[GUI] Launching MushIO GUI..." -ForegroundColor Cyan
Start-Process python -ArgumentList $gui
Write-Host "[GUI] Launched. Done." -ForegroundColor Green
