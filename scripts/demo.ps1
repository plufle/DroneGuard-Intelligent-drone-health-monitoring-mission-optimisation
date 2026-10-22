# DroneGuard live demo with the Raspberry Pi as the drone - run this on the laptop.
#
#   Double-click scripts\demo.bat      (or: powershell -File scripts\demo.ps1)
#   options:  -NoPi      don't open the Pi window (laptop-only demo)
#             -Port 8000 web port
#
# It (1) starts the DroneGuard website, or reuses one that's already running,
#    (2) finds the Pi with findpi if its address changed,
#    (3) opens a window that ssh's into the Pi and starts the drone there,
#    (4) opens the live page in the browser with the Pi selected.

param([switch]$NoPi, [int]$Port = 8000)
$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$py = Join-Path $root "python"
$venvPy = Join-Path $py ".venv\Scripts\python.exe"
$url = "http://127.0.0.1:$Port"
$findpi = Join-Path $env:USERPROFILE ".ssh\findpi.ps1"

function Say($text, $color = "Gray") { Write-Host $text -ForegroundColor $color }

if (-not (Test-Path $venvPy)) {
    Say "No Python environment at $venvPy" Red
    Say "Create it once:  cd $py ; python -m venv .venv ; .venv\Scripts\python -m pip install -r requirements.txt"
    exit 1
}

# ---- 1. the website ---------------------------------------------------------
function Get-Status { try { Invoke-RestMethod "$url/api/live/status" -TimeoutSec 2 } catch { $null } }

$status = Get-Status
if ($status -and -not $status.PSObject.Properties["laptop_ip"]) {
    # A DroneGuard server from before the Raspberry Pi option was added.
    $owner = (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue |
              Select-Object -First 1).OwningProcess
    Say "An older DroneGuard website is running (process $owner) - it has no Raspberry Pi option." Yellow
    $answer = Read-Host "Restart it with the current code? [Y/n]"
    if ($answer -match "^(n|no)$") { exit 1 }
    Stop-Process -Id $owner -Confirm:$false
    Start-Sleep -Seconds 1
    $status = $null
}
if (-not $status) {
    if (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue) {
        Say "Port $Port is used by another program. Close it, or run with -Port 8001." Red
        exit 1
    }
    Say "Starting the DroneGuard website in a new window..." Cyan
    Start-Process powershell -WorkingDirectory $py -ArgumentList @(
        "-NoExit", "-Command",
        "`$Host.UI.RawUI.WindowTitle = 'DroneGuard website (Ctrl+C to stop)'; & '$venvPy' -m uvicorn web.app:app --host 127.0.0.1 --port $Port")
    for ($i = 0; $i -lt 40 -and -not $status; $i++) { Start-Sleep -Milliseconds 500; $status = Get-Status }
    if (-not $status) { Say "The website did not start - look at its window for the error." Red; exit 1 }
}
Say "Website ready: $url   (laptop on Wi-Fi at $($status.laptop_ip))" Green

# ---- 2 + 3. the Pi ------------------------------------------------------------
if (-not $NoPi) {
    $ssh = @("-o", "BatchMode=yes", "-o", "ConnectTimeout=4")
    & ssh @ssh pi true 2>$null
    if ($LASTEXITCODE -ne 0 -and (Test-Path $findpi)) {
        Say "Pi not answering at its last address - looking for it on this Wi-Fi (findpi)..." Cyan
        & powershell -NoProfile -File $findpi
        & ssh @ssh pi true 2>$null
    }
    if ($LASTEXITCODE -eq 0) {
        Say "Opening a window on the Pi that starts the drone..." Cyan
    } else {
        Say "Could not log in to the Pi automatically (new address or new host key?)." Yellow
        Say "The Pi window will open anyway - answer its questions, or check the Pi is on this Wi-Fi."
    }
    Start-Process powershell -ArgumentList @(
        "-NoExit", "-Command",
        "`$Host.UI.RawUI.WindowTitle = 'Raspberry Pi drone (Ctrl+C to stop)'; ssh -t pi '~/droneguard/pi/start_drone.sh'")
}

# ---- 4. the browser ---------------------------------------------------------------
$page = if ($NoPi) { "$url/live" } else { "$url/live?source=pi" }
Start-Process $page
Say ""
Say "Live page: $page" Green
if (-not $NoPi) { Say "When the dot turns green (Raspberry Pi connected), press Take off." Green }
