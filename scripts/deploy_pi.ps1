# Copy the current DroneGuard code to the Raspberry Pi (run on the laptop).
#
#   powershell -File scripts\deploy_pi.ps1          copy, then run the tests on the Pi
#   powershell -File scripts\deploy_pi.ps1 -NoTest
#
# Copies python/droneguard, python/live, python/tests and python/pi into
# ~/droneguard on the Pi. The Pi's Python environment (.venv) is left alone.

param([switch]$NoTest)
$ErrorActionPreference = "Stop"

$py = Join-Path (Split-Path -Parent $PSScriptRoot) "python"
$tgz = Join-Path $env:TEMP "droneguard_pi.tgz"

Push-Location $py
try {
    tar --exclude=__pycache__ -czf $tgz droneguard live tests pi
} finally { Pop-Location }

ssh -o ConnectTimeout=5 pi true
if ($LASTEXITCODE -ne 0) {
    Write-Host "Can't reach the Pi - run findpi, then try again." -ForegroundColor Red
    exit 1
}
scp $tgz pi:/tmp/droneguard_pi.tgz
ssh pi "mkdir -p ~/droneguard && tar -xzf /tmp/droneguard_pi.tgz -C ~/droneguard && rm /tmp/droneguard_pi.tgz && sed -i 's/\r$//' ~/droneguard/pi/*.sh && chmod +x ~/droneguard/pi/*.sh && echo 'copied to ~/droneguard'"
Remove-Item $tgz

if (-not $NoTest) {
    Write-Host "Running the tests on the Pi (about a minute)..." -ForegroundColor Cyan
    ssh pi "cd ~/droneguard && .venv/bin/python -m unittest 2>&1 | tail -3"
}
