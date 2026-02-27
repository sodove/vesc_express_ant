# VESC Express build helper for Windows
# Prerequisites: ESP-IDF v5.2.2, Python 3.10+
#
# Usage:
#   .\build_idf.ps1              # build (default)
#   .\build_idf.ps1 flash        # flash
#   .\build_idf.ps1 fullclean    # clean

# Remove MSYSTEM env var that Git Bash sets — ESP-IDF rejects it
Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue

# Python 3.10 must be first in PATH (ESP-IDF venv is built against it)
# Override with PYTHON3_DIR env var if installed elsewhere
$pyDir = if ($env:PYTHON3_DIR) { $env:PYTHON3_DIR } else { "$env:LOCALAPPDATA\Programs\Python\Python310" }
if (Test-Path "$pyDir\python.exe") {
    $env:PATH = "$pyDir;$pyDir\Scripts;$env:PATH"
}

# Find IDF — override with IDF_PATH env var
$idfPath = if ($env:IDF_PATH) { $env:IDF_PATH } else { "C:\esp\esp-idf-v5.2.2" }
if (-not (Test-Path "$idfPath\export.ps1")) {
    Write-Error "ESP-IDF not found at $idfPath. Set IDF_PATH env var."
    exit 1
}

. "$idfPath\export.ps1"
Set-Location $PSScriptRoot
idf.py @args
