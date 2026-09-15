$ErrorActionPreference = "Stop"
$ToolDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$VenvDir = Join-Path $ToolDir ".venv"

if (-not (Test-Path (Join-Path $VenvDir "Scripts\python.exe"))) {
    python -m venv $VenvDir
    & (Join-Path $VenvDir "Scripts\python.exe") -m pip install -r (Join-Path $ToolDir "requirements.txt")
}

& (Join-Path $VenvDir "Scripts\python.exe") (Join-Path $ToolDir "app.py")
