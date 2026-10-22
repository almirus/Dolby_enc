param([string]$Batch = "build.bat", [string]$Log = "logs/build.txt")
$ErrorActionPreference = "Continue"
$root = $PSScriptRoot
Set-Location $root
if (-not (Test-Path logs)) { New-Item -ItemType Directory logs | Out-Null }
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $root "gen_rsp.ps1") | Out-Null
$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = "cmd.exe"
$psi.Arguments = "/c $Batch > $Log 2>&1"
$psi.WorkingDirectory = $root
$psi.UseShellExecute = $false
$proc = [System.Diagnostics.Process]::Start($psi)
$proc.WaitForExit(900000) | Out-Null
Get-Content $Log | Select-Object -Last 60
Write-Host "exit=$($proc.ExitCode)"
exit $proc.ExitCode
