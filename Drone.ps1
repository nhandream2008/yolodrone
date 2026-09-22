param(
    [ValidateSet('sim', 'sim-avoid', 'sim-slalom', 'sim-challenge', 'avoid', 'detect', 'detect-avoid', 'detect-slalom', 'detect-challenge', 'capture', 'view', 'monitor', 'replay', 'benchmark', 'scenario-matrix', 'unified', 'check', 'setup')]
    [string]$Action = 'check',
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Options
)
$ErrorActionPreference = 'Stop'
$linuxProject = (& wsl.exe -d Ubuntu-22.04 -u drone -- wslpath -a $PSScriptRoot).Trim()
if ($LASTEXITCODE -ne 0 -or -not $linuxProject.StartsWith('/')) {
    throw 'Khong truy cap duoc Ubuntu-22.04. Kiem tra: wsl --list --verbose.'
}
if ($Action -eq 'setup') {
    & wsl.exe -d Ubuntu-22.04 -u root -- env SETUP_USER=drone bash "$linuxProject/scripts/setup_ubuntu.sh"
}
else {
    & wsl.exe -d Ubuntu-22.04 -u drone -- bash "$linuxProject/scripts/run.sh" $Action @Options
}
exit $LASTEXITCODE
