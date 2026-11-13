param(
    [ValidateSet('Start','Stop')][string]$Action = 'Start',
    [switch]$NoBrowser
)
$ErrorActionPreference = 'Stop'
try {
    if ($PSScriptRoot -match '^\\\\(?:wsl\.localhost|wsl\$)\\([^\\]+)(\\.*)$') {
        $distro = $Matches[1]
        $linuxDirectory = $Matches[2].Replace('\','/')
        $arguments = @('-d', $distro)
    } else {
        $arguments = @()
        $linuxDirectory = (& wsl.exe --exec wslpath -u $PSScriptRoot | Out-String).Trim()
        if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve this folder through WSL.' }
    }
    & wsl.exe @arguments --exec bash "$linuxDirectory/dashboard.sh" $Action.ToLowerInvariant()
    if ($LASTEXITCODE -ne 0) { throw "WSL Dashboard $Action failed (exit $LASTEXITCODE). See the error and server.log above." }
    if ($Action -eq 'Start' -and -not $NoBrowser) {
        Start-Process 'http://localhost:8060'
    }
    exit 0
} catch {
    Write-Host "Dashboard error: $_" -ForegroundColor Red
    exit 1
}
