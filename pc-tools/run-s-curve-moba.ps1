$ErrorActionPreference = 'Stop'
Start-Transcript -Path (Join-Path $PSScriptRoot 'run-s-curve-moba.log') -Append -Force
& 'D:\RacecarTools\envs\yolov5\python.exe' (Join-Path $PSScriptRoot 'racecar_network.py') | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Cannot locate the configured car on the current network.' }
$taskConnection = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $PSScriptRoot 'racecar-connection.json') | ConvertFrom-Json
$taskHostOptions = @('-o', ('HostKeyAlias='+$taskConnection.verified_host_alias), '-o', ('UserKnownHostsFile='+$taskConnection.verified_known_hosts_file), '-o', 'StrictHostKeyChecking=yes')
$taskTarget = $taskConnection.username + '@' + $taskConnection.host
$taskWindowsBin = "$env:WINDIR\System32"
if (Test-Path -LiteralPath "$env:WINDIR\Sysnative\OpenSSH\ssh.exe") { $taskWindowsBin = "$env:WINDIR\Sysnative" }
$taskSsh = Join-Path $taskWindowsBin 'OpenSSH\ssh.exe'
$taskScp = Join-Path $taskWindowsBin 'OpenSSH\scp.exe'
Write-Host 'Starting recorded S-curve run. Ctrl+C requests a stop.'
& $taskSsh @taskHostOptions -tt -i $taskConnection.ssh_identity_file -p $taskConnection.port -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=25 -o ServerAliveInterval=3 -o ServerAliveCountMax=2 $taskTarget 'bash ~/racecar-tests/s-curve-test/run_s_curve.sh'
$taskRunExit = $LASTEXITCODE
$taskRemoteRun = (& $taskSsh @taskHostOptions -i $taskConnection.ssh_identity_file -p $taskConnection.port -o IdentitiesOnly=yes -o BatchMode=yes $taskTarget 'cat ~/racecar-tests/s-curve-test/latest_run.txt').Trim()
if ($taskRemoteRun -match '^/home/bianbu/racecar-tests/s-curve-test/runs/s-curve-\d{8}-\d{6}$') {
    $taskOutRoot = Join-Path (Split-Path $PSScriptRoot -Parent) 'reports\s-curve-runs'
    New-Item -ItemType Directory -Force -Path $taskOutRoot | Out-Null
    & $taskScp @taskHostOptions -r -i $taskConnection.ssh_identity_file -P $taskConnection.port -o IdentitiesOnly=yes -o BatchMode=yes "${taskTarget}:$taskRemoteRun" $taskOutRoot
    $taskLocalRun = Join-Path $taskOutRoot ($taskRemoteRun.Split('/')[-1])
    $taskTracking = Join-Path $taskLocalRun 'tracking'
    if (Test-Path -LiteralPath (Join-Path $taskTracking 'metadata.json')) {
        & 'D:\RacecarTools\envs\yolov5\python.exe' (Join-Path (Split-Path $PSScriptRoot -Parent) 'code\s-curve-test\analyze_tracking.py') $taskTracking
    }
    Write-Host "Results: $taskLocalRun"
}
Write-Host "Run finished, exit code $taskRunExit."
Stop-Transcript
