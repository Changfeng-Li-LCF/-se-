$ErrorActionPreference = 'Stop'
$ruleName = 'Racecar-ROS2-Car-UDP'
$resultPath = 'D:\RacecarWork\verification\racecar-firewall-result.txt'
try {
    $settings = @{
        Direction = 'Inbound'
        VMCreatorId = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
        Protocol = 'UDP'
        RemoteAddresses = 'Any'
        LocalPorts = '7400-7649'
        Action = 'Allow'
        Enabled = 'True'
    }
    $displayName = 'Racecar ROS2 UDP - any source IP'
    if (Get-NetFirewallHyperVRule -Name $ruleName -ErrorAction SilentlyContinue) {
        Set-NetFirewallHyperVRule -Name $ruleName -NewDisplayName $displayName @settings | Out-Null
    } else {
        New-NetFirewallHyperVRule -Name $ruleName -DisplayName $displayName @settings | Out-Null
    }
    Get-NetFirewallHyperVRule -Name $ruleName | Select-Object Name,Action,RemoteAddresses,LocalPorts | Format-List | Out-String | Set-Content -LiteralPath $resultPath -Encoding UTF8
} catch {
    $_.Exception.Message | Set-Content -LiteralPath $resultPath -Encoding UTF8
    exit 1
}
