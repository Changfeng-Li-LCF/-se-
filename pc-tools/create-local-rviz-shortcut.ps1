$ErrorActionPreference = 'Stop'
$desktopPath = [Environment]::GetFolderPath('Desktop')
$shortcutPath = Join-Path $desktopPath 'RViz（电脑运行）.lnk'
$shell = New-Object -ComObject WScript.Shell
$shortcut = $shell.CreateShortcut($shortcutPath)
$shortcut.TargetPath = "$env:WINDIR\System32\wsl.exe"
$shortcut.Arguments = '-d RacecarUbuntu2204 -- bash /mnt/d/RacecarWork/tools/run-rviz.sh'
$shortcut.WorkingDirectory = 'D:\RacecarWork'
$shortcut.Description = 'RViz runs on this computer using WSLg; 30 FPS; no SSH rendering.'
$shortcut.WindowStyle = 7
$shortcut.Save()
Write-Output $shortcutPath
