$ErrorActionPreference = 'Stop'
$menuPython = 'D:\RacecarTools\envs\yolov5\pythonw.exe'
if (-not (Test-Path -LiteralPath $menuPython)) { throw '未找到小车命令菜单所需的 Python 环境。' }
$menuScript = Join-Path $PSScriptRoot 'menu.py'
Start-Process -FilePath $menuPython -ArgumentList @(('"' + $menuScript + '"')) -WorkingDirectory $PSScriptRoot -WindowStyle Hidden
