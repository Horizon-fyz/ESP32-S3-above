<#
.SYNOPSIS
  便携式 ESP-IDF 启动脚本 —— 自动探测本机 ESP-IDF 安装位置, 兼容多台电脑.

.DESCRIPTION
  探测顺序 (取第一个有效项):
    1. 环境变量 $env:IDF_PATH                 (若其下存在 tools\idf.py 与 export.ps1)
    2. <根>\esp-idf*                          (直接克隆在根目录)
    3. <根>\frameworks\esp-idf*               (Espressif 官方安装器布局)
    4. <根>\*\esp-idf*                        (如 C:\esp\v6.0\esp-idf)
    5. <根>\*\*\esp-idf*
  其中 <根> 依次尝试: C:\Espressif  D:\Espressif  E:\Espressif  C:\esp  D:\esp  %USERPROFILE%\esp

  找到后 dot-source 该 IDF 自带的 export.ps1, 由 IDF 自己配好全部环境变量
  (工具链 / python venv / cmake / ninja / ESP_ROM_ELF_DIR 等), 再执行 idf.py.
  因此本脚本不含任何写死的绝对路径, 多台电脑可共用.

.EXAMPLE
  .\idf.ps1 build
  .\idf.ps1 -p COM3 flash monitor
  .\idf.ps1 -p COM3 monitor

.NOTES
  首次在新电脑上使用, 需先确保该电脑已安装 ESP-IDF (install.ps1 已执行)。
  若自动探测失败, 可手动设置环境变量后重试:
      $env:IDF_PATH = 'D:\path\to\esp-idf'
  注意: 本文件必须保存为 UTF-8 with BOM, 否则 PowerShell 5.1 会按 GBK 误读导致中文乱码。
#>

param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$IdfArgs
)

# 注意: 这里不能用 'Stop'.
# IDF 的 export.ps1 会把进度信息写到 stderr, 而 PS 5.1 在启用了输出重定向时
# (如 2>&1 / *>) 会把原生命令的 stderr 包装成终止性错误, 用 Stop 会导致脚本中途退出.
$ErrorActionPreference = 'Continue'

# 本项目使用的 ESP-IDF 版本 (用于多版本共存时优选)
$ProjectIdfVersion = '5.5'

function Test-IdfPath {
    param([string]$Path)
    if (-not $Path) { return $false }
    return (Test-Path (Join-Path $Path 'tools\idf.py')) -and (Test-Path (Join-Path $Path 'export.ps1'))
}

function Get-IdfCandidates {
    $found = New-Object System.Collections.Generic.List[string]

    if (Test-IdfPath $env:IDF_PATH) { $found.Add($env:IDF_PATH) }

    $roots = @(
        'C:\Espressif', 'D:\Espressif', 'E:\Espressif',
        'C:\esp', 'D:\esp',
        (Join-Path $env:USERPROFILE 'esp')
    )

    foreach ($root in $roots) {
        if (-not (Test-Path $root)) { continue }
        $patterns = @(
            (Join-Path $root 'esp-idf*'),
            (Join-Path $root 'frameworks\esp-idf*'),
            (Join-Path $root '*\esp-idf*'),
            (Join-Path $root '*\*\esp-idf*')
        )
        foreach ($pat in $patterns) {
            Get-Item -Path $pat -ErrorAction SilentlyContinue |
                Where-Object { $_.PSIsContainer } |
                ForEach-Object { $found.Add($_.FullName) }
        }
    }

    return ($found | Select-Object -Unique)
}

# ---------- 1. 探测 IDF ----------
$candidates = @(Get-IdfCandidates | Where-Object { Test-IdfPath $_ })

if ($candidates.Count -eq 0) {
    Write-Host "==============================================================" -ForegroundColor Red
    Write-Host " 未找到可用的 ESP-IDF 安装" -ForegroundColor Red
    Write-Host "==============================================================" -ForegroundColor Red
    Write-Host " 已搜索: C:\Espressif  D:\Espressif  E:\Espressif  C:\esp  D:\esp  $env:USERPROFILE\esp"
    Write-Host ""
    Write-Host " 请任选其一:"
    Write-Host "   1) 先安装 ESP-IDF (运行其 install.ps1)"
    Write-Host "   2) 手动指定后再运行本脚本:"
    Write-Host "        `$env:IDF_PATH = 'D:\your\esp-idf'"
    exit 1
}

# 多版本共存时优选与本项目匹配的版本; 同版本内 $env:IDF_PATH 优先 (它在列表最前)
$preferred = @($candidates | Where-Object { $_ -like "*$ProjectIdfVersion*" })
if ($preferred.Count -gt 0) {
    $idfPath = $preferred[0]
} elseif (Test-IdfPath $env:IDF_PATH) {
    $idfPath = $env:IDF_PATH
    Write-Host "警告: 未找到 v$ProjectIdfVersion, 改用 `$env:IDF_PATH" -ForegroundColor Yellow
} else {
    $idfPath = $candidates[0]
    Write-Host "警告: 未找到 v$ProjectIdfVersion, 改用 $idfPath" -ForegroundColor Yellow
}

Write-Host "使用 ESP-IDF: $idfPath" -ForegroundColor Cyan
if ($candidates.Count -gt 1) {
    Write-Host "  (本机检测到多个版本, 完整列表:)" -ForegroundColor DarkGray
    foreach ($c in $candidates) { Write-Host "    $c" -ForegroundColor DarkGray }
    Write-Host "  (如需换版本: `$env:IDF_PATH = '<上面的路径>' 后再运行)" -ForegroundColor DarkGray
}

# ---------- 2. 确保 python 可用 (export.ps1 需要调用 activate.py) ----------
if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    $pyCandidates = @(
        (Join-Path $env:USERPROFILE '.espressif\python_env\*\Scripts\python.exe'),
        (Join-Path $env:LOCALAPPDATA 'Programs\Python\*\python.exe'),
        'C:\Espressif\tools\idf-python\*\python.exe'
    )
    $py = Get-Item -Path $pyCandidates -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($py) {
        $env:PATH = "$($py.DirectoryName);$env:PATH"
        Write-Host "已把 python 加入 PATH: $($py.FullName)" -ForegroundColor DarkGray
    } else {
        Write-Host "警告: PATH 上找不到 python, export.ps1 可能失败" -ForegroundColor Yellow
    }
}

# ---------- 3. 加载环境并执行 ----------
. (Join-Path $idfPath 'export.ps1')

if (-not $IdfArgs -or $IdfArgs.Count -eq 0) { $IdfArgs = @('build') }

Write-Host "idf.py $($IdfArgs -join ' ')" -ForegroundColor Cyan
& idf.py @IdfArgs
