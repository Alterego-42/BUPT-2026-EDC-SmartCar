param(
    [string]$CcsRoot,
    [string]$SdkRoot,
    [switch]$Clean,
    [switch]$NoBuild,
    [switch]$NoFlash,
    [switch]$SkipProbeCheck,
    [int]$TimeoutSec = 120
)

$ErrorActionPreference = "Stop"

function Resolve-ExistingPath {
    param(
        [string[]]$Candidates,
        [string]$Name
    )

    foreach ($candidate in $Candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate)) {
            continue
        }
        $expanded = [Environment]::ExpandEnvironmentVariables($candidate)
        $matches = @(Get-Item -LiteralPath $expanded -ErrorAction SilentlyContinue)
        if ($matches.Count -gt 0) {
            return $matches[0].FullName
        }
    }

    throw "Cannot find $Name. Pass -$Name <path> or set it in this script."
}

function Find-FirstFile {
    param(
        [string[]]$Roots,
        [string]$Filter,
        [string]$Name
    )

    foreach ($root in $Roots) {
        if ([string]::IsNullOrWhiteSpace($root) -or -not (Test-Path -LiteralPath $root)) {
            continue
        }
        $found = Get-ChildItem -Path $root -Recurse -Filter $Filter -File -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if ($null -ne $found) {
            return $found.FullName
        }
    }

    throw "Cannot find $Name under: $($Roots -join ', ')"
}

function To-MakePath {
    param([string]$Path)
    return ($Path -replace "\\", "/")
}

function Invoke-Step {
    param(
        [string]$Title,
        [scriptblock]$Body
    )

    Write-Host ""
    Write-Host "==> $Title" -ForegroundColor Cyan
    & $Body
}

$ProjectRoot = $PSScriptRoot
$BuildDir = Join-Path $ProjectRoot "ticlang"
$OutFile = Join-Path $BuildDir "ecar_debug_mspm0g3507.out"
$Ccxml = Join-Path $BuildDir "mspm0g3507_xds110.ccxml"

if (-not (Test-Path -LiteralPath (Join-Path $BuildDir "makefile"))) {
    throw "Cannot find ticlang makefile. Script must stay in the ecar_debug_mspm0g3507 project root."
}

if ([string]::IsNullOrWhiteSpace($CcsRoot)) {
    $CcsRoot = Resolve-ExistingPath @(
        $env:CCS_ROOT,
        $env:CCS_INSTALL_DIR,
        "D:\ti\ccs\ccs",
        "C:\ti\ccs\ccs"
    ) "CcsRoot"
}

if (-not (Test-Path -LiteralPath (Join-Path $CcsRoot "ccs_base")) -and
    (Test-Path -LiteralPath (Join-Path $CcsRoot "ccs\ccs_base"))) {
    $CcsRoot = Join-Path $CcsRoot "ccs"
}

if ([string]::IsNullOrWhiteSpace($SdkRoot)) {
    $SdkRoot = Resolve-ExistingPath @(
        $env:MSPM0_SDK_INSTALL_DIR,
        "D:\ti\mspm0_sdk",
        "D:\ti\ccs\mspm0_sdk_2_10_00_04",
        "C:\ti\mspm0_sdk"
    ) "SdkRoot"
}

$Gmake = Join-Path $CcsRoot "utils\bin\gmake.exe"
$Dslite = Join-Path $CcsRoot "ccs_base\DebugServer\bin\DSLite.exe"
$SysconfigTool = Find-FirstFile @((Join-Path $CcsRoot "utils")) "sysconfig_cli.bat" "SysConfig CLI"
$TiArmClangExe = Find-FirstFile @(
    (Join-Path $CcsRoot "tools\compiler"),
    (Join-Path (Split-Path $CcsRoot -Parent) "ti_cgt_arm_llvm_4.0.2.LTS")
) "tiarmclang.exe" "TI Arm Clang"
$TiArmCompilerRoot = Split-Path (Split-Path $TiArmClangExe -Parent) -Parent

foreach ($required in @($Gmake, $Dslite, $SysconfigTool, $TiArmClangExe, $Ccxml)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Required file not found: $required"
    }
}

if (-not (Test-Path -LiteralPath (Join-Path $SdkRoot ".metadata\product.json"))) {
    throw "MSPM0 SDK product.json not found under $SdkRoot"
}

$MakeArgs = @(
    "-f", "makefile",
    "MSPM0_SDK_INSTALL_DIR=$(To-MakePath $SdkRoot)",
    "SYSCONFIG_TOOL=$(To-MakePath $SysconfigTool)",
    "TICLANG_ARMCOMPILER=$(To-MakePath $TiArmCompilerRoot)"
)

Write-Host "Project : $ProjectRoot"
Write-Host "CCS     : $CcsRoot"
Write-Host "SDK     : $SdkRoot"
Write-Host "Output  : $OutFile"

if ($Clean) {
    Invoke-Step "Clean" {
        Push-Location $BuildDir
        try {
            & $Gmake @MakeArgs clean
            if ($LASTEXITCODE -ne 0) { throw "Clean failed with exit code $LASTEXITCODE" }
        } finally {
            Pop-Location
        }
    }
}

if (-not $NoBuild) {
    Invoke-Step "Build" {
        Push-Location $BuildDir
        try {
            & $Gmake @MakeArgs
            if ($LASTEXITCODE -ne 0) { throw "Build failed with exit code $LASTEXITCODE" }
        } finally {
            Pop-Location
        }
    }
}

if (-not (Test-Path -LiteralPath $OutFile)) {
    throw "Build output not found: $OutFile"
}

if ($NoFlash) {
    Write-Host ""
    Write-Host "Build output is ready. Flash skipped because -NoFlash was specified." -ForegroundColor Yellow
    exit 0
}

if (-not $SkipProbeCheck) {
    Invoke-Step "Identify connected probe" {
        & $Dslite identifyProbe --config="$Ccxml"
        if ($LASTEXITCODE -ne 0) { throw "Probe check failed with exit code $LASTEXITCODE" }
    }
}

Invoke-Step "Flash and run target" {
    & $Dslite flash --config="$Ccxml" --verbose --timeout=$TimeoutSec --flash --verify --run "$OutFile"
    if ($LASTEXITCODE -ne 0) { throw "Flash failed with exit code $LASTEXITCODE" }
}

Write-Host ""
Write-Host "Done. Firmware was programmed and the target was started." -ForegroundColor Green
