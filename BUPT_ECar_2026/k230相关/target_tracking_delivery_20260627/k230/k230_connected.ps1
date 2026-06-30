param(
    [string]$Port = "",
    [switch]$Upload,
    [switch]$Run,
    [switch]$Debug,
    [double]$Seconds = 0,
    [switch]$NoControl,
    [switch]$Force,
    [double]$ForceYaw = 0.0,
    [double]$ForcePitch = 1.0,
    [switch]$Capture,
    [switch]$Diagnose,
    [switch]$Reset,
    [switch]$List
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$MainScript = Join-Path $ScriptDir "k230_target_uart.py"
$ConfigScript = Join-Path $ScriptDir "k230_target_config.py"
$BootScript = Join-Path $ScriptDir "main.py"
$CaptureScript = Join-Path $ScriptDir "tools\k230_capture_frame.py"
$ForceOnceScript = Join-Path $ScriptDir "tools\k230_force_once.py"
$RemoteMain = "/sdcard/k230_target_uart_v72.py"
$RemoteConfig = "/sdcard/k230_target_config_v72.py"
$RemoteBoot = "/sdcard/main.py"
$CaptureLocal = Join-Path (Split-Path -Parent $ScriptDir) "k230_capture_latest.jpg"
$DiagCaptureLocal = Join-Path (Split-Path -Parent $ScriptDir) "k230_diag_latest.jpg"

function Invoke-MpRemote {
    param([string[]]$ArgsList)

    $effectiveArgs = @()
    for ($i = 0; $i -lt $ArgsList.Count; $i++) {
        $effectiveArgs += $ArgsList[$i]
        if ($ArgsList[$i] -eq "connect" -and ($i + 1) -lt $ArgsList.Count) {
            $i++
            $effectiveArgs += $ArgsList[$i]
            if (($i + 1) -ge $ArgsList.Count -or $ArgsList[$i + 1] -ne "resume") {
                $effectiveArgs += "resume"
            }
        }
    }

    & python -m mpremote @effectiveArgs
    if ($LASTEXITCODE -ne 0) {
        throw "mpremote failed: $($effectiveArgs -join ' ')"
    }
}

function Write-K230ScriptFile {
    param(
        [string]$Path,
        [string]$Text
    )

    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($Path, $Text, $utf8NoBom)
}

function Stop-K230MpRemoteProcesses {
    Get-CimInstance Win32_Process -Filter "name = 'python.exe'" |
        Where-Object { $_.CommandLine -match 'mpremote' } |
        ForEach-Object {
            Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
        }
}

function Invoke-K230SerialBreak {
    param([string]$DevicePort)

    $py = @"
import time
import serial

port = r"$DevicePort"
try:
    ser = serial.Serial(port, 115200, timeout=0.15)
    time.sleep(0.15)
    for _ in range(16):
        ser.write(b"\x03")
        ser.flush()
        time.sleep(0.04)
    ser.write(b"\r\n")
    ser.flush()
    time.sleep(0.2)
    ser.close()
except Exception as exc:
    print("serial break failed: %s" % exc)
"@
    $py | python -
}

function Test-K230Repl {
    param([string]$DevicePort)

    $out = & python -m mpremote connect $DevicePort resume exec "print('REPL_READY')" 2>&1
    if ($LASTEXITCODE -eq 0 -and ($out -join "`n") -match "REPL_READY") {
        return $true
    }
    Write-Host ($out -join "`n")
    return $false
}

function Reset-K230ToRepl {
    param([string]$DevicePort)

    Write-Host "Resetting K230 to REPL..."
    Stop-K230MpRemoteProcesses

    for ($attempt = 1; $attempt -le 3; $attempt++) {
        Invoke-K230SerialBreak -DevicePort $DevicePort
        Start-Sleep -Milliseconds (700 * $attempt)
        if (Test-K230Repl -DevicePort $DevicePort) {
            Write-Host "K230 REPL_READY."
            return
        }
    }

    throw "K230 did not enter REPL after serial break attempts. Press the physical RST button, then retry."
}

function Get-K230Port {
    if ($Port) {
        return $Port
    }

    $devs = & python -m mpremote devs
    foreach ($line in $devs) {
        if ($line -match '^(COM\d+)\s+\S+\s+1209:abd1\b') {
            return $matches[1]
        }
    }

    $serials = Get-CimInstance Win32_SerialPort |
        Where-Object {
            $_.DeviceID -match '^COM\d+$' -and
            $_.Name -notmatch 'XDS110' -and
            ($_.Name -match 'USB|串行|Serial' -or $_.Description -match 'USB|串行|Serial')
        }

    if ($serials.Count -eq 1) {
        return $serials[0].DeviceID
    }

    throw "Cannot auto-detect K230 COM port. Run 'python -m mpremote devs' and pass -Port COMx."
}

function New-DebugScript {
    param(
        [double]$RunSeconds,
        [bool]$EnableDebug,
        [bool]$DisableControl,
        [bool]$ForceCommand,
        [double]$YawCommand,
        [double]$PitchCommand
    )

    if ($RunSeconds -le 0) {
        $RunSeconds = 10
    }

    $text = Get-Content -Raw $MainScript
    $postLoad = @()
    if ($EnableDebug -and -not $ForceCommand) {
        $postLoad += 'USB_DEBUG_ENABLED = True'
        $postLoad += 'TARGET_CANDIDATE_DEBUG = True'
    }
    $postLoad += "DEFAULT_RUN_SECONDS = $RunSeconds"
    if ($DisableControl) {
        $postLoad += 'CONTROL_OUTPUT_ENABLED = False'
    }
    if ($ForceCommand) {
        $postLoad += 'FORCE_COMMAND_ENABLED = True'
        $postLoad += ("FORCE_YAW = {0:F2}" -f $YawCommand)
        $postLoad += ("FORCE_PITCH = {0:F2}" -f $PitchCommand)
    }
    if ($postLoad.Count -gt 0) {
        $postText = $postLoad -join "`n"
        $text = $text -replace '(?m)^load_config_overrides\(\)\s*$', "load_config_overrides()`n$postText"
    }

    $tmp = Join-Path $env:TEMP ("k230_target_uart_debug_{0}.py" -f $PID)
    Write-K230ScriptFile -Path $tmp -Text $text
    return $tmp
}

function New-ForceOnceScript {
    param(
        [double]$RunSeconds,
        [double]$YawCommand,
        [double]$PitchCommand
    )

    if ($RunSeconds -le 0) {
        $RunSeconds = 1
    }

    $text = Get-Content -Raw $ForceOnceScript
    $text = $text -replace '(?m)^YAW\s*=\s*[-+0-9.]+\s*$', ("YAW = {0:F2}" -f $YawCommand)
    $text = $text -replace '(?m)^PITCH\s*=\s*[-+0-9.]+\s*$', ("PITCH = {0:F2}" -f $PitchCommand)
    $text = $text -replace '(?m)^SECONDS\s*=\s*[-+0-9.]+\s*$', ("SECONDS = {0:F2}" -f $RunSeconds)

    $tmp = Join-Path $env:TEMP ("k230_force_once_{0}.py" -f $PID)
    Write-K230ScriptFile -Path $tmp -Text $text
    return $tmp
}

function New-DiagnoseScript {
    $text = Get-Content -Raw $MainScript
    $postText = @(
        'DIAGNOSE_ONCE = True',
        'CONTROL_OUTPUT_ENABLED = False',
        'USB_DEBUG_ENABLED = False'
    ) -join "`n"
    $text = $text -replace '(?m)^load_config_overrides\(\)\s*$', "load_config_overrides()`n$postText"
    $tmp = Join-Path $env:TEMP ("k230_target_uart_diag_{0}.py" -f $PID)
    Write-K230ScriptFile -Path $tmp -Text $text
    return $tmp
}

if (-not ($Upload -or $Run -or $Capture -or $Diagnose -or $Reset -or $List -or $Force)) {
    $Upload = $true
    $Run = $true
    $Debug = $true
    $Seconds = 10
}

if ($Force) {
    $Run = $true
    $Debug = $true
}

$K230Port = Get-K230Port
Write-Host "K230 port: $K230Port"

if ($Upload -or $Run -or $Capture -or $Diagnose -or $Force -or $Reset) {
    Reset-K230ToRepl -DevicePort $K230Port
}

if ($List) {
    Invoke-MpRemote @("devs")
    Invoke-MpRemote @("connect", $K230Port, "fs", "ls", "/sdcard")
}

if ($Reset) {
    Write-Host "K230 is reset to REPL."
}

if ($Upload) {
    if (-not (Test-Path $MainScript)) {
        throw "Missing K230 script: $MainScript"
    }
    if (-not (Test-Path $ConfigScript)) {
        throw "Missing K230 config: $ConfigScript"
    }
    if (-not (Test-Path $BootScript)) {
        throw "Missing K230 boot script: $BootScript"
    }
    Write-Host "Uploading $MainScript -> $RemoteMain"
    Invoke-MpRemote @("connect", $K230Port, "fs", "cp", $MainScript, ":$RemoteMain")
    Write-Host "Uploading $ConfigScript -> $RemoteConfig"
    Invoke-MpRemote @("connect", $K230Port, "fs", "cp", $ConfigScript, ":$RemoteConfig")
    Write-Host "Uploading $BootScript -> $RemoteBoot"
    Invoke-MpRemote @("connect", $K230Port, "fs", "cp", $BootScript, ":$RemoteBoot")
    Write-Host "Standalone boot enabled: $RemoteBoot imports $RemoteMain and reads $RemoteConfig."
}

if ($Capture) {
    if (-not (Test-Path $CaptureScript)) {
        throw "Missing capture script: $CaptureScript"
    }
    Write-Host "Capturing camera frame..."
    Invoke-MpRemote @("connect", $K230Port, "run", $CaptureScript)
    Invoke-MpRemote @("connect", $K230Port, "fs", "cp", ":/sdcard/k230_capture.jpg", $CaptureLocal)
    Write-Host "Capture copied to $CaptureLocal"
}

if ($Diagnose) {
    $runScript = New-DiagnoseScript
    Write-Host "Running one-shot K230 diagnosis."
    try {
        Invoke-MpRemote @("connect", $K230Port, "run", $runScript)
        try {
            Invoke-MpRemote @("connect", $K230Port, "fs", "cp", ":/sdcard/k230_diag.jpg", $DiagCaptureLocal)
            Write-Host "Diagnostic capture copied to $DiagCaptureLocal"
        } catch {
            Write-Host "Diagnostic capture copy failed: $_"
        }
    } finally {
        Remove-Item -LiteralPath $runScript -ErrorAction SilentlyContinue
    }
}

if ($Run) {
    if ($Force) {
        if (-not (Test-Path $ForceOnceScript)) {
            throw "Missing K230 force script: $ForceOnceScript"
        }
        $runScript = New-ForceOnceScript -RunSeconds $Seconds `
            -YawCommand $ForceYaw `
            -PitchCommand $ForcePitch
        Write-Host ("Running UART-only force script for {0} second(s), yaw={1:F2}, pitch={2:F2}." -f `
                ($(if ($Seconds -gt 0) { $Seconds } else { 1 })), $ForceYaw, $ForcePitch)
        try {
            Invoke-MpRemote @("connect", $K230Port, "run", $runScript)
        } finally {
            Remove-Item -LiteralPath $runScript -ErrorAction SilentlyContinue
        }
    } elseif ($Debug -or $Seconds -gt 0) {
        $runScript = New-DebugScript -RunSeconds $Seconds `
            -EnableDebug $Debug.IsPresent `
            -DisableControl $NoControl.IsPresent `
            -ForceCommand $Force.IsPresent `
            -YawCommand $ForceYaw `
            -PitchCommand $ForcePitch
        Write-Host "Running local debug script for $Seconds second(s)."
        try {
            Invoke-MpRemote @("connect", $K230Port, "run", $runScript)
        } finally {
            Remove-Item -LiteralPath $runScript -ErrorAction SilentlyContinue
        }
    } else {
        Write-Host "Running deployed script from $RemoteMain. Press Ctrl+C to stop."
        Invoke-MpRemote @("connect", $K230Port, "exec", "exec(open('$RemoteMain').read())")
    }
}
