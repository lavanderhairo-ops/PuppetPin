# Installs the Puppet Pin OFX plugin for DaVinci Resolve (Edit and Fusion pages).
#
# The plugin is copied to %ProgramData%\PuppetPin\PuppetPin.ofx.bundle, and Resolve's OFX
# folder gets a link to it. Only creating that link needs administrator rights (one UAC
# prompt, the first time); later updates are a plain copy. Close Resolve before running.
param(
    [string]$Bundle = (Join-Path $PSScriptRoot "build\PuppetPin.ofx.bundle"),
    [switch]$Uninstall
)

$store = Join-Path $env:ProgramData "PuppetPin"
$storeBundle = Join-Path $store "PuppetPin.ofx.bundle"
$link = "C:\Program Files\Common Files\OFX\Plugins\PuppetPin.ofx.bundle"

function Test-Junction($path) {
    $item = Get-Item -LiteralPath $path -Force -ErrorAction SilentlyContinue
    return $item -and ($item.Attributes -band [IO.FileAttributes]::ReparsePoint)
}

function Invoke-Elevated($command) {
    $p = Start-Process powershell -Verb RunAs -Wait -PassThru -ArgumentList @(
        "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", $command)
    return $p.ExitCode
}

if (Get-Process -Name Resolve -ErrorAction SilentlyContinue) {
    Write-Host "DaVinci Resolve is running. Close it, then run this again." -ForegroundColor Yellow
    exit 2
}

if ($Uninstall) {
    if (Test-Path -LiteralPath $link) {
        Invoke-Elevated "Remove-Item -LiteralPath '$link' -Recurse -Force" | Out-Null
    }
    if (Test-Path $store) { Remove-Item -Recurse -Force $store }
    Write-Host "Puppet Pin removed."
    exit 0
}

if (-not (Test-Path (Join-Path $Bundle "Contents\Win64\PuppetPin.ofx"))) {
    Write-Host "Bundle not found: $Bundle (build first: ./build.sh plugin)" -ForegroundColor Red
    exit 1
}

New-Item -ItemType Directory -Force $store | Out-Null
if (Test-Path $storeBundle) { Remove-Item -Recurse -Force $storeBundle }
Copy-Item -Recurse -Force $Bundle $storeBundle
Write-Host "Copied plugin to $storeBundle"

if (-not (Test-Junction $link)) {
    Write-Host "Linking Resolve's OFX folder to it (one-time administrator prompt)..."
    $cmd = "if (Test-Path -LiteralPath '$link') { Remove-Item -LiteralPath '$link' -Recurse -Force }; " +
           "New-Item -ItemType Junction -Path '$link' -Target '$storeBundle' | Out-Null"
    Invoke-Elevated $cmd | Out-Null
    if (-not (Test-Junction $link)) {
        Write-Host "Could not create $link" -ForegroundColor Red
        exit 3
    }
}
Write-Host "Installed. Start DaVinci Resolve." -ForegroundColor Green
exit 0
