param(
    [string]$MSBuildPath = "",
    [ValidateSet("Debug", "Release")][string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
Push-Location -LiteralPath $PSScriptRoot
try {
    if ([string]::IsNullOrWhiteSpace($MSBuildPath)) {
        $installer = Join-Path ([Environment]::GetFolderPath('ProgramFilesX86')) 'Microsoft Visual Studio\Installer\vswhere.exe'
        if (Test-Path -LiteralPath $installer) {
            $found = & $installer -version '[16.0,17.0)' -products '*' -requires Microsoft.Component.MSBuild -find 'MSBuild\Current\Bin\MSBuild.exe'
            $MSBuildPath = $found | Select-Object -First 1
            if (-not $MSBuildPath) {
                $MSBuildPath = & $installer -latest -products '*' -requires Microsoft.Component.MSBuild -find 'MSBuild\Current\Bin\MSBuild.exe' | Select-Object -First 1
            }
        }
        if (-not $MSBuildPath) {
            $command = Get-Command MSBuild.exe -ErrorAction SilentlyContinue
            if ($command) { $MSBuildPath = $command.Source }
        }
    }
    if (-not $MSBuildPath -or -not (Test-Path -LiteralPath $MSBuildPath)) {
        throw 'MSBuild was not found. Install Visual Studio 2019 with .NET desktop development and the .NET Framework 4.7.2 targeting pack. You can also pass -MSBuildPath to this script.'
    }
    Write-Host "Using MSBuild: $MSBuildPath"
    & $MSBuildPath PcdConverter.sln /t:Rebuild "/p:Configuration=$Configuration" /nologo /verbosity:minimal
    if ($LASTEXITCODE -ne 0) { throw 'Build failed. Check the .NET desktop workload and .NET Framework 4.7.2 targeting pack in Visual Studio Installer.' }

    & (Join-Path $PSScriptRoot "Tests\bin\$Configuration\PcdConverter.Tests.exe")
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed.' }

    $publishDirectory = Join-Path $PSScriptRoot 'publish'
    New-Item -ItemType Directory -Path $publishDirectory -Force | Out-Null
    foreach ($file in @('PcdConverter.exe','PcdConverter.exe.config')) {
        Copy-Item -LiteralPath (Join-Path $PSScriptRoot "bin\$Configuration\$file") -Destination (Join-Path $publishDirectory $file) -Force
    }
    # Repository history is a Windows user preference; no TXT configuration is generated.
    Write-Host "Ready: $publishDirectory\PcdConverter.exe"
    Write-Host 'Keep PcdConverter.exe.config beside the executable. Requires .NET Framework 4.7.2 or later.'
} finally {
    Pop-Location
}
