param(
    [string]$MSBuildPath = '',
    [ValidateSet('Debug', 'Release')][string]$Configuration = 'Release'
)
$ErrorActionPreference = 'Stop'
Push-Location -LiteralPath $PSScriptRoot
try {
    if (-not $MSBuildPath) {
        $installer = Join-Path ([Environment]::GetFolderPath('ProgramFilesX86')) 'Microsoft Visual Studio\Installer\vswhere.exe'
        if (Test-Path -LiteralPath $installer) {
            $MSBuildPath = & $installer -version '[16.0,17.0)' -products '*' -requires Microsoft.Component.MSBuild -find 'MSBuild\Current\Bin\MSBuild.exe' | Select-Object -First 1
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
        throw 'Install Visual Studio 2019: .NET desktop development and .NET Framework 4.7.2 targeting pack. Or pass -MSBuildPath.'
    }
    # Build the tests project directly so solution-specific configuration mappings
    # cannot make this script accidentally run a stale Debug/Release test EXE.
    # Its project reference builds the WPF application with the same configuration.
    & $MSBuildPath 'Tests\RawDataSender.Tests.csproj' /t:Rebuild "/p:Configuration=$Configuration" /nologo /verbosity:minimal
    if ($LASTEXITCODE -ne 0) { throw 'Build failed.' }
    & (Join-Path $PSScriptRoot "Tests\bin\$Configuration\RawDataSender.Tests.exe")
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed. Published files were not updated.' }
    $publishDirectory = Join-Path $PSScriptRoot 'publish'
    New-Item -ItemType Directory -Path $publishDirectory -Force | Out-Null
    foreach ($file in @('RawDataSender.exe', 'RawDataSender.exe.config')) {
        Copy-Item -LiteralPath (Join-Path $PSScriptRoot "bin\$Configuration\$file") -Destination (Join-Path $publishDirectory $file) -Force
    }
    Write-Host "Ready: $publishDirectory\RawDataSender.exe"
    Write-Host 'Keep the EXE.config next to the EXE. Requires .NET Framework 4.7.2 or later; no Python/.NET SDK/NuGet runtime dependency.'
} finally { Pop-Location }
