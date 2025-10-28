<#
Removes transient build artifacts and logs from the working tree to keep the repo tidy.
Does NOT commit any changes.

Usage:
  pwsh scripts/cleanup-repo.ps1
#>

$ErrorActionPreference = 'Stop'

Write-Host "Cleaning transient artifacts..." -ForegroundColor Cyan

$pathsToRemove = @(
  'nupkgs',
  'boost-install.log',
  'build-final.log',
  'build-full.log',
  'build.log',
  'build2.log',
  'test-results.txt',
  'test-results-final.txt',
  'native/facade/build',
  'native/facade/*.dll',
  'native/facade/*.lib',
  'native/facade/*.exp',
  'native/facade/*.obj',
  'build-native/ur_rtde/build',
  'build-native/ur_rtde/bin',
  'build-native/ur_rtde/install',
  'build-native/ur_rtde/.git'
)

foreach ($p in $pathsToRemove) {
  Get-ChildItem -LiteralPath $p -Force -ErrorAction SilentlyContinue | ForEach-Object {
    Write-Host "Removing: $($_.FullName)" -ForegroundColor DarkYellow
    if ($_.PSIsContainer) { Remove-Item $_.FullName -Recurse -Force }
    else { Remove-Item $_.FullName -Force }
  }
}

Write-Host "Cleanup complete." -ForegroundColor Green

