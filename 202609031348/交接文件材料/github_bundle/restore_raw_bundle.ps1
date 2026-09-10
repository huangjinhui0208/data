$ErrorActionPreference = 'Stop'

$bundleDir = $PSScriptRoot
$handoffDir = Split-Path -Parent $bundleDir
$rawDir = Join-Path $handoffDir 'raw'
$archivePath = Join-Path $bundleDir 'raw_bundle.tar.gz'
$manifestPath = Join-Path $bundleDir 'RAW_BUNDLE_MANIFEST.csv'

if (Test-Path -LiteralPath $rawDir) {
    throw "Refusing to overwrite existing directory: $rawDir"
}

$rows = Import-Csv -LiteralPath $manifestPath
$archiveRow = $rows | Where-Object { $_.kind -eq 'archive' }
$partRows = $rows | Where-Object { $_.kind -eq 'part' } | Sort-Object name
if ($archiveRow.Count -ne 1 -or $partRows.Count -eq 0) {
    throw "Invalid bundle manifest: $manifestPath"
}

$output = [System.IO.File]::Create($archivePath)
try {
    foreach ($part in $partRows) {
        $partPath = Join-Path $bundleDir $part.name
        $actualPartHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $partPath).Hash.ToLowerInvariant()
        if ($actualPartHash -ne $part.sha256) {
            throw "Part checksum mismatch: $($part.name)"
        }
        $input = [System.IO.File]::OpenRead($partPath)
        try { $input.CopyTo($output) } finally { $input.Dispose() }
    }
} finally {
    $output.Dispose()
}

$actualArchiveHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $archivePath).Hash.ToLowerInvariant()
if ($actualArchiveHash -ne $archiveRow.sha256) {
    Remove-Item -LiteralPath $archivePath -Force
    throw 'Combined archive checksum mismatch.'
}

try {
    & tar.exe -xzf $archivePath -C $handoffDir
    if ($LASTEXITCODE -ne 0) { throw "tar extraction failed with exit code $LASTEXITCODE" }
} finally {
    Remove-Item -LiteralPath $archivePath -Force -ErrorAction SilentlyContinue
}

Write-Host "Restored and verified: $rawDir"

