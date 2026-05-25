# Rebuild layer.json "available" from .terrain files on disk (one rect per tile).
param(
    [Parameter(Mandatory = $true)]
    [string]$TilesetDir
)

$ErrorActionPreference = 'Stop'
$layerPath = Join-Path $TilesetDir 'layer.json'
if (-not (Test-Path $layerPath)) {
    Write-Error "layer.json not found: $layerPath"
}

$doc = Get-Content $layerPath -Raw | ConvertFrom-Json
$maxZ = 0
$tilesByLevel = @{}

Get-ChildItem $TilesetDir -Directory | ForEach-Object {
    $zText = $_.Name
    if ($zText -notmatch '^\d+$') { return }
    $z = [int]$zText
    if ($z -gt $maxZ) { $maxZ = $z }
    Get-ChildItem $_.FullName -Directory | ForEach-Object {
        $xText = $_.Name
        if ($xText -notmatch '^\d+$') { return }
        $x = [int]$xText
        Get-ChildItem $_.FullName -Filter '*.terrain' | ForEach-Object {
            $yText = $_.BaseName
            if ($yText -notmatch '^\d+$') { return }
            $y = [int]$yText
            if (-not $tilesByLevel.ContainsKey($z)) {
                $tilesByLevel[$z] = @()
            }
            $tilesByLevel[$z] += [PSCustomObject]@{ x = $x; y = $y }
        }
    }
}

$available = @()
for ($z = 0; $z -le $maxZ; $z++) {
    if ($tilesByLevel.ContainsKey($z)) {
        $levelRanges = $tilesByLevel[$z] | Sort-Object x, y | ForEach-Object {
            [PSCustomObject]@{
                startX = $_.x; endX = $_.x
                startY = $_.y; endY = $_.y
            }
        }
        $available += ,@($levelRanges)
    } else {
        $available += ,@()
    }
}

$doc.available = $available
$doc | ConvertTo-Json -Depth 6 | Set-Content $layerPath -Encoding UTF8
Write-Host "Updated $layerPath (levels 0..$maxZ, from disk scan)"
