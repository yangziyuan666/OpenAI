param(
    [Parameter(Mandatory = $true)]
    [string]$InputPath,

    [int]$MaxLinesPerChunk = 1200
)

$ErrorActionPreference = "Stop"

$inputItem = Get-Item -LiteralPath $InputPath
$roundDir = $inputItem.DirectoryName
$cleanPath = Join-Path $roundDir "02_obs_clean.txt"
$csvPath = Join-Path $roundDir "03_obs.csv"
$chunkDir = Join-Path $roundDir "chunks"

New-Item -ItemType Directory -Path $chunkDir -Force | Out-Null

$rawLines = Get-Content -LiteralPath $InputPath -Encoding UTF8

$obsLines = New-Object System.Collections.Generic.List[string]
$rows = New-Object System.Collections.Generic.List[object]

foreach ($line in $rawLines) {
    if ($line -notmatch "\[OBS\]" -and $line -notmatch "\[R[0-9]\]") {
        continue
    }

    $obsLines.Add($line)

    $time = ""
    if ($line -match "\[(\d{2}:\d{2}:\d{2}\.\d{3})\]") {
        $time = $Matches[1]
    }

    $obj = [ordered]@{
        time = $time
        tag = ""
        Q = ""
        VR = ""
        RR = ""
        CS = ""
        RG = ""
        RCNT = ""
        LL = ""
        RL = ""
        JL = ""
        JR = ""
        WR = ""
        LC = ""
        RC = ""
        EF = ""
        A = ""
        raw = $line
    }

    if ($line -match "\[(OBS|R[0-9])\]") { $obj.tag = $Matches[1] }
    if ($line -match "Q=\s*(-?\d+)") { $obj.Q = $Matches[1] }
    if ($line -match "VR=\s*(-?\d+)") { $obj.VR = $Matches[1] }
    if ($line -match "RR=\s*(-?\d+)") { $obj.RR = $Matches[1] }
    if ($line -match "CS=\s*(-?\d+)") { $obj.CS = $Matches[1] }
    if ($line -match "RG=\s*([0-9]+/[0-9]+)") { $obj.RG = $Matches[1] }
    if ($line -match "(RCNT|RC)=\s*(-?\d+)") { $obj.RCNT = $Matches[2] }
    if ($line -match "LL=\s*(-?\d+)") { $obj.LL = $Matches[1] }
    if ($line -match "RL=\s*(-?\d+)") { $obj.RL = $Matches[1] }
    if ($line -match "JL=\s*(-?\d+)") { $obj.JL = $Matches[1] }
    if ($line -match "JR=\s*(-?\d+)") { $obj.JR = $Matches[1] }
    if ($line -match "WR=\s*(-?\d+)") { $obj.WR = $Matches[1] }
    if ($line -match "LC=\s*(-?\d+)") { $obj.LC = $Matches[1] }
    if ($line -match "(?<![A-Z])RC=\s*(-?\d+)") { $obj.RC = $Matches[1] }
    if ($line -match "EF=\s*(-?\d+)") { $obj.EF = $Matches[1] }
    if ($line -match "A=\s*(-?\d+)") { $obj.A = $Matches[1] }

    $rows.Add([pscustomobject]$obj)
}

$obsLines | Set-Content -LiteralPath $cleanPath -Encoding UTF8
$rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding UTF8

$chunkIndex = 1
for ($i = 0; $i -lt $obsLines.Count; $i += $MaxLinesPerChunk) {
    $end = [Math]::Min($i + $MaxLinesPerChunk - 1, $obsLines.Count - 1)
    $chunkPath = Join-Path $chunkDir ("obs_chunk_{0:D3}.txt" -f $chunkIndex)
    $obsLines[$i..$end] | Set-Content -LiteralPath $chunkPath -Encoding UTF8
    $chunkIndex++
}

Write-Host "Input:  $InputPath"
Write-Host "Clean:  $cleanPath"
Write-Host "CSV:    $csvPath"
Write-Host "Chunks: $chunkDir"
Write-Host "Lines:  $($obsLines.Count)"
