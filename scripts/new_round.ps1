param(
    [Parameter(Mandatory = $true)]
    [string]$Name
)

$ErrorActionPreference = "Stop"

$repoDir = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$stamp = Get-Date -Format "yyyyMMdd_HHmm"
$safeName = $Name -replace '[\\/:*?"<>|]', '_'
$roundDir = Join-Path $repoDir ("rounds\{0}_{1}" -f $stamp, $safeName)

New-Item -ItemType Directory -Path $roundDir -Force | Out-Null

$templateDir = Join-Path $repoDir "templates"

function Copy-TemplateByPrefix {
    param(
        [string]$Prefix,
        [string]$DestinationName
    )

    $template = Get-ChildItem -LiteralPath $templateDir -Filter "$Prefix*.md" |
        Select-Object -First 1

    if (-not $template) {
        throw "Template not found for prefix: $Prefix"
    }

    Copy-Item -LiteralPath $template.FullName -Destination (Join-Path $roundDir $DestinationName) -Force
}

Copy-TemplateByPrefix -Prefix "00_" -DestinationName "00_issue.md"
Copy-TemplateByPrefix -Prefix "04_" -DestinationName "04_prompt_for_chatgpt.md"
Copy-TemplateByPrefix -Prefix "05_" -DestinationName "05_chatgpt_analysis.md"
Copy-TemplateByPrefix -Prefix "06_" -DestinationName "06_task_for_codex.md"

New-Item -ItemType File -Path (Join-Path $roundDir "01_raw_uart.txt") -Force | Out-Null
New-Item -ItemType File -Path (Join-Path $roundDir "07_codex_result.md") -Force | Out-Null

Write-Host $roundDir
