<#!
  Assembles src/uml/uml.mmd from:
    - Every *.mmd under src/uml/frc/ and src/uml/edu/ (per-class diagrams)
    - src/uml/relationships.mmd (associations — edit when types or wiring change)

  Mermaid has no portable #include; this script is the merge step.
  Run:  .\scripts\merge-uml.ps1
  Or:   .\gradlew mergeUml
#>
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
if (-not (Test-Path (Join-Path $repoRoot 'build.gradle'))) {
    throw "Could not find project root (build.gradle) above $PSScriptRoot"
}
$umlRoot = Join-Path $repoRoot 'src\uml'
$outFile = Join-Path $umlRoot 'uml.mmd'
$relFile = Join-Path $umlRoot 'relationships.mmd'

function Get-ClassBlockLines {
    param([string[]]$Lines)
    $start = -1
    for ($i = 0; $i -lt $Lines.Count; $i++) {
        if ($Lines[$i] -match '^\s+class\s+\w+\s*\{') {
            $start = $i
            break
        }
    }
    if ($start -lt 0) { return @() }
    $block = [System.Collections.ArrayList]::new()
    for ($i = $start; $i -lt $Lines.Count; $i++) {
        [void]$block.Add($Lines[$i])
        if ($i -gt $start -and $Lines[$i] -match '^\s+\}\s*$') { break }
    }
    return ,$block.ToArray()
}

 # Collect *.mmd files from both 'frc' and 'edu' and sort them by the normalized relative path
 $paths = @()
 if (Test-Path (Join-Path $umlRoot 'frc')) { $paths += (Join-Path $umlRoot 'frc') }
 if (Test-Path (Join-Path $umlRoot 'edu')) { $paths += (Join-Path $umlRoot 'edu') }
 $classFiles = @()
 if ($paths.Count -gt 0) {
     # Build objects with a 'Rel' property normalized to forward slashes and sort by that
     $classFiles = @(Get-ChildItem -Path $paths -Recurse -Filter '*.mmd' -File -ErrorAction SilentlyContinue |
        ForEach-Object {
            $rel = $_.FullName.Substring($umlRoot.Length + 1) -replace '\\','/'
            [PSCustomObject]@{ FullName = $_.FullName; Rel = $rel }
        } |
        # Use case-sensitive sort to match Unix 'sort' behavior where uppercase sorts before lowercase
    # Sort by lowercase Rel to perform a case-insensitive, deterministic ordering that matches the shell script
    Sort-Object -Property @{Expression = { $_.Rel.ToLowerInvariant() }} |
         ForEach-Object { Get-Item $_.FullName }
     )
 }

$chunks = [System.Collections.ArrayList]::new()
foreach ($f in $classFiles) {
    $raw = Get-Content -LiteralPath $f.FullName
    $lines = foreach ($line in $raw) { $line.TrimEnd("`r") }
    $block = Get-ClassBlockLines -Lines $lines
    if ($block.Count -eq 0) {
        Write-Warning "No class block in $($f.FullName)"
        continue
    }
    $rel = $f.FullName.Substring($umlRoot.Length + 1) -replace '\\', '/'
    [void]$chunks.Add("  %% --- $rel ---")
    foreach ($line in $block) { [void]$chunks.Add($line) }
    [void]$chunks.Add('')
}

$relLines = if (Test-Path $relFile) {
    Get-Content -LiteralPath $relFile | ForEach-Object { $_.TrimEnd("`r") }
} else {
    @()
}

$sb = [System.Text.StringBuilder]::new()
[void]$sb.AppendLine('classDiagram')
[void]$sb.AppendLine('  %% AUTO-GENERATED - do not edit by hand. Run: gradlew mergeUml')
[void]$sb.AppendLine('  %% LR layout reduces vertical crossing; associations below are simplified (see class members for full deps)')
[void]$sb.AppendLine('  direction LR')
[void]$sb.AppendLine('')
foreach ($line in $chunks) {
    [void]$sb.AppendLine($line)
}
foreach ($line in $relLines) {
    [void]$sb.AppendLine($line)
}

$utf8NoBom = New-Object System.Text.UTF8Encoding $false
[System.IO.File]::WriteAllText($outFile, $sb.ToString().TrimEnd() + "`n", $utf8NoBom)
Write-Host "Wrote $outFile ($($classFiles.Count) class files + relationships)"
