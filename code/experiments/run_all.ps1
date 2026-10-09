[CmdletBinding()]
param(
    [int]$Trials = 3,
    [int]$Frames = 60,
    [int]$CoreIterations = 5000,
    [int]$NeuroIterations = 1000000,
    [int]$PayloadKiB = 150,
    [int]$SurrogateRounds = 64,
    [double]$StressDuration = 5.0
)
$ErrorActionPreference = 'Stop'
$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
python (Join-Path $scriptDirectory 'run_all.py') `
    --trials $Trials `
    --frames $Frames `
    --core-iterations $CoreIterations `
    --neuro-iterations $NeuroIterations `
    --payload-kib $PayloadKiB `
    --surrogate-rounds $SurrogateRounds `
    --stress-duration $StressDuration
exit $LASTEXITCODE
