# Probe candidate IPs for the teleop web console (:9101 by default).
# Prints the first host that accepts TCP, then exits 0. No output / exit 1 if none.
param(
    [string]$Port = "9101",
    [string]$Hosts = "172.18.101.12,59.79.233.120"
)
$ErrorActionPreference = "SilentlyContinue"
$list = @()
foreach ($tok in ($Hosts -split "[,;\s]+")) {
    $h = $tok.Trim()
    if (-not $h) { continue }
    if ($h.StartsWith("192.168.1.")) { continue }
    if ($list -contains $h) { continue }
    $list += $h
}
$portNum = 9101
[void][int]::TryParse($Port, [ref]$portNum)
foreach ($h in $list) {
    $c = New-Object System.Net.Sockets.TcpClient
    try {
        $iar = $c.BeginConnect($h, $portNum, $null, $null)
        if ($iar.AsyncWaitHandle.WaitOne(400, $false)) {
            $c.EndConnect($iar)
            Write-Output $h
            exit 0
        }
    } catch {
    } finally {
        $c.Close()
    }
}
exit 1
