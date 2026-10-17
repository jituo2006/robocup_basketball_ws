[CmdletBinding()]
param(
    [ValidatePattern('^[A-Za-z0-9_.@:-]+$')][string]$HostAlias = 'robocup-linux',
    [ValidateRange(1024,65535)][int]$LocalPort = 18082,
    [ValidateRange(1024,65535)][int]$RemotePort = 8082,
    [switch]$NoBrowser
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Net.Http
$previewUrl = "http://127.0.0.1:$LocalPort"
function Get-PreviewStatus([string]$Url) {
    $handler = New-Object System.Net.Http.HttpClientHandler
    $handler.UseProxy = $false
    $client = New-Object System.Net.Http.HttpClient($handler)
    $client.Timeout = [TimeSpan]::FromSeconds(2)
    try { return ($client.GetStringAsync("$Url/status").GetAwaiter().GetResult() | ConvertFrom-Json) }
    catch { return $null }
    finally { $client.Dispose(); $handler.Dispose() }
}

# Only subscribe to existing camera/detection data; never start robot motion.
$previewStatus = Get-PreviewStatus $previewUrl
if ($previewStatus.service -ne 'front_camera_view') {
    Write-Host 'Connecting to the remote front camera viewer...'
    $remoteCommand = @'
python3 - PORT_NUMBER <<'FRONT_PREVIEW_PY'
import json, pathlib, subprocess, sys, time, urllib.request
p=pathlib.Path('/home/user/robocup_basketball_ws')
port=int(sys.argv[1])
url=f'http://127.0.0.1:{port}/status'
def status():
    try:
        with urllib.request.urlopen(url,timeout=1) as r: return json.load(r)
    except OSError: return None
data=status()
if data and data.get('service')!='front_camera_view':
    raise SystemExit('Remote port belongs to another service; choose another RemotePort.')
if not data:
    out=p/'test_artifacts/front_camera_view';out.mkdir(parents=True,exist_ok=True)
    with (out/'server.log').open('ab') as log:
        proc=subprocess.Popen(['bash',str(p/'tools/view_front_camera.sh'),'--port',str(port)],cwd=p,
             stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    for _ in range(30):
        data=status()
        if data: break
        if proc.poll() is not None: raise SystemExit((out/'server.log').read_text()[-1500:])
        time.sleep(.2)
if not data or data.get('service')!='front_camera_view': raise SystemExit('Viewer is not ready.')
print('Remote viewer ready. Camera live:',data['image_live'])
FRONT_PREVIEW_PY
'@
    $remoteCommand = $remoteCommand.Replace('PORT_NUMBER', $RemotePort.ToString())
    & ssh -T -o BatchMode=yes -o ConnectTimeout=8 $HostAlias $remoteCommand
    if ($LASTEXITCODE -ne 0) { throw 'Could not start the remote viewer. Check SSH and host network.' }

    # A previous tunnel may become usable again once the remote server is restarted.
    $previewStatus = Get-PreviewStatus $previewUrl
    if ($previewStatus.service -eq 'front_camera_view') {
        Write-Host "Front camera: $previewUrl"
        if (-not $NoBrowser) { Start-Process $previewUrl }
        return
    }

    # Refuse an occupied local port rather than touching an unrelated process.
    $probe = New-Object System.Net.Sockets.TcpListener([System.Net.IPAddress]::Loopback, $LocalPort)
    try { $probe.Start() }
    catch { throw "Local port $LocalPort is in use. Run with -LocalPort 18083." }
    finally { $probe.Stop() }
    $logBase = Join-Path $env:TEMP ("front-camera-tunnel-" + [Guid]::NewGuid().ToString('N'))
    $sshArgs = @('-N','-T','-o','BatchMode=yes','-o','ExitOnForwardFailure=yes',
                 '-o','ConnectTimeout=8','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=3',
                 '-L',"127.0.0.1:${LocalPort}:127.0.0.1:${RemotePort}",$HostAlias)
    $tunnel = Start-Process -FilePath (Get-Command ssh).Source -ArgumentList $sshArgs -PassThru `
        -WindowStyle Hidden -RedirectStandardOutput "$logBase.out" -RedirectStandardError "$logBase.err"
    $deadline = [DateTime]::UtcNow.AddSeconds(12)
    do {
        Start-Sleep -Milliseconds 250
        $previewStatus = Get-PreviewStatus $previewUrl
        if ($previewStatus.service -eq 'front_camera_view') { break }
        if ($tunnel.HasExited) { throw (Get-Content -Raw "$logBase.err") }
    } while ([DateTime]::UtcNow -lt $deadline)
    if ($previewStatus.service -ne 'front_camera_view') {
        if (-not $tunnel.HasExited) { $tunnel.Kill() }
        throw "Tunnel did not become ready. See $logBase.err"
    }
    Write-Host "SSH tunnel PID: $($tunnel.Id) (Stop-Process -Id $($tunnel.Id) to close it)"
}
Write-Host "Front camera: $previewUrl"
if (-not $NoBrowser) { Start-Process $previewUrl }
