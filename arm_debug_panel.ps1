param([switch]$Check)
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$settingsPath = Join-Path $root 'arm_debug_panel.local.json'
$controller = '.\.venv\Scripts\python.exe -X utf8 .\k210_camera\grasp_controller.py --config .\k210_camera\controller_pc_fixed_placement_trial.json'
$defaults = [ordered]@{
    'SSH 隧道' = 'ssh -n -o BatchMode=yes -o StrictHostKeyChecking=yes -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -L 127.0.0.1:18000:127.0.0.1:8000 pi@192.168.1.107'
    '启动服务' = 'ssh -n -o BatchMode=yes -o StrictHostKeyChecking=yes pi@192.168.1.107 "cd /home/pi/competition/mechanical-arm && if pgrep -u pi -fx ''[.]venv/bin/python -B -m arm_service --config config.serial.integration.json'' >/dev/null; then exit 0; fi; if curl -fsS --max-time 2 http://127.0.0.1:8000/v1/arm/status >/dev/null; then exit 0; fi; exec .venv/bin/python -B -m arm_service --config config.serial.integration.json"'
    '一键测试' = "$controller --once debug_{id} --execute {args}"
    '路径预览' = "$controller --preview"
    '空爪准备' = "$controller --prepare-empty --confirm-empty --execute"
    '单独拍照' = '.\.venv\Scripts\python.exe -X utf8 .\k210_camera\receive_photo.py --capture --baudrate 230400 --verify-jpeg'
    '查询状态' = 'curl.exe -sS --fail-with-body --max-time 5 http://127.0.0.1:18000/v1/arm/status'
    '读取姿态' = 'curl.exe -sS --fail-with-body --max-time 5 "http://127.0.0.1:18000/v1/arm/pose?refresh=true"'
    '松开夹爪' = 'curl.exe -sS --fail-with-body --max-time 10 -H "Content-Type: application/json" -d "{\"request_id\":\"release_{id}\",\"targets\":[{\"id\":1,\"position\":385}],\"duration_ms\":20}" http://127.0.0.1:18000/v1/arm/move'
    '一键复原' = 'ssh -n -o BatchMode=yes -o StrictHostKeyChecking=yes pi@192.168.1.107 "cd /home/pi/competition/mechanical-arm || exit 1; pkill -TERM -u pi -fx ''[.]venv/bin/python -B -m arm_service --config config.serial.integration.json''; for i in 1 2 3 4 5; do pgrep -u pi -fx ''[.]venv/bin/python -B -m arm_service --config config.serial.integration.json'' >/dev/null || break; sleep 1; done; if pgrep -u pi -fx ''[.]venv/bin/python -B -m arm_service --config config.serial.integration.json'' >/dev/null; then echo old-service-still-running; exit 1; fi; if curl -fsS --max-time 2 http://127.0.0.1:8000/v1/arm/status >/dev/null; then echo arm-service-still-listening; exit 1; fi; exec .venv/bin/python -B -m arm_service --config config.serial.integration.json"'
}
function Read-Commands($json) {
    $result = [ordered]@{}
    foreach ($key in $defaults.Keys) { $result[$key] = $defaults[$key] }
    foreach ($property in (ConvertFrom-Json -InputObject $json).PSObject.Properties) {
        if ($property.Value -isnot [string] -or !$property.Value.Trim()) {
            throw "命令必须是非空文本：$($property.Name)"
        }
        $result[$property.Name] = $property.Value
    }
    return $result
}
$script:commands = $defaults
if (Test-Path -LiteralPath $settingsPath) {
    $script:commands = Read-Commands (Get-Content -Raw -Encoding UTF8 -LiteralPath $settingsPath)
}

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[Windows.Forms.Application]::EnableVisualStyles()
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class PanelDesktop {
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr window, out uint process);
}
'@
[PanelDesktop]::SetProcessDPIAware() | Out-Null
$bg = [Drawing.Color]::FromArgb(22,26,35)
$panel = [Drawing.Color]::FromArgb(33,42,57)
$fg = [Drawing.Color]::FromArgb(227,235,246)
$form = New-Object Windows.Forms.Form
$form.Text = '药盒抓取 · 调试台'
$form.Size = New-Object Drawing.Size(1620,1110)
$form.AutoScaleMode = 'Dpi'
$form.StartPosition = 'CenterScreen'
$form.MinimumSize = New-Object Drawing.Size(900,620)
$form.BackColor = $bg
$form.ForeColor = $fg
$form.Font = New-Object Drawing.Font('Microsoft YaHei UI',10)
$layout = New-Object Windows.Forms.TableLayoutPanel
$layout.Dock = 'Fill'
$layout.Padding = New-Object Windows.Forms.Padding(24)
$layout.ColumnCount = 1
$layout.RowCount = 5
foreach ($height in 48,106,46,50) {
    $null = $layout.RowStyles.Add((New-Object Windows.Forms.RowStyle('Absolute',$height)))
}
$null = $layout.RowStyles.Add((New-Object Windows.Forms.RowStyle('Percent',100)))
$form.Controls.Add($layout)
$title = New-Object Windows.Forms.Label
$title.Text = '药盒抓取调试台'
$title.Dock = 'Fill'
$title.Font = New-Object Drawing.Font('Microsoft YaHei UI',16)
$layout.Controls.Add($title,0,0)
$buttons = New-Object Windows.Forms.FlowLayoutPanel
$buttons.Dock = 'Fill'
$buttons.AutoScroll = $true
$buttons.Padding = New-Object Windows.Forms.Padding(0,4,0,4)
$layout.Controls.Add($buttons,0,1)
$arguments = New-Object Windows.Forms.TextBox
$arguments.Dock = 'Fill'
$arguments.BackColor = $panel
$arguments.ForeColor = $fg
$arguments.AccessibleName = '一键测试附加参数'
$arguments.Tag = '仅追加到一键测试，例如 --confirm-empty'
$arguments.Text = $arguments.Tag
$arguments.Add_Enter({ if ($arguments.Text -eq $arguments.Tag) { $arguments.Clear() } })
$layout.Controls.Add($arguments,0,2)
$status = New-Object Windows.Forms.Label
$status.Text = '就绪 · 初始化需要已配置的 SSH 密钥'
$status.Dock = 'Fill'
$outputBar = New-Object Windows.Forms.FlowLayoutPanel
$outputBar.Dock = 'Fill'
$outputBar.Padding = New-Object Windows.Forms.Padding(0,6,0,4)
$status.AutoSize = $true
$status.Dock = 'None'
$history = New-Object Windows.Forms.ComboBox
$history.DropDownStyle = 'DropDownList'
$history.Width = 285
$history.Margin = New-Object Windows.Forms.Padding(0,4,12,4)
$history.AccessibleName = '运行记录'
$copy = New-Object Windows.Forms.Button
$copy.Text = '复制本次输出'
$copy.AutoSize = $true
$copy.Margin = New-Object Windows.Forms.Padding(0,2,14,4)
$status.Margin = New-Object Windows.Forms.Padding(0,7,0,0)
$outputBar.Controls.AddRange(@($history,$copy,$status))
$layout.Controls.Add($outputBar,0,3)
$log = New-Object Windows.Forms.RichTextBox
$log.Dock = 'Fill'
$log.ReadOnly = $true
$log.BackColor = $panel
$log.ForeColor = $fg
$log.Font = New-Object Drawing.Font('Consolas',10)
$layout.Controls.Add($log,0,4)
$jobs = New-Object System.Collections.ArrayList
$runs = New-Object System.Collections.ArrayList
function Show-Run {
    if ($history.SelectedIndex -lt 0) { return }
    $run = $runs[$history.SelectedIndex]
    $log.Text = "$($run.Label)`r`n> $($run.Command)`r`n`r`n$($run.Output)"
    $log.SelectionStart = $log.TextLength
    $log.ScrollToCaret()
}
$history.Add_SelectedIndexChanged({ Show-Run })
$copy.Add_Click({
    if ($history.SelectedIndex -ge 0) {
        $output = $runs[$history.SelectedIndex].Output
        if ($output) { [Windows.Forms.Clipboard]::SetText($output); $status.Text = '已复制所选运行的输出' }
    }
})
function New-Run($name,$command) {
    $run = @{ Label="$(Get-Date -Format 'HH:mm:ss')  $name"; Command=$command; Output='' }
    $null = $runs.Add($run)
    $null = $history.Items.Add($run.Label)
    $history.SelectedIndex = $history.Items.Count-1
    return $run
}
function Write-Log($line) {
    $log.AppendText("$line`r`n")
    if ($log.TextLength -gt 180000) {
        $log.Select(0,$log.TextLength-150000)
        $log.SelectedText = ''
    }
    $log.SelectionStart = $log.TextLength
    $log.ScrollToCaret()
}
function Start-Command($name,$persistent=$false) {
    $extra = if ($name -eq '一键测试' -and $arguments.Text -ne $arguments.Tag) { $arguments.Text.Trim() } else { '' }
    if ($extra -match '[&|<>^%!`r`n]') { throw '附加参数只接受命令行参数。完整指令请在设置中编辑。' }
    $id = [Guid]::NewGuid().ToString('N')
    $command = $script:commands[$name].Replace('{id}',$id).Replace('{args}',$extra)
    $prefix = Join-Path ([IO.Path]::GetTempPath()) "grasp-panel-$id"
    $stdout = "$prefix.out"
    $stderr = "$prefix.err"
    $process = Start-Process -FilePath $env:ComSpec -ArgumentList "/d /s /c `"chcp 65001>nul & $command`"" -WorkingDirectory $root -WindowStyle Hidden -PassThru -RedirectStandardOutput $stdout -RedirectStandardError $stderr
    $run = New-Run $name $command
    $null = $jobs.Add(@{ Name=$name; Process=$process; Out=$stdout; Err=$stderr; Persistent=$persistent; OutLength=0; ErrLength=0; Run=$run })
    $status.Text = "运行中 · $name"
}
function Test-ArmReady {
    $reply = Invoke-RestMethod -Uri 'http://127.0.0.1:18000/v1/arm/status' -TimeoutSec 1
    if (!$reply.ok -or $reply.data.connection -ne 'connected') { throw '机械臂服务尚未连接舵机。' }
    return $reply.data
}
$script:readyDeadline = $null
$script:readyAfter = [DateTime]::MinValue
function Invoke-Action($name) {
    if (@($jobs | Where-Object { !$_.Persistent -and !$_.Process.HasExited }).Count) {
        throw '请等当前命令结束。'
    }
    switch ($name) {
        '一键初始化' {
            if (!@($jobs | Where-Object { $_.Name -eq 'SSH 隧道' -and !$_.Process.HasExited }).Count) {
                try { $null = Test-ArmReady } catch { Start-Command 'SSH 隧道' $true }
            }
            if (!@($jobs | Where-Object { $_.Name -in @('启动服务','一键复原') -and !$_.Process.HasExited }).Count) {
                Start-Command '启动服务' $true
            }
            $script:readyDeadline = [DateTime]::Now.AddSeconds(25)
        }
        '一键复原' {
            if ($script:readyDeadline) { throw '请等机械臂连接检查完成。' }
            $arm = Test-ArmReady
            if ($null -ne $arm.active_task_id -or $arm.upload_in_progress) { throw '机械臂任务仍在运行。' }
            if ([Windows.Forms.MessageBox]::Show('将重启树莓派机械臂服务。请确认夹爪已空、运动范围已清空。','确认复原','YesNo') -ne 'Yes') { return }
            Start-Command $name $true
            $script:readyAfter = [DateTime]::Now.AddSeconds(7)
            $script:readyDeadline = [DateTime]::Now.AddSeconds(28)
        }
        default {
            if ($name -eq '空爪准备' -and [Windows.Forms.MessageBox]::Show('请确认夹爪内没有物体。','确认空爪','YesNo') -ne 'Yes') { return }
            if ($name -in @('一键测试','松开夹爪','空爪准备')) {
                if ($script:readyDeadline) { throw '请等机械臂连接检查完成。' }
                $arm = Test-ArmReady
                if ($null -ne $arm.active_task_id -or $arm.upload_in_progress) { throw '机械臂任务仍在运行。' }
            }
            Start-Command $name
        }
    }
}
function Edit-Settings {
    $editor = New-Object Windows.Forms.Form
    $editor.Text = '指令设置 · 新增 JSON 键即可添加按钮'
    $editor.Size = New-Object Drawing.Size(850,560)
    $editor.BackColor = $bg
    $inputBox = New-Object Windows.Forms.TextBox
    $inputBox.Multiline = $true
    $inputBox.ScrollBars = 'Both'
    $inputBox.WordWrap = $false
    $inputBox.Dock = 'Fill'
    $inputBox.BackColor = $panel
    $inputBox.ForeColor = $fg
    $inputBox.Text = $script:commands | ConvertTo-Json
    $editor.Controls.Add($inputBox)
    $save = New-Object Windows.Forms.Button
    $save.Text = '保存指令'
    $save.Dock = 'Bottom'
    $save.Add_Click({
        try {
            $updated = Read-Commands $inputBox.Text
            $updated | ConvertTo-Json | Set-Content -Encoding UTF8 -LiteralPath $settingsPath
            $script:commands = $updated
            $editor.Close()
        } catch { [Windows.Forms.MessageBox]::Show($_.Exception.Message) | Out-Null }
    })
    $editor.Controls.Add($save)
    $null = $editor.ShowDialog($form)
}
function Add-Buttons {
    $buttons.Controls.Clear()
    $names = @('一键初始化','一键测试','路径预览','松开夹爪','单独拍照','一键复原','空爪准备','查询状态','读取姿态','VNC 终端指令','指令设置')
    $names += @($script:commands.Keys | Where-Object { $_ -notin $defaults.Keys })
    foreach ($name in $names) {
        $button = New-Object Windows.Forms.Button
        $button.Text = $name
        $button.AutoSize = $true
        $button.MinimumSize = New-Object Drawing.Size(108,36)
        $button.Margin = New-Object Windows.Forms.Padding(0,4,10,6)
        $button.FlatStyle = 'Flat'
        $button.BackColor = if ($name -eq '一键测试') { [Drawing.Color]::FromArgb(35,91,155) } else { $panel }
        $button.ForeColor = $fg
        $button.Add_Click({
            param($sender,$eventArgs)
            try {
                if ($sender.Text -eq '指令设置') { Edit-Settings; Add-Buttons }
                elseif ($sender.Text -eq 'VNC 终端指令') {
                    $remote = 'cd /home/pi/competition/mechanical-arm && .venv/bin/python -B -m arm_service --config config.serial.integration.json'
                    [Windows.Forms.Clipboard]::SetText($remote)
                    if ([Windows.Forms.MessageBox]::Show('指令已复制。选择“是”后，请在 5 秒内切到 TigerVNC 并点进树莓派终端。面板会尝试粘贴，按回车前请核对。复原时先按现场流程停止旧服务。','TigerVNC','YesNo') -eq 'Yes') {
                        $vncTimer = New-Object Windows.Forms.Timer
                        $vncTimer.Interval = 5000
                        $vncTimer.Add_Tick({
                            param($source,$eventArgs)
                            $source.Stop()
                            $source.Dispose()
                            [uint32]$foregroundId = 0
                            [PanelDesktop]::GetWindowThreadProcessId([PanelDesktop]::GetForegroundWindow(),[ref]$foregroundId) | Out-Null
                            $viewer = Get-Process -Id $foregroundId -ErrorAction SilentlyContinue
                            if ($viewer -and $viewer.ProcessName -match '^vncviewer') {
                                [Windows.Forms.SendKeys]::SendWait('^+v')
                            } else {
                                $status.Text = '未检测到前台 TigerVNC · 命令仍在剪贴板'
                            }
                        })
                        $vncTimer.Start()
                    }
                }
                else { Invoke-Action $sender.Text }
            } catch { Write-Log $_.Exception.Message; $status.Text = '操作未完成' }
        })
        $buttons.Controls.Add($button)
    }
}
Add-Buttons
if ($Check) {
    Read-Commands ($defaults | ConvertTo-Json) | Out-Null
    $form.Dispose()
    Write-Output 'WinForms and command settings OK'
    return
}
$timer = New-Object Windows.Forms.Timer
$timer.Interval = 750
$timer.Add_Tick({
    foreach ($job in @($jobs)) {
        foreach ($stream in @('Out','Err')) {
            $content = [IO.File]::ReadAllText($job[$stream],[Text.Encoding]::UTF8)
            $key = $stream+'Length'
            if ($content.Length -gt $job[$key]) {
                $chunk = $content.Substring($job[$key])
                if ($stream -eq 'Err') { $chunk = "[stderr]`r`n$chunk" }
                $job.Run.Output += $chunk
                $job[$key] = $content.Length
                if ($history.SelectedIndex -ge 0 -and [object]::ReferenceEquals($runs[$history.SelectedIndex],$job.Run)) { Show-Run }
            }
        }
        if ($job.Process.HasExited) {
            $status.Text = "$($job.Name) · 退出码 $($job.Process.ExitCode)"
            Write-Log $status.Text
            $jobs.Remove($job)
            Remove-Item -LiteralPath $job.Out,$job.Err -ErrorAction SilentlyContinue
        }
    }
    if ($script:readyDeadline -and [DateTime]::Now -ge $script:readyAfter) {
        try {
            $arm = Test-ArmReady
            $status.Text = "机械臂已连接 · 活动任务 $($arm.active_task_id)"
            $script:readyDeadline = $null
        } catch {
            if ([DateTime]::Now -gt $script:readyDeadline) {
                $status.Text = '连接未就绪 · 查看 SSH 和服务日志'
                $script:readyDeadline = $null
            }
        }
    }
})
$form.Add_FormClosing({
    param($sender,$eventArgs)
    if (@($jobs | Where-Object { !$_.Persistent -and !$_.Process.HasExited }).Count) {
        $eventArgs.Cancel = $true
        Write-Log '请等当前命令结束后再关闭。'
        return
    }
    $timer.Stop()
    foreach ($job in @($jobs)) {
        if (!$job.Process.HasExited) { & taskkill.exe /PID $job.Process.Id /T /F 2>$null | Out-Null }
        Remove-Item -LiteralPath $job.Out,$job.Err -ErrorAction SilentlyContinue
    }
})
$timer.Start()
$null = $form.ShowDialog()
