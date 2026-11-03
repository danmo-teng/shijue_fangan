"""Opt-in startup uhubctl operation for the existing debug entry, no SDK/UART.

Kernel enumeration and a successful request do not prove VBUS went low.
The caller must still obtain actual Pose/SDK input results in its own session.
"""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

USB_ROOT = Path('/sys/bus/usb/devices')
T265_IDS = {('03e7','2150'), ('8087','0b37')}
# This RDK's paired hub reports ganged switching. The explicit startup option
# authorizes this known shared-hub experiment, not forcing arbitrary hubs.
RDK_GANGED_IDS = {('05e3','0610'), ('05e3','0626')}


def usb_inventory(root=USB_ROOT):
    result=[]
    for p in sorted(root.iterdir()):
        if not re.fullmatch(r'(?:usb\d+|\d+-\d+(?:\.\d+)*)',p.name):continue
        try:
            row={'name':p.name}
            for key in ('idVendor','idProduct','busnum','devnum','product','serial'):
                f=p/key
                if f.exists():row[key]=f.read_text().strip()
            if 'idVendor' in row and 'idProduct' in row:result.append(row)
        except OSError:
            continue  # Enumeration can change during an explicit power cycle.
    return result


def recovery_plan(devices):
    cameras=[d for d in devices if (d['idVendor'],d['idProduct']) in T265_IDS]
    if len(cameras)!=1:
        raise RuntimeError(f'USB恢复需唯一枚举的T265/boot设备，当前{len(cameras)}个；不猜测端口')
    camera=cameras[0];name=camera['name']
    if '.' in name:
        hub_name,port=name.rsplit('.',1);location=hub_name
    else:
        bus,port=name.split('-',1);hub_name='usb'+bus;location=bus
    hub=next((d for d in devices if d['name']==hub_name),None)
    if hub is None:raise RuntimeError('未找到T265父Hub，未执行USB操作')
    force=(hub['idVendor'],hub['idProduct']) in RDK_GANGED_IDS
    return dict(camera=camera,hub=hub,location=location,port=int(port),
                force_known_rdk_ganged=force,may_affect_shared_usb_camera=force)


def recover_t265(log_path):
    """One explicit cycle before opening the sole SDK owner. Never retries."""
    log_path=Path(log_path)
    result=dict(method='uhubctl_cycle',requested=True,command_completed=False,
                result_scope='USB operation only; inspect raw Pose and wheel_sdk results in this same session',
                vbus_power_off_verified=False,reboot_fault_recovery_verified=False,
                sdk_pose_verified=False,commands=[],before=usb_inventory())
    boot=Path('/proc/sys/kernel/random/boot_id')
    if boot.exists():result['host_boot_id']=boot.read_text().strip()
    result['started_monotonic_ns']=time.monotonic_ns()
    command=None;cycle_started=False

    def run(args,timeout=25):
        entry=dict(argv=args,returncode=None);result['commands'].append(entry)
        try:completed=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
        except subprocess.TimeoutExpired:
            entry['error']='command_timeout';raise
        entry.update(returncode=completed.returncode,stdout=completed.stdout,stderr=completed.stderr)
        return completed

    try:
        plan=recovery_plan(result['before']);result['target']=plan
        executable=shutil.which('uhubctl')
        if executable is None and os.access('/usr/local/sbin/uhubctl',os.X_OK):
            executable='/usr/local/sbin/uhubctl'  # Desktop launchers may omit sbin from PATH.
        if executable is None:raise RuntimeError('未安装uhubctl；本机测试使用官方v2.6.0，不自动下载/更改权限')
        prefix=[] if os.geteuid()==0 else ['sudo','-n']
        camera=plan['camera']
        node=f"/dev/bus/usb/{int(camera['busnum']):03d}/{int(camera['devnum']):03d}"
        fuser=shutil.which('fuser')
        if fuser is None:raise RuntimeError('缺少fuser，不能确认是否已有SDK占用T265')
        occupied=run(prefix+[fuser,node],timeout=5)
        if occupied.returncode==0:raise RuntimeError('T265正被其它进程使用；先关闭原SDK/地图，未执行USB恢复')
        if occupied.returncode!=1:raise RuntimeError('无法核对T265设备占用，未执行USB恢复')
        version=run([executable,'--version'],timeout=5)
        result['uhubctl_version']=version.stdout.strip()
        # -N avoids querying strings from the potentially stuck USB device.
        command=prefix+[executable,'-N','-l',plan['location'],'-p',str(plan['port'])]
        if plan['force_known_rdk_ganged']:command+=['-f']
        hub_status=run(command);result['before_hub_status']=hub_status.stdout
        if hub_status.returncode!=0:raise RuntimeError('uhubctl无法访问所选Hub，未执行电源循环')
        print(f"[USB RECOVERY] hub={plan['location']} port={plan['port']} shared_hub={plan['may_affect_shared_usb_camera']}; one 5s cycle",flush=True)
        # Keep uhubctl's paired USB2/USB3 handling. No whole-hub -R reset and
        # no 'all ports' operation are silently added.
        cycle_started=True
        cycle=run(command+['-a','cycle','-d','5'])
        result['command_completed']=cycle.returncode==0
        if cycle.returncode!=0:raise RuntimeError('uhubctl请求失败，查看本会话t265_usb_recovery.json')
        deadline=time.monotonic()+10
        while True:
            result['after']=usb_inventory()
            result['t265_after']=[d for d in result['after'] if (d['idVendor'],d['idProduct']) in T265_IDS]
            if result['t265_after'] or time.monotonic()>=deadline:break
            time.sleep(.2)
        result['enumerated_after']=bool(result['t265_after'])
        result['device_identity_changed']=result['t265_after']!=[camera]
        if not result['enumerated_after']:raise RuntimeError('USB操作后未枚举到T265；未启动第二个SDK或自动重试')
        result['after_hub_status']=run(command).stdout
        return result
    except (OSError,subprocess.TimeoutExpired,RuntimeError,KeyboardInterrupt) as error:
        result['error']=str(error)
        if cycle_started and not result['command_completed']:
            # A failed/interrupted cycle may have left the port off. Restore
            # that same port, without repeating the cycle or widening scope.
            result['restore_on_attempted']=True
            try:result['restore_on_returncode']=run(command+['-a','on'],timeout=10).returncode
            except (OSError,subprocess.TimeoutExpired) as restore_error:
                result['restore_on_error']=str(restore_error)
        if isinstance(error,KeyboardInterrupt):raise
        raise RuntimeError('T265 USB恢复：'+str(error)) from error
    finally:
        result['finished_monotonic_ns']=time.monotonic_ns()
        log_path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
