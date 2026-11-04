"""Fast firmware preparation for the existing debug entry; no SDK/UART owner.
Enumeration is not Pose evidence. Failed logical reset experiments are recorded
in docs/T265_USB_FAST_START_20261104.md, not repeated as a startup ritual.
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
# Topology evidence only; fast startup does not issue hub power commands.
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
    """Prepare the sole SDK owner's device without an unconditional power cycle.

    Runtime PID is not proof of a healthy Pose stream. The caller still starts
    its existing SDK owner and verifies real Pose/SDK results. On this RDK, all
    tried logical resets failed to recover the cold-boot fault; do not repeat
    those resets on every launch or advertise them as a successful recovery.
    """
    log_path=Path(log_path)
    result=dict(method='existing_boot_tool_no_forced_cycle',requested=True,
        result_scope='firmware/runtime enumeration only; caller must verify actual SDK Pose',
        command_completed=False,vbus_power_off_verified=False,
        reboot_fault_recovery_verified=False,sdk_pose_verified=False,
        forced_power_cycle=False,fixed_wait_seconds=0,commands=[],before=usb_inventory())
    boot=Path('/proc/sys/kernel/random/boot_id')
    if boot.exists():result['host_boot_id']=boot.read_text().strip()
    result['started_monotonic_ns']=time.monotonic_ns()

    def run(args,timeout):
        entry=dict(argv=args,returncode=None);result['commands'].append(entry)
        try:completed=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
        except subprocess.TimeoutExpired:
            entry['error']='command_timeout';raise
        entry.update(returncode=completed.returncode,stdout=completed.stdout,stderr=completed.stderr)
        return completed

    try:
        plan=recovery_plan(result['before']);result['target']=plan
        camera=plan['camera'];prefix=[] if os.geteuid()==0 else ['sudo','-n']
        node=f"/dev/bus/usb/{int(camera['busnum']):03d}/{int(camera['devnum']):03d}"
        fuser=shutil.which('fuser')
        if fuser is None:raise RuntimeError('缺少fuser，不能核对T265设备占用')
        occupied=run(prefix+[fuser,node],5)
        if occupied.returncode==0:raise RuntimeError('T265正被其它进程使用；请关闭原SDK/地图，未复位设备')
        if occupied.returncode!=1:raise RuntimeError('无法核对T265设备占用')
        if (camera['idVendor'],camera['idProduct'])==('8087','0b37'):
            result.update(action='runtime_pid_skip_reset',runtime_enumerated=True)
            print('[T265 START] runtime PID present; skip reset, verify Pose in the existing SDK owner',flush=True)
            return result
        loader=Path(__file__).resolve().parents[2]/'t265_trajectory/build/t265_boot'
        if not os.access(loader,os.X_OK):raise RuntimeError('缺少现有t265_boot固件加载工具')
        print('[T265 START] boot PID; upload now, no fixed power-cycle delay',flush=True)
        completed=run([str(loader),'--timeout-ms','1000','--chunk-kib','256'],15)
        result.update(action='firmware_upload',command_completed=completed.returncode==0,
                      after=usb_inventory())
        result['runtime_enumerated']=any((d['idVendor'],d['idProduct'])==('8087','0b37') for d in result['after'])
        if completed.returncode!=0 or not result['runtime_enumerated']:
            detail=(completed.stderr or completed.stdout).strip()[-600:]
            raise RuntimeError('T265固件启动失败；未执行无效的重复断电循环。'+detail)
        print('[T265 START] firmware uploaded; verify real Pose in the existing SDK owner',flush=True)
        return result
    except (OSError,subprocess.TimeoutExpired,RuntimeError) as error:
        result['error']=str(error)
        raise RuntimeError('T265启动准备：'+str(error)) from error
    finally:
        result['finished_monotonic_ns']=time.monotonic_ns()
        result['elapsed_s']=(result['finished_monotonic_ns']-result['started_monotonic_ns'])/1e9
        log_path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
