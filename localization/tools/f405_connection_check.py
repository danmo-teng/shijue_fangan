"""Read-only check invoked by the EXISTING T265 debug entry's --check-chassis.
No standalone executable, T265, radar, motion, STOP, reset or stream subscription.
"""
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import json
import re
import time
import hashlib
import subprocess

from f405_test_client import ChassisTestClient, firmware_compatible

OWNER = "f405_connection_check"
FILES = {"metadata.json","result.json","session_complete.json","uart_rx.jsonl",
         "status.json","status.jsonl","chassis_identity.json","chassis_link.jsonl",
         "chassis_extended.jsonl","chassis_telemetry.jsonl","imu_telemetry.jsonl","chassis_actions.jsonl"}


def prune(root, current):
    def eligible(p):
        if p.is_symlink() or not p.is_dir() or not re.fullmatch(r"\d{8}_\d{6}_check(?:_\d+)?",p.name):return None
        try:
            m=json.loads((p/"session_complete.json").read_text());meta=json.loads((p/"metadata.json").read_text())
            if m.get("owner")!=OWNER or meta.get("owner")!=OWNER or m.get("complete") is not True:return None
            if type(m.get("session_index")) is not int:return None
            if any(f.is_symlink() or not f.is_file() or f.name not in FILES for f in p.iterdir()):return None
            return m["session_index"]
        except (OSError,ValueError):return None
    if eligible(current) is None:return []
    old=[(eligible(p),p) for p in root.iterdir() if p.resolve()!=current.resolve()]
    old=sorted(((i,p) for i,p in old if i is not None),key=lambda x:(x[0],x[1].name),reverse=True)
    removed=[]
    for _,p in old[4:]:
        if eligible(p) is None:continue
        for f in p.iterdir():f.unlink()
        p.rmdir();removed.append(p.name)
    return removed


def check_connection(options, root):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    stamp=datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S")+"_check"
    run=root/stamp;i=1
    while run.exists():run=root/f"{stamp}_{i:02d}";i+=1
    run.mkdir()
    indices=[]
    for p in root.glob("*/session_complete.json"):
        try:
            m=json.loads(p.read_text())
            if m.get("owner")==OWNER and type(m.get("session_index")) is int:indices.append(m["session_index"])
        except (OSError,ValueError):pass
    metadata=dict(owner=OWNER,created_local=datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
        uart=options.uart,baud=options.baud,identity_policy=options.chassis_profile,
        read_only=True,t265_started=False,motor_commands_sent=0,stream_subscription_sent=False)
    project=Path(__file__).resolve().parents[2]
    metadata["upper_git_commit"]=subprocess.check_output(["git","rev-parse","HEAD"],cwd=project,text=True).strip()
    metadata["upper_tracked_changes"]=subprocess.check_output(["git","diff","--name-only","HEAD"],cwd=project,text=True).splitlines()
    metadata["client_source_sha256"]={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
        for name in ("f405_test_client.py","f405_timed_protocol.py","f405_connection_check.py")}
    (run/"metadata.json").write_text(json.dumps(metadata,indent=2)+"\n")
    client=ChassisTestClient(options.uart,options.baud,run/"status.json",run/"status.jsonl",
        identity_policy=options.chassis_profile,asserted_firmware=options.actual_lower_firmware,
        identity_confirmed=(options.actual_lower_mode=="NormalDebug" and options.actual_lower_service=="MOTIONDEBUG"
                            and firmware_compatible(options.actual_lower_firmware)))
    complete=False
    try:
        client.start()
        deadline=time.monotonic()+1.
        while time.monotonic()<deadline:
            client.poll();time.sleep(.02)
        status,stats=client.status_snapshot();link=client.connection_snapshot()
        fresh=bool(status and 0<=time.monotonic_ns()-status["timestamp_monotonic_ns"]<=1_000_000_000)
        result=dict(log_directory=str(run),connection_ok=link["motion_identity_ready"] and fresh,
            chassis_link=link,status=status,statistics=stats,fresh_legacy_0x1a=fresh,
            motor_commands_sent=0,capture_time_qualified=False,sdk_time_qualified=False,
            physical_motion_verified=False)
        (run/"result.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
        complete=client.log_error is None
    finally:
        client.close()
    if complete:
        (run/"session_complete.json").write_text(json.dumps(dict(owner=OWNER,complete=True,
            session_index=max(indices,default=0)+1,accuracy_verified=False))+"\n")
        result["old_check_sessions_removed"]=prune(root,run)
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    return 0 if result["connection_ok"] else 2
