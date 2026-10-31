#!/usr/bin/env python3
"""Independent STL50B2 + primary-yaw diagnostic. No competition/T265 imports.

One ChassisTestClient owns UART4; radar has its own distinct serial port.
Motion is ONLY an explicit terminal command after a static checkpoint.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import re
import select
import signal
import subprocess
import sys
import time
import uuid
import webbrowser
from zoneinfo import ZoneInfo

import serial

from f405_test_client import ChassisTestClient, IMPLEMENTATION_COMMIT, DOCUMENT_COMMIT
from f407_motion_protocol import turn_ccw_frame
from lidar_yaw_debug.diagnostic import DiagnosticState, load_config
from lidar_yaw_debug.crop import split_samples
from lidar_yaw_debug.lidar_reader import LidarStream
from lidar_yaw_debug.lidar_timing import LidarTimingError
from lidar_yaw_debug.protocol import command_frame
from lidar_yaw_debug.viewer import Viewer

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT/"localization/config/lidar_yaw_debug.json"
DEFAULT_LOG_ROOT = ROOT/"rescue_map/runtime/history/lidar_yaw_map"
OWNER = "lidar_yaw_map"
SESSION_FILES = frozenset({"metadata.json","config.json","events.jsonl","scans.jsonl","results.jsonl",
    "lidar_rx.jsonl","lidar_tx.jsonl","f407_motion_status.json","f407_motion_status.jsonl",
    "uart_rx.jsonl","chassis_telemetry.jsonl","imu_telemetry.jsonl","chassis_actions.jsonl",
    "analysis.json","session_complete.json"})


def prune_completed_sessions(root, current):
    """After saving current, retain five complete OWNED sessions; no recursion."""
    def eligible(path):
        if path.is_symlink() or not path.is_dir() or not re.fullmatch(r"\d{8}_\d{6}_lidar_yaw(?:_\d+)?",path.name):
            return None
        try:
            marker = json.loads((path/"session_complete.json").read_text())
            metadata = json.loads((path/"metadata.json").read_text())
            if marker.get("owner")!=OWNER or marker.get("complete") is not True or metadata.get("owner")!=OWNER:
                return None
            if not isinstance(marker.get("session_index"),int): return None
            children = list(path.iterdir())
            if any(p.is_symlink() or not p.is_file() or p.name not in SESSION_FILES for p in children): return None
            return marker["session_index"]
        except (OSError,ValueError): return None
    if eligible(current) is None: return []
    others = [(eligible(p),p) for p in root.iterdir() if p.resolve()!=current.resolve()]
    others = sorted(((i,p) for i,p in others if i is not None),key=lambda x:(x[0],x[1].name),reverse=True)
    removed=[]
    for _,path in others[4:]:
        if eligible(path) is None: continue
        for child in path.iterdir(): child.unlink()
        path.rmdir();removed.append(path.name)
    return removed


HELP = """仅本地终端控制，地图只读；正角逆时针。命令：
  scan_start / scan_stop       雷达A0+A2(10Hz) / A1，不控制底盘
  stationary <说明>           人工确认车已停稳在地面；进入静止阶段
  fixture <摆放说明>           用已确认的1号135°与当前累计yaw建立固定参考
  yaw <场地角deg> <标线说明>   按独立地面方向重新建立参考，不修改gyro
  seed <中心X m> <中心Y m> <说明>  给出附近物理分支（不是新增测量）
  review_static <核对说明>     连续静止双轴几何+人工核对，非精度资格
  phase_rotate                先静止核对后才能进入；旋转中不修正位置
  turn <CCW角deg> [deg/s]      人工触发旧TURN适配，默认30deg/s
  stop                        共用底盘客户端的STOP，等匹配回执
  mark <实测或现象记录>        卷尺/地面角度、动作起止、原地中心移动等
  quit                        退出；若本会话发过动作则请求STOP
完整回执不是物理角度真值。转动结束并实物停稳后，再stationary记录雷达位置。
"""


class LidarYawApp:
    def __init__(self, options):
        self.options = options
        self.config = load_config(options.config)
        self.state = DiagnosticState(self.config)
        self.client = None
        self.lidar = None
        self.viewer = None
        self.run_dir = None
        self.inbox = queue.Queue(maxsize=2048)
        self.overflow = False
        self.sent_motion = False
        self.stop_requested = False
        self.log_failed = False
        self.last_raw_wheel = None
        self.latest_result = None
        self.last_render_ns = 0
        self.scans_seen = 0
        self.static_fixes = 0
        self.rotation_scans = 0
        self.unsubscribe = None

    def log(self, name, data):
        try:
            with (self.run_dir/name).open("a",encoding="utf-8") as stream:
                stream.write(json.dumps(data,ensure_ascii=False,allow_nan=False)+"\n")
        except (OSError,ValueError):
            self.log_failed = True
            raise

    def event(self, event, **data):
        self.log("events.jsonl",dict(event=event,timestamp_monotonic_ns=time.monotonic_ns(),**data))

    def telemetry(self, event):
        # Subscriber never performs serial I/O or expensive geometry.
        if event["kind"] not in ("imu","raw_wheel_counts"): return
        try: self.inbox.put_nowait(event)
        except queue.Full: self.overflow = True

    def drain_chassis(self):
        if self.client.link_failed and self.state.yaw.offset_rad is not None:
            self.state.yaw.invalidate("CHASSIS_LINK_FAILED_REFERENCE_RECHECK")
            self.state.static_review=None
            self.event("field_reference_invalidated",reason=self.state.yaw.reason)
        if self.overflow:
            self.overflow = False
            while not self.inbox.empty(): self.inbox.get_nowait()
            self.state.yaw.invalidate("SUBSCRIPTION_OVERFLOW")
            self.state.static_review = None
            self.event("chassis_subscription_overflow_no_replay")
        revision = self.state.yaw.revision
        while not self.inbox.empty():
            event = self.inbox.get_nowait()
            if event["kind"]=="imu": self.state.yaw.ingest(event["sample"])
            else:
                self.last_raw_wheel = event["sample"]
                self.state.note_wheels(event["sample"])
        if self.state.yaw.revision!=revision:
            self.state.static_review = None
            self.state.static_history.clear()
            self.event("field_reference_invalidated",reason=self.state.yaw.reason)
        for result in self.client.drain_action_results():
            self.event("chassis_action_result",result=result,physical_stopped_verified=False)
            print("底盘回执：",result["outcome"],result.get("status",{}).get("progress"),flush=True)

    def new_reader(self, reason):
        m = self.config["installation"]
        self.lidar_epoch = uuid.uuid4().hex
        # ALL qualification defaults remain false. 1ms tick is a diagnostic
        # hypothesis, not a calibrated capture clock. Preserve display guards.
        reader_config = dict(bearing_sign=1,bearing_zero_deg=0.,physical_direction=1,
            center_raw_deg=m["center_raw_deg"],fov_deg=m["display_fov_deg"],
            packet_timing=self.config["packet_timing"])
        self.reader = LidarStream(reader_config,sensor_epoch=self.lidar_epoch)
        self.event("lidar_host_segment",reason=reason,host_segment_id=self.lidar_epoch,
                   device_boot_epoch=None,capture_time_qualified=False)

    def start(self):
        o = self.options
        lidar_port = o.lidar or self.config["lidar"]["port"]
        if os.path.realpath(lidar_port)==os.path.realpath(o.chassis):
            raise ValueError("radar and chassis must not share a physical UART")
        o.output_root.mkdir(parents=True,exist_ok=True)
        name = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S")+"_lidar_yaw"
        self.run_dir = o.output_root/name
        i=1
        while self.run_dir.exists():
            self.run_dir=o.output_root/f"{name}_{i:02d}";i+=1
        self.run_dir.mkdir()
        indices=[]
        for p in o.output_root.glob("*/session_complete.json"):
            try:
                m=json.loads(p.read_text())
                if m.get("owner")==OWNER and isinstance(m.get("session_index"),int): indices.append(m["session_index"])
            except (OSError,ValueError): pass
        self.session_index=max(indices,default=0)+1
        (self.run_dir/"config.json").write_text(json.dumps(self.config,ensure_ascii=False,indent=2)+"\n")
        self.client = ChassisTestClient(o.chassis,115200,self.run_dir/"f407_motion_status.json",
            self.run_dir/"f407_motion_status.jsonl",identity_confirmed=(
                o.actual_lower_mode=="NormalDebug" and o.actual_lower_service=="MOTIONDEBUG" and
                (o.actual_lower_firmware=="unconfirmed" or
                 (len(o.actual_lower_firmware)>=7 and IMPLEMENTATION_COMMIT.startswith(o.actual_lower_firmware)))))
        self.unsubscribe=self.client.telemetry(self.telemetry)
        metadata=dict(owner=OWNER,mode="independent_static_rotation_diagnostic",session_id=uuid.uuid4().hex,
            timestamp_monotonic_ns=time.monotonic_ns(),
            created_local=datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),upper_commit=subprocess.check_output(
                ["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),
            expected_lower_firmware=IMPLEMENTATION_COMMIT,actual_lower_firmware=o.actual_lower_firmware,
            actual_lower_mode=o.actual_lower_mode,actual_lower_service=o.actual_lower_service,
            identity_attested_by_uart=False,contract_document_commit=DOCUMENT_COMMIT,
            upper_tracked_changes=subprocess.check_output(["git","diff","--name-only","HEAD"],cwd=ROOT,text=True).splitlines(),
            chassis_client="f405_test_client.ChassisTestClient",chassis_uart=o.chassis,
            lidar_port=lidar_port,lidar_baud=o.baud or self.config["lidar"]["baud"],
            map_conversion=self.config["map"],startup_fixture=self.config["startup_fixture"],
            qualification=self.config["qualification"],configuration_path=str(o.config),
            no_competition_output=True,no_t265_process=True,control_pose_published=False)
        (self.run_dir/"metadata.json").write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+"\n")
        self.client.start()
        self.lidar=serial.Serial(lidar_port,metadata["lidar_baud"],timeout=.02,write_timeout=.2,exclusive=True)
        self.new_reader("serial_connection_open")
        self.viewer=Viewer(o.http_port)
        self.event("session_started")
        print(f"日志：{self.run_dir}\n只读地图：http://127.0.0.1:{self.viewer.server.server_port}\n{HELP}",flush=True)
        if o.open_browser: webbrowser.open(f"http://127.0.0.1:{self.viewer.server.server_port}")

    def command(self, text):
        words=text.strip().split()
        if not words: return
        now=time.monotonic_ns();name=words[0]
        self.event("operator_command",text=text)
        if name in ("quit","q"):
            self.stop_requested=True
        elif name=="help": print(HELP,flush=True)
        elif name in ("scan_start","scan_stop"):
            packets = [(0xA0,0),(0xA2,3600)] if name=="scan_start" else [(0xA1,0)]
            for mode,value in packets:
                data=command_frame(mode,value)
                self.log("lidar_tx.jsonl",dict(timestamp_monotonic_ns=now,data_hex=data.hex(),command=name))
                if self.lidar.write(data)!=len(data): raise RuntimeError("partial lidar command write")
        elif name=="stationary":
            if len(words)<2: raise ValueError("describe physical settled observation")
            if self.client.pending is not None: raise ValueError("wait for action/STOP receipt, then confirm physical settling")
            self.state.operator_stationary=True;self.state.phase="static";self.state.static_history.clear()
            self.event("manual_stationary_declaration",description=" ".join(words[1:]),not_ground_truth_measurement=True)
        elif name in ("yaw","fixture"):
            if not self.state.operator_stationary or self.client.pending is not None or not self.state.wheel_quiet(now):
                raise ValueError("first stationary <physical settled description>")
            if name=="fixture":
                angle=self.config["startup_fixture"]["field_heading_deg"]
                desc="operator confirms original startup placement: "+" ".join(words[1:])
                if len(words)<2: raise ValueError("describe confirmed fixture placement for this session")
            else:
                if len(words)<3: raise ValueError("yaw <field deg> <physical reference>")
                angle=float(words[1]);desc=" ".join(words[2:])
            reference=self.state.yaw.align(angle,desc,now)
            if name=="fixture": reference["source"]="operator_confirmed_zone1_startup_fixture"
            self.state.static_review=None;self.state.static_history.clear()
            self.event("field_yaw_reference",reference=reference)
        elif name=="seed":
            if len(words)<4: raise ValueError("seed <centre X> <centre Y> <physical branch description>")
            if self.client.pending is not None: raise ValueError("cannot change map branch while an action is pending")
            self.state.seed(float(words[1]),float(words[2])," ".join(words[3:]))
            self.event("operator_map_branch",branch=self.state.branch)
        elif name=="review_static":
            checkpoint=self.state.review_static(" ".join(words[1:]),now)
            self.event("static_checkpoint_reviewed",checkpoint=checkpoint)
        elif name=="phase_rotate":
            self.state.enter_rotation(now);self.event("rotation_phase_entered")
        elif name=="turn":
            if self.state.phase!="rotate" or not self.state.static_review or self.state.yaw.field_rad(now) is None:
                raise ValueError("complete static review and phase_rotate with a valid field reference first")
            if self.state.static_review["yaw_reference_revision"]!=self.state.yaw.revision:
                raise ValueError("field reference changed; repeat static checkpoint")
            if len(words) not in (2,3): raise ValueError("turn <CCW deg> [deg/s]")
            if now-self.last_render_ns>350_000_000:
                raise ValueError("no recent lidar sector; resume the chain before requesting a turn")
            angle=float(words[1]);rate=float(words[2]) if len(words)==3 else 30.
            turn_ccw_frame(0,angle,rate)  # Validate before marking any possible UART write.
            before=self.client.action_number
            try: action=self.client.turn_ccw(angle,rate)
            finally: self.sent_motion=self.sent_motion or self.client.action_number!=before
            self.event("turn_sent",action=action,command_target_is_ground_truth=False)
        elif name=="stop":
            self.sent_motion=True
            self.event("stop_requested",action=self.client.stop(),physical_stopped_verified=False)
        elif name=="mark":
            if len(words)<2: raise ValueError("mark <measurement or observation>")
            self.event("manual_measurement_or_observation",description=" ".join(words[1:]),
                snapshot=self.latest_result,command_progress_is_truth=False)
        else: raise ValueError("unknown command; enter help")

    def observe(self, scan, now):
        m=self.config["installation"]
        raw=[[p.raw_angle_deg,math.hypot(*p.point_lidar_m)*1000,p.intensity] for p in scan.points]
        points,guard,rejected=split_samples(raw,m["center_raw_deg"],m["fov_deg"],m["display_fov_deg"],m["excluded_raw_ranges_deg"])
        info=dict(scan_id=scan.scan_id,host_receive_monotonic_ns=now,
            estimated_first_point_ns=scan.first_point_ns,estimated_last_point_ns=scan.last_point_ns,
            time_map_version=scan.time_map_version,capture_time_qualified=scan.timing_qualified,
            time_uncertainty_ns=None,timestamp_basis="unverified_receive_anchored_tick_reconstruction",
            physical_points=len(scan.points),accepted_points=len(points),guard_points=len(guard),masked_points=len(rejected))
        self.log("scans.jsonl",dict(scan=info,points=[asdict(p) for p in scan.points]))
        result=self.state.observe(points,now,info)
        self.scans_seen+=1
        self.static_fixes+=int(result.get("rank_xy",0)>0)
        self.rotation_scans+=int(self.state.phase=="rotate")
        result.update(timestamp_monotonic_ns=now,source="STL50B2_PLUS_PRIMARY_YAW_DIAGNOSTIC",
            mounting="inverted",sector_center_raw_deg=m["center_raw_deg"],sector_fov_deg=m["fov_deg"],
            display_fov_deg=m["display_fov_deg"],excluded_raw_ranges_deg=m["excluded_raw_ranges_deg"],
            usable_angular_measure_deg=m["usable_angular_measure_deg"],points=len(points),
            guard_points=guard.tolist(),rejected_points=rejected.tolist(),arena_size_m=[3.,3.],
            laser_to_base=[*m["extrinsic"][:2],math.radians(m["extrinsic"][2])],pose_reference="vehicle_center",
            vehicle_side_m=m["vehicle_side_m"],vehicle_front_vertex_deg=m["vehicle_front_vertex_deg"],
            reader_stats=dict(self.reader.stats),current_reference_reason=self.state.yaw.reason)
        result["actual_lower_identity"] = dict(mode=self.options.actual_lower_mode,
            service=self.options.actual_lower_service,firmware=self.options.actual_lower_firmware,
            firmware_attested_by_uart=False)
        xy=result.get("candidate_center_xy_m") or result.get("display_prior_center_xy_m")
        if xy is not None and result["field_yaw_rad"] is not None:
            result["partial_constraint_pose"]=[xy[0]+1.5,xy[1]+1.5,result["field_yaw_rad"]]
        self.latest_result=result
        self.log("results.jsonl",result)
        self.viewer.update(points.tolist(),result)
        self.last_render_ns=now

    def close(self, complete):
        stop_confirmed=None
        if self.client is not None:
            if self.sent_motion and self.client.serial is not None:
                try:
                    action=self.client.stop();deadline=time.monotonic()+1.
                    self.event("exit_stop_requested",action=action)
                    while self.client.pending is not None and time.monotonic()<deadline: time.sleep(.01)
                    stop_confirmed=bool(self.client.pending is None and self.client.last_action and
                        self.client.last_action["host_action_id"]==action["host_action_id"] and
                        self.client.last_action["outcome"]=="reported_stopped")
                except (OSError,RuntimeError,serial.SerialException): stop_confirmed=False
            self.client.close()
            complete=complete and self.client.log_error is None
        if self.unsubscribe is not None: self.unsubscribe()
        if self.lidar is not None: self.lidar.close()
        if self.viewer is not None: self.viewer.close()
        if self.run_dir is not None:
            summary=dict(scans=self.scans_seen,static_geometry_candidates=self.static_fixes,rotation_scans=self.rotation_scans,
                accuracy_verified=False,capture_time_qualified=False,translation_compensated=False,
                exit_stop_receipt_confirmed=stop_confirmed,physical_stopped_verified=False)
            summary["receipt_correlation_scope"]="current host action, receive-after-send seq/opcode; no MCU boot proof"
            (self.run_dir/"analysis.json").write_text(json.dumps(summary,indent=2)+"\n")
            self.event("session_stopped",summary=summary)
            if complete and not self.log_failed and self.scans_seen:
                (self.run_dir/"session_complete.json").write_text(json.dumps(dict(owner=OWNER,complete=True,
                    session_index=self.session_index,accuracy_verified=False))+"\n")
                removed=prune_completed_sessions(self.options.output_root,self.run_dir)
                self.event("session_retention",keep_complete_sessions=5,removed=removed)

    def run(self):
        complete=False
        try:
            self.start()
            signal.signal(signal.SIGTERM,lambda *_:setattr(self,"stop_requested",True))
            while not self.stop_requested:
                self.drain_chassis();self.client.poll()
                if select.select([sys.stdin],[],[],0)[0]:
                    line=sys.stdin.readline()
                    if line=="": self.stop_requested=True
                    else:
                        try: self.command(line)
                        except (ValueError,RuntimeError,serial.SerialException) as error:
                            self.event("operator_request_rejected",text=line.strip(),reason=str(error))
                            print("未执行：",error,flush=True)
                waiting=self.lidar.in_waiting
                if waiting>8192:
                    data=self.lidar.read(min(waiting,65536));now=time.monotonic_ns()
                    self.log("lidar_rx.jsonl",dict(timestamp_monotonic_ns=now,data_hex=data.hex(),
                        host_segment_id=self.lidar_epoch,not_used_reason="SERIAL_BACKLOG"))
                    self.event("lidar_backlog_rejected",waiting_bytes=waiting)
                    self.state.static_history.clear();self.state.static_review=None
                    self.new_reader("backlog_recorded_but_not_used")
                    self.viewer.update([],dict(status="SERIAL_BACKLOG",phase=self.state.phase,
                        field_yaw_rad=self.state.yaw.field_rad(now),position_observed_axes=[],rank_xy=0,
                        qualified_pose=False,full_pose_valid=False,translation_compensated=False,rotation_compensated=False))
                    continue
                data=self.lidar.read(min(waiting or 1,4096));now=time.monotonic_ns()
                if data:
                    self.log("lidar_rx.jsonl",dict(timestamp_monotonic_ns=now,data_hex=data.hex(),host_segment_id=self.lidar_epoch))
                    try:
                        for scan in self.reader.feed(data,now): self.observe(scan,now)
                    except LidarTimingError as error:
                        self.event("lidar_segment_rejected",reason=str(error))
                        self.state.static_history.clear();self.state.static_review=None
                        self.new_reader("discontinuity_discarded_no_replay")
                if now-self.last_render_ns>350_000_000:
                    # No stale valid-looking position/heading or repeated scan timestamp.
                    self.viewer.update([],dict(status="NO_CURRENT_SCAN",phase=self.state.phase,
                        field_yaw_rad=self.state.yaw.field_rad(now),position_observed_axes=[],rank_xy=0,
                        last_scan_age_s=None if not self.last_render_ns else (now-self.last_render_ns)/1e9,
                        qualified_pose=False,full_pose_valid=False,translation_compensated=False,rotation_compensated=False))
            complete=True
        except KeyboardInterrupt: complete=True
        finally: self.close(complete)
        return 0


def arguments():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config",type=Path,default=DEFAULT_CONFIG)
    ap.add_argument("--chassis",default="/dev/ttyS1")
    ap.add_argument("--lidar",help="default config /dev/ttyS6; must differ from chassis")
    ap.add_argument("--baud",type=int,help="default config 230400; no automatic baud commands")
    ap.add_argument("--http-port",type=int,default=8766)
    ap.add_argument("--open-browser",action="store_true")
    ap.add_argument("--output-root",type=Path,default=DEFAULT_LOG_ROOT)
    ap.add_argument("--actual-lower-mode",default="NormalDebug",choices=("NormalDebug","unconfirmed"))
    ap.add_argument("--actual-lower-service",default="unconfirmed",choices=("MOTIONDEBUG","unconfirmed"))
    ap.add_argument("--actual-lower-firmware",default="unconfirmed")
    return ap.parse_args()


if __name__=="__main__":
    try: raise SystemExit(LidarYawApp(arguments()).run())
    except (ValueError,RuntimeError,OSError) as error:
        print(f"雷达/yaw验证未启动或中断：{error}",file=sys.stderr)
        raise SystemExit(1)
