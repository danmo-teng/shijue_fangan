#!/usr/bin/env python3
"""Dynamic fence localization TEST entry. Reuses the T265 map's SDK/UART owner.
Explicit buttons are the only source of chassis motion. No competition output.
"""
import argparse
import json
import math
from pathlib import Path
import queue
import socket
import sys
import threading
import time
import uuid
import cv2
import numpy as np

import t265_f407_motion_map as base
from async_debug_log import AsyncDebugLog
from f405_timed_measurements import TimedMeasurements
from lidar_yaw_debug.diagnostic import load_config
from lidar_yaw_debug.worker import RadarWorker

DYNAMIC_FILES={'dynamic_events.jsonl','dynamic_scans.jsonl','dynamic_localization.jsonl',
    'dynamic_status.json','dynamic_summary.json','pose_motion.jsonl','lidar_rx.jsonl','lidar_tx.jsonl',
    'lidar_worker_events.jsonl','lidar_worker_summary.json'}
base.SESSION_FILES=base.SESSION_FILES|DYNAMIC_FILES


class DynamicMapApp(base.MotionMapApp):
    def __init__(self,options):
        cv2.setNumThreads(1)  # UI resize must not compete with sensor workers.
        super().__init__(options)
        self.dynamic_config=load_config(options.dynamic_config)
        self.radar=RadarWorker()
        self.pose_address='\0lidar-vio-'+uuid.uuid4().hex
        self.pose_socket=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        self.pose_socket.bind(self.pose_address);self.pose_socket.listen(1);self.pose_socket.settimeout(.05)
        self.dynamic_stop=threading.Event();self.pose_thread=None;self.yaw_thread=None
        self.dynamic_log=None;self.dynamic_unsubscribe=None
        self.chassis_inbox=queue.Queue(maxsize=2048);self.clock_map=TimedMeasurements()
        self.timed_yaw_seen=False;self.last_yaw_id=None;self.pose_packets=0;self.imu_packets=0
        self.dynamic_error=None;self.dynamic_snapshot={};self.path=[];self.vio_path=[]
        self.last_worker_input_drops=0
        self.last_dynamic_stamp=0;self.exit_requested=False
        self.capture_chain_present=False
        self.auto_reference_requested=False
        self.speed_mm_s=200;self.turn_rate_deg_s=60;self.distance_mm=500

    def extra_localizer_args(self):
        return ['--debug-pose-socket','@'+self.pose_address[1:]]

    def dlog(self,name,value):
        if self.dynamic_log:self.dynamic_log.submit(name,value)

    def devent(self,event,**data):
        self.dlog('dynamic_events.jsonl',dict(event=event,timestamp_monotonic_ns=time.monotonic_ns(),**data))

    def start_session(self):
        self.message='放在1号位、车头135°，点击“开始定位”；动作仅由按钮触发'
        self.render_screen()  # Prepare static glyphs before any sensor threads.
        super().start_session()
        self.dynamic_log=AsyncDebugLog(self.run_dir)
        if self.options.lidar==self.options.uart:raise ValueError('雷达与底盘不能共用同一UART')
        self.dynamic_unsubscribe=self.bridge.telemetry(self.dynamic_telemetry,readonly=True)
        if self.bridge.device_identity:self.clock_map.set_identity(self.bridge.device_identity)
        self.bridge.start_timed_measurements()  # Sync is diagnostic; primary yaw keeps its native sign.
        self.pose_thread=threading.Thread(target=self.read_pose,name='dynamic-pose-receiver',daemon=True)
        self.pose_thread.start()
        self.yaw_thread=threading.Thread(target=self.read_yaw,name='dynamic-yaw-forwarder',daemon=True)
        self.yaw_thread.start()
        self.radar.start(self.dynamic_config,self.options.lidar,self.options.lidar_baud,self.run_dir)
        m=base.read_json(self.run_dir/'metadata.json')
        m.update(localization_mode='lidar_t265_dynamic_debug',initial_pose=dict(x_m=-1.35,y_m=1.35,yaw_deg=135),
            physical_placement_confirmed=False,lidar_port=self.options.lidar,lidar_baud=self.options.lidar_baud,
            lidar_installation=self.dynamic_config['installation'],motion_compensation='SE2_per_point_translation_and_primary_yaw',
            pose_transport='bounded nonblocking same-SDK Unix SEQPACKET',dynamic_time_policy='timing_diagnostic_not_admission',
            qualification=self.dynamic_config['qualification'],control_pose_published=False,
            hardware_speed_accuracy_verified=False,maximum_test_command_speed_mm_s=700)
        m['wheel_and_primary_gyro_independent_of_sdk_input']=False if self.options.wheel_input=='on' else None
        m['sdk_device_internal_state_clean_verified']=False
        m['lidar_execution']='dedicated_spawn_process_parser_deskew_fit'
        m['lidar_worker_pid']=self.radar.process.pid
        m['line_sampler']='uniform_batched_ordered_pairs_160_v1'
        (self.run_dir/'metadata.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')
        self.message='放在1号位、车头135°，点击“开始定位”；动作仅由按钮触发'

    def dynamic_telemetry(self,event):
        if event['kind'] not in ('imu','timed_identity','timed_clock_sync','timed_gyro_diagnostic','device_identity_changed'):return
        try:self.chassis_inbox.put_nowait(event)
        except queue.Full:self.dynamic_error='主yaw队列溢出；已计为数据缺口'

    def drain_yaw(self):
        for _ in range(2048):
            try:event=self.chassis_inbox.get_nowait()
            except queue.Empty:break
            s=event['sample'];kind=event['kind']
            if kind=='timed_identity':self.clock_map.set_identity(s)
            elif kind=='timed_clock_sync':self.clock_map.sync(s)
            elif kind=='device_identity_changed':
                self.radar.send('invalidate',dict(reason='device_identity_changed'));self.timed_yaw_seen=False;self.last_yaw_id=None
                self.devent('reference_invalidated',reason='device_identity_changed')
            elif kind=='timed_gyro_diagnostic':
                if not self.clock_map.identity or s['boot_epoch']!=self.clock_map.identity['boot_epoch']:continue
                identity=(s['boot_epoch'],s['yaw_source_id'])
                if identity==self.last_yaw_id:continue
                flags=s['yaw_flags']
                if not flags&1 or flags&(4|8|32|1024):continue
                estimate,uncertainty,reasons=self.clock_map.map_time(s['yaw_receive_us'],s['receive_monotonic_ns'])
                t=estimate if estimate is not None and estimate>=0 else s['receive_monotonic_ns']
                self.radar.send('yaw',dict(time_ns=t,yaw_deg=s['yaw_mdeg']/1000.,source=dict(source='versioned_primary_yaw',raw=s,
                    time_qualified=False,time_uncertainty_ns=uncertainty,time_diagnostics=reasons)))
                self.last_yaw_id=identity;self.timed_yaw_seen=True;self.imu_packets+=1
            elif kind=='imu' and not self.timed_yaw_seen and s.get('wire_valid'):
                self.radar.send('yaw',dict(time_ns=s['timestamp_monotonic_ns'],yaw_deg=s['yaw_deg'],
                    source=dict(source='legacy_primary_yaw_receive',raw=s,time_qualified=False)))
                self.imu_packets+=1

    def read_pose(self):
        connection=None
        while not self.dynamic_stop.is_set():
            try:
                if connection is None:
                    connection,_=self.pose_socket.accept();connection.settimeout(.05)
                    connection.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,262144)
                data=connection.recv(16384)
                if not data:break
            except socket.timeout:continue
            except OSError:break
            try:
                p=json.loads(data);self.radar.send('pose',p);self.pose_packets+=1;self.dlog('pose_motion.jsonl',p)
            except (ValueError,KeyError,TypeError) as e:self.dynamic_error='Pose解析: '+str(e)
        if connection is not None:connection.close()

    def read_yaw(self):
        while not self.dynamic_stop.wait(.005):
            self.drain_yaw()

    def update_data(self):
        super().update_data()
        for message in self.radar.poll():
            if message['kind']=='reference_set':
                self.path=[];self.vio_path=[];self.dynamic_error=None
                self.devent('field_reference_set',reference=message['reference'])
                self.message='定位已开始；初始朝向朝外，先后退可向场内移动'
            elif message['kind']=='reference_rejected':self.message=message['error']
            elif message['kind']=='error':raise RuntimeError('雷达进程：'+message['error'])
        state=self.radar.state;self.dynamic_snapshot=state.get('last',{})
        if (self.radar.process is not None and self.radar.process.exitcode is not None
                and not self.dynamic_stop.is_set()):
            self.logging_failed=True
            raise RuntimeError('雷达工作进程已退出，查看lidar_worker_events.jsonl')
        if self.radar.input_drops!=self.last_worker_input_drops:
            self.last_worker_input_drops=self.radar.input_drops
            self.devent('bounded_motion_queue_overflow',dropped=self.radar.input_drops)
        if (self.options.sensor_check_seconds and not self.dynamic_snapshot.get('ready')
                and not self.auto_reference_requested and self.radar.pending_reference is None
                and state.get('have_tracked_pose') and state.get('have_primary_yaw')):
            self.radar.align('desktop sensor check; initial field position is only a hypothesis',physical_confirmed=False)
            self.auto_reference_requested=True
        self.input_snapshot=self.wheel_input.snapshot(self.wheel_sdk_status) if self.wheel_input else {}
        if self.dynamic_snapshot.get('ready'):
            stamp=self.dynamic_snapshot['sample_monotonic_ns']
            if stamp!=self.last_dynamic_stamp:
                self.path.append(self.dynamic_snapshot['center_xy_m']);self.path=self.path[-5000:];self.last_dynamic_stamp=stamp
                self.vio_path.append(self.dynamic_snapshot['t265_center_xy_m']);self.vio_path=self.vio_path[-5000:]
        if self.dynamic_log:self.dynamic_log.submit('dynamic_status.json',self.dynamic_snapshot,atomic=True)

    def render_logical(self):
        # Chinese rasterization is expensive and can hold the Python GIL.
        # Cache static labels/buttons; per-frame numbers use OpenCV primitives.
        key=(self.speed_mm_s,self.turn_rate_deg_s,self.options.wheel_input)
        if getattr(self,'background_key',None)!=key:
            canvas=np.full((base.LOGICAL_HEIGHT,base.LOGICAL_WIDTH,3),245,np.uint8)
            text=base.TextPainter(font_scale=1./self.render_scale);self.hitboxes.clear()
            cv2.rectangle(canvas,(30,60),(880,910),(70,70,70),2)
            for value in (-1.,-.5,0.,.5,1.):
                x,y=self.world_to_pixel(value,value)
                cv2.line(canvas,(x,60),(x,910),(215,215,215));cv2.line(canvas,(30,y),(880,y),(215,215,215))
            text.add('雷达＋T265动态定位 / 中心原点 / 1号位135°',(30,20),20,(35,35,35),True)
            text.add('绿点：雷达；蓝：T265车中心；橙：雷达修正后',(30,930),16,(35,35,35))
            text.add('测试显示，精度待实测；不向正式比赛发布',(30,960),14,(70,70,70))
            self.draw_panel(canvas,text)
            self.background=text.paint(canvas);self.background_key=key
            self.background_hitboxes=dict(self.hitboxes)
        canvas=self.background.copy();self.hitboxes=dict(self.background_hitboxes)
        s=self.dynamic_snapshot
        self.draw_path(canvas,self.vio_path,(220,120,50),2)
        self.draw_path(canvas,self.path,(30,130,235),2)
        for p in s.get('points_field',[]):
            q=self.world_to_pixel(*p)
            if 30<=q[0]<=880 and 60<=q[1]<=910:cv2.circle(canvas,q,1,(70,160,40),-1)
        def number(value,y,color=(225,225,225)):
            cv2.putText(canvas,str(value),(920,y),cv2.FONT_HERSHEY_SIMPLEX,.65,color,1,cv2.LINE_AA)
        if s.get('ready'):
            x,y=s['center_xy_m'];point=self.world_to_pixel(x,y);a=math.radians(s['yaw_deg'])
            cv2.circle(canvas,point,7,(30,130,235),-1)
            cv2.arrowedLine(canvas,point,self.world_to_pixel(x+.15*math.cos(a),y+.15*math.sin(a)),(30,130,235),3)
            number(f'X {x:+.3f}m  Y {y:+.3f}m',88,(230,220,70))
            number(f'Yaw {s["yaw_deg"]:.1f} deg',120)
        else:number('WAIT: place at zone 1',88)
        number('Wall: '+('/'.join(s.get('current_wall_axes',[])) or 'VIO'),153)
        number(f'Scan {self.radar.state.get("scans_received",0)} / Fit {self.radar.state.get("fits",0)}',183)
        number(f'Pose {self.pose_packets} / Yaw {self.imu_packets}',213)
        wheel=getattr(self,'input_snapshot',{});count=wheel.get('sdk_api_success_samples',0)
        stage='OFF' if self.options.wheel_input=='off' else ('API_OK' if count else 'CFG_OK' if wheel.get('config_loaded') else 'WAIT_CFG')
        number(f'{stage} / {count} / {wheel.get("sdk_api_rate_hz",0):.0f}Hz',849)
        age=wheel.get('sdk_success_age_ms');age_text='--' if age is None else f'{age:.0f}'
        cv2.putText(canvas,f'last API {age_text}ms / reject {(self.wheel_sdk_status or {}).get("rejected_samples",0)}',
            (920,868),cv2.FONT_HERSHEY_SIMPLEX,.45,(200,200,200),1,cv2.LINE_AA)
        number(self.motion_status.get('state_name','WAIT'),886,(140,210,240))
        msg=('前mm,左mm,角deg；未知?：'+self.measurement_buffer) if self.measurement_buffer is not None else (self.dynamic_error or self.message)
        if getattr(self,'message_key',None)!=msg:
            patch=np.full((110,380,3),(26,27,30),np.uint8)
            text=base.TextPainter(font_scale=1./self.render_scale)
            text.add(msg,(20,10),13,(130,210,240),max_width=330)
            self.message_image=text.paint(patch);self.message_key=msg
        canvas[900:1010,900:1280]=self.message_image
        return canvas

    def draw_panel(self,canvas,text):
        x=920;cv2.rectangle(canvas,(900,0),(1280,1024),(26,27,30),-1)
        text.add('运动定位测试',(x,20),24,(240,240,240),True)
        self.button(canvas,text,'start_dynamic','1号位已摆好，开始定位',(x,240,1250,285))
        self.button(canvas,text,'speed_slow','200mm/s',(x,310,1078,350),self.speed_mm_s==200)
        self.button(canvas,text,'speed_fast','700mm/s 上限',(1090,310,1250,350),self.speed_mm_s==700)
        text.add(f'转速上限 {self.turn_rate_deg_s}°/s',(x,355),12,(200,200,200))
        for i,(name,label) in enumerate([('forward','前进50cm'),('backward','后退50cm'),('lateral','左移50cm'),('right','右移50cm')]):
            col,row=i%2,i//2;self.button(canvas,text,name,label,(x+col*170,380+row*60,x+col*170+160,430+row*60))
        self.button(canvas,text,'turn_90','左转90°',(x,515,1078,565))
        self.button(canvas,text,'turn_-90','右转90°',(1090,515,1250,565))
        self.button(canvas,text,'stop','停止',(x,590,1250,665),False,(55,55,150))
        self.button(canvas,text,'mark','停稳后记点',(x,690,1078,735))
        self.button(canvas,text,'measure','录入实测',(1090,690,1250,735))
        self.button(canvas,text,'quit','结束并保存',(x,755,1250,800))
        text.add('SDK轮速输入 / 成功数 / 底盘状态',(x,815),14,(210,210,210))

    def select_at(self,x,y):
        name=next((n for n,b in self.hitboxes.items() if b[0]<=x<=b[2] and b[1]<=y<=b[3]),None)
        if name is None:return
        if name=='start_dynamic':
            if self.bridge.pending:self.message='先结束当前动作再建立起点';return
            try:
                request=self.radar.align('operator button: vehicle at zone1 (-1.35,+1.35), heading135')
                self.devent('field_reference_requested',request_id=request);self.message='正在建立1号位参考'
            except ValueError as e:self.message=str(e)
        elif name=='speed_slow':self.speed_mm_s=200;self.turn_rate_deg_s=60
        elif name=='speed_fast':self.speed_mm_s=700;self.turn_rate_deg_s=360
        elif name=='stop':self.active_test=None;self.send_stop('用户动态地图停止')
        elif name=='quit':self.exit_requested=True
        elif name=='mark':
            try:
                self.mark_settled_reference()
                self.settled_reference_mark['dynamic_localization']=self.dynamic_snapshot
                self.devent('operator_settled_mark',test_id=self.test_id,marker=self.settled_reference_mark)
                self.message=f'实验{self.test_id}终点已冻结；点击录入实测，填写卷尺/地面角度'
            except ValueError as e:self.message=str(e)
        elif name=='measure':
            if self.active_test is not None or not self.settled_reference_mark:self.message='先停稳后记点，再录入实测'
            else:self.measurement_buffer=''
        elif self.radar.pending_reference is not None or not self.dynamic_snapshot.get('ready'):self.message='请先摆好1号位并开始定位'
        else:super().select_at(x,y)

    def handle_key(self,key):
        if self.measurement_buffer is not None:return super().handle_key(key)
        if self.exit_requested or key&255 in (27,ord('q'),ord('Q')):return False
        if key&255 in (32,ord('s'),ord('S')):self.active_test=None;self.send_stop('用户停止键')
        return True

    def save_ground_truth(self,text):
        super().save_ground_truth(text)
        self.devent('dynamic_ground_truth',test_id=self.test_id,measurement=text,
            reference=self.options.reference_description,marker=self.settled_reference_mark)

    def run(self):
        result=super().run()
        if self.options.sensor_check_seconds and not self.capture_chain_present:
            raise RuntimeError('检查未收到完整Pose/主yaw/雷达链路；查看dynamic_summary与localizer.log')
        return result

    def stop_session(self):
        if self.bridge and self.bridge.serial is not None and self.last_sent_command is not None:
            try:self.send_stop('动态地图退出停车')
            except (OSError,RuntimeError):pass
            self.last_sent_command=None  # Parent must not submit a second exit STOP.
        self.dynamic_stop.set()
        if self.dynamic_unsubscribe:self.dynamic_unsubscribe();self.dynamic_unsubscribe=None
        for worker in (self.pose_thread,self.yaw_thread):
            if worker:
                worker.join(timeout=3)
                if worker.is_alive():self.logging_failed=True;self.dynamic_error='处理线程退出未完成'
        self.radar.close()
        self.pose_socket.close()
        if self.dynamic_log:
            summary=dict(self.radar.state,pose_packets=self.pose_packets,primary_yaw_packets=self.imu_packets,
                motion_queue_drops=self.radar.input_drops,worker_stopped_cleanly=self.radar.stopped_cleanly,
                parent_error=self.dynamic_error)
            self.capture_chain_present=bool(summary.get('capture_chain_present'))
            self.logging_failed=(self.logging_failed or not self.radar.stopped_cleanly
                or not summary.get('logging_healthy',False))
            self.dynamic_log.submit('dynamic_summary.json',summary,atomic=True);self.dynamic_log.close()
            self.logging_failed=self.logging_failed or not self.dynamic_log.healthy()
        super().stop_session()


def arguments():
    ap=argparse.ArgumentParser(add_help=False)
    ap.add_argument('--lidar',default='/dev/ttyS6');ap.add_argument('--lidar-baud',type=int,default=230400)
    ap.add_argument('--dynamic-config',type=Path,default=base.PROJECT_ROOT/'localization/config/lidar_yaw_debug.json')
    extra,remaining=ap.parse_known_args();old=sys.argv
    if not any(x=='--wheel-input' or x.startswith('--wheel-input=') for x in remaining):
        remaining += ['--wheel-input','on']  # User confirmed dynamic default, 2026-11-03.
    try:sys.argv=[old[0],*remaining];o=base.arguments()
    finally:sys.argv=old
    for k,v in vars(extra).items():setattr(o,k,v)
    if o.wheel_fixture!='off' or o.check_chassis:raise ValueError('dynamic entry does not accept synthetic/chassis-check mode')
    o.zone=1;o.center_origin=False;o.heading_offset_deg=0.
    return o


if __name__=='__main__':
    try:raise SystemExit(DynamicMapApp(arguments()).run())
    except (ValueError,RuntimeError,OSError) as e:print('动态定位：'+str(e),file=sys.stderr);raise SystemExit(2)
