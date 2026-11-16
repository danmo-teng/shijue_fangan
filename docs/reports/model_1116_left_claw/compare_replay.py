import sys,json,hashlib
from pathlib import Path
import cv2,numpy as np
R=Path(__file__).resolve().parents[3];sys.path[:0]=[str(R/'vision'),str(R/'competition_rescue')]
from rescue_vision.yolo26 import X5Yolo26,DEFAULT_MODEL
from rescue_vision.vse import VseScaler
from run_yolo_x5 import load_labels,DEFAULT_LABELS,bgr_to_nv12
out=R/'docs/reports/model_1116_left_claw';out.mkdir(exist_ok=True)
old=R/'yolo26n/x5_output_640_1020/yolo26n_6cls_640_bpu_bayese_640x640_nv12.bin';new=DEFAULT_MODEL
models={n:X5Yolo26(p,load_labels(DEFAULT_LABELS),.2,.45,0,[0]) for n,p in [('old',old),('1116',new)]}
print('MODELS',[(n,str(p),hashlib.sha256(p.read_bytes()).hexdigest()) for n,p in [('old',old),('1116',new)]],flush=True)
print('SHAPES',models['1116'].shapes,flush=True)
root=R/'rescue_map/runtime/history/competition_wheel_input/20261116_155030_competition_wheel'
t0=1816.777909754;scaler=VseScaler(1280,1024,640,512);results=[]
try:
 for clip,lo,hi,mode in [(4,172.8,182.7,48),(8,276.5,279.6,23)]:
  rows=[json.loads(x) for x in (root/f'safe_zone_replay_{clip:04}.jsonl').read_text().splitlines()]
  rows=[x for x in rows if lo<=x['snapshot_monotonic_ns']/1e9-t0<=hi and x['stm']['mode']==mode and x['stm']['camera_pitch_cdeg']==14000]
  rows=[rows[i] for i in np.linspace(0,len(rows)-1,min(12,len(rows)),dtype=int)]
  video=cv2.VideoCapture(str(root/f'safe_zone_replay_{clip:04}.avi'))
  for row in rows:
   idx=row['video_frame_index'];video.set(cv2.CAP_PROP_POS_FRAMES,idx);ok,full=video.read();assert ok
   # Remove only the separate text header; recorded overlays remain. No inpainting.
   picture=cv2.resize(full[96:],(1280,1024),interpolation=cv2.INTER_LINEAR)
   nv12=bgr_to_nv12(picture)
   record=dict(clip=clip,video_frame_index=idx,frame_sequence=row['frame_sequence'],seconds=row['snapshot_monotonic_ns']/1e9-t0,original_detections=row['detections'])
   # Fixed manually chosen vicinity of the visible left-claw block; NOT a runtime ROI.
   # Count a green box only when its centre is in this area and it has plausible size.
   for name,m in models.items():
    dets,_=m.infer_nv12(nv12,1280,1024,scaler)
    boxes=[dict(class_name=d.class_name,confidence=float(d.confidence),bbox=list(map(int,d.bbox))) for d in dets]
    hits=[d for d in boxes if d['class_name']=='green_supply' and 270<=d['bbox'][0]+d['bbox'][2]/2<=650 and 130<=d['bbox'][1]+d['bbox'][3]/2<=550 and d['bbox'][2]*d['bbox'][3]>=12000]
    record[name]=dict(left_green_found=bool(hits),left_green_confidence=max([d['confidence'] for d in hits],default=None),detections=boxes)
   results.append(record)
   if len(results)==1 or idx==rows[len(rows)//2]['video_frame_index']:
    cv2.imwrite(str(out/f'clip{clip:02d}_frame{idx}_replay.jpg'),full)
  video.release()
finally:scaler.close()
meta=dict(note='Paired offline replay, MJPEG compressed 960x768 camera region resized to 1280x1024; original detection/ROI overlays remain. Not original raw inference inputs, no field accuracy claim.',models={n:dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for n,p in [('old',old),('1116',new)]},preprocess='existing VSE 1280x1024 -> 640x512 + original NV12 letterbox',score_threshold=.2,nms_threshold=.45,results=results)
(out/'comparison.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
for clip in (4,8):
 rr=[x for x in results if x['clip']==clip];print('RESULT',clip,'n',len(rr),'old',sum(x['old']['left_green_found'] for x in rr),'1116',sum(x['1116']['left_green_found'] for x in rr),'original_frame_with_green',sum(any(d['class']=='green_supply' for d in x['original_detections'])for x in rr),flush=True)
print('DONE',len(results),flush=True)
