"""Offline paired BPU/VSE comparison; never opens camera, UART or motors."""
from pathlib import Path
import sys,json,hashlib,time
import cv2,numpy as np
R=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(R/'vision'),str(R/'competition_rescue')]
from rescue_vision.yolo26 import X5Yolo26
from rescue_vision.vse import VseScaler
from run_yolo_x5 import load_labels,DEFAULT_LABELS,bgr_to_nv12
out=Path(__file__).parent
paths={n:R/f'yolo26n/x5_output_640_{n}/yolo26n_6cls_640_{n}_bpu_bayese_640x640_nv12.bin' for n in ['1116','1117']}
models={n:X5Yolo26(p,load_labels(DEFAULT_LABELS),.2,.45,0,[0]) for n,p in paths.items()}
print('SHAPES',models['1117'].shapes,flush=True)
records=[];samples=[]
oldroot=R/'rescue_map/runtime/history/competition_wheel_input/20261116_155030_competition_wheel'
for row in json.loads((R/'docs/reports/model_1116_left_claw/comparison.json').read_text())['results']:
 samples.append(('left_green',oldroot,row['clip'],row['video_frame_index']))
root=R/'rescue_map/runtime/history/competition_wheel_input/20261117_113433_competition_wheel'
for clip in range(1,5):
 rows=[json.loads(l) for l in (root/f'safe_zone_replay_{clip:04}.jsonl').read_text().splitlines()]
 for i in np.linspace(0,len(rows)-1,min(8,len(rows)),dtype=int):samples.append(('mixed_scene',root,clip,rows[i]['video_frame_index']))
 far=[r for r in rows if any(d['class'] not in ['safe_red','safe_blue'] and d['bbox'][3]<100 for d in r['detections'])]
 for i in np.linspace(0,len(far)-1,min(4,len(far)),dtype=int) if far else []:samples.append(('far_small',root,clip,far[i]['video_frame_index']))
scaler=VseScaler(1280,1024,640,512)
try:
 for idx,(group,folder,clip,frame) in enumerate(samples):
  video=cv2.VideoCapture(str(folder/f'safe_zone_replay_{clip:04}.avi'));video.set(cv2.CAP_PROP_POS_FRAMES,frame);ok,full=video.read();video.release();assert ok
  picture=cv2.resize(full[96:],(1280,1024),interpolation=cv2.INTER_LINEAR);nv12=bgr_to_nv12(picture)
  row=dict(group=group,folder=str(folder.relative_to(R)),clip=clip,video_frame_index=frame)
  for n in (['1116','1117'] if idx%2==0 else ['1117','1116']):
   dets,timing=models[n].infer_nv12(nv12,1280,1024,scaler)
   boxes=[dict(class_name=d.class_name,confidence=float(d.confidence),bbox=list(map(int,d.bbox))) for d in dets]
   left=[d for d in boxes if d['class_name']=='green_supply' and 270<=d['bbox'][0]+d['bbox'][2]/2<=650 and 130<=d['bbox'][1]+d['bbox'][3]/2<=550 and d['bbox'][2]*d['bbox'][3]>=12000]
   row[n]=dict(detections=boxes,timing_ms=timing,left_green_found=bool(left))
  records.append(row)
  if group=='far_small' or idx in [0,21,29,37,45]:cv2.imwrite(str(out/f'sample_{idx:03}.jpg'),full)
 # Same picture repeated, exclude first five calls for both backends.
 timing={}
 for n,m in models.items():
  values=[]
  for i in range(30):
   ds,t=m.infer_nv12(nv12,1280,1024,scaler)
   if i>=5:values.append(t)
  timing[n]={k:float(np.median([d[k] for d in values])) for k in values[0]}
  ds,cpu=m.infer(picture)
  assert all(np.isfinite(d.confidence) and d.bbox[2]>0 and d.bbox[3]>0 for d in ds)
  print('TIMING',n,timing[n],flush=True)
finally:scaler.close()
meta=dict(models={n:dict(path=str(p.relative_to(R)),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),shapes=models[n].shapes) for n,p in paths.items()},labels=load_labels(DEFAULT_LABELS),note='MJPEG replay contains original overlays, 960x768 resized to1280x1024; not clean raw frames or independent annotated validation. No full-match FPS or ground recall claim.',threshold=.2,nms=.45,median_timing_ms=timing,results=records)
(out/'comparison.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
for group in ['left_green','mixed_scene','far_small']:
 rr=[r for r in records if r['group']==group];print('RESULT',group,len(rr),{n:dict(left_green=sum(r[n]['left_green_found'] for r in rr),boxes=sum(len(r[n]['detections']) for r in rr)) for n in models},flush=True)
