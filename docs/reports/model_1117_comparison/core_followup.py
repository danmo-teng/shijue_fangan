import sys,json,cv2
from pathlib import Path
R=Path('/home/sunrise/RDK_X5/shijue_fangan');sys.path[:0]=[str(R/'vision')]
from rescue_vision.yolo26 import X5Yolo26
from rescue_vision.vse import VseScaler
from run_yolo_x5 import load_labels,DEFAULT_LABELS,bgr_to_nv12
models={n:X5Yolo26(R/f'yolo26n/x5_output_640_{n}/yolo26n_6cls_640_{n}_bpu_bayese_640x640_nv12.bin',load_labels(DEFAULT_LABELS),.2,.45,0,[0]) for n in ['1116','1117']}
c=cv2.VideoCapture(str(R/'rescue_map/runtime/history/competition_wheel_input/20261117_113433_competition_wheel/safe_zone_replay_0002.avi'));sc=VseScaler(1280,1024,640,512);out=[]
try:
 for idx in range(76,89):
  c.set(cv2.CAP_PROP_POS_FRAMES,idx);ok,im=c.read();assert ok
  nv=bgr_to_nv12(cv2.resize(im[96:],(1280,1024)))
  row={'video_frame_index':idx}
  for n,m in models.items():
   ds,_=m.infer_nv12(nv,1280,1024,sc)
   row[n]=[{'class_name':d.class_name,'confidence':float(d.confidence),'bbox':d.bbox} for d in ds if d.class_name=='core_black']
  out.append(row);print(idx,{n:[round(d['confidence'],3) for d in row[n]] for n in models},flush=True)
finally:c.release();sc.close()
(R/'docs/reports/model_1117_comparison/core_followup.json').write_text(json.dumps(out,indent=2)+'\n')
