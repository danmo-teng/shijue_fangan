from pathlib import Path
import json,sys,collections,numpy as np
R=Path(__file__).resolve().parents[3];sys.path.insert(0,str(R/'competition_rescue'))
from protocol import crc16_modbus
out=[]
for name,t0 in [('20261117_222902_competition_wheel',542.883069747),('20261117_223218_competition_wheel',739.991161081)]:
 p=R/'rescue_map/runtime/history/competition_wheel_input'/name
 result={'session':name,'t0':t0,'identity':json.loads((p/'chassis_identity.json').read_text()),'mode_transitions':[]}
 ctx=None;buf=b'';prev=None
 with (p/'uart_rx_native.jsonl').open() as stream:
  for lineno,l in enumerate(stream,1):
   try:d=json.loads(l)
   except ValueError:continue
   buf+=bytes.fromhex(d['data_hex'])
   while len(buf)>=15:
    k=buf.find(b'\xa3\xb3')
    if k<0:buf=buf[-1:];break
    buf=buf[k:]
    if len(buf)<15:break
    f=buf[:15]
    if f[14]!=195 or crc16_modbus(f[2:12])!=int.from_bytes(f[12:14],'little'):buf=buf[1:];continue
    buf=buf[15:]
    if f[2]==30:ctx=f
    elif f[2]==23:
     key=(f[5],int.from_bytes(ctx[4:6],'big') if ctx and ctx[3]==f[3] else None)
     if key!=prev:
      result['mode_transitions'].append({'t':d['timestamp_monotonic_ns']/1e9-t0,'mode':key[0],'task':key[1],'flags':f[4],'source_line':lineno,'hex':f.hex(),'context_hex':ctx.hex() if ctx else None});prev=key
     ctx=None
 gyro=[]
 with (p/'motion_feedback.jsonl').open() as stream:
  for l in stream:
   d=json.loads(l)
   if d.get('type')==8:gyro.append((d['receive_monotonic_ns']/1e9-t0,d['rate_mdps']/1000))
 if name.endswith('222902_competition_wheel'):
  result['turn_rates']={f'{a}-{b}':{'median_abs_deg_s':float(np.median([abs(v) for t,v in gyro if a<=t<=b])),'max_abs_deg_s':max(abs(v) for t,v in gyro if a<=t<=b)} for a,b in [(54.09,54.67),(101.5,103.)]}
  for task in [20307,20310]:print(name,'task',task,[(round(d['t'],3),d['mode']) for d in result['mode_transitions'] if d['task']==task])
  print(result['turn_rates'])
 print(name,'mode50',[(round(d['t'],3),d['task']) for d in result['mode_transitions'] if d['mode']==50]);out.append(result)
(Path(__file__).parent/'raw_status_evidence.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
