#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import cv2
import numpy as np

from rescue_vision.camera import LatestFrameCamera, resolve_camera_device
from rescue_vision.config import load_config, save_config
from rescue_vision.detector import TraditionalDetector
from rescue_vision.localizer import GroundLocalizer
from rescue_vision.tuning import auto_sample_profile, diagnose_frame


HTML = r'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>RDK X5 救援视觉调试器</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#10151c;color:#e8edf3;font-family:Arial,"Microsoft YaHei",sans-serif}
header{position:sticky;top:0;z-index:5;background:#18212c;padding:12px 20px;display:flex;gap:12px;align-items:center;border-bottom:1px solid #364454}
button,select,input{font-size:16px}button{padding:9px 14px;background:#2878c8;color:white;border:0;border-radius:6px;cursor:pointer}button.warn{background:#ba6123}button.good{background:#23864b}
main{display:grid;grid-template-columns:minmax(660px,1.35fr) minmax(520px,1fr);gap:16px;padding:16px}.panel{background:#18212c;border:1px solid #334151;border-radius:8px;padding:14px}
#canvas{width:100%;max-width:1000px;aspect-ratio:16/9;background:#000;cursor:crosshair;display:block}.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:8px 0}.status{color:#7fd6ff}.help{color:#aebdca;font-size:14px;line-height:1.55}
.section{margin-top:14px;border-top:1px solid #354454;padding-top:12px}.section h3{margin:0 0 10px;color:#75c7ff}.control{display:grid;grid-template-columns:115px minmax(180px,1fr) 90px;gap:10px;align-items:center;margin:7px 0}.control input[type=range]{width:100%;height:28px}.control input[type=number],input[type=text]{width:100%;padding:7px;background:#0e141b;color:white;border:1px solid #4c6175;border-radius:4px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:12px}.badge{padding:4px 8px;border-radius:12px;background:#35495d;font-size:13px}.danger{background:#8a3b3b}.ok{background:#246c46}pre{white-space:pre-wrap;background:#0d131a;padding:10px;border-radius:5px;max-height:220px;overflow:auto}
@media(max-width:1150px){main{grid-template-columns:1fr}.control{grid-template-columns:105px minmax(200px,1fr) 85px}}
</style></head><body>
<header><b>RDK X5 救援视觉 Web 编辑器</b><span id="connection" class="status">连接中...</span><button onclick="saveConfig()" class="good">保存全部配置</button><button onclick="toggleFreeze()" id="freezeBtn">冻结画面</button></header>
<main><div><div class="panel"><div class="row"><label>调节组：</label><select id="classSelect" onchange="selectClass()"></select><span id="groupBadge" class="badge"></span><span id="keyBadge" class="badge"></span></div>
<div class="row"><label>组号</label><input id="groupIndex" type="number" min="1" max="99" style="width:75px"><label style="width:85px">组显示名称</label><input id="displayName" type="text"><button onclick="applyGroupMeta()">修改组号/组名</button><button onclick="applyAll()">应用参数</button></div>
<div class="row"><label>参考阈值</label><select id="referenceSelect" onchange="selectReference()"></select><label>参考名称</label><input id="referenceName" type="text" style="max-width:180px"><button onclick="addReference()" class="good">复制并新增参考</button><button onclick="deleteReference()" class="warn">删除当前参考</button></div>
<canvas id="canvas" width="1280" height="720"></canvas>
<div class="row"><button onclick="setView('original')">原图</button><button onclick="setView('mask')">白黑掩膜</button><button onclick="setView('annotated')">候选与分类</button><button class="warn" onclick="autoSample()">用框选区域自动取值</button></div>
<div class="help">鼠标在画面上按住左键框住一个目标（尽量少带背景），点击“自动取值”。程序会用稳健分位数估算HSV/Lab，并估算面积、长宽比、填充率、实心度；之后再用右侧控件微调。框选时建议先冻结。</div>
<pre id="sampleResult">尚未框选采样</pre></div>
<div class="panel section"><h3>现场光照与摄像头诊断</h3><div class="row"><button onclick="diagnose()">分析框选区域并估算曝光</button><button onclick="autoWhiteBalance()">灰卡自动锁白平衡</button><button onclick="autoFocus()">框选目标自动扫焦</button></div><div class="row"><label>曝光</label><input id="exposure" type="number" min="4" max="50" value="15" style="width:85px"><label>白平衡</label><input id="wb" type="number" min="2800" max="6500" value="4500" style="width:100px"><label>焦距</label><input id="focus" type="number" min="0" max="1023" value="264" style="width:90px"><button onclick="applyCamera()" class="warn">写入摄像头</button></div><pre id="diagnosis">曝光：框住灰卡或典型物资。白平衡：把哑光灰卡放入框内。焦距：框住最远仍需可靠识别的位置。</pre></div></div>
<div class="panel"><h3>当前分类规则</h3><div id="ruleText" class="help"></div><div class="row"><label><input id="useShape" type="checkbox"> 启用形状硬过滤</label><label><input id="useSize" type="checkbox"> 启用毫米尺寸（仅标定后有效）</label></div>
<div class="section"><h3>颜色空间与融合</h3><div class="row"><label>融合：</label><select id="fusion"><option value="and">HSV AND Lab（误检少）</option><option value="or">HSV OR Lab（漏检少）</option><option value="hsv">仅HSV</option><option value="lab">仅Lab</option></select></div><div id="colorControls"></div></div>
<div class="section"><h3>形态学、候选与分类</h3><div id="shapeControls"></div></div>
<div class="section"><h3>多帧确认</h3><div id="trackControls"></div></div>
<div class="row"><button onclick="applyAll()" class="good">应用全部参数</button><span class="help">数值框可直接输入，无需逐格拖动滑条。</span></div></div></main>
<script>
let state=null, view='original', frozen=false, rect=null, dragging=false, start=null, image=new Image();const canvas=document.getElementById('canvas'),ctx=canvas.getContext('2d');
const defs=[['H min','hsv',0,0,179,1],['S min','hsv',1,0,255,1],['V min','hsv',2,0,255,1],['H max','hsv',3,0,179,1],['S max','hsv',4,0,255,1],['V max','hsv',5,0,255,1],['L min','lab',0,0,255,1],['A min','lab',1,0,255,1],['B min','lab',2,0,255,1],['L max','lab',3,0,255,1],['A max','lab',4,0,255,1],['B max','lab',5,0,255,1]];
const shapes=[['开运算核(基准)','morphology.open',0,31,1],['闭运算核(基准)','morphology.close',0,31,1],['面积最小(基准)','candidate.area_px.0',0,200000,10],['面积最大(基准)','candidate.area_px.1',10,500000,10],['长宽比最小','candidate.aspect.0',1,12,.05],['长宽比最大','candidate.aspect.1',1,15,.05],['填充率最小','candidate.extent_min',0,1,.01],['实心度最小','candidate.solidity_min',0,1,.01],['颜色占比最小','candidate.color_fill_min',0,1,.01],['局部对比最小','candidate.contrast_min',0,150,1],['多边形顶点最小','candidate.vertices.0',3,15,1],['多边形顶点最大','candidate.vertices.1',3,20,1],['分类阈值','score_min',0,1,.01],['颜色权重','weights.color',0,1,.01],['形状权重','weights.shape',0,1,.01],['尺寸权重','weights.size',0,1,.01]];
const tracks=[['确认命中帧数','confirmation.min_hits',1,30,1],['允许丢失帧数','confirmation.max_misses',0,60,1],['匹配距离','confirmation.match_distance',10,500,5]];
function getPath(o,p){return p.split('.').reduce((a,k)=>a[k],o)}function setPath(o,p,v){let a=p.split('.'),x=o;for(let i=0;i<a.length-1;i++)x=x[a[i]];x[a.at(-1)]=v}
function control(id,label,min,max,step,value){return `<div class="control"><label>${label}</label><input type="range" min="${min}" max="${max}" step="${step}" value="${value}" oninput="document.getElementById('${id}n').value=this.value"><input id="${id}n" type="number" min="${min}" max="${max}" step="${step}" value="${value}" oninput="this.previousElementSibling.value=this.value"></div>`}
async function api(path,body){let o={};if(body!==undefined)o={method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};let r=await fetch(path,o);let j=await r.json();if(!r.ok)throw Error(j.error||r.statusText);return j}
async function loadState(){state=await api('/api/state');canvas.width=state.camera.width;canvas.height=state.camera.height;canvas.style.aspectRatio=`${state.camera.width}/${state.camera.height}`;exposure.max=Math.max(4,Math.floor(9000/state.camera.fps));connection.textContent=`已连接 ${state.camera.width}x${state.camera.height}@${state.camera.fps}  解码 ${state.metrics.decode_fps.toFixed(1)}fps  识别 ${state.metrics.vision_fps.toFixed(1)}fps  耗时 ${state.metrics.vision_ms.toFixed(2)}ms`;let s=classSelect;if(!s.options.length){state.classes.forEach(c=>s.add(new Option(`${c.index}. ${c.name} [${c.key}]`,c.key)))}s.value=state.selected;renderForm()}
function activeShapes(){return shapes.filter(d=>state.profile.kind==='core_black'||(!d[1].includes('contrast')&&!d[1].includes('vertices')))}
function renderForm(){let p=state.profile,c=state.classes.find(x=>x.key===state.selected);classSelect.innerHTML='';state.classes.forEach(x=>classSelect.add(new Option(`${x.index}. ${x.name} [${x.key}]`,x.key)));classSelect.value=state.selected;referenceSelect.innerHTML='';state.references.forEach(x=>referenceSelect.add(new Option(`${x.index}. ${x.name}`,x.index)));referenceSelect.value=state.selected_reference;referenceName.value=state.selected_reference===0?'基础参考':(p.reference_name||`参考${state.selected_reference}`);referenceName.disabled=state.selected_reference===0;displayName.value=c.name;groupIndex.value=c.index;groupIndex.max=state.classes.length;groupBadge.textContent=`组号 ${c.index}`;keyBadge.textContent=`内部键 ${state.selected} · ${state.references.length}组参考`;fusion.value=p.fusion;useShape.checked=p.candidate.use_shape!==false;useSize.checked=p.candidate.use_size!==false;ruleText.innerHTML=state.rule_description.map(x=>`<div>• ${x}</div>`).join('');colorControls.innerHTML=defs.map((d,i)=>control('c'+i,d[0],d[3],d[4],d[5],p[d[1]][d[2]])).join('');shapeControls.innerHTML=activeShapes().map((d,i)=>control('s'+i,d[0],d[2],d[3],d[4],getPath(p,d[1])??0)).join('');trackControls.innerHTML=tracks.map((d,i)=>control('t'+i,d[0],d[2],d[3],d[4],getPath(p,d[1]))).join('')}
function formProfile(){let p=structuredClone(state.profile);p.display_name=displayName.value;if(state.selected_reference>0)p.reference_name=referenceName.value.trim()||`参考${state.selected_reference}`;p.fusion=fusion.value;p.candidate.use_shape=useShape.checked;p.candidate.use_size=useSize.checked;defs.forEach((d,i)=>p[d[1]][d[2]]=Number(document.getElementById('c'+i+'n').value));activeShapes().forEach((d,i)=>setPath(p,d[1],Number(document.getElementById('s'+i+'n').value)));tracks.forEach((d,i)=>setPath(p,d[1],Number(document.getElementById('t'+i+'n').value)));return p}
async function applyAll(){let j=await api('/api/profile',{profile:formProfile()});state=j.state;document.getElementById('connection').textContent='参数已应用（未保存）';renderForm()}
async function applyGroupMeta(){let j=await api('/api/group',{display_name:displayName.value,index:Number(groupIndex.value)});state=j.state;renderForm();connection.textContent='组号/组名已修改（未保存）'}
async function selectClass(){state=(await api('/api/select',{class_name:classSelect.value})).state;rect=null;renderForm()}
async function selectReference(){state=(await api('/api/reference/select',{index:Number(referenceSelect.value)})).state;rect=null;renderForm()}
async function addReference(){let name=prompt('新参考名称，例如：侧视、斜视、远距离','侧视');if(name===null)return;state=(await api('/api/reference/add',{name:name})).state;renderForm();connection.textContent='已复制当前参数为新参考，请框选新角度目标后自动取值'}
async function deleteReference(){if(state.selected_reference===0){alert('基础参考不能删除');return}if(!confirm('确定删除当前参考阈值？'))return;state=(await api('/api/reference/delete',{})).state;renderForm()}
async function saveConfig(){await applyAll();await api('/api/save',{});connection.textContent='配置已保存到JSON'}
async function toggleFreeze(){let j=await api('/api/freeze',{});frozen=j.frozen;freezeBtn.textContent=frozen?'恢复实时画面':'冻结画面'}async function setView(v){view=v;await api('/api/view',{view:v})}
async function autoSample(){if(!rect){alert('请先在目标画面上拖出矩形');return}let j=await api('/api/sample',{rectangle:rect});state=j.state;sampleResult.textContent=JSON.stringify(j.result,null,2);renderForm()}
async function diagnose(){let j=await api('/api/diagnose',{rectangle:rect});diagnosis.textContent=JSON.stringify(j,null,2);if(j.suggested_exposure!=null)exposure.value=j.suggested_exposure}
async function applyCamera(){let j=await api('/api/camera',{exposure:Number(exposure.value),white_balance:Number(wb.value),focus:Number(focus.value)});diagnosis.textContent=j.output+'\n重新点击“分析当前画面”验证。'}
async function autoWhiteBalance(){if(!rect){alert('请先框住占满选区的哑光灰卡');return}diagnosis.textContent='正在按灰卡区域扫描白平衡，约需3秒...';let j=await api('/api/auto-white-balance',{rectangle:rect});wb.value=j.white_balance_temperature;diagnosis.textContent=JSON.stringify(j,null,2)}
async function autoFocus(){if(!rect){alert('请先框住600～800mm处带清晰边缘的目标');return}diagnosis.textContent='正在粗扫和精扫焦距，约需3秒...';let j=await api('/api/auto-focus',{rectangle:rect});focus.value=j.best_focus;diagnosis.textContent=JSON.stringify(j,null,2)}
function pos(e){let r=canvas.getBoundingClientRect();return [Math.round((e.clientX-r.left)*canvas.width/r.width),Math.round((e.clientY-r.top)*canvas.height/r.height)]}canvas.onmousedown=e=>{dragging=true;start=pos(e);rect=null};canvas.onmousemove=e=>{if(dragging){let q=pos(e);rect=[start[0],start[1],q[0],q[1]]}};canvas.onmouseup=e=>{dragging=false;let q=pos(e);rect=[start[0],start[1],q[0],q[1]]};
function loop(){let started=performance.now(),interval=1000/state.preview.fps;image.onload=()=>{ctx.imageSmoothingEnabled=view!=='mask';ctx.drawImage(image,0,0,canvas.width,canvas.height);if(rect){ctx.strokeStyle='#ffff00';ctx.lineWidth=3;ctx.strokeRect(rect[0],rect[1],rect[2]-rect[0],rect[3]-rect[1])}setTimeout(loop,Math.max(0,interval-(performance.now()-started)))};image.onerror=()=>setTimeout(loop,250);image.src=`/image/current.jpg?t=${Date.now()}`}loadState().then(loop);setInterval(async()=>{try{let x=await api('/api/metrics');connection.textContent=`已连接 ${state.camera.width}x${state.camera.height}@${state.camera.fps}  解码 ${x.decode_fps.toFixed(1)}fps  识别 ${x.vision_fps.toFixed(1)}fps  耗时 ${x.vision_ms.toFixed(2)}ms  预览 ${x.preview_ms.toFixed(2)}ms/${x.preview_kb.toFixed(0)}KB  帧龄 ${x.frame_age_ms.toFixed(2)}ms`}catch(e){connection.textContent='连接断开'}},1000);
</script></body></html>'''


def nested_merge(target: dict[str, Any], source: dict[str, Any]) -> None:
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            nested_merge(target[key], value)
        else:
            target[key] = value


class Runtime:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.args.device = resolve_camera_device(args.device)
        self.config = load_config(args.config)
        camera_config = self.config.setdefault("camera", {})
        self.width = int(args.width or camera_config.get("width", 1280))
        self.height = int(args.height or camera_config.get("height", 720))
        self.camera_fps = int(args.camera_fps or camera_config.get("fps", 180))
        camera_config.update({"width": self.width, "height": self.height, "fps": self.camera_fps})
        self.localizer = GroundLocalizer.load(args.homography, (self.width, self.height))
        self.detector = TraditionalDetector(self.config, self.localizer)
        self.camera = LatestFrameCamera(
            args.device, self.width, self.height, self.camera_fps,
            decoder=args.decoder, decode_fps=args.decode_fps,
        )
        self.lock = threading.RLock()
        self.selected = next(iter(self.config["classes"]))
        self.selected_reference = 0
        self.running = True
        self.frozen = False
        self.frozen_frame: np.ndarray | None = None
        self.image: bytes = b""
        self.active_view = "original"
        self.latest_detections = []
        self.latest_valid_mask: np.ndarray | None = None
        self.latest_detection_class = self.selected
        self.preview_thread: threading.Thread | None = None
        self.metrics = {"decode_fps": 0.0, "vision_fps": 0.0, "vision_ms": 0.0, "preview_ms": 0.0, "preview_kb": 0.0, "frame_age_ms": 0.0}
        self.last_raw: np.ndarray | None = None
        self.startup_event = threading.Event()
        self.startup_error: str | None = None

    def rule_description(self) -> list[str]:
        profile = self.active_profile()
        rules = profile["candidate"]
        result = ["掩膜：颜色分割只是候选；白色最终掩膜只保留同时通过面积、形状、评分等规则的目标，小色块和不合格候选保持黑色。"]
        if rules.get("use_shape", True):
            result.append("形状：启用长宽比、矩形填充率、轮廓实心度硬过滤，并参与置信度评分。")
        else:
            result.append("形状：已关闭，仅颜色容易受到同色物体干扰。")
        if profile.get("kind") == "core_black":
            result.append("黑色核心物资：额外使用候选与周围背景的亮度对比、多边形顶点数，抑制阴影和场地缝隙。")
        if rules.get("use_size", True):
            result.append("尺寸：完成地面单应标定后使用毫米尺寸；未标定时自动跳过尺寸评分。")
        confirmation = self.config["classes"][self.selected]["confirmation"]
        result.append(f"多帧：命中{confirmation['min_hits']}帧后确认，允许丢失{confirmation['max_misses']}帧。")
        return result

    def active_profile(self) -> dict[str, Any]:
        base = self.config["classes"][self.selected]
        if self.selected_reference == 0:
            return base
        references = base.setdefault("references", [])
        index = self.selected_reference - 1
        if not 0 <= index < len(references):
            self.selected_reference = 0
            return base
        return references[index]

    def state(self) -> dict[str, Any]:
        with self.lock:
            classes = [{"index": index + 1, "key": key, "name": value.get("display_name", key)} for index, (key, value) in enumerate(self.config["classes"].items())]
            base = self.config["classes"][self.selected]
            references = [{"index": 0, "name": "基础参考"}] + [
                {"index": index, "name": item.get("reference_name", f"参考{index}")}
                for index, item in enumerate(base.get("references", []), start=1)
            ]
            preview = {"width": self.args.preview_width, "height": self.args.preview_height, "fps": self.args.web_fps, "quality": self.args.preview_quality}
            return {"selected": self.selected, "selected_reference": self.selected_reference, "references": references, "classes": classes, "profile": self.active_profile(), "rule_description": self.rule_description(), "calibrated": self.localizer.calibrated, "camera": {"width": self.width, "height": self.height, "fps": self.camera_fps}, "preview": preview, "metrics": dict(self.metrics)}

    @staticmethod
    def _jpeg(image: np.ndarray, quality: int) -> bytes:
        ok, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return data.tobytes() if ok else b""

    def loop(self) -> None:
        try:
            self.camera.start()
        except Exception as error:
            self.startup_error = str(error)
            self.running = False
            self.startup_event.set()
            return
        self.startup_event.set()
        self.preview_thread = threading.Thread(target=self.preview_loop, daemon=True)
        self.preview_thread.start()
        report = time.perf_counter(); decoded0 = vision_count = 0
        while self.running:
            error = self.camera.check_error()
            if error:
                print(error)
                self.running = False
                break
            packet = self.camera.latest()
            if packet is None:
                time.sleep(0.002); continue
            with self.lock:
                if self.frozen:
                    if self.frozen_frame is None: self.frozen_frame = packet.image.copy()
                    frame = self.frozen_frame.copy()
                else:
                    self.frozen_frame = None; frame = packet.image.copy()
                selected = self.selected; selected_reference = self.selected_reference
                active_view = self.active_view
                runtime = self.config["runtime"]
                if self.args.vision_fps is not None:
                    target_fps = self.args.vision_fps
                elif selected in runtime.get("danger_classes", []):
                    target_fps = float(runtime.get("danger_fps", 120))
                elif selected in runtime.get("zone_classes", []):
                    target_fps = float(runtime.get("zone_fps", 30))
                else:
                    target_fps = float(runtime.get("material_fps", 90))
                period = 1.0 / max(target_fps, 1.0)
            started = time.perf_counter()
            detections, debug = self.detector.detect(
                frame,
                [selected],
                collect_rejected=False,
                reference_index=selected_reference,
                collect_debug=active_view == "mask",
            )
            cost = (time.perf_counter() - started) * 1000.0
            with self.lock:
                self.last_raw = frame
                self.metrics["vision_ms"] = cost
                self.metrics["frame_age_ms"] = (time.monotonic_ns() - packet.published_ns) / 1_000_000.0
                self.latest_detections = detections
                self.latest_detection_class = selected
                if active_view == "mask":
                    self.latest_valid_mask = debug["valid_masks"].get(selected)
            vision_count += 1
            now = time.perf_counter()
            if now - report >= 1.0:
                decoded = self.camera.decoded_count(); elapsed = now - report
                with self.lock:
                    self.metrics["decode_fps"] = (decoded - decoded0) / elapsed; self.metrics["vision_fps"] = vision_count / elapsed
                decoded0, vision_count, report = decoded, 0, now
            sleep = period - (time.perf_counter() - started)
            if sleep > 0: time.sleep(sleep)
        self.camera.stop()
        if self.preview_thread is not None:
            self.preview_thread.join(timeout=2)

    def preview_loop(self) -> None:
        period = 1.0 / self.args.web_fps
        next_preview = time.perf_counter()
        while self.running:
            now = time.perf_counter()
            if now < next_preview:
                time.sleep(min(next_preview - now, 0.005))
                continue
            next_preview = max(next_preview + period, now)
            packet = self.camera.latest()
            if packet is None:
                time.sleep(0.005)
                continue
            with self.lock:
                active_view = self.active_view
                frozen_frame = None if self.frozen_frame is None else self.frozen_frame.copy()
                detections = list(self.latest_detections)
                detection_class = self.latest_detection_class
                valid_mask = None if self.latest_valid_mask is None else self.latest_valid_mask.copy()
            source = frozen_frame if frozen_frame is not None else packet.image
            if active_view == "mask":
                mask = valid_mask if valid_mask is not None else np.zeros(source.shape[:2], np.uint8)
                rendered = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                interpolation = cv2.INTER_NEAREST
            elif active_view == "annotated":
                rendered = source.copy()
                for detection in detections:
                    x, y, width, height = detection.bbox
                    cv2.rectangle(rendered, (x, y), (x + width, y + height), (0, 255, 0), 2)
                    cv2.putText(rendered, f"{detection_class} {detection.confidence:.2f}", (x, max(15, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
                interpolation = cv2.INTER_AREA
            else:
                rendered = source
                interpolation = cv2.INTER_AREA
            preview_started = time.perf_counter()
            preview = cv2.resize(rendered, (self.args.preview_width, self.args.preview_height), interpolation=interpolation)
            encoded = self._jpeg(preview, self.args.preview_quality)
            with self.lock:
                self.image = encoded
                self.metrics["preview_ms"] = (time.perf_counter() - preview_started) * 1000.0
                self.metrics["preview_kb"] = len(encoded) / 1024.0

    def get_camera_control(self, name: str) -> int | None:
        try:
            output = subprocess.check_output(["v4l2-ctl", "-d", self.args.device, f"--get-ctrl={name}"], text=True, stderr=subprocess.STDOUT)
            return int(output.strip().split(":")[-1].strip().split()[0])
        except Exception:
            return None

    def set_camera(self, values: dict[str, Any]) -> str:
        max_exposure = max(4, int(9000 / max(self.camera_fps, 1)))
        exposure = max(4, min(max_exposure, int(values["exposure"])))
        white_balance = max(2800, min(6500, int(values["white_balance"])))
        focus = max(0, min(1023, int(values["focus"])))
        commands = {"auto_exposure": 1, "exposure_time_absolute": exposure, "white_balance_automatic": 0, "white_balance_temperature": white_balance, "focus_automatic_continuous": 0, "focus_absolute": focus}
        output = []
        for key, value in commands.items():
            result = subprocess.run(["v4l2-ctl", "-d", self.args.device, f"--set-ctrl={key}={value}"], text=True, capture_output=True)
            if result.returncode: raise RuntimeError(result.stderr or result.stdout)
            output.append(f"{key}={value}")
        return "摄像头参数已写入：" + ", ".join(output)

    def set_camera_control(self, name: str, value: int) -> None:
        result = subprocess.run(
            ["v4l2-ctl", "-d", self.args.device, f"--set-ctrl={name}={int(value)}"],
            text=True,
            capture_output=True,
        )
        if result.returncode:
            raise RuntimeError((result.stderr or result.stdout).strip())

    @staticmethod
    def crop_rectangle(frame: np.ndarray, rectangle: Any) -> np.ndarray:
        if not rectangle or len(rectangle) != 4:
            return frame
        x1, y1, x2, y2 = map(int, rectangle)
        x1, x2 = sorted((max(0, x1), min(frame.shape[1], x2)))
        y1, y2 = sorted((max(0, y1), min(frame.shape[0], y2)))
        if x2 - x1 < 12 or y2 - y1 < 12:
            raise ValueError("框选区域太小，请重新框选")
        return frame[y1:y2, x1:x2]

    def auto_white_balance(self, rectangle: Any) -> dict[str, Any]:
        self.set_camera_control("white_balance_automatic", 0)

        def scan(values: list[int]) -> list[tuple[float, int, list[float]]]:
            scores: list[tuple[float, int, list[float]]] = []
            for value in values:
                self.set_camera_control("white_balance_temperature", value)
                time.sleep(0.09)
                packet = self.camera.latest()
                if packet is None:
                    continue
                roi = self.crop_rectangle(packet.image, rectangle)
                means = [float(v) for v in cv2.mean(roi)[:3]]
                imbalance = float(np.std(means) / max(np.mean(means), 1.0))
                scores.append((imbalance, value, means))
            return scores

        coarse = scan(list(range(2800, 6501, 250)))
        if not coarse:
            raise RuntimeError("白平衡扫描期间没有取得图像")
        coarse_best = min(coarse)[1]
        fine = scan(list(range(max(2800, coarse_best - 250), min(6500, coarse_best + 250) + 1, 50)))
        imbalance, temperature, means = min(coarse + fine)
        self.set_camera_control("white_balance_temperature", temperature)
        return {
            "white_balance_temperature": temperature,
            "gray_card_mean_bgr": [round(value, 2) for value in means],
            "channel_imbalance": round(imbalance, 4),
            "mode": "manual_locked",
            "advice": "已选择使框选灰卡B/G/R均值最接近的具体色温并锁定。框内必须主要是中性灰卡；光源变化后重新执行。",
        }

    @staticmethod
    def sharpness_score(frame: np.ndarray) -> float:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())

    def auto_focus(self, rectangle: Any) -> dict[str, Any]:
        self.set_camera_control("focus_automatic_continuous", 0)
        def scan(values: list[int]) -> list[tuple[float, int]]:
            scores: list[tuple[float, int]] = []
            for value in values:
                self.set_camera_control("focus_absolute", value)
                time.sleep(0.09)
                packet = self.camera.latest()
                if packet is None:
                    continue
                roi = self.crop_rectangle(packet.image, rectangle)
                scores.append((self.sharpness_score(roi), value))
            return scores

        coarse_values = list(range(0, 1024, 64))
        coarse = scan(coarse_values)
        if not coarse:
            raise RuntimeError("自动对焦期间没有取得图像")
        coarse_best = max(coarse)[1]
        fine_values = list(range(max(0, coarse_best - 64), min(1023, coarse_best + 64) + 1, 8))
        results = coarse + scan(fine_values)
        best_score, best_focus = max(results)
        self.set_camera_control("focus_absolute", best_focus)
        return {
            "best_focus": best_focus,
            "best_sharpness": round(best_score, 2),
            "samples": len(results),
            "mode": "manual_locked",
            "advice": "焦距已按框选目标的拉普拉斯清晰度粗扫、精扫并锁定。车和相机位置固定后不要再开连续自动对焦。",
        }


class Handler(BaseHTTPRequestHandler):
    runtime: Runtime
    def log_message(self, fmt, *args):
        return
    def send_json(self, value: Any, status=200):
        data = json.dumps(value, ensure_ascii=False).encode(); self.send_response(status); self.send_header("Content-Type", "application/json; charset=utf-8"); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)
    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            data = HTML.encode(); self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data); return
        if path == "/api/state": self.send_json(self.runtime.state()); return
        if path == "/api/metrics": self.send_json(dict(self.runtime.metrics)); return
        if path.startswith("/image/"):
            with self.runtime.lock: data = self.runtime.image
            self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.send_header("Cache-Control", "no-store"); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data); return
        self.send_error(404)
    def body(self):
        return json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))) or b"{}")
    def do_POST(self):
        try:
            path=urlparse(self.path).path; body=self.body(); r=self.runtime
            if path == "/api/select":
                with r.lock:
                    if body["class_name"] not in r.config["classes"]: raise ValueError("未知类别")
                    r.selected=body["class_name"]; r.selected_reference=0
                    r.latest_detections=[]; r.latest_valid_mask=None
                self.send_json({"state":r.state()}); return
            if path == "/api/reference/select":
                with r.lock:
                    index = int(body.get("index", 0)); count = len(r.config["classes"][r.selected].get("references", []))
                    if index < 0 or index > count: raise ValueError("未知参考阈值")
                    r.selected_reference = index
                self.send_json({"state":r.state()}); return
            if path == "/api/reference/add":
                with r.lock:
                    base = r.config["classes"][r.selected]
                    source = copy.deepcopy(r.active_profile())
                    source.pop("references", None)
                    source["reference_name"] = str(body.get("name", "")).strip() or f"参考{len(base.get('references', [])) + 1}"
                    base.setdefault("references", []).append(source)
                    r.selected_reference = len(base["references"])
                    r.detector.update_config(r.config)
                self.send_json({"state":r.state()}); return
            if path == "/api/reference/delete":
                with r.lock:
                    if r.selected_reference == 0: raise ValueError("基础参考不能删除")
                    base = r.config["classes"][r.selected]
                    base.setdefault("references", []).pop(r.selected_reference - 1)
                    r.selected_reference = 0; r.detector.update_config(r.config)
                self.send_json({"state":r.state()}); return
            if path == "/api/profile":
                with r.lock:
                    base = r.config["classes"][r.selected]
                    updated = body["profile"]
                    if r.selected_reference == 0:
                        references = base.get("references", [])
                        updated["references"] = references
                        r.config["classes"][r.selected] = updated
                    else:
                        base["confirmation"] = copy.deepcopy(updated.get("confirmation", base["confirmation"]))
                        r.config["classes"][r.selected].setdefault("references", [])[r.selected_reference - 1] = updated
                    r.detector.update_config(r.config)
                self.send_json({"state":r.state()}); return
            if path == "/api/group":
                with r.lock:
                    profile=r.config["classes"][r.selected]
                    profile["display_name"]=str(body.get("display_name",r.selected)).strip() or r.selected
                    items=list(r.config["classes"].items()); current=next(i for i,item in enumerate(items) if item[0]==r.selected)
                    item=items.pop(current); target=max(0,min(len(items),int(body.get("index",current+1))-1)); items.insert(target,item)
                    r.config["classes"]=dict(items); r.detector.update_config(r.config)
                self.send_json({"state":r.state()}); return
            if path == "/api/sample":
                with r.lock:
                    if r.last_raw is None: raise ValueError("尚无摄像头图像")
                    result=auto_sample_profile(r.last_raw, tuple(map(int,body["rectangle"])), r.active_profile(), r.config.get("threshold_reference_resolution")); r.detector.update_config(r.config)
                self.send_json({"result":result,"state":r.state()}); return
            if path == "/api/freeze":
                with r.lock: r.frozen=not r.frozen
                self.send_json({"frozen":r.frozen}); return
            if path == "/api/view":
                view = str(body.get("view", "original"))
                if view not in {"original", "mask", "annotated"}: raise ValueError("未知画面类型")
                with r.lock:
                    r.active_view = view
                    if view == "mask": r.latest_valid_mask = None
                self.send_json({"view":view}); return
            if path == "/api/save": save_config(r.args.config,r.config); self.send_json({"ok":True}); return
            if path == "/api/diagnose":
                with r.lock:
                    if r.last_raw is None: raise ValueError("尚无图像")
                    frame = r.last_raw.copy()
                roi = r.crop_rectangle(frame, body.get("rectangle"))
                max_exposure = max(4, int(9000 / max(r.camera_fps, 1)))
                result=diagnose_frame(roi,r.get_camera_control("exposure_time_absolute"),max_exposure)
                self.send_json(result); return
            if path == "/api/camera": self.send_json({"output":r.set_camera(body)}); return
            if path == "/api/auto-white-balance": self.send_json(r.auto_white_balance(body.get("rectangle"))); return
            if path == "/api/auto-focus": self.send_json(r.auto_focus(body.get("rectangle"))); return
            self.send_error(404)
        except Exception as exc: self.send_json({"error":str(exc)},400)


def main() -> int:
    root=Path(__file__).resolve().parent; p=argparse.ArgumentParser(description="SSH友好的救援视觉Web编辑器")
    p.add_argument("--device",default="/dev/video0"); p.add_argument("--width",type=int); p.add_argument("--height",type=int); p.add_argument("--camera-fps",type=int); p.add_argument("--decoder",choices=("jpu","software"),default="jpu"); p.add_argument("--decode-fps",type=float,default=60); p.add_argument("--vision-fps",type=float); p.add_argument("--preview-width",type=int,default=640); p.add_argument("--preview-height",type=int,default=512); p.add_argument("--preview-quality",type=int,default=72); p.add_argument("--web-fps",type=float,default=10); p.add_argument("--host",default="127.0.0.1"); p.add_argument("--port",type=int,default=8080)
    p.add_argument("--config",default=str(root/"config"/"rescue_vision.json")); p.add_argument("--homography",default=str(root/"config"/"homography.txt")); args=p.parse_args()
    if not 8 <= args.web_fps <= 12: p.error("--web-fps必须在8..12之间")
    if not 70 <= args.preview_quality <= 75: p.error("--preview-quality必须在70..75之间")
    if args.preview_width <= 0 or args.preview_height <= 0: p.error("预览宽高必须大于0")
    runtime=Runtime(args)
    Handler.runtime=runtime
    thread=threading.Thread(target=runtime.loop,daemon=True)
    thread.start()
    if not runtime.startup_event.wait(timeout=8):
        runtime.running=False
        thread.join(timeout=3)
        raise RuntimeError("等待摄像头启动超时")
    if runtime.startup_error:
        thread.join(timeout=3)
        raise RuntimeError(f"摄像头启动失败：{runtime.startup_error}")
    server=ThreadingHTTPServer((args.host,args.port),Handler)
    def stop(_s,_f):
        runtime.running=False
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    def monitor_runtime():
        while runtime.running:
            time.sleep(0.1)
        if not getattr(server, "_BaseServer__shutdown_request", False):
            server.shutdown()
    threading.Thread(target=monitor_runtime,daemon=True).start()
    print(f"摄像头：{runtime.args.device}")
    print(f"Web编辑器已启动：http://{args.host}:{args.port}"); print(f"SSH端口转发：ssh -L {args.port}:127.0.0.1:{args.port} root@RDK_IP，然后电脑浏览器打开 http://127.0.0.1:{args.port}")
    try: server.serve_forever()
    finally: runtime.running=False; thread.join(timeout=3); server.server_close()
    return 0
if __name__=="__main__": raise SystemExit(main())
