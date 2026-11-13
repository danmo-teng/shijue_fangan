"""Opt-in observations for the independent YOLO benchmark, never formal execution.

Wrappers delegate to the unchanged camera, SDK, decoder and queue operations.
No affinity, scheduler, throttle, queue capacity or sample acceptance is changed.
"""
from __future__ import annotations

import csv
from functools import partial
import json
import os
from pathlib import Path
import threading
import time
import zlib

import numpy as np

from rescue_vision.camera import LatestFrameCamera
from rescue_vision.yolo26 import X5Yolo26
from rescue_vision import yolo26_pipeline as pipeline_module

FORMAL_WORKER = pipeline_module._worker


def write_csv(path, rows):
    if rows:
        with Path(path).open('w') as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def percentiles(values):
    return dict(zip(('p50', 'p95', 'p99', 'max'), map(float, np.percentile(values, [50, 95, 99, 100])))) if len(values) else None


class ObservedCamera(LatestFrameCamera):
    def __init__(self, *args, **kwargs):
        self.capture_events, self.decode_events = [], []
        self.last_decode = {}
        super().__init__(*args, **kwargs)
        original_sink = self.sink

        class SinkProbe:
            metadata = {}

            def emit(self, name, *args):
                sample = original_sink.emit(name, *args)
                if sample is not None and name == 'pull-sample':
                    buffer = sample.get_buffer()
                    self.metadata = dict(gst_offset=int(buffer.offset), gst_offset_end=int(buffer.offset_end),
                        gst_pts_ns=int(buffer.pts), gst_duration_ns=int(buffer.duration))
                return sample

        self.source_probe = SinkProbe()

    @property
    def _latest(self):
        return self._observed_latest

    @_latest.setter
    def _latest(self, packet):
        self._observed_latest = packet
        if packet is not None:
            self.decode_events.append(dict(frame_id=packet.frame_id, capture_s=packet.published_ns/1e9,
                decoded_ready_s=time.monotonic(), sampled_luma_crc32=zlib.crc32(
                    np.ascontiguousarray(packet.image[:1024:16, ::16])), **self.last_decode))

    def _on_sample(self, sink):
        result = super()._on_sample(self.source_probe)
        with self._lock:
            compressed = self._compressed
        if compressed is not None:
            frame_id, _, captured_ns = compressed
            if not self.capture_events or self.capture_events[-1]['capture_id'] != frame_id:
                self.capture_events.append(dict(capture_id=frame_id, capture_s=captured_ns/1e9,
                    **self.source_probe.metadata))
        return result

    def start(self):
        # start() constructs the sole decoder. Only this benchmark's Python
        # process sees the temporary factory; the actual decoding is delegated.
        from rescue_vision import jpu
        original = jpu.JpuDecoder
        camera = self

        class DecoderProbe:
            def __init__(self, *args, **kwargs):
                self.decoder = original(*args, **kwargs)

            def __getattr__(self, name):
                return getattr(self.decoder, name)

            def decode_nv12(self, jpeg):
                begin, cpu = time.monotonic(), time.thread_time()
                image = self.decoder.decode_nv12(jpeg)
                camera.last_decode = dict(decode_start_s=begin, decode_end_s=time.monotonic(),
                    decode_cpu_ms=(time.thread_time()-cpu)*1000)
                return image

        jpu.JpuDecoder = DecoderProbe
        try:
            return super().start()
        finally:
            jpu.JpuDecoder = original


class ObservedDetector(X5Yolo26):
    def run(self, tensor):
        begin, cpu = time.monotonic(), time.thread_time()
        outputs = super().run(tensor)
        self.runtime_observation = dict(profile_runtime_start_s=begin, profile_runtime_end_s=time.monotonic(),
            profile_runtime_cpu_ms=(time.thread_time()-cpu)*1000)
        return outputs

    def decode(self, outputs):
        cpu = time.thread_time()
        result = super().decode(outputs)
        self.decode_cpu_ms = (time.thread_time()-cpu)*1000
        return result

    def _observe(self, function, *args):
        begin, cpu = time.monotonic(), time.thread_time()
        detections, timing = function(*args)
        timing.update(profile_infer_begin_s=begin, profile_infer_end_s=time.monotonic(),
            profile_infer_cpu_ms=(time.thread_time()-cpu)*1000,
            profile_decode_cpu_ms=self.decode_cpu_ms, **self.runtime_observation)
        return detections, timing

    def infer_nv12(self, *args):
        return self._observe(super().infer_nv12, *args)

    def infer(self, *args):
        return self._observe(super().infer, *args)


def observed_worker(*args, directory):
    from rescue_vision import yolo26
    calls, completions = [], []
    request_queue, result_queue = args[2], args[3]

    class Requests:
        last = None

        def get(self, *a, **kw):
            request = request_queue.get(*a, **kw)
            return self.record(request, 'blocking_get')

        def get_nowait(self):
            return self.record(request_queue.get_nowait(), 'drain_latest')

        def record(self, request, method):
            self.last = dict(frame_id=request[2], dequeue_s=time.monotonic(), method=method)
            calls.append(self.last)
            return request

    requests = Requests()

    class Results:
        def put(self, item):
            request, detections, timing = item
            begin = time.monotonic()
            timing.update(profile_dequeue_s=requests.last['dequeue_s'], profile_put_start_s=begin)
            result = result_queue.put(item)
            completions.append(dict(frame_id=request[2], put_begin_s=begin, put_end_s=time.monotonic()))
            return result

    original_detector = yolo26.X5Yolo26
    yolo26.X5Yolo26 = ObservedDetector
    delegated = list(args)
    delegated[2], delegated[3] = requests, Results()
    try:
        FORMAL_WORKER(*delegated)
    finally:
        yolo26.X5Yolo26 = original_detector
        Path(directory, 'profile_worker.json').write_text(json.dumps(dict(dequeues=calls, completions=completions)))


class ObservedPipeline(pipeline_module.Yolo26Pipeline):
    def __init__(self, *args, directory, **kwargs):
        self.submissions, self.receptions = [], []
        self.receive_calls = self.empty_receives = 0
        original = pipeline_module._worker
        pipeline_module._worker = partial(observed_worker, directory=str(directory))
        try:
            super().__init__(*args, **kwargs)
        finally:
            pipeline_module._worker = original

    def submit(self, packet):
        begin = time.monotonic()
        accepted = super().submit(packet)
        self.submissions.append(dict(frame_id=packet.frame_id, submit_begin_s=begin,
            submit_end_s=time.monotonic(), accepted=accepted))
        return accepted

    def receive(self):
        begin = time.monotonic()
        result = super().receive()
        self.receive_calls += 1
        if result is None:
            self.empty_receives += 1
        else:
            end = time.monotonic()
            packet, _, timing = result
            timing.update(profile_receive_begin_s=begin, profile_receive_end_s=end)
            self.receptions.append(dict(frame_id=packet.frame_id, receive_begin_s=begin, receive_end_s=end))
        return result


class ResourceSampler:
    """1 Hz proc/sysfs reader; percentages use 100% per fully occupied CPU."""

    def __init__(self, roles, directory):
        self.roles, self.directory = dict(roles), Path(directory)
        self.samples, self.previous = [], {}
        self.thread_snapshots = {}
        self.stop = threading.Event()
        self.ticks = os.sysconf('SC_CLK_TCK')
        self.page_bytes = os.sysconf('SC_PAGE_SIZE')
        thermal = Path('/sys/class/thermal/thermal_zone1/temp')
        self.thermal_paths = [thermal] if thermal.exists() else []
        self.thread = threading.Thread(target=self._loop, name='benchmark-proc-sampler', daemon=True)

    def start(self):
        self.thread_snapshots['start'] = self.snapshot_threads()
        self.thread.start()

    def snapshot_threads(self):
        now = time.monotonic()
        return dict(monotonic_s=now, roles={name:{thread.name:self.stat(thread,now)
            for thread in (Path('/proc')/str(pid)).glob('task/*')} for name,pid in self.roles.items()})

    @staticmethod
    def read(path):
        try:
            return Path(path).read_text().strip()
        except OSError:
            return None

    def stat(self, path, now):
        raw = self.read(path/'stat')
        if raw is None:
            return None
        parts = raw[raw.rfind(')')+2:].split()
        ticks = int(parts[11])+int(parts[12])
        old = self.previous.get(str(path))
        result = dict(comm=raw[raw.find('(')+1:raw.rfind(')')], state=parts[0],
            cpu_percent=None if old is None else (ticks-old[1])/self.ticks/(now-old[0])*100,
            rss_bytes=int(parts[21])*self.page_bytes, last_cpu=int(parts[36]))
        self.previous[str(path)] = (now, ticks)
        sched = self.read(path/'schedstat')
        if sched:
            values = list(map(int, sched.split()))
            key = str(path)+'sched'
            previous = self.previous.get(key)
            result['runqueue_wait_ms'] = None if previous is None else (values[1]-previous[1])/1e6
            self.previous[key] = (now, values[1])
        return result

    def _loop(self):
        while not self.stop.is_set():
            begin, cpu = time.monotonic(), time.thread_time()
            sample = dict(monotonic_s=begin, roles={}, cpus={}, bpu={}, thermal_millideg={})
            for name, pid in self.roles.items():
                path = Path('/proc')/str(pid)
                process = self.stat(path, begin)
                if process is None:
                    continue
                try:
                    process['affinity'] = sorted(os.sched_getaffinity(pid))
                except (ProcessLookupError, PermissionError):
                    pass
                sample['roles'][name] = process
            for line in (self.read('/proc/stat') or '').splitlines():
                fields = line.split()
                if fields and fields[0].startswith('cpu'):
                    values = list(map(int, fields[1:9]))
                    total, idle = sum(values), values[3]+values[4]
                    key = fields[0]
                    old = self.previous.get(key)
                    sample['cpus'][key] = None if old is None or total==old[0] else (1-(idle-old[1])/(total-old[0]))*100
                    self.previous[key] = (total, idle)
            for name, path in dict(utilization_percent='/sys/devices/system/bpu/bpu0/ratio',
                    frequency_hz='/sys/class/devfreq/3a000000.bpu/cur_freq',
                    cpu_frequency_khz='/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq').items():
                value = self.read(path)
                sample['bpu'][name] = int(value) if value and value.isdigit() else value
            for path in self.thermal_paths:
                sample['thermal_millideg'][self.read(path.with_name('type')) or path.parent.name] = self.read(path)
            sample['sampler_wall_ms'] = (time.monotonic()-begin)*1000
            sample['sampler_cpu_ms'] = (time.thread_time()-cpu)*1000
            self.samples.append(sample)
            self.stop.wait(max(0, 1-(time.monotonic()-begin)))

    def close(self):
        self.stop.set()
        self.thread.join(3)
        self.thread_snapshots['end'] = self.snapshot_threads()
        self.directory.mkdir(exist_ok=True, parents=True)
        (self.directory/'profile_resources.json').write_text(json.dumps(self.samples, indent=2))
        (self.directory/'profile_threads.json').write_text(json.dumps(self.thread_snapshots, indent=2))


def finish_profile(directory, camera, detector, rows, measure_start, end):
    directory = Path(directory)
    if camera is None or detector is None:
        return {}
    captures = camera.capture_events
    decoded = camera.decode_events
    submissions = detector.submissions
    write_csv(directory/'profile_captures.csv', captures)
    write_csv(directory/'profile_decoded.csv', decoded)
    write_csv(directory/'profile_submissions.csv', submissions)
    write_csv(directory/'profile_receptions.csv', detector.receptions)
    worker_path = directory/'profile_worker.json'
    worker = json.loads(worker_path.read_text()) if worker_path.exists() else {}
    decoded_by_id = {r['frame_id']:r for r in decoded}
    submitted_by_id = {r['frame_id']:r for r in submissions if r['accepted']}
    joined = []
    for r in rows:
        d, s = decoded_by_id.get(r['frame_id']), submitted_by_id.get(r['frame_id'])
        if d is None or s is None:
            continue
        joined.append(dict(frame_id=r['frame_id'],
            capture_to_decode_ready_ms=(d['decoded_ready_s']-r['capture_s'])*1000,
            jpeg_wait_before_decode_ms=(d['decode_start_s']-r['capture_s'])*1000,
            jpu_decode_ms=(d['decode_end_s']-d['decode_start_s'])*1000,
            decode_to_submit_ms=(s['submit_begin_s']-d['decoded_ready_s'])*1000,
            copy_and_submit_ms=(s['submit_end_s']-s['submit_begin_s'])*1000,
            # Worker dequeue can race the parent's return from queue.put.
            queue_wait_lower_ms=(r['profile_dequeue_s']-s['submit_end_s'])*1000,
            queue_wait_upper_ms=(r['profile_dequeue_s']-s['submit_begin_s'])*1000,
            dequeue_to_infer_ms=(r['profile_infer_begin_s']-r['profile_dequeue_s'])*1000,
            infer_to_result_put_ms=(r['profile_put_start_s']-r['profile_infer_end_s'])*1000,
            result_put_to_receive_ms=(r['profile_receive_end_s']-r['profile_put_start_s'])*1000,
            receive_copy_ms=(r['profile_receive_end_s']-r['profile_receive_begin_s'])*1000,
            total_to_snapshot_ms=r['latency_ms']))
    write_csv(directory/'profile_latency_parts.csv', joined)
    def inside(rows, key):
        return [r for r in rows if measure_start<=r[key]<end]
    stages = dict(camera_mjpeg_received=(captures,'capture_s'), decoded_published=(decoded,'decoded_ready_s'),
        submit_attempts=(submissions,'submit_begin_s'), submit_accepted=([r for r in submissions if r['accepted']],'submit_begin_s'),
        worker_completed=(worker.get('completions',[]),'put_begin_s'), parent_received=(detector.receptions,'receive_end_s'))
    elapsed = max(0, end-measure_start)
    summary = dict(sample_period_s=1, latency_parts_ms={}, stages={}, profile_receive_calls=detector.receive_calls,
        profile_empty_receive_calls=detector.empty_receives,
        notes=['Camera timestamps are host JPEG reception, not sensor exposure.',
            'Queue lower/upper bounds include parent put return races; negative lower bounds are retained.',
            'CPU% is 100 per busy CPU; BPU ratio is a sampled driver statistic, not a per-frame integral.',
            'Instrumented test only: no timing admission, affinity, scheduling or queue-rule changes.'])
    current_captures = inside(captures, 'capture_s')
    current_decoded = inside(decoded, 'decoded_ready_s')
    offsets = [r['gst_offset'] for r in current_captures if r.get('gst_offset',2**64-1)!=2**64-1]
    pts = [r['gst_pts_ns'] for r in current_captures if r.get('gst_pts_ns',2**64-1)!=2**64-1]
    checksums = [r['sampled_luma_crc32'] for r in current_decoded]
    summary['source_identity'] = dict(gst_offsets_observed=len(offsets), gst_offsets_unique=len(set(offsets)),
        gst_pts_interarrival_ms=percentiles(np.diff(pts)/1e6) if len(pts)>1 else None,
        decoded_luma_samples=len(checksums), decoded_luma_samples_unique=len(set(checksums)),
        checksum_scope='Every 16th Y row/column, CRC32; observations only, never used to admit/reject frames.',
        limitation='GStreamer source offsets/PTS and changing pixels do not calibrate physical exposure timing.')
    for key in joined[0] if joined else ():
        if key != 'frame_id':
            summary['latency_parts_ms'][key] = percentiles([r[key] for r in joined])
    for name,(events,key) in stages.items():
        events = inside(events,key)
        windows = [sum(measure_start+i<=r[key]<measure_start+i+1 for r in events) for i in range(int(elapsed))]
        summary['stages'][name] = dict(count=len(events), fps=len(events)/elapsed if elapsed else 0, fps_1s=windows,
            interarrival_ms=percentiles(np.diff([r[key] for r in events])*1000) if len(events)>1 else None)
    resources_path = directory/'profile_resources.json'
    resources = inside(json.loads(resources_path.read_text()),'monotonic_s') if resources_path.exists() else []
    summary['resources'] = dict(roles={}, bpu={}, sampler_wall_ms=percentiles([r['sampler_wall_ms'] for r in resources]),
        sampler_cpu_ms=percentiles([r['sampler_cpu_ms'] for r in resources]),
        system_cpu_percent=percentiles([r['cpus']['cpu'] for r in resources if r['cpus'].get('cpu') is not None]))
    for key in ('utilization_percent','frequency_hz','cpu_frequency_khz'):
        summary['resources']['bpu'][key] = percentiles([r['bpu'][key] for r in resources if isinstance(r['bpu'].get(key),int)])
    for name in {name for r in resources for name in r['roles']}:
        stats = [r['roles'][name] for r in resources if name in r['roles']]
        summary['resources']['roles'][name] = dict(cpu_percent=percentiles([r['cpu_percent'] for r in stats if r['cpu_percent'] is not None]),
            rss_bytes=percentiles([r['rss_bytes'] for r in stats]), affinity=stats[-1].get('affinity'))
    display_file=directory/'profile_display.csv'
    if display_file.exists():
        with display_file.open() as file:
            displayed=list(csv.DictReader(file))
        displayed=[r for r in displayed if measure_start<=float(r['begin_s'])<end]
        summary['display_ms']={key:percentiles([float(r[key]) for r in displayed]) for key in (
            'shared_slot_read_ms','render_and_imshow_ms','highgui_event_ms')}
    (directory/'profile_summary.json').write_text(json.dumps(summary, indent=2))
    return summary


def cached_performance(args):
    """Three clearly labeled cached-frame ceilings, without camera or GUI."""
    from run_yolo_x5 import load_labels, nv12_letterbox
    from rescue_vision.vse import VseScaler
    import hashlib
    import cv2
    cv2.setNumThreads(1)
    source = np.fromfile(args.cached_perf, np.uint8).reshape(1536,1280)
    directory = args.output/(time.strftime('%Y%m%d_%H%M%S')+f'_{time.time_ns()%1000000:06d}_cached_profile')
    directory.mkdir(parents=True)
    detector = X5Yolo26(args.model, load_labels(args.labels), args.score_thres,args.nms_thres,args.priority,args.bpu_cores)
    scaler = VseScaler(1280,1024,640,512)
    sampler = ResourceSampler({'cached_benchmark':os.getpid()}, directory)
    sampler.start()
    tensor,_,_ = nv12_letterbox(scaler.scale(source),640,512,640,640)
    (directory/'input_640x640.nv12').write_bytes(tensor.tobytes())
    summaries, records = [], []
    try:
        for mode in ('runtime_only','prepared_plus_postprocess','full_preprocess_runtime_postprocess'):
            begin = time.monotonic()
            measure_start, deadline = begin+args.warmup, begin+args.warmup+args.duration
            times, cpu_times = [], []
            while time.monotonic()<deadline:
                start,cpu=time.monotonic(),time.thread_time()
                if mode=='runtime_only':
                    detector.run(tensor)
                elif mode=='prepared_plus_postprocess':
                    detector._infer_prepared(tensor,1280,1024,.5,0,64,start)
                else:
                    detector.infer_nv12(source,1280,1024,scaler)
                finish=time.monotonic()
                if measure_start<=finish<deadline:
                    times.append((finish-start)*1000)
                    cpu_times.append((time.thread_time()-cpu)*1000)
                    records.append(dict(mode=mode,start_s=start,finish_s=finish,wall_ms=times[-1],main_thread_cpu_ms=cpu_times[-1]))
            summaries.append(dict(mode=mode,source='same_cached_frame_not_live_camera',measure_start_s=measure_start,
                end_s=deadline,frames=len(times),fps=len(times)/args.duration,wall_ms=percentiles(times),main_thread_cpu_ms=percentiles(cpu_times)))
            print(json.dumps(summaries[-1]),flush=True)
    finally:
        sampler.close()
        scaler.close()
    report=dict(model=str(args.model),model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
        source='same_cached_frame_not_live_camera',runs=summaries,output=str(directory))
    write_csv(directory/'cached_frames.csv',records)
    (directory/'summary.json').write_text(json.dumps(report,indent=2))
    print('Cached profile:',directory,flush=True)
    return 0
