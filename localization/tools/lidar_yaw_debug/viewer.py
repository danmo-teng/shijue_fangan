"""Read-only localhost map, reusing the reviewed lidar presentation geometry."""
import json
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .presentation import presentation


class Viewer:
    def __init__(self, port):
        self.lock = threading.Lock()
        self.state = {"points": [], "result": None}
        self.updated = time.monotonic()
        owner = self
        html = Path(__file__).with_name("view.html").read_bytes()
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/state":
                    with owner.lock:
                        payload = json.dumps(dict(owner.state,age_s=time.monotonic()-owner.updated),allow_nan=False).encode()
                    content_type = "application/json"
                elif self.path == "/":
                    payload,content_type = html,"text/html; charset=utf-8"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type",content_type)
                self.send_header("Cache-Control","no-store")
                self.send_header("Content-Length",str(len(payload)))
                self.end_headers()
                try: self.wfile.write(payload)
                except (BrokenPipeError,ConnectionResetError): pass
            def log_message(self,*args): pass
        self.server = ThreadingHTTPServer(("127.0.0.1",port),Handler)
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def update(self, points, result):
        display = presentation(points,result)
        # Legacy geometry is corner-coordinate internally; the prominent readout
        # is explicitly competition-centred. This does not publish to competition.
        display["coordinate_frame"] = "fixed_corner_axes; displayed_center=corner-(1.5,1.5)"
        display["label"] = "静止几何诊断（未标定精度）" if result.get("rank_xy")==2 else "部分/历史参考（未观测轴不作为测量）"
        with self.lock:
            self.state = dict(points=points,result=result,display=display)
            self.updated = time.monotonic()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)
