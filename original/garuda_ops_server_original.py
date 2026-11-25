from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse
import json, mimetypes
ROOT=Path(__file__).resolve().parent
class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*a,**kw): super().__init__(*a,directory=str(ROOT/'web'),**kw)
    def do_GET(self):
        path=urlparse(self.path).path
        if path=='/api/dataset':
            data=(ROOT/'data'/'observations.json').read_bytes()
            self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data);return
        if path=='/api/health':
            data=b'{"status":"ok","mode":"dataset-driven simulation"}'
            self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data);return
        if path.startswith('/media/'):
            filename=path.removeprefix('/media/')
            if filename!='airsim.mp4': self.send_error(404);return
            target=ROOT/'media'/filename
            if not target.is_file(): self.send_error(404);return
            size=target.stat().st_size
            rng=self.headers.get('Range','')
            start,end=0,size-1
            if rng.startswith('bytes='):
                try:
                    a,b=rng[6:].split('-',1);start=int(a) if a else 0;end=int(b) if b else end
                except ValueError: self.send_error(416);return
            if start> end or start>=size: self.send_error(416);return
            self.send_response(206 if rng else 200)
            self.send_header('Content-Type','video/mp4')
            self.send_header('Accept-Ranges','bytes')
            self.send_header('Content-Length',str(end-start+1))
            if rng:self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
            self.end_headers()
            with target.open('rb') as f:
                f.seek(start)
                remaining=end-start+1
                while remaining:
                    chunk=f.read(min(65536,remaining))
                    if not chunk:break
                    self.wfile.write(chunk);remaining-=len(chunk)
            return
        if path=='/':self.path='/index.html'
        return super().do_GET()
if __name__=='__main__':
    print('GARUDA OPS DATASET DASHBOARD: http://localhost:8000')
    ThreadingHTTPServer(('127.0.0.1',8000),Handler).serve_forever()
