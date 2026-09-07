import os
import json
import subprocess
from http.server import HTTPServer, SimpleHTTPRequestHandler

PORT = 8080
DIRECTORY = "/home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/visualizer"
CONFIG_YAML_PATH = "/home/kms/dev/LSM_repository/config/waypoints.yaml"
SCRIPT_PATH = "/home/kms/.gemini/antigravity/brain/2757ddc0-5e70-47e4-a33a-9d06f351cd20/scratch/update_user_coords.py"

class EditorHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def do_POST(self):
        if self.path == '/api/save-yaml':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                payload = json.loads(post_data.decode('utf-8'))
                yaml_text = payload.get('yaml_text', '')

                if not yaml_text:
                    raise ValueError("Empty YAML content")

                # Write directly to top-level config/waypoints.yaml
                with open(CONFIG_YAML_PATH, "w", encoding="utf-8") as f:
                    f.write(yaml_text)

                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                res = {"status": "success", "message": "waypoints.yaml가 성공적으로 저장되었습니다!"}
                self.wfile.write(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                res = {"status": "error", "message": str(e)}
                self.wfile.write(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            return

        elif self.path == '/api/recalculate-origin':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                payload = json.loads(post_data.decode('utf-8'))
                
                ox = float(payload.get('origin_x_pt', 140.0))
                oy = float(payload.get('origin_y_pt', 900.0))

                # Update SCRIPT_PATH ORIGIN constants
                with open(SCRIPT_PATH, "r", encoding="utf-8") as f:
                    script_code = f.read()
                
                import re
                script_code = re.sub(r'ORIGIN_X_PT = [\d\.]+', f'ORIGIN_X_PT = {ox}', script_code)
                script_code = re.sub(r'ORIGIN_Y_PT = [\d\.]+', f'ORIGIN_Y_PT = {oy}', script_code)
                
                with open(SCRIPT_PATH, "w", encoding="utf-8") as f:
                    f.write(script_code)

                # Run update_user_coords.py
                result = subprocess.run(["python3", SCRIPT_PATH], capture_output=True, text=True)

                if result.returncode == 0:
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    res = {"status": "success", "message": f"새 원점 ({ox}pt, {oy}pt) 기준으로 waypoints.yaml 재계산 완료!"}
                    self.wfile.write(json.dumps(res, ensure_ascii=False).encode('utf-8'))
                else:
                    raise RuntimeError(result.stderr)
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                res = {"status": "error", "message": str(e)}
                self.wfile.write(json.dumps(res, ensure_ascii=False).encode('utf-8'))
            return

        super().do_POST()

def run_server():
    server_address = ('', PORT)
    httpd = HTTPServer(server_address, EditorHandler)
    print(f"Server running on port {PORT}...")
    httpd.serve_forever()

if __name__ == '__main__':
    run_server()
