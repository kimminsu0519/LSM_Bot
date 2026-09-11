#!/usr/bin/env python3
import os
import sys
import http.server
import socketserver
import threading

PORT = 8080

try:
    import rclpy
    from rclpy.node import Node
    from ament_index_python.packages import get_package_share_directory
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

class WebServerHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

def run_standalone_server(web_dir):
    print(f"[LSM FMS Web Server] Serving directory: {web_dir}")
    print(f"[LSM FMS Web Server] Open Dashboard at: http://localhost:{PORT}")
    
    class DirectoryHandler(WebServerHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=web_dir, **kwargs)

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", PORT), DirectoryHandler) as httpd:
        httpd.serve_forever()

if HAS_ROS2:
    class LSMFMSWebServerNode(Node):
        def __init__(self):
            super().__init__('lsm_fms_web_server_node')
            self.get_logger().info('LSM FMS Web Server Node Starting...')
            
            try:
                package_share = get_package_share_directory('lsm_fms_web')
                self.web_dir = os.path.join(package_share, 'web')
            except Exception:
                self.web_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../web'))
                
            self.get_logger().info(f'Serving Web Dashboard from: {self.web_dir}')
            
            t = threading.Thread(target=run_standalone_server, args=(self.web_dir,), daemon=True)
            t.start()

    def main(args=None):
        rclpy.init(args=args)
        node = LSMFMSWebServerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
else:
    def main():
        web_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../web'))
        run_standalone_server(web_dir)

if __name__ == '__main__':
    main()
