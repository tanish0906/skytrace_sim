from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import queue
import random
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def velocity_to_quaternion(vx, vy, vz, bank_rad=0.0):
    horiz_speed = math.hypot(vx, vy)
    yaw = math.atan2(vy, vx)
    pitch = -math.atan2(vz, max(horiz_speed, 1e-6))
    roll = bank_rad

    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def make_wireframe_radar(stamp, range_m, pan_angle, tilt_angle, fov):
    m = Marker()
    m.header.frame_id = 'radar_base'
    m.header.stamp = stamp
    m.ns = 'skytrace'
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 2.0
    m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 1.0, 0.5

    corners = []
    half_fov = fov / 2.0
    for dy in [-half_fov, half_fov]:
        for dz in [-half_fov, half_fov]:
            x, y, z = range_m, range_m * math.tan(dy), range_m * math.tan(dz)
            x1 = x * math.cos(tilt_angle) + z * math.sin(tilt_angle)
            z1 = -x * math.sin(tilt_angle) + z * math.cos(tilt_angle)
            x2 = x1 * math.cos(pan_angle) - y * math.sin(pan_angle)
            y2 = x1 * math.sin(pan_angle) + y * math.cos(pan_angle)
            corners.append((x2, y2, z1))

    p0 = Point(x=0.0, y=0.0, z=0.0)
    for c in corners:
        m.points.extend([p0, Point(x=c[0], y=c[1], z=c[2])])

    order = [0, 1, 3, 2]
    for i in range(4):
        c1 = corners[order[i]]
        c2 = corners[order[(i + 1) % 4]]
        m.points.extend([
            Point(x=c1[0], y=c1[1], z=c1[2]),
            Point(x=c2[0], y=c2[1], z=c2[2]),
        ])
    return m


def make_fighter(stamp, m_id, x, y, z, vx, vy, vz, spoofed):
    m = Marker()
    m.header.frame_id = 'radar_base'
    m.header.stamp = stamp
    m.ns = 'skytrace'
    m.id = m_id + 10
    m.type = Marker.MESH_RESOURCE
    m.action = Marker.ADD
    m.mesh_resource = 'package://skytrace_sim/meshes/fighter_jet.stl'
    m.scale.x = m.scale.y = m.scale.z = 3.0
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
    qx, qy, qz, qw = velocity_to_quaternion(vx, vy, vz, 0.0)
    m.pose.orientation.x, m.pose.orientation.y = float(qx), float(qy)
    m.pose.orientation.z, m.pose.orientation.w = float(qz), float(qw)

    m.mesh_use_embedded_materials = False
    if spoofed:
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
    else:
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.9, 1.0, 1.0
    return m


def make_direction_arrow(stamp, m_id, x, y, z, vx, vy, vz, spoofed):
    m = Marker()
    m.header.frame_id = 'radar_base'
    m.header.stamp = stamp
    m.ns = 'skytrace'
    m.id = m_id + 100
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
    qx, qy, qz, qw = velocity_to_quaternion(vx, vy, vz, 0.0)
    m.pose.orientation.x, m.pose.orientation.y = float(qx), float(qy)
    m.pose.orientation.z, m.pose.orientation.w = float(qz), float(qw)

    m.scale.x = 100.0
    m.scale.y = 8.0
    m.scale.z = 8.0

    if spoofed:
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 0.9
    else:
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.7
    return m


def make_spoof_highlight(stamp, m_id, x, y, z, label='SPOOF DETECTED'):
    text_m = Marker()
    text_m.header.frame_id = 'radar_base'
    text_m.header.stamp = stamp
    text_m.ns = 'skytrace'
    text_m.id = m_id + 200
    text_m.type = Marker.TEXT_VIEW_FACING
    text_m.action = Marker.ADD
    text_m.pose.position.x, text_m.pose.position.y, text_m.pose.position.z = float(x), float(y), float(z + 80.0)
    text_m.text = label
    text_m.scale.z = 40.0
    text_m.color.r, text_m.color.g, text_m.color.b, text_m.color.a = 1.0, 0.1, 0.1, 1.0

    halo_m = Marker()
    halo_m.header.frame_id = 'radar_base'
    halo_m.header.stamp = stamp
    halo_m.ns = 'skytrace'
    halo_m.id = m_id + 300
    halo_m.type = Marker.SPHERE
    halo_m.action = Marker.ADD
    halo_m.pose.position.x, halo_m.pose.position.y, halo_m.pose.position.z = float(x), float(y), float(z)
    halo_m.scale.x = halo_m.scale.y = halo_m.scale.z = 250.0
    halo_m.color.r, halo_m.color.g, halo_m.color.b, halo_m.color.a = 1.0, 0.0, 0.0, 0.4

    return [text_m, halo_m]


class ReusableHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class SkytraceSimNode(Node):
    def __init__(self):
        super().__init__('skytrace_sim_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/skytrace/markers', 10)
        self.telemetry_pub = self.create_publisher(String, '/skytrace/telemetry', 10)

        pkg_share = get_package_share_directory('skytrace_sim')
        self.html_path = os.path.join(pkg_share, 'web', 'index.html')

        self.lock = threading.Lock()
        self.sse_clients = []
        self.log_history = deque(maxlen=80)
        self.paused = False
        self.sim_time = 0.0
        self.last_wall_time = time.time()
        self.last_log_time = -1.0
        self.radar_range = 3000.0
        self.extra_targets = []
        self.httpd = None

        self._start_web_server(port=8080)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self._emit_log('INFO', 'SKYTRACE restored | GUI available at http://localhost:8080')

    def _emit_log(self, level: str, msg: str):
        if level == 'WARN':
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)
        with self.lock:
            self.log_history.append({'level': level, 'msg': msg})

    def _start_web_server(self, port=8080):
        node_ref = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                return

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    try:
                        with open(node_ref.html_path, 'rb') as f:
                            body = f.read()
                        self.send_response(200)
                        self.send_header('Content-Type', 'text/html; charset=utf-8')
                        self.send_header('Content-Length', str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)
                    except OSError:
                        self.send_error(404, 'web/index.html not found')
                elif self.path == '/events':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache')
                    self.send_header('Connection', 'keep-alive')
                    self.end_headers()
                    q = queue.Queue(maxsize=20)
                    with node_ref.lock:
                        node_ref.sse_clients.append(q)
                    try:
                        while True:
                            payload = q.get()
                            self.wfile.write(f'data: {payload}\n\n'.encode('utf-8'))
                            self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        pass
                    finally:
                        with node_ref.lock:
                            if q in node_ref.sse_clients:
                                node_ref.sse_clients.remove(q)
                else:
                    self.send_error(404)

            def do_POST(self):
                if self.path == '/api/control':
                    length = int(self.headers.get('Content-Length', 0))
                    raw = self.rfile.read(length).decode('utf-8') if length else '{}'
                    try:
                        cmd = json.loads(raw)
                    except json.JSONDecodeError:
                        cmd = {}
                    node_ref.handle_control(cmd)
                    resp = b'{"ok":true}'
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.end_headers()
                    self.wfile.write(resp)
                else:
                    self.send_error(404)

        try:
            self.httpd = ReusableHTTPServer(('0.0.0.0', port), Handler)
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()
        except OSError as e:
            self.get_logger().warn(f'Web GUI server could not bind port {port}: {e}')

    def handle_control(self, cmd: dict):
        action = cmd.get('action')
        if action == 'toggle_pause':
            with self.lock:
                self.paused = not self.paused
                state = 'PAUSED' if self.paused else 'RESUMED'
            self._emit_log('INFO', f'GUI command: Simulation {state}')
        elif action == 'reset':
            with self.lock:
                self.sim_time = 0.0
                self.last_log_time = -1.0
                self.extra_targets.clear()
                self.paused = False
            self._emit_log('INFO', 'GUI command: Simulation RESET')
        elif action == 'inject':
            t_type = cmd.get('type', 'ghost')
            with self.lock:
                idx = len(self.extra_targets) + 1
                tid = f'GHOST-INJ-{idx:02d}' if t_type == 'ghost' else f'SPOOF-INJ-{idx:02d}'
                self.extra_targets.append({
                    'id': tid,
                    'type': t_type,
                    'radius': random.uniform(600.0, 2400.0),
                    'speed': random.uniform(0.35, 0.85),
                    'phase': random.uniform(0.0, 2.0 * math.pi),
                    'alt_base': random.uniform(250.0, 750.0),
                })
            self._emit_log('WARN', f'GUI injected {t_type.upper()} target: {tid}')

    def timer_callback(self):
        now = time.time()
        dt = now - self.last_wall_time
        self.last_wall_time = now

        with self.lock:
            if not self.paused:
                self.sim_time += dt
            sim_time = self.sim_time
            paused = self.paused
            extra_copy = list(self.extra_targets)

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        pan_angle = (sim_time * 1.0) % (2.0 * math.pi)
        pan_deg = math.degrees(pan_angle)
        marker_array.markers.append(
            make_wireframe_radar(
                stamp, self.radar_range, pan_angle, math.radians(10.0), math.radians(35.0)
            )
        )

        should_log = (not paused) and ((sim_time - self.last_log_time) >= 1.0)
        if should_log:
            self.last_log_time = sim_time

        tracks_payload = []

        for i in range(9):
            if i < 6:
                t_type = 'verified'
                track_id = f'AG-{101 + i}'
                conf = 96 + (i % 3)
            elif i < 8:
                t_type = 'suspicious'
                track_id = f'SPOOF-0{i - 5}'
                conf = 78
            else:
                t_type = 'ghost'
                track_id = 'GHOST-01'
                conf = 92

            is_spoofed = (i >= 6)
            radius = 800.0 + (i * 200.0)
            speed = 0.5 + (i * 0.05)
            angle = sim_time * speed + (i * math.pi / 3.0)

            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 300.0 + (i * 50.0) + (100.0 * math.sin(sim_time + i))

            vx = -radius * speed * math.sin(angle)
            vy = radius * speed * math.cos(angle)
            vz = 100.0 * math.cos(sim_time + i)

            marker_array.markers.append(make_fighter(stamp, i, x, y, z, vx, vy, vz, is_spoofed))
            marker_array.markers.append(make_direction_arrow(stamp, i, x, y, z, vx, vy, vz, is_spoofed))

            if is_spoofed:
                marker_array.markers.extend(make_spoof_highlight(stamp, i, x, y, z))

            speed_ms = math.hypot(vx, math.hypot(vy, vz))
            heading = int((90.0 - math.degrees(math.atan2(vy, vx))) % 360)
            rng = int(math.hypot(x, y))

            tracks_payload.append({
                'id': track_id,
                'x': round(x, 1),
                'y': round(y, 1),
                'alt': round(z, 1),
                'speed': round(speed_ms, 1),
                'range': rng,
                'hdg': heading,
                'type': t_type,
                'conf': conf,
            })

            if should_log and (i == 0 or i == 6):
                status = 'SPOOF DETECTED' if is_spoofed else 'Genuine Aircraft'
                log_msg = f'{status} | Speed: {speed_ms:.1f} m/s | Range: {rng}m'
                self._emit_log('WARN' if is_spoofed else 'INFO', log_msg)

        for idx, ext in enumerate(extra_copy):
            ang = sim_time * ext['speed'] + ext['phase']
            x = ext['radius'] * math.cos(ang)
            y = ext['radius'] * math.sin(ang)
            z = ext['alt_base'] + 60.0 * math.sin(sim_time + idx)
            vx = -ext['radius'] * ext['speed'] * math.sin(ang)
            vy = ext['radius'] * ext['speed'] * math.cos(ang)
            vz = 60.0 * math.cos(sim_time + idx)
            speed_ms = math.hypot(vx, math.hypot(vy, vz))
            heading = int((90.0 - math.degrees(math.atan2(vy, vx))) % 360)
            rng = int(math.hypot(x, y))

            marker_array.markers.append(make_fighter(stamp, 50 + idx, x, y, z, vx, vy, vz, True))
            marker_array.markers.append(make_direction_arrow(stamp, 50 + idx, x, y, z, vx, vy, vz, True))
            marker_array.markers.extend(make_spoof_highlight(stamp, 50 + idx, x, y, z, f"{ext['id']} | SPOOF"))

            tracks_payload.append({
                'id': ext['id'],
                'x': round(x, 1),
                'y': round(y, 1),
                'alt': round(z, 1),
                'speed': round(speed_ms, 1),
                'range': rng,
                'hdg': heading,
                'type': ext['type'],
                'conf': 90 if ext['type'] == 'ghost' else 75,
            })

        self.publisher_.publish(marker_array)

        with self.lock:
            payload_str = json.dumps({
                'sim_time': round(sim_time, 2),
                'pan_deg': round(pan_deg, 1),
                'radar_range': self.radar_range,
                'paused': paused,
                'tracks': tracks_payload,
                'logs': list(self.log_history),
            })
            clients = list(self.sse_clients)

        self.telemetry_pub.publish(String(data=payload_str))

        for q in clients:
            try:
                q.put_nowait(payload_str)
            except queue.Full:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = SkytraceSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.httpd:
        node.httpd.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
