from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import queue
import random
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


# --- Radar Origin & OpenSky Coverage (23.04 sq deg -> 1 API credit/poll) ---
RADAR_LAT = 30.9010          # Change to any lat/lon you want to monitor
RADAR_LON = 75.8573
BBOX_HALF_DEG = 2.4          # +/- 2.4 deg (~250 km radius; covers DEL/IXC/ATQ corridor)
POLL_INTERVAL_SEC = 15.0     # Safe polling cadence for OpenSky rate limits
MAX_URDF_SLOTS = 9           # Matches aircraft_0 .. aircraft_8 URDF publishers

REAL_RADAR_RANGE_M = 250000.0    # 250 km real-world radar range (sent to AEROGUARD GUI)
VISUAL_RADAR_RANGE_M = 3000.0    # 3000 m visual cone in RViz2
VISUAL_XY_SCALE = VISUAL_RADAR_RANGE_M / REAL_RADAR_RANGE_M
VISUAL_Z_SCALE = 0.08            # 10,000 m cruise altitude -> 800 m in RViz2
RADAR_DISH_Z = 140.0
YAW_OFFSET_DEG = 0.0             # Set to 90/-90/180 if your jet nose needs rotation

OPENSKY_STATES_URL = 'https://opensky-network.org/api/states/all'
OPENSKY_TOKEN_URL = (
    'https://auth.opensky-network.org/auth/realms/opensky-network/'
    'protocol/openid-connect/token'
)


def get_urdf_root_link(urdf_path: str) -> str:
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        links = {el.attrib['name'] for el in root.findall('link')}
        children = {
            el.find('child').attrib['link']
            for el in root.findall('joint')
            if el.find('child') is not None
        }
        roots = links - children
        return next(iter(roots)) if roots else 'base_link'
    except Exception:
        return 'base_link'


def euler_to_quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def velocity_to_quaternion(vx, vy, vz, bank_rad=0.0):
    horiz = math.hypot(vx, vy)
    yaw = math.atan2(vy, vx) + math.radians(YAW_OFFSET_DEG)
    pitch = -math.atan2(vz, max(horiz, 1e-6))
    return euler_to_quaternion(bank_rad, pitch, yaw)


def latlon_to_local_m(lat: float, lon: float):
    east_m = (lon - RADAR_LON) * 111320.0 * math.cos(math.radians(RADAR_LAT))
    north_m = (lat - RADAR_LAT) * 110540.0
    return east_m, north_m


def make_delete_marker(stamp, m_id):
    m = Marker()
    m.header.frame_id = 'radar_base'
    m.header.stamp = stamp
    m.ns = 'skytrace'
    m.id = m_id
    m.action = Marker.DELETE
    return m



def make_ground_radar_station(stamp, pan_angle, tilt_angle, range_m=3000.0):
    """Detailed military 3D phased-array air-defense radar site + volumetric sweep & PPI rings."""
    markers = []
    cp, sp = math.cos(pan_angle), math.sin(pan_angle)
    ct, st = math.cos(tilt_angle), math.sin(tilt_angle)

    def rot_pos(lx, ly, lz):
        return (lx * cp - ly * sp, lx * sp + ly * cp, lz)

    def dish_pos(dx, dy, dz):
        # Pitch backward by tilt_angle so +X array face looks upward, then rotate by pan_angle
        tx = dx * ct - dz * st
        tz = dx * st + dz * ct
        return (tx * cp - dy * sp, tx * sp + dy * cp, RADAR_DISH_Z + tz)

    def add_prim(m_id, m_type, pos, scale, rgba, q=(0.0, 0.0, 0.0, 1.0)):
        m = Marker()
        m.header.frame_id = 'radar_base'
        m.header.stamp = stamp
        m.ns = 'skytrace'
        m.id = m_id
        m.type = m_type
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
            float(q[0]), float(q[1]), float(q[2]), float(q[3])
        )
        m.scale.x, m.scale.y, m.scale.z = float(scale[0]), float(scale[1]), float(scale[2])
        m.color.r, m.color.g, m.color.b, m.color.a = float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3])
        markers.append(m)

    # --- 1. Concrete Foundation & Perimeter ---
    add_prim(900, Marker.CYLINDER, (0.0, 0.0, 5.0), (310.0, 310.0, 12.0), (0.36, 0.38, 0.40, 1.0))
    add_prim(901, Marker.CYLINDER, (0.0, 0.0, 12.0), (225.0, 225.0, 4.0), (0.21, 0.23, 0.25, 1.0))
    add_prim(902, Marker.CYLINDER, (0.0, 0.0, 19.0), (105.0, 105.0, 16.0), (0.32, 0.34, 0.36, 1.0))

    # --- 2. Command & Control (C2) Shelter, SATCOM Dome & Diesel Generator ---
    q_bunker = euler_to_quaternion(0.0, 0.0, 0.45)
    add_prim(903, Marker.CUBE, (-118.0, -78.0, 34.0), (96.0, 52.0, 44.0), (0.28, 0.32, 0.25, 1.0), q_bunker)
    # Armored shelter roof HVAC unit
    add_prim(904, Marker.CUBE, (-128.0, -83.0, 60.0), (28.0, 24.0, 12.0), (0.20, 0.22, 0.23, 1.0), q_bunker)
    # Rooftop SATCOM radome
    add_prim(905, Marker.SPHERE, (-96.0, -68.0, 68.0), (26.0, 26.0, 24.0), (0.86, 0.88, 0.89, 1.0))
    # Diesel power generator trailer + exhaust stack
    add_prim(906, Marker.CUBE, (108.0, -86.0, 26.0), (62.0, 34.0, 30.0), (0.34, 0.36, 0.28, 1.0), q_bunker)
    add_prim(907, Marker.CYLINDER, (120.0, -82.0, 48.0), (5.0, 5.0, 22.0), (0.15, 0.16, 0.17, 1.0))

    # --- 3. Steel Lattice Radar Tower & Elevated Catwalk ---
    # Central armored waveguide / power core
    add_prim(908, Marker.CYLINDER, (0.0, 0.0, 64.0), (30.0, 30.0, 82.0), (0.24, 0.27, 0.30, 1.0))
    # Service platform deck
    add_prim(909, Marker.CYLINDER, (0.0, 0.0, 106.0), (112.0, 112.0, 6.0), (0.34, 0.38, 0.42, 1.0))

    # Lattice legs, cross-bracing, catwalk safety railing & C2 comms mast
    truss = Marker()
    truss.header.frame_id = 'radar_base'
    truss.header.stamp = stamp
    truss.ns = 'skytrace'
    truss.id = 910
    truss.type = Marker.LINE_LIST
    truss.action = Marker.ADD
    truss.scale.x = 4.2
    truss.color.r, truss.color.g, truss.color.b, truss.color.a = 0.50, 0.55, 0.60, 1.0

    tiers = [(26.0, 44.0), (53.0, 38.0), (80.0, 32.0), (105.0, 28.0)]
    for t_idx in range(len(tiers) - 1):
        z0, r0 = tiers[t_idx]
        z1, r1 = tiers[t_idx + 1]
        for k in range(4):
            a0 = k * math.pi / 2.0 + math.pi / 4.0
            a1 = ((k + 1) % 4) * math.pi / 2.0 + math.pi / 4.0
            p00 = Point(x=r0 * math.cos(a0), y=r0 * math.sin(a0), z=z0)
            p01 = Point(x=r0 * math.cos(a1), y=r0 * math.sin(a1), z=z0)
            p10 = Point(x=r1 * math.cos(a0), y=r1 * math.sin(a0), z=z1)
            p11 = Point(x=r1 * math.cos(a1), y=r1 * math.sin(a1), z=z1)
            # Corner leg, horizontal strut, and X-bracing
            truss.points.extend([p00, p10, p10, p11, p00, p11, p01, p10])

    # Circular safety handrail on catwalk
    rail_segs = 16
    rail_r = 54.0
    for k in range(rail_segs):
        a0 = 2.0 * math.pi * k / rail_segs
        a1 = 2.0 * math.pi * (k + 1) / rail_segs
        p_bot = Point(x=rail_r * math.cos(a0), y=rail_r * math.sin(a0), z=109.0)
        p_top0 = Point(x=rail_r * math.cos(a0), y=rail_r * math.sin(a0), z=120.0)
        p_top1 = Point(x=rail_r * math.cos(a1), y=rail_r * math.sin(a1), z=120.0)
        truss.points.extend([p_bot, p_top0, p_top0, p_top1])

    # Tall communications mast on C2 bunker + cable tray to tower
    truss.points.extend([
        Point(x=-145.0, y=-92.0, z=56.0), Point(x=-145.0, y=-92.0, z=125.0),
        Point(x=-155.0, y=-92.0, z=115.0), Point(x=-135.0, y=-92.0, z=115.0),
        Point(x=-78.0, y=-54.0, z=16.0), Point(x=-18.0, y=-12.0, z=16.0),
        Point(x=80.0, y=-64.0, z=16.0), Point(x=18.0, y=-12.0, z=16.0),
    ])
    markers.append(truss)

    # 4 Red aviation obstruction warning beacons around the platform
    beacons = Marker()
    beacons.header.frame_id = 'radar_base'
    beacons.header.stamp = stamp
    beacons.ns = 'skytrace'
    beacons.id = 911
    beacons.type = Marker.SPHERE_LIST
    beacons.action = Marker.ADD
    beacons.scale.x = beacons.scale.y = beacons.scale.z = 8.0
    beacons.color.r, beacons.color.g, beacons.color.b, beacons.color.a = 1.0, 0.12, 0.08, 1.0
    for k in range(4):
        ang = k * math.pi / 2.0
        beacons.points.append(Point(x=54.0 * math.cos(ang), y=54.0 * math.sin(ang), z=123.0))
    beacons.points.append(Point(x=-145.0, y=-92.0, z=127.0))
    markers.append(beacons)

    # --- 4. Rotating Slew-Ring Turntable & Dual Elevation Yoke ---
    q_pan = euler_to_quaternion(0.0, 0.0, pan_angle)
    q_dish = euler_to_quaternion(0.0, -tilt_angle, pan_angle)

    # Heavy bearing collar
    add_prim(912, Marker.CYLINDER, (0.0, 0.0, 115.0), (74.0, 74.0, 12.0), (0.18, 0.20, 0.22, 1.0))
    # Armored rotating turret cabin
    add_prim(913, Marker.CUBE, rot_pos(-8.0, 0.0, 132.0), (66.0, 62.0, 26.0), (0.29, 0.33, 0.26, 1.0), q_pan)
    # Left & Right trunnion yoke arms holding the array
    add_prim(914, Marker.CUBE, rot_pos(6.0, 42.0, 142.0), (22.0, 12.0, 44.0), (0.24, 0.27, 0.22, 1.0), q_pan)
    add_prim(915, Marker.CUBE, rot_pos(6.0, -42.0, 142.0), (22.0, 12.0, 44.0), (0.24, 0.27, 0.22, 1.0), q_pan)

    # --- 5. Tilted 3D Phased-Array (AESA) Antenna Assembly ---
    # Heavy metallic rear bezel / structural backing frame
    add_prim(916, Marker.CUBE, dish_pos(12.0, 0.0, 12.0), (12.0, 144.0, 84.0), (0.22, 0.25, 0.27, 1.0), q_dish)
    # Front active phased-array radome face
    add_prim(917, Marker.CUBE, dish_pos(20.0, 0.0, 12.0), (6.0, 132.0, 72.0), (0.35, 0.40, 0.34, 1.0), q_dish)
    # Rear transmitter / liquid-cooling electronics cabinet (acts as counterweight)
    add_prim(918, Marker.CUBE, dish_pos(-12.0, 0.0, 8.0), (36.0, 68.0, 46.0), (0.26, 0.30, 0.24, 1.0), q_dish)
    # Rear cooling blower cylinders
    add_prim(919, Marker.CYLINDER, dish_pos(-32.0, 18.0, 10.0), (18.0, 18.0, 26.0), (0.18, 0.20, 0.22, 1.0), q_dish)
    add_prim(920, Marker.CYLINDER, dish_pos(-32.0, -18.0, 10.0), (18.0, 18.0, 26.0), (0.18, 0.20, 0.22, 1.0), q_dish)
    # Top-mounted Secondary IFF (Friend/Foe) Horizontal Bar Antenna
    add_prim(921, Marker.CUBE, dish_pos(16.0, 0.0, 62.0), (8.0, 118.0, 11.0), (0.55, 0.58, 0.52, 1.0), q_dish)
    # IFF mounting struts
    add_prim(922, Marker.CUBE, dish_pos(14.0, 32.0, 56.0), (6.0, 6.0, 10.0), (0.20, 0.22, 0.24, 1.0), q_dish)
    add_prim(923, Marker.CUBE, dish_pos(14.0, -32.0, 56.0), (6.0, 6.0, 10.0), (0.20, 0.22, 0.24, 1.0), q_dish)

    # T/R Phased-Array Element Grid lines across the front panel face
    grid_m = Marker()
    grid_m.header.frame_id = 'radar_base'
    grid_m.header.stamp = stamp
    grid_m.ns = 'skytrace'
    grid_m.id = 924
    grid_m.type = Marker.LINE_LIST
    grid_m.action = Marker.ADD
    grid_m.scale.x = 2.0
    grid_m.color.r, grid_m.color.g, grid_m.color.b, grid_m.color.a = 0.08, 0.92, 1.0, 0.85

    face_x = 23.8
    for row in (-24.0, -8.0, 8.0, 24.0, 40.0):
        p1 = dish_pos(face_x, -64.0, row)
        p2 = dish_pos(face_x, 64.0, row)
        grid_m.points.extend([Point(x=p1[0], y=p1[1], z=p1[2]), Point(x=p2[0], y=p2[1], z=p2[2])])
    for col in (-60.0, -40.0, -20.0, 0.0, 20.0, 40.0, 60.0):
        p1 = dish_pos(face_x, col, -24.0)
        p2 = dish_pos(face_x, col, 48.0)
        grid_m.points.extend([Point(x=p1[0], y=p1[1], z=p1[2]), Point(x=p2[0], y=p2[1], z=p2[2])])
    # IFF dipole elements along the top bar
    for col in (-50.0, -30.0, -10.0, 10.0, 30.0, 50.0):
        p1 = dish_pos(20.5, col, 57.0)
        p2 = dish_pos(20.5, col, 67.0)
        grid_m.points.extend([Point(x=p1[0], y=p1[1], z=p1[2]), Point(x=p2[0], y=p2[1], z=p2[2])])
    markers.append(grid_m)

    # --- 6. Volumetric Translucent Radar Sweep Fan + Ground PPI Sweep Trail ---
    sweep = Marker()
    sweep.header.frame_id = 'radar_base'
    sweep.header.stamp = stamp
    sweep.ns = 'skytrace'
    sweep.id = 925
    sweep.type = Marker.TRIANGLE_LIST
    sweep.action = Marker.ADD
    sweep.pose.orientation.w = 1.0
    sweep.scale.x = sweep.scale.y = sweep.scale.z = 1.0

    ox, oy, oz = dish_pos(24.0, 0.0, 12.0)
    p_orig = Point(x=float(ox), y=float(oy), z=float(oz))
    trail_rad = math.radians(52.0)
    steps = 10
    fan_r = range_m * math.cos(tilt_angle)
    fan_z = oz + range_m * math.sin(tilt_angle)

    for s_idx in range(steps):
        f0 = s_idx / float(steps)
        f1 = (s_idx + 1) / float(steps)
        a0 = pan_angle - f0 * trail_rad
        a1 = pan_angle - f1 * trail_rad

        al0 = 0.28 * ((1.0 - f0) ** 1.6)
        al1 = 0.28 * ((1.0 - f1) ** 1.6)
        c_org = ColorRGBA(r=0.1, g=0.95, b=1.0, a=float(al0 * 0.95))
        c0 = ColorRGBA(r=0.0, g=0.85, b=1.0, a=float(al0 * 0.45))
        c1 = ColorRGBA(r=0.0, g=0.85, b=1.0, a=float(al1 * 0.45))

        pa = Point(x=float(ox + fan_r * math.cos(a0)), y=float(oy + fan_r * math.sin(a0)), z=float(fan_z))
        pb = Point(x=float(ox + fan_r * math.cos(a1)), y=float(oy + fan_r * math.sin(a1)), z=float(fan_z))
        # Double-sided so visible from any camera pitch
        sweep.points.extend([p_orig, pb, pa, p_orig, pa, pb])
        sweep.colors.extend([c_org, c1, c0, c_org, c0, c1])

        # Ground PPI sweep shadow hovering cleanly above desert floor
        gal0 = 0.18 * ((1.0 - f0) ** 1.8)
        gal1 = 0.18 * ((1.0 - f1) ** 1.8)
        gc0 = ColorRGBA(r=0.0, g=0.95, b=0.70, a=float(gal0))
        gc1 = ColorRGBA(r=0.0, g=0.95, b=0.70, a=float(gal1))
        g_org = Point(x=0.0, y=0.0, z=22.0)
        ga = Point(x=float(range_m * math.cos(a0)), y=float(range_m * math.sin(a0)), z=22.0)
        gb = Point(x=float(range_m * math.cos(a1)), y=float(range_m * math.sin(a1)), z=22.0)
        sweep.points.extend([g_org, gb, ga])
        sweep.colors.extend([gc0, gc1, gc0])

    markers.append(sweep)

    # --- 7. Tactical Ground Range Rings (1 km, 2 km, 3 km) & Azimuth Ticks ---
    rings = Marker()
    rings.header.frame_id = 'radar_base'
    rings.header.stamp = stamp
    rings.ns = 'skytrace'
    rings.id = 926
    rings.type = Marker.LINE_LIST
    rings.action = Marker.ADD
    rings.scale.x = 3.5
    rings.color.r, rings.color.g, rings.color.b, rings.color.a = 0.10, 0.90, 0.80, 0.42

    ring_z = 26.0
    ring_segs = 64
    for rr in (range_m / 3.0, 2.0 * range_m / 3.0, range_m):
        for k in range(ring_segs):
            a0 = 2.0 * math.pi * k / ring_segs
            a1 = 2.0 * math.pi * (k + 1) / ring_segs
            rings.points.extend([
                Point(x=rr * math.cos(a0), y=rr * math.sin(a0), z=ring_z),
                Point(x=rr * math.cos(a1), y=rr * math.sin(a1), z=ring_z),
            ])
    for deg in range(0, 360, 30):
        ar = math.radians(deg)
        inner_r = 220.0 if (deg % 90 == 0) else (range_m - 220.0)
        rings.points.extend([
            Point(x=inner_r * math.cos(ar), y=inner_r * math.sin(ar), z=ring_z),
            Point(x=range_m * math.cos(ar), y=range_m * math.sin(ar), z=ring_z),
        ])
    markers.append(rings)

    # --- 8. Station Tactical Callout ---
    label = Marker()
    label.header.frame_id = 'radar_base'
    label.header.stamp = stamp
    label.ns = 'skytrace'
    label.id = 927
    label.type = Marker.TEXT_VIEW_FACING
    label.action = Marker.ADD
    label.pose.position.x, label.pose.position.y, label.pose.position.z = 0.0, -240.0, 88.0
    label.text = 'SKYTRACE 3D AESA RADAR [OPENSKY LIVE]'
    label.scale.z = 42.0
    label.color.r, label.color.g, label.color.b, label.color.a = 0.35, 0.98, 1.0, 1.0
    markers.append(label)

    return markers


def make_wireframe_radar(stamp, range_m, pan_angle, tilt_angle, fov):
    m = Marker()
    m.header.frame_id = 'radar_base'
    m.header.stamp = stamp
    m.ns = 'skytrace'
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 2.5
    m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.92, 1.0, 0.65

    ct, st = math.cos(tilt_angle), math.sin(tilt_angle)
    cp, sp = math.cos(pan_angle), math.sin(pan_angle)
    tx = 24.0 * ct - 12.0 * st
    tz = 24.0 * st + 12.0 * ct
    origin_x = tx * cp
    origin_y = tx * sp
    origin_z = RADAR_DISH_Z + tz

    corners = []
    half_fov = fov / 2.0
    for dy in [-half_fov, half_fov]:
        for dz in [-half_fov, half_fov]:
            x, y, z = range_m, range_m * math.tan(dy), range_m * math.tan(dz)
            x1 = x * ct - z * st
            z1 = x * st + z * ct
            x2 = x1 * cp - y * sp
            y2 = x1 * sp + y * cp
            corners.append((origin_x + x2, origin_y + y2, origin_z + z1))

    p0 = Point(x=origin_x, y=origin_y, z=origin_z)
    for c in corners:
        m.points.extend([p0, Point(x=c[0], y=c[1], z=c[2])])

    # Center radar boresight beam line
    bx = origin_x + range_m * ct * cp
    by = origin_y + range_m * ct * sp
    bz = origin_z + range_m * st
    m.points.extend([p0, Point(x=bx, y=by, z=bz)])

    order = [0, 1, 3, 2]
    for i in range(4):
        c1 = corners[order[i]]
        c2 = corners[order[(i + 1) % 4]]
        m.points.extend([
            Point(x=c1[0], y=c1[1], z=c1[2]),
            Point(x=c2[0], y=c2[1], z=c2[2]),
        ])
    return m


def make_front_vector(stamp, m_id, x, y, z, vx, vy, vz, spoofed):
    m = Marker()
    m.header.frame_id = 'radar_base'
    m.header.stamp = stamp
    m.ns = 'skytrace'
    m.id = m_id + 100
    m.action = Marker.DELETE
    return m


def make_callsign_label(stamp, m_id, x, y, z, text, spoofed):
    m = Marker()
    m.header.frame_id = 'radar_base'
    m.header.stamp = stamp
    m.ns = 'skytrace'
    m.id = m_id + 400
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = float(z + 110.0)
    m.text = text
    m.scale.z = 38.0
    if spoofed:
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.25, 0.2, 1.0
    else:
        m.color.r, m.color.g, m.color.b, m.color.a = 0.75, 1.0, 0.85, 0.95
    return m


def make_spoof_halo(stamp, m_id, x, y, z):
    halo = Marker()
    halo.header.frame_id = 'radar_base'
    halo.header.stamp = stamp
    halo.ns = 'skytrace'
    halo.id = m_id + 300
    halo.type = Marker.SPHERE
    halo.action = Marker.ADD
    halo.pose.position.x = float(x)
    halo.pose.position.y = float(y)
    halo.pose.position.z = float(z)
    halo.scale.x = halo.scale.y = halo.scale.z = 260.0
    halo.color.r, halo.color.g, halo.color.b, halo.color.a = 1.0, 0.0, 0.0, 0.3
    return halo


class OpenSkyClient:
    """Fetches OpenSky /states/all using optional OAuth2 client credentials or anonymous mode."""

    def __init__(self, client_id='', client_secret=''):
        self.client_id = client_id.strip()
        self.client_secret = client_secret.strip()
        self._token = None
        self._token_expiry = 0.0

    def _get_bearer_token(self):
        if not (self.client_id and self.client_secret):
            return None
        now = time.time()
        if self._token and now < self._token_expiry:
            return self._token

        body = urllib.parse.urlencode({
            'grant_type': 'client_credentials',
            'client_id': self.client_id,
            'client_secret': self.client_secret,
        }).encode('utf-8')
        req = urllib.request.Request(
            OPENSKY_TOKEN_URL,
            data=body,
            headers={
                'Content-Type': 'application/x-www-form-urlencoded',
                'User-Agent': 'SKYTRACE-ROS2/1.0',
            },
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode('utf-8'))
            self._token = data.get('access_token')
            expires_in = float(data.get('expires_in', 1800))
            self._token_expiry = now + max(60.0, expires_in - 120.0)
            return self._token

    def fetch_states(self, lamin, lomin, lamax, lomax):
        query = urllib.parse.urlencode({
            'lamin': f'{lamin:.4f}',
            'lomin': f'{lomin:.4f}',
            'lamax': f'{lamax:.4f}',
            'lomax': f'{lomax:.4f}',
        })
        headers = {'User-Agent': 'SKYTRACE-ROS2/1.0'}
        token = self._get_bearer_token()
        if token:
            headers['Authorization'] = f'Bearer {token}'

        req = urllib.request.Request(f'{OPENSKY_STATES_URL}?{query}', headers=headers)
        with urllib.request.urlopen(req, timeout=12) as resp:
            remaining = resp.headers.get('X-Rate-Limit-Remaining', '?')
            payload = json.loads(resp.read().decode('utf-8'))
            return payload.get('states') or [], remaining







def make_sand_ground(stamp):
    """Lightweight desert sand floor + gentle wind-swept dunes (<4 KB per frame)."""
    markers = []

    # Main 7.5 km x 7.5 km desert floor (top surface at z = -1.0, cleanly below radar pad)
    floor = Marker()
    floor.header.frame_id = 'radar_base'
    floor.header.stamp = stamp
    floor.ns = 'skytrace'
    floor.id = 890
    floor.type = Marker.CUBE
    floor.action = Marker.ADD
    floor.pose.position.x = 0.0
    floor.pose.position.y = 0.0
    floor.pose.position.z = -4.0
    floor.pose.orientation.w = 1.0
    floor.scale.x = 7500.0
    floor.scale.y = 7500.0
    floor.scale.z = 6.0
    floor.color.r, floor.color.g, floor.color.b, floor.color.a = 0.82, 0.70, 0.48, 1.0
    markers.append(floor)

    # Compacted darker sand apron around the radar base
    apron = Marker()
    apron.header.frame_id = 'radar_base'
    apron.header.stamp = stamp
    apron.ns = 'skytrace'
    apron.id = 889
    apron.type = Marker.CYLINDER
    apron.action = Marker.ADD
    apron.pose.position.x = 0.0
    apron.pose.position.y = 0.0
    apron.pose.position.z = -0.8
    apron.pose.orientation.w = 1.0
    apron.scale.x = 650.0
    apron.scale.y = 650.0
    apron.scale.z = 1.2
    apron.color.r, apron.color.g, apron.color.b, apron.color.a = 0.74, 0.62, 0.43, 1.0
    markers.append(apron)

    # Gentle rolling desert sand dunes around the perimeter (x, y, sx, sy, sz, yaw, shade)
    dunes = [
        (1100.0,  950.0, 1300.0, 720.0,  95.0,  0.4, 0.04),
        (-1350.0, 820.0, 1450.0, 780.0, 110.0, -0.3, -0.02),
        (1450.0, -1100.0, 1500.0, 820.0, 105.0,  0.7, 0.03),
        (-1200.0, -1300.0, 1350.0, 750.0, 100.0, -0.5, -0.03),
        (2400.0,  250.0, 1600.0, 900.0, 130.0,  0.2, 0.05),
        (-2450.0, -150.0, 1550.0, 850.0, 125.0, -0.2, -0.02),
        (150.0,  2350.0, 1650.0, 880.0, 120.0,  0.8, 0.02),
        (-250.0, -2300.0, 1600.0, 860.0, 115.0, -0.7, -0.04),
        (2350.0,  2100.0, 1400.0, 800.0, 135.0,  0.5, 0.04),
        (-2250.0, 2150.0, 1450.0, 820.0, 130.0, -0.4, 0.01),
        (2200.0, -2250.0, 1500.0, 850.0, 125.0,  0.3, -0.03),
        (-2350.0, -2200.0, 1450.0, 800.0, 130.0, -0.6, 0.02),
        (850.0, -2100.0, 1150.0, 680.0,  90.0,  0.1, 0.03),
        (-950.0, 1950.0, 1200.0, 700.0,  95.0, -0.1, -0.02),
    ]
    for idx, (dx, dy, sx, sy, sz, yaw, shade) in enumerate(dunes):
        d = Marker()
        d.header.frame_id = 'radar_base'
        d.header.stamp = stamp
        d.ns = 'skytrace'
        d.id = 870 + idx
        d.type = Marker.SPHERE
        d.action = Marker.ADD
        d.pose.position.x = dx
        d.pose.position.y = dy
        d.pose.position.z = -28.0
        d.pose.orientation.z = math.sin(yaw * 0.5)
        d.pose.orientation.w = math.cos(yaw * 0.5)
        d.scale.x = sx
        d.scale.y = sy
        d.scale.z = sz
        d.color.r = min(1.0, max(0.0, 0.84 + shade))
        d.color.g = min(1.0, max(0.0, 0.72 + shade))
        d.color.b = min(1.0, max(0.0, 0.50 + shade))
        d.color.a = 1.0
        markers.append(d)

    return markers


class ReusableHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class SkytraceOpenSkyNode(Node):
    def __init__(self):
        super().__init__('skytrace_opensky_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/skytrace/markers', 10)
        self.telemetry_pub = self.create_publisher(String, '/skytrace/telemetry', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        pkg_share = get_package_share_directory('skytrace_sim')
        urdf_path = os.path.join(pkg_share, 'urdf', 'aircraft.urdf')
        self.html_path = os.path.join(pkg_share, 'web', 'index.html')
        self.root_link = get_urdf_root_link(urdf_path)

        client_id = os.environ.get('OPENSKY_CLIENT_ID', '')
        client_secret = os.environ.get('OPENSKY_CLIENT_SECRET', '')
        self.opensky = OpenSkyClient(client_id, client_secret)

        self.lock = threading.Lock()
        self.sse_clients = []
        self.log_history = deque(maxlen=80)
        self.live_aircraft = []
        self.prev_positions = {}
        self.extra_targets = []
        self.paused = False
        self.sim_time = 0.0
        self.last_wall_time = time.time()
        self.httpd = None
        self.stop_event = threading.Event()

        self._start_web_server(port=8080)
        self.poll_thread = threading.Thread(target=self._opensky_poll_loop, daemon=True)
        self.poll_thread.start()

        self.timer = self.create_timer(0.05, self.timer_callback)
        mode = 'OAuth2 Authenticated' if (client_id and client_secret) else 'Anonymous'
        self._emit_log(
            'INFO',
            f'SKYTRACE OpenSky Live ({mode}) | Origin: ({RADAR_LAT:.2f}, {RADAR_LON:.2f}) | GUI: http://localhost:8080',
        )

    def _emit_log(self, level: str, msg: str):
        if level == 'WARN':
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)
        with self.lock:
            self.log_history.append({'level': level, 'msg': msg})

    def _opensky_poll_loop(self):
        lamin = RADAR_LAT - BBOX_HALF_DEG
        lamax = RADAR_LAT + BBOX_HALF_DEG
        lomin = RADAR_LON - BBOX_HALF_DEG
        lomax = RADAR_LON + BBOX_HALF_DEG

        while not self.stop_event.is_set():
            try:
                states, remaining = self.opensky.fetch_states(lamin, lomin, lamax, lomax)
                now = time.time()
                parsed = []

                for s in states:
                    icao24 = (s[0] or '').strip().upper()
                    callsign = (s[1] or '').strip()
                    lon = s[5]
                    lat = s[6]
                    baro_alt = s[7]
                    on_ground = bool(s[8])
                    vel = s[9]
                    track = s[10]
                    vrate = s[11]
                    geo_alt = s[13]

                    if lat is None or lon is None or on_ground:
                        continue

                    ex, ny = latlon_to_local_m(lat, lon)
                    rng_m = math.hypot(ex, ny)
                    if rng_m > REAL_RADAR_RANGE_M:
                        continue

                    alt_m = geo_alt if geo_alt is not None else (baro_alt if baro_alt is not None else 3500.0)
                    speed_ms = float(vel) if vel is not None else 210.0
                    hdg_deg = float(track) if track is not None else 0.0
                    vz_ms = float(vrate) if vrate is not None else 0.0

                    hdg_rad = math.radians(hdg_deg)
                    vx = speed_ms * math.sin(hdg_rad)
                    vy = speed_ms * math.cos(hdg_rad)

                    # Physics-based live spoofing / anomaly checks
                    t_type = 'verified'
                    conf = 97
                    anomaly_note = ''

                    if baro_alt is not None and geo_alt is not None and abs(geo_alt - baro_alt) > 1200.0:
                        t_type = 'suspicious'
                        conf = 68
                        anomaly_note = 'GNSS ALT DRIFT'
                    elif icao24 in self.prev_positions:
                        px, py, pt = self.prev_positions[icao24]
                        dt_p = max(1.0, now - pt)
                        implied_spd = math.hypot(ex - px, ny - py) / dt_p
                        if implied_spd > 450.0:
                            t_type = 'suspicious'
                            conf = 62
                            anomaly_note = 'POS JUMP SPOOF'
                    elif not callsign:
                        t_type = 'suspicious'
                        conf = 79
                        anomaly_note = 'NO CALLSIGN'

                    self.prev_positions[icao24] = (ex, ny, now)
                    track_id = callsign if callsign else f'ICAO-{icao24[:6]}'

                    parsed.append({
                        'id': track_id,
                        'icao24': icao24,
                        'x_m': ex,
                        'y_m': ny,
                        'alt_m': max(300.0, float(alt_m)),
                        'vx': vx,
                        'vy': vy,
                        'vz': vz_ms,
                        'speed': speed_ms,
                        'hdg': int(round(hdg_deg)) % 360,
                        'range_m': rng_m,
                        'type': t_type,
                        'conf': conf,
                        'note': anomaly_note,
                        'stamp_wall': now,
                    })

                # Keep closest aircraft inside the radar sector
                parsed.sort(key=lambda a: a['range_m'])
                with self.lock:
                    self.live_aircraft = parsed[:20]

                shown = min(len(parsed), MAX_URDF_SLOTS)
                self._emit_log(
                    'INFO',
                    f'OpenSky poll OK: {len(parsed)} live flights in 250km ({shown} URDF active) | Credits left: {remaining}',
                )
                for a in parsed[:MAX_URDF_SLOTS]:
                    if a['type'] != 'verified':
                        self._emit_log(
                            'WARN',
                            f"ANOMALY {a['id']} ({a['note']}) | Alt: {a['alt_m']:.0f}m | Rng: {a['range_m']/1000.0:.1f}km",
                        )

            except urllib.error.HTTPError as e:
                retry = e.headers.get('X-Rate-Limit-Retry-After-Seconds', '')
                self._emit_log('WARN', f'OpenSky HTTP {e.code} (retry after {retry or POLL_INTERVAL_SEC}s)')
                if e.code == 429 and retry.isdigit():
                    self.stop_event.wait(min(120.0, float(retry)))
                    continue
            except Exception as e:
                self._emit_log('WARN', f'OpenSky fetch error: {e}')

            self.stop_event.wait(POLL_INTERVAL_SEC)

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
            self._emit_log('INFO', f'GUI command: Radar {state}')
        elif action == 'reset':
            with self.lock:
                self.extra_targets.clear()
                self.paused = False
            self._emit_log('INFO', 'GUI command: Cleared injected threats')
        elif action == 'inject':
            t_type = cmd.get('type', 'ghost')
            with self.lock:
                idx = len(self.extra_targets) + 1
                tid = f'GHOST-INJ-{idx:02d}' if t_type == 'ghost' else f'SPOOF-INJ-{idx:02d}'
                self.extra_targets.append({
                    'id': tid,
                    'type': t_type,
                    'radius_m': random.uniform(50000.0, 210000.0),
                    'ang_speed': random.uniform(0.015, 0.035),
                    'phase': random.uniform(0.0, 2.0 * math.pi),
                    'alt_m': random.uniform(4000.0, 11000.0),
                })
            self._emit_log('WARN', f'Injected synthetic {t_type.upper()} threat: {tid}')

    def timer_callback(self):
        now = time.time()
        dt = now - self.last_wall_time
        self.last_wall_time = now

        with self.lock:
            if not self.paused:
                self.sim_time += dt
            sim_time = self.sim_time
            paused = self.paused
            live_copy = list(self.live_aircraft)
            extra_copy = list(self.extra_targets)

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        pan_angle = (sim_time * 1.0) % (2.0 * math.pi)
        pan_deg = math.degrees(pan_angle)
        tilt_angle = math.radians(10.0)

        marker_array.markers.extend(make_sand_ground(stamp))
        marker_array.markers.extend(make_ground_radar_station(stamp, pan_angle, tilt_angle))
        marker_array.markers.append(
            make_wireframe_radar(stamp, VISUAL_RADAR_RANGE_M, pan_angle, tilt_angle, math.radians(35.0))
        )

        tracks_payload = []

        # Map up to 9 closest live OpenSky flights onto URDF aircraft_0 .. aircraft_8
        for i in range(MAX_URDF_SLOTS):
            if i < len(live_copy):
                ac = live_copy[i]
                elapsed = 0.0 if paused else min(25.0, max(0.0, now - ac['stamp_wall']))

                real_x = ac['x_m'] + ac['vx'] * elapsed
                real_y = ac['y_m'] + ac['vy'] * elapsed
                real_alt = max(300.0, ac['alt_m'] + ac['vz'] * elapsed)
                real_rng = int(math.hypot(real_x, real_y))

                vis_x = real_x * VISUAL_XY_SCALE
                vis_y = real_y * VISUAL_XY_SCALE
                vis_z = max(240.0, min(1400.0, real_alt * VISUAL_Z_SCALE))

                qx, qy, qz, qw = velocity_to_quaternion(ac['vx'], ac['vy'], ac['vz'], bank_rad=0.0)

                tf_msg = TransformStamped()
                tf_msg.header.stamp = stamp
                tf_msg.header.frame_id = 'radar_base'
                tf_msg.child_frame_id = f'aircraft_{i}_{self.root_link}'
                tf_msg.transform.translation.x = float(vis_x)
                tf_msg.transform.translation.y = float(vis_y)
                tf_msg.transform.translation.z = float(vis_z)
                tf_msg.transform.rotation.x = float(qx)
                tf_msg.transform.rotation.y = float(qy)
                tf_msg.transform.rotation.z = float(qz)
                tf_msg.transform.rotation.w = float(qw)
                self.tf_broadcaster.sendTransform(tf_msg)

                is_spoof = ac['type'] != 'verified'
                marker_array.markers.append(
                    make_front_vector(stamp, i, vis_x, vis_y, vis_z, ac['vx'], ac['vy'], ac['vz'], is_spoof)
                )
                label_txt = f"{ac['id']} | {int(real_alt)}m"
                if ac['note']:
                    label_txt = f"{ac['id']} | {ac['note']}"
                marker_array.markers.append(
                    make_callsign_label(stamp, i, vis_x, vis_y, vis_z, label_txt, is_spoof)
                )
                if is_spoof:
                    marker_array.markers.append(make_spoof_halo(stamp, i, vis_x, vis_y, vis_z))
                else:
                    marker_array.markers.append(make_delete_marker(stamp, i + 300))

                tracks_payload.append({
                    'id': ac['id'],
                    'x': round(real_x, 1),
                    'y': round(real_y, 1),
                    'alt': round(real_alt, 1),
                    'speed': round(ac['speed'], 1),
                    'range': real_rng,
                    'hdg': ac['hdg'],
                    'type': ac['type'],
                    'conf': ac['conf'],
                })
            else:
                # Park unused URDF slots out of view and clear their markers
                tf_msg = TransformStamped()
                tf_msg.header.stamp = stamp
                tf_msg.header.frame_id = 'radar_base'
                tf_msg.child_frame_id = f'aircraft_{i}_{self.root_link}'
                tf_msg.transform.translation.z = -5000.0
                tf_msg.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(tf_msg)
                marker_array.markers.append(make_delete_marker(stamp, i + 100))
                marker_array.markers.append(make_delete_marker(stamp, i + 300))
                marker_array.markers.append(make_delete_marker(stamp, i + 400))

        # Include remaining live flights (beyond 9) in the AEROGUARD dashboard & radar scope
        for ac in live_copy[MAX_URDF_SLOTS:]:
            elapsed = 0.0 if paused else min(25.0, max(0.0, now - ac['stamp_wall']))
            real_x = ac['x_m'] + ac['vx'] * elapsed
            real_y = ac['y_m'] + ac['vy'] * elapsed
            real_alt = max(300.0, ac['alt_m'] + ac['vz'] * elapsed)
            tracks_payload.append({
                'id': ac['id'],
                'x': round(real_x, 1),
                'y': round(real_y, 1),
                'alt': round(real_alt, 1),
                'speed': round(ac['speed'], 1),
                'range': int(math.hypot(real_x, real_y)),
                'hdg': ac['hdg'],
                'type': ac['type'],
                'conf': ac['conf'],
            })

        # Synthetic spoof/ghost threats injected from AEROGUARD GUI
        for idx, ext in enumerate(extra_copy):
            ang = sim_time * ext['ang_speed'] + ext['phase']
            real_x = ext['radius_m'] * math.cos(ang)
            real_y = ext['radius_m'] * math.sin(ang)
            real_alt = ext['alt_m']
            vx = -ext['radius_m'] * ext['ang_speed'] * math.sin(ang)
            vy = ext['radius_m'] * ext['ang_speed'] * math.cos(ang)
            speed_ms = math.hypot(vx, vy)
            hdg = int((90.0 - math.degrees(math.atan2(vy, vx))) % 360)

            vis_x = real_x * VISUAL_XY_SCALE
            vis_y = real_y * VISUAL_XY_SCALE
            vis_z = max(240.0, min(1400.0, real_alt * VISUAL_Z_SCALE))
            mid = 50 + idx

            marker_array.markers.append(make_front_vector(stamp, mid, vis_x, vis_y, vis_z, vx, vy, 0.0, True))
            marker_array.markers.append(make_callsign_label(stamp, mid, vis_x, vis_y, vis_z, f"{ext['id']} | SPOOF", True))
            marker_array.markers.append(make_spoof_halo(stamp, mid, vis_x, vis_y, vis_z))

            tracks_payload.append({
                'id': ext['id'],
                'x': round(real_x, 1),
                'y': round(real_y, 1),
                'alt': round(real_alt, 1),
                'speed': round(speed_ms, 1),
                'range': int(math.hypot(real_x, real_y)),
                'hdg': hdg,
                'type': ext['type'],
                'conf': 90 if ext['type'] == 'ghost' else 74,
            })

        self.publisher_.publish(marker_array)

        with self.lock:
            payload_str = json.dumps({
                'sim_time': round(sim_time, 2),
                'pan_deg': round(pan_deg, 1),
                'radar_range': REAL_RADAR_RANGE_M,
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
    node = SkytraceOpenSkyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_event.set()
    if node.httpd:
        node.httpd.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
