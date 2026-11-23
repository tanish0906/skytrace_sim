from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import queue
import random
import threading
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


# Adjust to 90.0, -90.0, or 180.0 if your CAD aircraft nose faces sideways or backward
YAW_OFFSET_DEG = 0.0
RADAR_DISH_Z = 140.0


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
    horiz_speed = math.hypot(vx, vy)
    yaw = math.atan2(vy, vx) + math.radians(YAW_OFFSET_DEG)
    pitch = -math.atan2(vz, max(horiz_speed, 1e-6))
    return euler_to_quaternion(bank_rad, pitch, yaw)



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
    label.text = 'SKYTRACE 3D AESA RADAR'
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


def make_spoof_highlight(stamp, m_id, x, y, z, label='SPOOF DETECTED'):
    text_m = Marker()
    text_m.header.frame_id = 'radar_base'
    text_m.header.stamp = stamp
    text_m.ns = 'skytrace'
    text_m.id = m_id + 200
    text_m.type = Marker.TEXT_VIEW_FACING
    text_m.action = Marker.ADD
    text_m.pose.position.x = float(x)
    text_m.pose.position.y = float(y)
    text_m.pose.position.z = float(z + 115.0)
    text_m.text = label
    text_m.scale.z = 45.0
    text_m.color.r, text_m.color.g, text_m.color.b, text_m.color.a = 1.0, 0.1, 0.1, 1.0

    halo_m = Marker()
    halo_m.header.frame_id = 'radar_base'
    halo_m.header.stamp = stamp
    halo_m.ns = 'skytrace'
    halo_m.id = m_id + 300
    halo_m.type = Marker.SPHERE
    halo_m.action = Marker.ADD
    halo_m.pose.position.x = float(x)
    halo_m.pose.position.y = float(y)
    halo_m.pose.position.z = float(z)
    halo_m.scale.x = halo_m.scale.y = halo_m.scale.z = 260.0
    halo_m.color.r, halo_m.color.g, halo_m.color.b, halo_m.color.a = 1.0, 0.0, 0.0, 0.3

    return [text_m, halo_m]







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


class SkytraceUrdfNode(Node):
    def __init__(self):
        super().__init__('skytrace_urdf_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/skytrace/markers', 10)
        self.telemetry_pub = self.create_publisher(String, '/skytrace/telemetry', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        pkg_share = get_package_share_directory('skytrace_sim')
        urdf_path = os.path.join(pkg_share, 'urdf', 'aircraft.urdf')
        self.html_path = os.path.join(pkg_share, 'web', 'index.html')
        self.root_link = get_urdf_root_link(urdf_path)

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
        self._emit_log(
            'INFO',
            f'SKYTRACE URDF active | Root link: {self.root_link} | GUI: http://localhost:8080',
        )

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
                    'speed': random.uniform(35, 85),
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
        tilt_angle = math.radians(10.0)

        # Ground radar system + rotating 360-degree wireframe beam
        marker_array.markers.extend(make_sand_ground(stamp))
        marker_array.markers.extend(make_ground_radar_station(stamp, pan_angle, tilt_angle))
        marker_array.markers.append(
            make_wireframe_radar(stamp, self.radar_range, pan_angle, tilt_angle, math.radians(35.0))
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
            speed = 2+ (i * 0.05)
            angle = sim_time * speed + (i * math.pi / 3.0)

            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 1500.0 + (i * 50.0) + (100.0 * math.sin(sim_time + i))

            vx = -radius * speed * math.sin(angle)
            vy = radius * speed * math.cos(angle)
            vz = 100.0 * math.cos(sim_time + i)
            qx, qy, qz, qw = velocity_to_quaternion(vx, vy, vz, bank_rad=0.0)

            # Broadcast live TF2 transform for each URDF aircraft
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = 'radar_base'
            tf_msg.child_frame_id = f'aircraft_{i}_{self.root_link}'
            tf_msg.transform.translation.x = float(x)
            tf_msg.transform.translation.y = float(y)
            tf_msg.transform.translation.z = float(z)
            tf_msg.transform.rotation.x = float(qx)
            tf_msg.transform.rotation.y = float(qy)
            tf_msg.transform.rotation.z = float(qz)
            tf_msg.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(tf_msg)

            # Small heading vector directly in front of each aircraft
            marker_array.markers.append(
                make_front_vector(stamp, i, x, y, z, vx, vy, vz, is_spoofed)
            )

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

            marker_array.markers.append(
                make_front_vector(stamp, 50 + idx, x, y, z, vx, vy, vz, True)
            )
            marker_array.markers.extend(
                make_spoof_highlight(stamp, 50 + idx, x, y, z, f"{ext['id']} | SPOOF")
            )
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
    node = SkytraceUrdfNode()
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
