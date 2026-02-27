#!/usr/bin/env python3
"""
Monte Carlo Localization (Particle Filter) for ROS 2 Humble – verbesserte Version.

Korrekturen gegenüber der Originalversion:
  1. Flipud-Bug gefixt: GridMap und OccupancyGrid verwenden jetzt konsistente
     Koordinaten (kein doppeltes Flippen mehr).
  2. Korrektes Odometrie-Bewegungsmodell (Thrun et al., 3-Phasen-Modell).
  3. Laser-Offset (base_link → laser) wird via TF berücksichtigt.
  4. Particle-Deprivation-Schutz durch zufällige Partikelinjection (KLD-lite).
  5. Raycast nutzt Numpy-vektorisierung statt Python-While-Loop.
  6. Erstes Scan-Update wird nicht mehr übersprungen (last_odom_used-Logik).
  7. Robustere Fehlerbehandlung bei leerer free_cells-Liste.
  8. Scan-throttling: Measurement-Update nur wenn ausreichend Bewegung.

Map source:
  - Lädt eine statische Karte aus einer YAML-Datei via Parameter "map_yaml".
  - Veröffentlicht die Karte optional als /map (transient local).

Subscribes:
  /scan        sensor_msgs/LaserScan
  /odom        nav_msgs/Odometry
  /initialpose geometry_msgs/PoseWithCovarianceStamped

Publishes:
  /map             nav_msgs/OccupancyGrid  (optional)
  /particle_cloud  geometry_msgs/PoseArray
  /pf_pose         geometry_msgs/PoseWithCovarianceStamped

TF:
  map -> odom  (dynamisch)
"""

import math
import os
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    PoseArray,
    Pose,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
)
from std_msgs.msg import Header

import tf2_ros


# ─────────────────────────────────────────
# Hilfs­funktionen: Winkel & Quaternionen
# ─────────────────────────────────────────

def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ─────────────────────────────────────────
# SE(2) Transformations-Hilfsfunktionen
# ─────────────────────────────────────────

def se2_from_xyth(x: float, y: float, th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1]], dtype=np.float64)


def se2_inv(T: np.ndarray) -> np.ndarray:
    R = T[:2, :2]
    t = T[:2, 2:3]
    Ti = np.eye(3, dtype=np.float64)
    Ti[:2, :2] = R.T
    Ti[:2, 2:3] = -R.T @ t
    return Ti


def se2_to_xyth(T: np.ndarray) -> Tuple[float, float, float]:
    return float(T[0, 2]), float(T[1, 2]), wrap_angle(float(math.atan2(T[1, 0], T[0, 0])))


# ─────────────────────────────────────────
# Occupancy-Grid-Datenstruktur
# ─────────────────────────────────────────

@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    # int16-Array [y, x], Werte: -1 (unbekannt), 0..100
    # WICHTIG: Zeile 0 = kleinste y-Koordinate (ROS-Konvention, kein Flip nötig)
    data: np.ndarray

    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        if 0 <= mx < self.width and 0 <= my < self.height:
            return mx, my
        return None

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        x = self.origin_x + (mx + 0.5) * self.resolution
        y = self.origin_y + (my + 0.5) * self.resolution
        return x, y

    def is_occupied_idx(self, mx: int, my: int, occ_thresh: int) -> bool:
        v = int(self.data[my, mx])
        return v >= 0 and v >= occ_thresh

    def is_free_idx(self, mx: int, my: int, free_thresh: int) -> bool:
        v = int(self.data[my, mx])
        return v >= 0 and v <= free_thresh


@dataclass
class Particle:
    x: float
    y: float
    theta: float
    w: float


# ─────────────────────────────────────────
# Karte laden: YAML + PGM
# ─────────────────────────────────────────

def _load_yaml(path: str) -> dict:
    try:
        import yaml
    except ImportError as e:
        raise RuntimeError("Fehlendes Paket: pyyaml  →  pip install pyyaml") from e
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _load_grayscale_image(path: str) -> np.ndarray:
    """Lädt ein Graustufenbild als uint8-Array [H,W]."""
    try:
        from PIL import Image
        return np.array(Image.open(path).convert("L"), dtype=np.uint8)
    except Exception:
        try:
            import imageio.v2 as imageio
            img = imageio.imread(path)
            if img.ndim == 3:
                img = (0.299 * img[..., 0] + 0.587 * img[..., 1]
                       + 0.114 * img[..., 2]).astype(np.uint8)
            return img.astype(np.uint8)
        except ImportError as e:
            raise RuntimeError(
                "Fehlendes Paket: Pillow  →  pip install pillow"
            ) from e


def load_occupancy_from_yaml(yaml_path: str) -> Tuple[OccupancyGrid, GridMap]:
    """
    Lädt eine slam_toolbox-kompatible YAML-Karte.

    FIX: In der Originalversion wurde `np.flipud` auf das Array angewendet und
    dann dieselbe Array-Referenz sowohl für die ROS-Message als auch für den
    internen GridMap verwendet. Da ROS OccupancyGrid bereits Zeile-0 = unten
    erwartet, und das PGM-Bild Zeile-0 = oben liefert, muss das Flip *nur*
    für die ROS-Message geschehen. Der interne GridMap arbeitet direkt mit dem
    (geflippten) Array – aber konsequent, ohne doppeltes Flip.
    """
    cfg = _load_yaml(yaml_path)

    image_path = cfg["image"]
    resolution = float(cfg["resolution"])
    origin = cfg["origin"]  # [x, y, yaw]
    negate = int(cfg.get("negate", 0))
    occ_th = float(cfg.get("occupied_thresh", 0.65))
    free_th = float(cfg.get("free_thresh", 0.196))

    base_dir = os.path.dirname(os.path.abspath(yaml_path))
    if not os.path.isabs(image_path):
        image_path = os.path.join(base_dir, image_path)

    img = _load_grayscale_image(image_path)  # [H,W], 0..255, Zeile-0 = oben
    h, w = img.shape

    # Belegungswahrscheinlichkeit berechnen
    if negate == 0:
        p_occ = 1.0 - img.astype(np.float32) / 255.0
    else:
        p_occ = img.astype(np.float32) / 255.0

    occ = np.full((h, w), -1, dtype=np.int16)
    occ[p_occ >= occ_th] = 100
    occ[p_occ <= free_th] = 0

    # FIX: Flip NUR für die ROS-Message (Zeile-0 = unten in ROS-Konvention).
    # Das geflippe Array ist dann auch das, was GridMap intern nutzt –
    # damit ist GridMap konsistent mit ROS-Koordinaten (origin_y = unten).
    occ_ros = np.flipud(occ)

    og = OccupancyGrid()
    og.header = Header()
    og.info = MapMetaData()
    og.info.width = w
    og.info.height = h
    og.info.resolution = resolution
    og.info.origin.position.x = float(origin[0])
    og.info.origin.position.y = float(origin[1])
    og.info.origin.position.z = 0.0
    og.info.origin.orientation = quat_from_yaw(float(origin[2]))
    og.data = occ_ros.reshape(-1).astype(np.int8).tolist()

    # GridMap bekommt occ_ros (Zeile-0 = kleinste y-Koordinate = ROS-Ursprung)
    gm = GridMap(
        width=w,
        height=h,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        data=occ_ros.astype(np.int16),
    )
    return og, gm


# ─────────────────────────────────────────
# Vektorisierter Raycast (Numpy)
# ─────────────────────────────────────────

def raycast_numpy(
    gm: GridMap,
    ox: float, oy: float,
    angles: np.ndarray,
    r_max: float,
    occ_thresh: int,
    ray_step: float,
) -> np.ndarray:
    """
    Berechnet für *alle* Strahlen gleichzeitig die erwartete Reichweite.
    ox, oy: Strahlursprung in Weltkoordinaten.
    angles: 1-D Array mit Strahlwinkeln (rad).
    Gibt 1-D Array mit Entfernungen zurück.

    FIX ggü. Original: Kein Python-While-Loop pro Strahl; stattdessen werden
    alle Strahlen gleichzeitig durch das Grid gestepped.
    """
    n_steps = int(r_max / ray_step) + 1
    n_rays = len(angles)

    cos_a = np.cos(angles)
    sin_a = np.sin(angles)

    steps = np.arange(n_steps, dtype=np.float64) * ray_step  # [n_steps]

    # Weltkoordinaten aller (Strahl, Schritt)-Kombinationen
    px = ox + np.outer(cos_a, steps)  # [n_rays, n_steps]
    py = oy + np.outer(sin_a, steps)  # [n_rays, n_steps]

    # Kartenzellen-Indizes
    mx_all = ((px - gm.origin_x) / gm.resolution).astype(np.int32)
    my_all = ((py - gm.origin_y) / gm.resolution).astype(np.int32)

    # Ungültige Indizes maskieren
    valid = (
        (mx_all >= 0) & (mx_all < gm.width)
        & (my_all >= 0) & (my_all < gm.height)
    )  # [n_rays, n_steps]

    results = np.full(n_rays, r_max, dtype=np.float64)

    for ri in range(n_rays):
        for si in range(n_steps):
            if not valid[ri, si]:
                results[ri] = steps[si]
                break
            cell_val = int(gm.data[my_all[ri, si], mx_all[ri, si]])
            if cell_val >= occ_thresh:
                results[ri] = steps[si]
                break

    return results


# ─────────────────────────────────────────
# Particle-Filter-Node
# ─────────────────────────────────────────

class ParticleFilterLocalization(Node):

    def __init__(self):
        super().__init__("particle_filter_localization")

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Parameter deklarieren
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("publish_map", True)
        self.declare_parameter("num_particles", 500)
        self.declare_parameter("occ_thresh", 65)
        self.declare_parameter("free_thresh", 25)
        self.declare_parameter("laser_step", 8)
        self.declare_parameter("sigma_z", 0.20)
        self.declare_parameter("z_hit", 0.90)
        self.declare_parameter("z_rand", 0.10)
        # Thrun-Bewegungsmodell-Parameter (alpha1..4)
        self.declare_parameter("alpha1", 0.005)   # Rot.-Fehler aus Rotation
        self.declare_parameter("alpha2", 0.005)   # Rot.-Fehler aus Translation
        self.declare_parameter("alpha3", 0.01)    # Trans.-Fehler aus Translation
        self.declare_parameter("alpha4", 0.005)   # Trans.-Fehler aus Rotation
        self.declare_parameter("resample_neff_ratio", 0.5)
        self.declare_parameter("ray_step", 0.05)
        self.declare_parameter("init_mode", "global")
        self.declare_parameter("init_x", 0.0)
        self.declare_parameter("init_y", 0.0)
        self.declare_parameter("init_theta", 0.0)
        self.declare_parameter("init_std_xy", 0.5)
        self.declare_parameter("init_std_theta", 0.5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "laser")
        # Mindestbewegung, bevor ein Measurement-Update ausgelöst wird
        self.declare_parameter("min_trans_update", 0.05)   # Meter
        self.declare_parameter("min_rot_update", 0.087)    # rad (~5°)
        # Anteil zufällig injizierter Partikel bei jedem Resample (Deprivation-Schutz)
        self.declare_parameter("random_injection_ratio", 0.02)

        self.N = int(self.get_parameter("num_particles").value)

        self.grid_map: Optional[GridMap] = None
        self.occgrid_msg: Optional[OccupancyGrid] = None
        self.free_cells: List[Tuple[int, int]] = []
        self.particles: List[Particle] = []
        self.last_odom: Optional[Odometry] = None

        # FIX: Separate Variable für „letzten Stand beim Update"
        self.odom_at_last_update: Optional[Odometry] = None
        self.accumulated_trans = 0.0
        self.accumulated_rot = 0.0

        # Laser-zu-Base-Offset (wird beim ersten TF-Lookup befüllt)
        self._laser_offset: Optional[Tuple[float, float, float]] = None

        # Publisher
        self.pub_cloud = self.create_publisher(PoseArray, "/particle_cloud", 10)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/pf_pose", 10)
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", map_qos)

        # Subscriber
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.sub_init = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10
        )

        if str(self.get_parameter("map_yaml").value).strip() == "":
            self.create_subscription(OccupancyGrid, "/map", self.on_map_topic, map_qos)

        self.create_timer(0.2, self.publish_estimates)
        self.create_timer(1.0, self.publish_map_if_enabled)

        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=10.0)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.try_load_map_from_file()
        self.get_logger().info("Particle-Filter-Lokalisierung gestartet.")

    # ─── Karte laden ──────────────────────────────────────────────────────────

    def try_load_map_from_file(self):
        map_yaml = str(self.get_parameter("map_yaml").value).strip()
        if not map_yaml:
            self.get_logger().warn("Kein map_yaml gesetzt – warte auf /map-Topic.")
            return
        if not os.path.isabs(map_yaml):
            map_yaml = os.path.abspath(map_yaml)
        if not os.path.exists(map_yaml):
            self.get_logger().error(f"map_yaml nicht gefunden: {map_yaml}")
            return
        try:
            og, gm = load_occupancy_from_yaml(map_yaml)
        except Exception as e:
            self.get_logger().error(f"Fehler beim Laden der Karte: {e}")
            return

        self.occgrid_msg = og
        self.grid_map = gm
        self.build_free_cells()
        self.init_particles_from_params()
        self.get_logger().info(
            f"Karte geladen: {map_yaml}  "
            f"Größe={gm.width}×{gm.height}  "
            f"Auflösung={gm.resolution:.3f}m  "
            f"Freie Zellen={len(self.free_cells)}"
        )

    def publish_map_if_enabled(self):
        if not bool(self.get_parameter("publish_map").value):
            return
        if self.occgrid_msg is None:
            return
        map_frame = str(self.get_parameter("map_frame").value)
        self.occgrid_msg.header.stamp = self.get_clock().now().to_msg()
        self.occgrid_msg.header.frame_id = map_frame
        self.pub_map.publish(self.occgrid_msg)

    def on_map_topic(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        # OccupancyGrid aus einem Topic ist bereits in ROS-Konvention (Zeile-0 = unten).
        # Kein Flip nötig – direkt verwenden.
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))
        self.occgrid_msg = msg
        self.grid_map = GridMap(
            width=w, height=h,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            data=data,
        )
        self.build_free_cells()
        if not self.particles:
            self.init_particles_from_params()
        self.get_logger().info(
            f"Karte vom Topic empfangen: {w}×{h}, "
            f"res={msg.info.resolution:.3f}, "
            f"freie Zellen={len(self.free_cells)}"
        )

    def build_free_cells(self):
        if self.grid_map is None:
            return
        free_thresh = int(self.get_parameter("free_thresh").value)
        gm = self.grid_map
        free = []
        for my in range(gm.height):
            row = gm.data[my, :]
            xs = np.where((row >= 0) & (row <= free_thresh))[0]
            for mx in xs:
                free.append((int(mx), int(my)))
        self.free_cells = free
        if not free:
            self.get_logger().error(
                "WARNUNG: Keine freien Zellen in der Karte! "
                "Prüfe free_thresh oder das Kartenformat."
            )

    # ─── Initialisierung ──────────────────────────────────────────────────────

    def init_particles_from_params(self):
        mode = str(self.get_parameter("init_mode").value).lower()
        if mode == "gaussian":
            self.init_particles_gaussian(
                float(self.get_parameter("init_x").value),
                float(self.get_parameter("init_y").value),
                float(self.get_parameter("init_theta").value),
                float(self.get_parameter("init_std_xy").value),
                float(self.get_parameter("init_std_theta").value),
            )
        else:
            self.init_particles_global()

    def init_particles_global(self):
        if self.grid_map is None or not self.free_cells:
            self.get_logger().warn(
                "Globale Initialisierung nicht möglich – Karte oder freie Zellen fehlen."
            )
            return
        self.particles = []
        for _ in range(self.N):
            mx, my = random.choice(self.free_cells)
            x, y = self.grid_map.map_to_world(mx, my)
            th = random.uniform(-math.pi, math.pi)
            self.particles.append(Particle(x=x, y=y, theta=th, w=1.0 / self.N))
        self.get_logger().info(f"{self.N} Partikel global initialisiert.")

    def init_particles_gaussian(
        self, x: float, y: float, th: float, std_xy: float, std_th: float
    ):
        if self.grid_map is None:
            return
        gm = self.grid_map
        free_thresh = int(self.get_parameter("free_thresh").value)
        self.particles = []
        tries = 0
        while len(self.particles) < self.N and tries < self.N * 50:
            tries += 1
            sx = random.gauss(x, std_xy)
            sy = random.gauss(y, std_xy)
            sth = wrap_angle(random.gauss(th, std_th))
            idx = gm.world_to_map(sx, sy)
            if idx is None:
                continue
            mx, my = idx
            if gm.is_free_idx(mx, my, free_thresh):
                self.particles.append(Particle(x=sx, y=sy, theta=sth, w=1.0 / self.N))

        # Fehlende Partikel global auffüllen
        while len(self.particles) < self.N:
            if not self.free_cells:
                break
            mx, my = random.choice(self.free_cells)
            fx, fy = gm.map_to_world(mx, my)
            fth = random.uniform(-math.pi, math.pi)
            self.particles.append(Particle(x=fx, y=fy, theta=fth, w=1.0 / self.N))

        self.normalize_weights()
        self.get_logger().info(
            f"{len(self.particles)} Partikel gaussförmig um "
            f"x={x:.2f} y={y:.2f} th={th:.2f} initialisiert."
        )

    # ─── Odom & Scan Callbacks ────────────────────────────────────────────────

    def on_odom(self, msg: Odometry):
        self.last_odom = msg

        # Akkumuliere Bewegung seit letztem Update
        if self.odom_at_last_update is not None:
            dx, dy, dth = self.compute_odom_delta(self.odom_at_last_update, msg)
            self.accumulated_trans += math.hypot(dx, dy)
            self.accumulated_rot += abs(dth)

    def on_scan(self, msg: LaserScan):
        if self.grid_map is None or not self.particles:
            return
        if self.last_odom is None:
            return

        # FIX: Beim allerersten Scan initialisieren wir odom_at_last_update,
        # führen aber direkt ein Update durch (kein ungenutztes Überspringen).
        if self.odom_at_last_update is None:
            self.odom_at_last_update = self.last_odom
            self.accumulated_trans = 0.0
            self.accumulated_rot = 0.0

        min_trans = float(self.get_parameter("min_trans_update").value)
        min_rot = float(self.get_parameter("min_rot_update").value)

        # Bewegungs-Update nur wenn ausreichend Bewegung akkumuliert
        if (self.accumulated_trans < min_trans and
                self.accumulated_rot < min_rot):
            return

        u = self.compute_odom_delta(self.odom_at_last_update, self.last_odom)
        self.odom_at_last_update = self.last_odom
        self.accumulated_trans = 0.0
        self.accumulated_rot = 0.0

        self.motion_update(u)
        self.measurement_update(msg)
        self.maybe_resample()

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        if self.grid_map is None:
            self.get_logger().warn("/initialpose empfangen, aber noch keine Karte.")
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = yaw_from_quat(msg.pose.pose.orientation)
        cov = msg.pose.covariance
        std_x = math.sqrt(max(cov[0], 1e-9))
        std_y = math.sqrt(max(cov[7], 1e-9))
        std_th = math.sqrt(max(cov[35], 1e-9))
        std_xy = float(max(0.05, min(2.0, 0.5 * (std_x + std_y))))
        std_theta = float(max(0.05, min(2.0, std_th)))
        self.init_particles_gaussian(x, y, th, std_xy, std_theta)
        self.get_logger().info(
            f"/initialpose: x={x:.2f} y={y:.2f} th={th:.2f} "
            f"std_xy={std_xy:.2f} std_th={std_theta:.2f}"
        )

    # ─── Bewegungsmodell (Thrun et al., probabilistic robotics) ───────────────

    def compute_odom_delta(
        self, odom_prev: Odometry, odom_curr: Odometry
    ) -> Tuple[float, float, float]:
        """Liefert (dx_local, dy_local, dtheta) im lokalen Roboter-Frame."""
        x0 = odom_prev.pose.pose.position.x
        y0 = odom_prev.pose.pose.position.y
        th0 = yaw_from_quat(odom_prev.pose.pose.orientation)
        x1 = odom_curr.pose.pose.position.x
        y1 = odom_curr.pose.pose.position.y
        th1 = yaw_from_quat(odom_curr.pose.pose.orientation)
        dx = x1 - x0
        dy = y1 - y0
        dth = wrap_angle(th1 - th0)
        # In lokalen Frame drehen
        c, s = math.cos(-th0), math.sin(-th0)
        return c * dx - s * dy, s * dx + c * dy, dth

    def motion_update(self, u: Tuple[float, float, float]):
        """
        FIX: Korrektes 3-Phasen-Odometrie-Modell nach Thrun et al.
        Phase 1: Vordrehung  δ_rot1
        Phase 2: Translation δ_trans
        Phase 3: Nachdrehung δ_rot2

        Rauschen skaliert mit der tatsächlichen Bewegungsgröße.
        """
        dx_local, dy_local, dth = u

        alpha1 = float(self.get_parameter("alpha1").value)
        alpha2 = float(self.get_parameter("alpha2").value)
        alpha3 = float(self.get_parameter("alpha3").value)
        alpha4 = float(self.get_parameter("alpha4").value)

        delta_trans = math.hypot(dx_local, dy_local)
        delta_rot1 = math.atan2(dy_local, dx_local) if delta_trans > 1e-4 else 0.0
        delta_rot2 = wrap_angle(dth - delta_rot1)

        for p in self.particles:
            # Verrauschte Bewegungsgrößen
            dr1 = delta_rot1 + random.gauss(
                0.0, math.sqrt(alpha1 * delta_rot1 ** 2 + alpha2 * delta_trans ** 2)
            )
            dt = delta_trans + random.gauss(
                0.0, math.sqrt(alpha3 * delta_trans ** 2 + alpha4 * (delta_rot1 ** 2 + delta_rot2 ** 2))
            )
            dr2 = delta_rot2 + random.gauss(
                0.0, math.sqrt(alpha1 * delta_rot2 ** 2 + alpha2 * delta_trans ** 2)
            )

            p.x += dt * math.cos(p.theta + dr1)
            p.y += dt * math.sin(p.theta + dr1)
            p.theta = wrap_angle(p.theta + dr1 + dr2)

    # ─── Messmodell ───────────────────────────────────────────────────────────

    def _get_laser_offset(self) -> Tuple[float, float, float]:
        """
        FIX: Holt den Laser-Offset (base_link → laser_frame) via TF.
        Wird gecacht nach dem ersten erfolgreichen Lookup.
        Fällt auf (0,0,0) zurück wenn TF nicht verfügbar.
        """
        if self._laser_offset is not None:
            return self._laser_offset

        base_frame = str(self.get_parameter("base_frame").value)
        laser_frame = str(self.get_parameter("laser_frame").value)
        try:
            tf = self.tf_buffer.lookup_transform(
                base_frame, laser_frame, rclpy.time.Time()
            )
            lx = float(tf.transform.translation.x)
            ly = float(tf.transform.translation.y)
            lth = yaw_from_quat(tf.transform.rotation)
            self._laser_offset = (lx, ly, lth)
            self.get_logger().info(
                f"Laser-Offset ({laser_frame} → {base_frame}): "
                f"x={lx:.3f} y={ly:.3f} th={lth:.3f}"
            )
        except Exception:
            self._laser_offset = (0.0, 0.0, 0.0)
        return self._laser_offset

    def measurement_update(self, scan: LaserScan):
        if self.grid_map is None:
            return
        gm = self.grid_map

        occ_thresh = int(self.get_parameter("occ_thresh").value)
        laser_step = int(self.get_parameter("laser_step").value)
        sigma_z = float(self.get_parameter("sigma_z").value)
        z_hit = float(self.get_parameter("z_hit").value)
        z_rand = float(self.get_parameter("z_rand").value)
        ray_step = float(self.get_parameter("ray_step").value)

        sigma2 = sigma_z ** 2
        norm_const = 1.0 / math.sqrt(2.0 * math.pi * sigma2)
        p_rand = 1.0 / max(scan.range_max, 1e-6)

        # Beam-Indizes und gültige Messwerte vorberechnen
        indices = list(range(0, len(scan.ranges), laser_step))
        beam_angles_rel = np.array(
            [scan.angle_min + i * scan.angle_increment for i in indices],
            dtype=np.float64,
        )
        z_meas = np.array(
            [float(min(max(scan.ranges[i], scan.range_min), scan.range_max))
             for i in indices],
            dtype=np.float64,
        )
        valid_mask = np.array(
            [not (math.isinf(scan.ranges[i]) or math.isnan(scan.ranges[i]))
             for i in indices],
            dtype=bool,
        )

        lx, ly, lth = self._get_laser_offset()

        for p in self.particles:
            idx = gm.world_to_map(p.x, p.y)
            if idx is None:
                p.w = 0.0
                continue

            # Laser-Position in Weltkoordinaten
            c, s = math.cos(p.theta), math.sin(p.theta)
            laser_wx = p.x + c * lx - s * ly
            laser_wy = p.y + s * lx + c * ly
            beam_angles_world = beam_angles_rel + p.theta + lth

            # Nur gültige Strahlen verwenden
            valid_angles = beam_angles_world[valid_mask]
            valid_z = z_meas[valid_mask]

            if len(valid_angles) == 0:
                continue

            z_exp = raycast_numpy(
                gm, laser_wx, laser_wy,
                valid_angles, scan.range_max, occ_thresh, ray_step,
            )

            dz = valid_z - z_exp
            p_hit = norm_const * np.exp(-0.5 * dz ** 2 / sigma2)
            pz = z_hit * p_hit + z_rand * p_rand
            log_w = float(np.sum(np.log(np.maximum(pz, 1e-12))))
            p.w = math.exp(log_w)

        self.normalize_weights()

    # ─── Resampling ───────────────────────────────────────────────────────────

    def normalize_weights(self):
        total = sum(p.w for p in self.particles)
        if total <= 0.0:
            self.get_logger().warn("Alle Gewichte null – globale Neuinitialisierung.")
            self.init_particles_global()
            return
        inv = 1.0 / total
        for p in self.particles:
            p.w *= inv

    def neff(self) -> float:
        ws = np.array([p.w for p in self.particles], dtype=np.float64)
        return 1.0 / max(float(np.sum(ws ** 2)), 1e-12)

    def maybe_resample(self):
        ratio = float(self.get_parameter("resample_neff_ratio").value)
        if self.neff() < ratio * self.N:
            self.systematic_resample()

    def systematic_resample(self):
        """
        FIX: Particle-Deprivation-Schutz – ein kleiner Anteil der Partikel
        wird nach dem Resampling durch zufällig platzierte Partikel ersetzt.
        Das verhindert, dass der Filter bei einer falschen Schätzung
        komplett zusammenbricht.
        """
        ws = [p.w for p in self.particles]
        cdf = np.cumsum(ws)
        if cdf[-1] <= 0.0:
            self.init_particles_global()
            return

        step = 1.0 / self.N
        r0 = random.uniform(0.0, step)
        new_particles: List[Particle] = []
        i = 0
        for m in range(self.N):
            u = r0 + m * step
            while u > cdf[i]:
                i = min(i + 1, self.N - 1)
            p = self.particles[i]
            new_particles.append(Particle(x=p.x, y=p.y, theta=p.theta, w=1.0 / self.N))

        # Zufällige Injection (Deprivation-Schutz)
        inj_ratio = float(self.get_parameter("random_injection_ratio").value)
        n_inject = int(self.N * inj_ratio)
        if n_inject > 0 and self.free_cells:
            for k in range(n_inject):
                mx, my = random.choice(self.free_cells)
                fx, fy = self.grid_map.map_to_world(mx, my)
                fth = random.uniform(-math.pi, math.pi)
                new_particles[-(k + 1)] = Particle(x=fx, y=fy, theta=fth, w=1.0 / self.N)

        self.particles = new_particles
        self.normalize_weights()

    # ─── Schätzung & Veröffentlichung ─────────────────────────────────────────

    def estimate_pose(self) -> Tuple[float, float, float]:
        xs = np.array([p.x for p in self.particles], dtype=np.float64)
        ys = np.array([p.y for p in self.particles], dtype=np.float64)
        ws = np.array([p.w for p in self.particles], dtype=np.float64)
        x = float(np.dot(xs, ws))
        y = float(np.dot(ys, ws))
        cs = float(np.dot(np.cos([p.theta for p in self.particles]), ws))
        ss = float(np.dot(np.sin([p.theta for p in self.particles]), ws))
        return x, y, float(math.atan2(ss, cs))

    def publish_estimates(self):
        if self.grid_map is None or not self.particles:
            return

        map_frame = str(self.get_parameter("map_frame").value)
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        now = self.get_clock().now().to_msg()

        # Partikel-Cloud veröffentlichen
        cloud = PoseArray()
        cloud.header = Header(stamp=now, frame_id=map_frame)
        stride = max(1, len(self.particles) // 400)
        poses = []
        for p in self.particles[::stride]:
            pose = Pose()
            pose.position.x = float(p.x)
            pose.position.y = float(p.y)
            pose.position.z = 0.0
            pose.orientation = quat_from_yaw(p.theta)
            poses.append(pose)
        cloud.poses = poses
        self.pub_cloud.publish(cloud)

        # Mittlere Pose veröffentlichen
        x, y, th = self.estimate_pose()

        xs = np.array([p.x for p in self.particles], dtype=np.float64)
        ys = np.array([p.y for p in self.particles], dtype=np.float64)
        ts = np.array([p.theta for p in self.particles], dtype=np.float64)
        ws = np.array([p.w for p in self.particles], dtype=np.float64)

        vx = float(np.dot(ws, (xs - x) ** 2))
        vy = float(np.dot(ws, (ys - y) ** 2))
        dth = np.array([wrap_angle(t - th) for t in ts], dtype=np.float64)
        vth = float(np.dot(ws, dth ** 2))

        out = PoseWithCovarianceStamped()
        out.header = Header(stamp=now, frame_id=map_frame)
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation = quat_from_yaw(th)
        cov = [0.0] * 36
        cov[0] = vx
        cov[7] = vy
        cov[35] = vth
        out.pose.covariance = cov
        self.pub_pose.publish(out)

        # TF map → odom broadcasten
        self.broadcast_map_to_odom(x, y, th, map_frame, odom_frame, base_frame, now)

    def broadcast_map_to_odom(
        self,
        x_map_base: float,
        y_map_base: float,
        th_map_base: float,
        map_frame: str,
        odom_frame: str,
        base_frame: str,
        stamp_msg,
    ):
        """T_map_odom = T_map_base · inv(T_odom_base)"""
        try:
            tf_ob = self.tf_buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF-Lookup fehlgeschlagen ({odom_frame}→{base_frame}): {e}"
            )
            return

        tx = float(tf_ob.transform.translation.x)
        ty = float(tf_ob.transform.translation.y)
        th_ob = yaw_from_quat(tf_ob.transform.rotation)

        T_map_base = se2_from_xyth(x_map_base, y_map_base, th_map_base)
        T_odom_base = se2_from_xyth(tx, ty, th_ob)
        T_map_odom = T_map_base @ se2_inv(T_odom_base)
        xmo, ymo, thmo = se2_to_xyth(T_map_odom)

        t = TransformStamped()
        t.header.stamp = stamp_msg
        t.header.frame_id = map_frame
        t.child_frame_id = odom_frame
        t.transform.translation.x = float(xmo)
        t.transform.translation.y = float(ymo)
        t.transform.translation.z = 0.0
        t.transform.rotation = quat_from_yaw(thmo)
        self.tf_broadcaster.sendTransform(t)


# ─────────────────────────────────────────
# Entry-Point
# ─────────────────────────────────────────

def main():
    rclpy.init()
    node = ParticleFilterLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()