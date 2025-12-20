"""Gazebo-style demo application with local simulation, GUI, and web UI."""

from __future__ import annotations

import argparse
import json
import math
import random
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ai_robotics_core import (
    Action,
    BasicSafetyGate,
    Observation,
    RunLogger,
    RuntimeConfig,
    RuntimeLoop,
    SpeedLimits,
    State,
)
from ai_robotics_core.adapters import PolicyAdapter, RobotAdapter, SensorAdapter
from ai_robotics_sim.base import SimulationBridge


@dataclass
class DemoConfig:
    backend: str = "mock"
    duration_s: float = 30.0
    loop_hz: float = 10.0
    output_dir: str = "data/runs"
    run_id: str = "gazebo-demo"
    enable_gui: bool = True
    enable_web: bool = True
    web_port: int = 8080
    world: Dict[str, Any] = field(default_factory=dict)
    ros: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    radius: float


@dataclass
class WorldConfig:
    width_m: float = 10.0
    height_m: float = 10.0
    lidar_samples: int = 181
    lidar_max_range_m: float = 6.0
    lidar_fov_rad: float = math.pi
    camera_width: int = 24
    camera_height: int = 18
    obstacles: List[Obstacle] = field(
        default_factory=lambda: [
            Obstacle(2.5, 7.0, 0.6),
            Obstacle(7.2, 2.6, 0.8),
            Obstacle(6.4, 6.8, 0.5),
        ]
    )


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class MockGazeboBridge(SimulationBridge):
    def __init__(self, config: WorldConfig) -> None:
        self._config = config
        self._connected = False

    def connect(self) -> None:
        self._connected = True

    def step(self) -> None:
        if not self._connected:
            raise RuntimeError("Simulation bridge not connected.")

    def disconnect(self) -> None:
        self._connected = False


class MockGazeboWorld:
    def __init__(self, config: WorldConfig) -> None:
        self._config = config
        self._pose = Pose2D(x=config.width_m * 0.5, y=config.height_m * 0.5, yaw=0.0)
        self._linear = 0.0
        self._angular = 0.0
        self._time_s = 0.0

    @property
    def pose(self) -> Pose2D:
        return self._pose

    @property
    def velocity(self) -> Tuple[float, float]:
        return self._linear, self._angular

    @property
    def time_s(self) -> float:
        return self._time_s

    @property
    def config(self) -> WorldConfig:
        return self._config

    def step(self, linear: float, angular: float, dt: float) -> None:
        self._linear = linear
        self._angular = angular
        self._pose.yaw = _wrap_angle(self._pose.yaw + angular * dt)
        self._pose.x += linear * math.cos(self._pose.yaw) * dt
        self._pose.y += linear * math.sin(self._pose.yaw) * dt
        self._pose.x = max(0.2, min(self._config.width_m - 0.2, self._pose.x))
        self._pose.y = max(0.2, min(self._config.height_m - 0.2, self._pose.y))
        self._time_s += dt

    def lidar_scan(self) -> Tuple[List[float], List[float]]:
        angles: List[float] = []
        ranges: List[float] = []
        half = self._config.lidar_fov_rad * 0.5
        start = -half
        step = self._config.lidar_fov_rad / (self._config.lidar_samples - 1)
        for i in range(self._config.lidar_samples):
            angle = start + i * step
            world_angle = self._pose.yaw + angle
            distance = _ray_cast(
                origin=(self._pose.x, self._pose.y),
                angle=world_angle,
                config=self._config,
            )
            angles.append(angle)
            ranges.append(distance)
        return angles, ranges

    def camera_frame(self) -> Dict[str, Any]:
        width = self._config.camera_width
        height = self._config.camera_height
        base = _front_obstacle_intensity(self._pose, self._config)
        pixels: List[int] = []
        for y in range(height):
            for x in range(width):
                ripple = (x * 11 + y * 17) % 23
                value = max(0, min(255, base + ripple - 10))
                pixels.append(value)
        return {"width": width, "height": height, "mode": "L", "pixels": pixels}


class GazeboSensorAdapter(SensorAdapter):
    def __init__(self, world: MockGazeboWorld) -> None:
        self._world = world

    def start(self) -> None:
        return None

    def read(self) -> Observation:
        angles, ranges = self._world.lidar_scan()
        sensors = {
            "lidar": {
                "angles_rad": angles,
                "ranges_m": ranges,
                "max_range_m": self._world.config.lidar_max_range_m,
            },
            "camera": self._world.camera_frame(),
        }
        return Observation(timestamp_s=self._world.time_s, sensors=sensors, frame_id="base_link")

    def stop(self) -> None:
        return None


class GazeboRobotAdapter(RobotAdapter):
    def __init__(self, world: MockGazeboWorld, dt: float) -> None:
        self._world = world
        self._dt = dt

    def start(self) -> None:
        return None

    def read_state(self) -> State:
        linear, angular = self._world.velocity
        pose = {
            "x": self._world.pose.x,
            "y": self._world.pose.y,
            "yaw": self._world.pose.yaw,
        }
        velocity = {"linear": linear, "angular": angular}
        return State(timestamp_s=self._world.time_s, pose=pose, velocity=velocity)

    def apply(self, action: Action) -> None:
        linear = float(action.command.get("linear", 0.0))
        angular = float(action.command.get("angular", 0.0))
        self._world.step(linear, angular, self._dt)

    def stop(self) -> None:
        return None


class LoopingPolicy(PolicyAdapter):
    def __init__(self, linear: float = 0.2, angular: float = 0.3) -> None:
        self._linear = linear
        self._angular = angular

    def start(self) -> None:
        return None

    def act(self, observation: Observation, state: State) -> Action:
        _ = observation
        _ = state
        return Action(
            timestamp_s=observation.timestamp_s,
            command={"linear": self._linear, "angular": self._angular},
            frame_id="base_link",
        )

    def stop(self) -> None:
        return None


class DemoStateStore:
    def __init__(self, config: WorldConfig) -> None:
        self._lock = threading.Lock()
        self._config = config
        self._last: Dict[str, Any] = {}
        self._path: List[Dict[str, float]] = []

    def update(self, observation: Observation, state: State, action: Action) -> None:
        with self._lock:
            self._last = {
                "observation": _asdict(observation),
                "state": _asdict(state),
                "action": _asdict(action),
            }
            self._path.append({"x": state.pose["x"], "y": state.pose["y"]})
            if len(self._path) > 240:
                self._path.pop(0)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            payload = dict(self._last)
            payload["world"] = {
                "width_m": self._config.width_m,
                "height_m": self._config.height_m,
                "obstacles": [
                    {"x": obs.x, "y": obs.y, "radius": obs.radius}
                    for obs in self._config.obstacles
                ],
                "path": list(self._path),
            }
        return payload


class DemoLogger:
    def __init__(self, logger: RunLogger, store: DemoStateStore) -> None:
        self._logger = logger
        self._store = store

    def start(self) -> None:
        self._logger.start()

    def log_step(self, observation: Observation, state: State, action: Action) -> None:
        self._store.update(observation, state, action)
        self._logger.log_step(observation, state, action)

    def stop(self) -> None:
        self._logger.stop()


class DemoWebServer:
    def __init__(self, store: DemoStateStore, host: str, port: int) -> None:
        self._store = store
        self._server = ThreadingHTTPServer((host, port), self._handler_factory())
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def _handler_factory(self):
        store = self._store

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path == "/api/state":
                    payload = store.snapshot()
                    body = json.dumps(payload).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path == "/":
                    body = _web_index().encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_response(404)
                self.end_headers()

            def log_message(self, format: str, *args: Any) -> None:
                _ = format
                _ = args

        return Handler

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)


class DemoGUI:
    def __init__(self, store: DemoStateStore, config: WorldConfig) -> None:
        import tkinter as tk

        self._store = store
        self._config = config
        self._root = tk.Tk()
        self._root.title("Gazebo Turtlebot3 Demo")
        self._root.geometry("860x520")
        self._root.configure(bg="#151515")

        self._world_canvas = tk.Canvas(self._root, width=520, height=520, bg="#101820")
        self._world_canvas.pack(side=tk.LEFT)
        self._camera_canvas = tk.Canvas(self._root, width=320, height=240, bg="#1f2b38")
        self._camera_canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=False)

        self._status = tk.Label(
            self._root,
            text="Starting...",
            bg="#151515",
            fg="#e0e4dd",
            font=("Helvetica", 12, "bold"),
        )
        self._status.pack(side=tk.TOP, fill=tk.X)

        self._camera_cells: List[int] = []
        self._init_camera_grid()

        self._root.after(100, self._update)

    def _init_camera_grid(self) -> None:
        width = self._config.camera_width
        height = self._config.camera_height
        cell_w = 320 / width
        cell_h = 240 / height
        for y in range(height):
            for x in range(width):
                x0 = x * cell_w
                y0 = y * cell_h
                rect = self._camera_canvas.create_rectangle(
                    x0,
                    y0,
                    x0 + cell_w,
                    y0 + cell_h,
                    fill="#202b38",
                    outline="",
                )
                self._camera_cells.append(rect)

    def _update(self) -> None:
        snapshot = self._store.snapshot()
        self._draw_world(snapshot)
        self._draw_camera(snapshot)
        self._root.after(120, self._update)

    def _draw_world(self, snapshot: Dict[str, Any]) -> None:
        self._world_canvas.delete("all")
        world = snapshot.get("world", {})
        width = world.get("width_m", self._config.width_m)
        height = world.get("height_m", self._config.height_m)
        scale_x = 520 / width
        scale_y = 520 / height

        for obs in world.get("obstacles", []):
            x = obs["x"] * scale_x
            y = (height - obs["y"]) * scale_y
            r = obs["radius"] * scale_x
            self._world_canvas.create_oval(x - r, y - r, x + r, y + r, fill="#354f52")

        path = world.get("path", [])
        for idx in range(1, len(path)):
            p0 = path[idx - 1]
            p1 = path[idx]
            x0 = p0["x"] * scale_x
            y0 = (height - p0["y"]) * scale_y
            x1 = p1["x"] * scale_x
            y1 = (height - p1["y"]) * scale_y
            self._world_canvas.create_line(x0, y0, x1, y1, fill="#4e8098")

        state = snapshot.get("state", {})
        pose = state.get("pose", {})
        x = pose.get("x", 0.0) * scale_x
        y = (height - pose.get("y", 0.0)) * scale_y
        yaw = pose.get("yaw", 0.0)
        self._world_canvas.create_oval(x - 10, y - 10, x + 10, y + 10, fill="#ffd166")
        self._world_canvas.create_line(
            x,
            y,
            x + 20 * math.cos(yaw),
            y - 20 * math.sin(yaw),
            fill="#ff6b35",
            width=2,
        )

        lidar = snapshot.get("observation", {}).get("sensors", {}).get("lidar", {})
        angles = lidar.get("angles_rad", [])
        ranges = lidar.get("ranges_m", [])
        for angle, distance in zip(angles[::6], ranges[::6], strict=False):
            world_angle = yaw + angle
            lx = pose.get("x", 0.0) + distance * math.cos(world_angle)
            ly = pose.get("y", 0.0) + distance * math.sin(world_angle)
            self._world_canvas.create_line(
                x,
                y,
                lx * scale_x,
                (height - ly) * scale_y,
                fill="#8ecae6",
            )

        self._status.config(text=f"t={state.get('timestamp_s', 0.0):.2f}s")

    def _draw_camera(self, snapshot: Dict[str, Any]) -> None:
        camera = snapshot.get("observation", {}).get("sensors", {}).get("camera", {})
        pixels = camera.get("pixels", [])
        if not pixels:
            return
        for idx, value in enumerate(pixels):
            shade = max(0, min(255, int(value)))
            color = f"#{shade:02x}{shade:02x}{shade:02x}"
            self._camera_canvas.itemconfig(self._camera_cells[idx], fill=color)

    def run(self) -> None:
        self._root.mainloop()

    def close(self) -> None:
        self._root.destroy()


class RosContext:
    def __init__(self, namespace: str) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
        except ImportError as exc:
            raise RuntimeError(
                "ROS backend requires rclpy (ROS 2 Humble or newer). "
                "Install ROS 2 and source the environment before running."
            ) from exc

        self._rclpy = rclpy
        self._executor_cls = SingleThreadedExecutor
        self._namespace = namespace
        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()

    @property
    def node(self):
        if self._node is None:
            raise RuntimeError("ROS context not started.")
        return self._node

    def start(self) -> None:
        self._rclpy.init()
        self._executor = self._executor_cls()
        self._node = self._rclpy.create_node("atlasforge_demo", namespace=self._namespace)
        self._executor.add_node(self._node)
        self._running.set()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while self._running.is_set():
            self._executor.spin_once(timeout_sec=0.1)

    def shutdown(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._executor:
            self._executor.shutdown()
        if self._node:
            self._node.destroy_node()
        self._rclpy.shutdown()


class RosGazeboSensors(SensorAdapter):
    def __init__(self, node, topics: Dict[str, str], world_config: WorldConfig) -> None:
        from sensor_msgs.msg import Image, LaserScan

        self._node = node
        self._world_config = world_config
        self._lock = threading.Lock()
        self._last_scan: Optional[Dict[str, Any]] = None
        self._last_camera: Optional[Dict[str, Any]] = None
        self._frame_id = topics.get("frame_id", "base_link")

        self._node.create_subscription(LaserScan, topics["scan"], self._on_scan, 10)
        self._node.create_subscription(Image, topics["camera"], self._on_camera, 10)

    def start(self) -> None:
        return None

    def read(self) -> Observation:
        with self._lock:
            scan = dict(self._last_scan) if self._last_scan else None
            camera = dict(self._last_camera) if self._last_camera else None

        timestamp_s = 0.0
        if scan:
            timestamp_s = scan["timestamp_s"]
        elif camera:
            timestamp_s = camera["timestamp_s"]

        sensors = {
            "lidar": scan or {"angles_rad": [], "ranges_m": [], "max_range_m": 0.0},
            "camera": camera
            or {
                "width": self._world_config.camera_width,
                "height": self._world_config.camera_height,
                "mode": "L",
                "pixels": [],
            },
        }
        return Observation(timestamp_s=timestamp_s, sensors=sensors, frame_id=self._frame_id)

    def stop(self) -> None:
        return None

    def _on_scan(self, msg) -> None:
        angles = [
            msg.angle_min + idx * msg.angle_increment for idx in range(len(msg.ranges))
        ]
        with self._lock:
            self._last_scan = {
                "timestamp_s": _ros_stamp_to_float(msg.header.stamp),
                "angles_rad": angles,
                "ranges_m": list(msg.ranges),
                "max_range_m": float(msg.range_max),
            }

    def _on_camera(self, msg) -> None:
        frame = _ros_image_to_frame(
            msg,
            target_width=self._world_config.camera_width,
            target_height=self._world_config.camera_height,
        )
        with self._lock:
            self._last_camera = {
                "timestamp_s": _ros_stamp_to_float(msg.header.stamp),
                "width": frame["width"],
                "height": frame["height"],
                "mode": "L",
                "pixels": frame["pixels"],
            }


class RosGazeboRobot(RobotAdapter):
    def __init__(self, node, topics: Dict[str, str]) -> None:
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        self._node = node
        self._lock = threading.Lock()
        self._last_odom: Optional[Dict[str, Any]] = None
        self._frame_id = topics.get("frame_id", "base_link")
        self._publisher = self._node.create_publisher(Twist, topics["cmd_vel"], 10)
        self._node.create_subscription(Odometry, topics["odom"], self._on_odom, 10)

    def start(self) -> None:
        return None

    def read_state(self) -> State:
        with self._lock:
            odom = dict(self._last_odom) if self._last_odom else None

        if not odom:
            return State(
                timestamp_s=0.0,
                pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
                velocity={"linear": 0.0, "angular": 0.0},
            )

        return State(
            timestamp_s=odom["timestamp_s"],
            pose=odom["pose"],
            velocity=odom["velocity"],
        )

    def apply(self, action: Action) -> None:
        from geometry_msgs.msg import Twist

        msg = Twist()
        msg.linear.x = float(action.command.get("linear", 0.0))
        msg.angular.z = float(action.command.get("angular", 0.0))
        self._publisher.publish(msg)

    def stop(self) -> None:
        return None

    def _on_odom(self, msg) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = _yaw_from_quaternion(pose.orientation)
        with self._lock:
            self._last_odom = {
                "timestamp_s": _ros_stamp_to_float(msg.header.stamp),
                "pose": {"x": float(pose.position.x), "y": float(pose.position.y), "yaw": yaw},
                "velocity": {
                    "linear": float(twist.linear.x),
                    "angular": float(twist.angular.z),
                },
            }


def _build_ros_adapters(
    ros_config: Dict[str, Any],
    world_config: WorldConfig,
) -> Tuple[SensorAdapter, RobotAdapter, RosContext]:
    namespace = ros_config.get("namespace", "")
    topics = ros_config.get(
        "topics",
        {
            "cmd_vel": "/cmd_vel",
            "odom": "/odom",
            "scan": "/scan",
            "camera": "/camera/color/image_raw",
        },
    )
    topics = dict(topics)
    topics.setdefault("frame_id", ros_config.get("frame_id", "base_link"))

    context = RosContext(namespace=namespace)
    context.start()
    sensors = RosGazeboSensors(context.node, topics, world_config)
    robot = RosGazeboRobot(context.node, topics)
    return sensors, robot, context


def run_demo(
    duration_s: float,
    loop_hz: float,
    output_dir: Path,
    run_id: str,
    enable_gui: bool,
    enable_web: bool,
    web_port: int,
    backend: str,
    ros_config: Dict[str, Any],
    world_config: WorldConfig,
) -> None:
    if backend == "ros":
        sensors, robot, ros_context = _build_ros_adapters(ros_config, world_config)
    else:
        world = MockGazeboWorld(world_config)
        sensors = GazeboSensorAdapter(world)
        robot = GazeboRobotAdapter(world, dt=1.0 / loop_hz)
        ros_context = None

    store = DemoStateStore(world_config)
    policy = LoopingPolicy()
    safety = BasicSafetyGate(limits=SpeedLimits(linear_mps=0.22, angular_rps=2.84))
    logger = DemoLogger(RunLogger(output_dir, run_id), store)
    loop = RuntimeLoop(
        sensors=sensors,
        robot=robot,
        policy=policy,
        safety=safety,
        logger=logger,
        config=RuntimeConfig(loop_hz=loop_hz, max_cycle_time_s=1.0 / loop_hz),
    )

    stop_event = threading.Event()
    web_server = None
    gui = None

    if enable_web:
        web_server = DemoWebServer(store, host="127.0.0.1", port=web_port)
        web_server.start()

    if enable_gui:
        gui = DemoGUI(store, world_config)

    def runner() -> None:
        loop.start()
        start = time.perf_counter()
        period = 1.0 / loop_hz
        try:
            while not stop_event.is_set():
                loop_start = time.perf_counter()
                loop.run_once()
                if duration_s > 0 and time.perf_counter() - start >= duration_s:
                    stop_event.set()
                sleep_for = period - (time.perf_counter() - loop_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            loop.stop()

    runner_thread = threading.Thread(target=runner, daemon=True)
    runner_thread.start()

    if gui:
        gui.run()
        stop_event.set()

    runner_thread.join()
    if web_server:
        web_server.stop()
    if ros_context:
        ros_context.shutdown()


def _asdict(data: Any) -> Dict[str, Any]:
    if hasattr(data, "__dict__"):
        return dict(data.__dict__)
    return dict(data)


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _ros_stamp_to_float(stamp) -> float:
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _ros_image_to_frame(msg, target_width: int, target_height: int) -> Dict[str, Any]:
    width = int(msg.width)
    height = int(msg.height)
    if width == 0 or height == 0:
        return {"width": target_width, "height": target_height, "pixels": []}

    encoding = (msg.encoding or "").lower()
    channels = 1 if encoding == "mono8" else 3
    if encoding not in {"mono8", "rgb8", "bgr8"}:
        return {"width": target_width, "height": target_height, "pixels": []}

    data = memoryview(msg.data)
    pixels: List[int] = []
    for y in range(target_height):
        src_y = int(y * height / target_height)
        row_base = src_y * msg.step
        for x in range(target_width):
            src_x = int(x * width / target_width)
            idx = row_base + src_x * channels
            if idx + channels > len(data):
                pixels.append(0)
                continue
            if channels == 1:
                pixels.append(int(data[idx]))
            else:
                if encoding == "bgr8":
                    b, g, r = data[idx : idx + 3]
                else:
                    r, g, b = data[idx : idx + 3]
                pixels.append(int((r + g + b) / 3))
    return {"width": target_width, "height": target_height, "pixels": pixels}


def _coalesce(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _load_config(path: Path) -> DemoConfig:
    if not path.exists():
        return DemoConfig()

    if path.suffix.lower() == ".json":
        data = json.loads(path.read_text(encoding="utf-8"))
    elif path.suffix.lower() in {".yaml", ".yml"}:
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError(
                "YAML config requires PyYAML. Install it or use a JSON config instead."
            ) from exc
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    else:
        raise ValueError("Unsupported config format. Use JSON or YAML.")

    return _config_from_dict(data or {})


def _config_from_dict(data: Dict[str, Any]) -> DemoConfig:
    config = DemoConfig()
    for key in (
        "backend",
        "duration_s",
        "loop_hz",
        "output_dir",
        "run_id",
        "enable_gui",
        "enable_web",
        "web_port",
    ):
        if key in data:
            setattr(config, key, data[key])
    config.world = dict(data.get("world", {}))
    config.ros = dict(data.get("ros", {}))
    return config


def _world_config_from_dict(data: Dict[str, Any]) -> WorldConfig:
    base = WorldConfig()
    obstacles = data.get("obstacles")
    if obstacles is None:
        obstacle_list = [Obstacle(obs.x, obs.y, obs.radius) for obs in base.obstacles]
    else:
        obstacle_list = [Obstacle(**obs) for obs in obstacles]
    return WorldConfig(
        width_m=float(data.get("width_m", base.width_m)),
        height_m=float(data.get("height_m", base.height_m)),
        lidar_samples=int(data.get("lidar_samples", base.lidar_samples)),
        lidar_max_range_m=float(data.get("lidar_max_range_m", base.lidar_max_range_m)),
        lidar_fov_rad=float(data.get("lidar_fov_rad", base.lidar_fov_rad)),
        camera_width=int(data.get("camera_width", base.camera_width)),
        camera_height=int(data.get("camera_height", base.camera_height)),
        obstacles=obstacle_list,
    )


def _ray_cast(origin: Tuple[float, float], angle: float, config: WorldConfig) -> float:
    ox, oy = origin
    dx = math.cos(angle)
    dy = math.sin(angle)
    max_range = config.lidar_max_range_m

    boundary = _ray_intersect_bounds(ox, oy, dx, dy, config.width_m, config.height_m)
    obstacle = max_range
    for obs in config.obstacles:
        hit = _ray_intersect_circle(ox, oy, dx, dy, obs)
        if hit is not None:
            obstacle = min(obstacle, hit)
    return min(max_range, boundary, obstacle)


def _ray_intersect_bounds(
    ox: float, oy: float, dx: float, dy: float, width: float, height: float
) -> float:
    candidates: List[float] = []
    if abs(dx) > 1e-6:
        for x_bound in (0.0, width):
            t = (x_bound - ox) / dx
            if t > 0:
                y = oy + t * dy
                if 0.0 <= y <= height:
                    candidates.append(t)
    if abs(dy) > 1e-6:
        for y_bound in (0.0, height):
            t = (y_bound - oy) / dy
            if t > 0:
                x = ox + t * dx
                if 0.0 <= x <= width:
                    candidates.append(t)
    if not candidates:
        return float("inf")
    return min(candidates)


def _ray_intersect_circle(
    ox: float, oy: float, dx: float, dy: float, obs: Obstacle
) -> Optional[float]:
    cx = obs.x
    cy = obs.y
    r = obs.radius
    ox_c = ox - cx
    oy_c = oy - cy
    b = 2 * (ox_c * dx + oy_c * dy)
    c = ox_c * ox_c + oy_c * oy_c - r * r
    disc = b * b - 4 * c
    if disc < 0:
        return None
    sqrt_disc = math.sqrt(disc)
    t0 = (-b - sqrt_disc) / 2
    t1 = (-b + sqrt_disc) / 2
    hits = [t for t in (t0, t1) if t > 0]
    if not hits:
        return None
    return min(hits)


def _front_obstacle_intensity(pose: Pose2D, config: WorldConfig) -> int:
    intensity = 30
    rng = random.Random(13)
    for obs in config.obstacles:
        dx = obs.x - pose.x
        dy = obs.y - pose.y
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        if abs(_wrap_angle(angle - pose.yaw)) < math.radians(30):
            score = max(0.0, config.lidar_max_range_m - distance)
            intensity = max(intensity, int(40 + (score / config.lidar_max_range_m) * 160))
    return min(255, intensity + rng.randint(-5, 5))


def _web_index() -> str:
    return """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1"/>
    <title>Gazebo Turtlebot3 Demo</title>
    <style>
      :root {
        color-scheme: light;
        --bg: #f6f4ef;
        --panel: #ffffff;
        --ink: #14213d;
        --accent: #fca311;
        --muted: #8d99ae;
      }
      body {
        margin: 0;
        font-family: "Fira Sans", "IBM Plex Sans", "Trebuchet MS", sans-serif;
        background: radial-gradient(circle at 10% 20%, #f7ede2, #e5e0da 55%, #d0c7bb);
        color: var(--ink);
      }
      header {
        padding: 18px 24px;
        display: flex;
        justify-content: space-between;
        align-items: center;
      }
      .badge {
        background: var(--accent);
        color: #111;
        padding: 6px 12px;
        border-radius: 999px;
        font-weight: 700;
        font-size: 12px;
        letter-spacing: 0.08em;
      }
      .layout {
        display: grid;
        grid-template-columns: 2fr 1fr;
        gap: 18px;
        padding: 0 24px 24px;
      }
      .panel {
        background: var(--panel);
        border-radius: 16px;
        padding: 16px;
        box-shadow: 0 16px 32px rgba(20, 33, 61, 0.08);
      }
      canvas {
        width: 100%;
        border-radius: 12px;
        background: #101820;
      }
      .status {
        margin-top: 12px;
        color: var(--muted);
        font-size: 14px;
      }
    </style>
  </head>
  <body>
    <header>
      <div>
        <h1 style="margin: 0;">Gazebo Turtlebot3 Demo</h1>
        <div class="status" id="status">Waiting for data...</div>
      </div>
      <div class="badge">LIVE</div>
    </header>
    <div class="layout">
      <div class="panel">
        <h2 style="margin-top: 0;">World + Lidar</h2>
        <canvas id="world" width="720" height="520"></canvas>
      </div>
      <div class="panel">
        <h2 style="margin-top: 0;">Camera</h2>
        <canvas id="camera" width="240" height="180"></canvas>
      </div>
    </div>
    <script>
      const worldCanvas = document.getElementById("world");
      const worldCtx = worldCanvas.getContext("2d");
      const cameraCanvas = document.getElementById("camera");
      const cameraCtx = cameraCanvas.getContext("2d");
      const statusEl = document.getElementById("status");

      function drawWorld(payload) {
        const world = payload.world || {};
        const pose = (payload.state || {}).pose || {};
        const sensors = (payload.observation || {}).sensors || {};
        const lidar = sensors.lidar || {};
        const width = world.width_m || 10;
        const height = world.height_m || 10;
        const scaleX = worldCanvas.width / width;
        const scaleY = worldCanvas.height / height;

        worldCtx.clearRect(0, 0, worldCanvas.width, worldCanvas.height);
        worldCtx.fillStyle = "#101820";
        worldCtx.fillRect(0, 0, worldCanvas.width, worldCanvas.height);

        (world.obstacles || []).forEach((obs) => {
          worldCtx.fillStyle = "#354f52";
          worldCtx.beginPath();
          worldCtx.arc(
            obs.x * scaleX,
            (height - obs.y) * scaleY,
            obs.radius * scaleX,
            0,
            Math.PI * 2
          );
          worldCtx.fill();
        });

        const path = world.path || [];
        worldCtx.strokeStyle = "#4e8098";
        worldCtx.lineWidth = 1;
        worldCtx.beginPath();
        path.forEach((p, idx) => {
          const x = p.x * scaleX;
          const y = (height - p.y) * scaleY;
          if (idx === 0) {
            worldCtx.moveTo(x, y);
          } else {
            worldCtx.lineTo(x, y);
          }
        });
        worldCtx.stroke();

        const robotX = (pose.x || 0) * scaleX;
        const robotY = (height - (pose.y || 0)) * scaleY;
        const yaw = pose.yaw || 0;
        worldCtx.fillStyle = "#ffd166";
        worldCtx.beginPath();
        worldCtx.arc(robotX, robotY, 10, 0, Math.PI * 2);
        worldCtx.fill();

        worldCtx.strokeStyle = "#ff6b35";
        worldCtx.lineWidth = 2;
        worldCtx.beginPath();
        worldCtx.moveTo(robotX, robotY);
        worldCtx.lineTo(robotX + 22 * Math.cos(yaw), robotY - 22 * Math.sin(yaw));
        worldCtx.stroke();

        const angles = lidar.angles_rad || [];
        const ranges = lidar.ranges_m || [];
        worldCtx.strokeStyle = "rgba(142, 202, 230, 0.8)";
        worldCtx.lineWidth = 1;
        for (let i = 0; i < angles.length; i += 6) {
          const angle = yaw + angles[i];
          const distance = ranges[i] || 0;
          const lx = (pose.x || 0) + distance * Math.cos(angle);
          const ly = (pose.y || 0) + distance * Math.sin(angle);
          worldCtx.beginPath();
          worldCtx.moveTo(robotX, robotY);
          worldCtx.lineTo(lx * scaleX, (height - ly) * scaleY);
          worldCtx.stroke();
        }
      }

      function drawCamera(payload) {
        const camera = ((payload.observation || {}).sensors || {}).camera || {};
        const width = camera.width || 0;
        const height = camera.height || 0;
        const pixels = camera.pixels || [];
        if (!width || !height || pixels.length === 0) {
          return;
        }
        const image = cameraCtx.createImageData(width, height);
        for (let i = 0; i < pixels.length; i++) {
          const v = pixels[i];
          const idx = i * 4;
          image.data[idx] = v;
          image.data[idx + 1] = v;
          image.data[idx + 2] = v;
          image.data[idx + 3] = 255;
        }
        cameraCtx.putImageData(image, 0, 0);
      }

      async function tick() {
        try {
          const response = await fetch("/api/state", { cache: "no-store" });
          if (!response.ok) {
            throw new Error("no data");
          }
          const payload = await response.json();
          drawWorld(payload);
          drawCamera(payload);
          const time = ((payload.state || {}).timestamp_s || 0).toFixed(2);
          statusEl.textContent = "t=" + time + "s, logging to data/runs";
        } catch (err) {
          statusEl.textContent = "Waiting for data...";
        }
      }

      setInterval(tick, 200);
      tick();
    </script>
  </body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a Gazebo-style Turtlebot3 demo.")
    parser.add_argument(
        "--config",
        default="config/sim/gazebo_demo.json",
        help="Path to demo config (JSON or YAML).",
    )
    parser.add_argument("--backend", choices=("mock", "ros"), default=None)
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Duration in seconds (0 = forever).",
    )
    parser.add_argument("--loop-hz", type=float, default=None, help="Control loop frequency.")
    parser.add_argument("--output-dir", default=None, help="Directory for JSONL logs.")
    parser.add_argument("--run-id", default=None, help="Run identifier for the log file.")
    parser.add_argument(
        "--gui",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Enable/disable the local GUI.",
    )
    parser.add_argument(
        "--web",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Enable/disable the web UI.",
    )
    parser.add_argument("--web-port", type=int, default=None, help="Port for the web UI.")
    args = parser.parse_args()

    config = _load_config(Path(args.config))
    world_config = _world_config_from_dict(config.world)

    run_demo(
        duration_s=float(_coalesce(args.duration, config.duration_s)),
        loop_hz=float(_coalesce(args.loop_hz, config.loop_hz)),
        output_dir=Path(_coalesce(args.output_dir, config.output_dir)),
        run_id=str(_coalesce(args.run_id, config.run_id)),
        enable_gui=bool(_coalesce(args.gui, config.enable_gui)),
        enable_web=bool(_coalesce(args.web, config.enable_web)),
        web_port=int(_coalesce(args.web_port, config.web_port)),
        backend=str(_coalesce(args.backend, config.backend)),
        ros_config=config.ros,
        world_config=world_config,
    )


if __name__ == "__main__":
    main()
