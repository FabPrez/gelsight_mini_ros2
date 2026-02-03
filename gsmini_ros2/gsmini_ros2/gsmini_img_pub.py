#!/usr/bin/env python3
# gsmini_image_pub.py

import os
import sys
import importlib
from types import ModuleType


# ---------------- minimal manual path shim ----------------

# --- set your absolute path here (point to the folder that contains "gsrobotics" or to "gsrobotics" itself) ---
GSROBOTICS_PATH = "/home/fabioprez/projects/fruit_ws/src/gelsight_mini_ros2"  # <<-- replace with your absolute path

def _add_root(root_path: str):
    root_path = os.path.abspath(os.path.expanduser(root_path))
    # if user pointed directly to the gsrobotics folder, use its parent
    if os.path.basename(root_path) == "gsrobotics":
        root = os.path.dirname(root_path)
    # if user pointed to a folder that contains gsrobotics, use it directly
    elif os.path.isdir(os.path.join(root_path, "gsrobotics")):
        root = root_path
    else:
        raise RuntimeError(f"Il percorso fornito non contiene 'gsrobotics': {root_path}")
    if root not in sys.path:
        sys.path.insert(0, root)
    return root

# Use the manually set path; do NOT read environment variables
chosen_root = _add_root(GSROBOTICS_PATH)

# optional: register compatibility aliases so imports like `from utilities.logger import ...`
# inside the external repo still work (keeps you from changing upstream code)
try:
    gs = importlib.import_module("gsrobotics")
    try:
        util_pkg = importlib.import_module("gsrobotics.utilities")
    except Exception:
        util_pkg = None
    try:
        config_mod = importlib.import_module("gsrobotics.config")
    except Exception:
        config_mod = None

    if util_pkg is not None and "utilities" not in sys.modules:
        sys.modules["utilities"] = util_pkg
    if config_mod is not None and "config" not in sys.modules:
        sys.modules["config"] = config_mod
except Exception:
    # non fallire qui: gli import reali produrranno errori se qualcosa non va
    pass
# ---------------- end shim ----------------

import argparse
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from typing import Optional
import cv2
import numpy as np

from gsrobotics.utilities.gelsightmini import GelSightMini
from gsrobotics.utilities.logger import log_message
from gsrobotics.config import GSConfig

## It is taking the config file from the gsrobotics package
class GelSightPublisher(Node):
    def __init__(
        self,
        node_name: str,
        topic: str,
        rate_hz: float,
        frame_id: str,
        gs_config_path: Optional[str] = None,
        device_id: Optional[int] = 1,
        warmup_seconds: float = 5.0,
        no_frame_log_interval: float = 5.0,
    ):
        super().__init__(node_name)
        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.frame_id = frame_id
        self.rate_hz = max(0.5, float(rate_hz))
        self.period = 1.0 / self.rate_hz
        self.device_id = device_id
        self.warmup_seconds = float(warmup_seconds)
        self._last_no_frame_log = 0.0
        self._no_frame_log_interval = float(no_frame_log_interval)

        # load config
        cfg_path = gs_config_path if gs_config_path is not None else "default_config.json"
        try:
            self.gs_config = GSConfig(cfg_path).config
        except Exception as e:
            self.get_logger().info(f"Impossibile caricare GSConfig da {cfg_path}: {e}. Uso valori di default.")
            self.gs_config = None

        target_w = getattr(self.gs_config, "camera_width", 640) if self.gs_config else 640
        target_h = getattr(self.gs_config, "camera_height", 480) if self.gs_config else 480
        border_fraction = getattr(self.gs_config, "border_fraction", 0.1) if self.gs_config else 0.1

        # instantiate camera object
        self.cam = GelSightMini(target_width=target_w, target_height=target_h, border_fraction=border_fraction)

        # select device BEFORE start
        if self.device_id is not None and hasattr(self.cam, "select_device"):
            try:
                self.get_logger().info(f"Selecting camera device id = {self.device_id} (before start)")
                self.cam.select_device(self.device_id)
            except Exception as e:
                self.get_logger().info(f"select_device({self.device_id}) raised: {e}")

        # start camera
        try:
            self.cam.start()
            self.get_logger().info("GelSight camera started.")
        except Exception as e:
            self.get_logger().info(f"Errore avviando la camera: {e}")

        # warmup visualization
        self._warmup_visualize()

        # start timer to publish frames
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.get_logger().info(f"Publishing on {topic} at {self.rate_hz} Hz (frame_id={self.frame_id})")

    def _warmup_visualize(self):
        """Show frames with OpenCV for warmup_seconds to verify camera is producing frames."""
        self.get_logger().info(f"Visualizing GelSight frames for {self.warmup_seconds:.1f} seconds (warmup)...")
        t0 = time.time()
        shown_any = False
        while time.time() - t0 < self.warmup_seconds:
            try:
                frame = self.cam.update(0.0)
            except Exception as e:
                self.get_logger().info(f"Warmup: cam.update() exception: {e}")
                frame = None

            if frame is None:
                time.sleep(0.01)
                continue

            shown_any = True
            try:
                cv2.imshow("GelSight Mini (warmup)", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
            except Exception as e:
                # if display fails, just log once and continue (do not raise)
                self.get_logger().info(f"cv2.imshow failed during warmup: {e}")
                break

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        if shown_any:
            self.get_logger().info("Warmup visualization done: frames were shown.")
        else:
            self.get_logger().info("Warmup visualization finished: no frames received during warmup.")

    def timer_callback(self):
        try:
            frame = self.cam.update(0.0)
            
        except Exception as e:
            self.get_logger().info(f"Error calling cam.update(): {e}")
            frame = None

        if frame is None:
            now = time.time()
            if now - self._last_no_frame_log > self._no_frame_log_interval:
                self.get_logger().info("No frame available from GelSight camera (still waiting)...")
                self._last_no_frame_log = now
            return
        try:
            frame = np.ascontiguousarray(frame)  # assicurati sia contiguo

            # pubblica come RGB (più veloce, evita conversioni inutili)
            if frame.ndim == 2 or (frame.ndim == 3 and frame.shape[2] == 1):
                encoding = "mono8"
            else:
                encoding = "rgb8"

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
            img_msg.header = Header()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.frame_id
            self.pub.publish(img_msg)

        except Exception as e:
            self.get_logger().info(f"Errore converting/publishing frame: {e}")

    def destroy_node(self):
        try:
            if hasattr(self, "cam") and self.cam is not None:
                if hasattr(self.cam, "camera") and self.cam.camera is not None:
                    try:
                        self.cam.camera.release()
                    except Exception:
                        pass
                if hasattr(self.cam, "stop"):
                    try:
                        self.cam.stop()
                    except Exception:
                        pass
        finally:
            super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="GelSight Mini ROS2 image publisher")
    parser.add_argument("--topic", type=str, default="/gsmini/image_raw", help="Topic to publish images")
    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate (Hz)")
    parser.add_argument("--frame-id", type=str, default="gsmini_camera", help="frame_id for image header")
    parser.add_argument("--gs-config", type=str, default=None, help="path to gs config JSON")
    parser.add_argument("--device-id", type=int, default=1, help="camera device id (optional). Default=1")
    parser.add_argument("--warmup-seconds", type=float, default=5.0, help="seconds to visualize frames on startup")
    args, _unknown = parser.parse_known_args()

    rclpy.init()
    node = GelSightPublisher(
        "gsmini_image_pub",
        args.topic,
        args.rate,
        args.frame_id,
        gs_config_path=args.gs_config,
        device_id=args.device_id,
        warmup_seconds=args.warmup_seconds,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down GelSight publisher...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()