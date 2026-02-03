#!/usr/bin/env python3
import time
import argparse
import cv2

# ---------------- compatibility shim (put this at top of your script) ----------------
import os
import sys
import importlib
from types import ModuleType

current_dir = os.path.dirname(os.path.abspath(__file__))

root = current_dir
while True:
    if os.path.isdir(os.path.join(root, "gsrobotics")):
        break
    parent = os.path.dirname(root)
    if parent == root:
        raise RuntimeError("Impossibile trovare la cartella 'gsrobotics' risalendo da: " + current_dir)
    root = parent

if root not in sys.path:
    sys.path.insert(0, root)

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
    pass
# ---------------- end shim ----------------

from gsrobotics.utilities.gelsightmini import GelSightMini
from gsrobotics.utilities.logger import log_message
from gsrobotics.config import GSConfig


def main():
    parser = argparse.ArgumentParser(
        description="Check connection to GelSight Mini (with FPS measurement)"
    )
    parser.add_argument("--gs-config", type=str, default=None)
    parser.add_argument(
        "--device-id",
        type=int,
        default=1,
        help="Index del device camera da usare"
    )
    parser.add_argument(
        "--measure-interval",
        type=float,
        default=1.0,
        help="Seconds between FPS reports (default: 1.0)"
    )
    args = parser.parse_args()

    # stesso flusso del viewer
    if args.gs_config is not None:
        log_message(f"Provided config path: {args.gs_config}")
    else:
        log_message("Using default config path './default_config.json'")
        args.gs_config = "default_config.json"

    gs_config = GSConfig(args.gs_config).config

    print("🔎 Verifica connessione GelSight Mini...")

    cam = GelSightMini(
        target_width=gs_config.camera_width,
        target_height=gs_config.camera_height,
        border_fraction=gs_config.border_fraction,
    )

    try:
        if args.device_id is not None:
            print(f"🎯 Seleziono device ID = {args.device_id}")
            cam.select_device(args.device_id)

        cam.start()
        print("✅ Stream avviato. Attendo un frame...")

        print("👀 Visualizzazione frame (premi 'q' per uscire)")

        # --- FPS measurement state ---
        total_frames = 0
        total_update_time = 0.0
        t_start = time.time()
        t_last_report = t_start
        frames_since_last = 0
        measure_interval = float(args.measure_interval)

        while True:
            t_before_update = time.time()
            frame = cam.update(0.0)
            t_after_update = time.time()

            update_dt = t_after_update - t_before_update
            if frame is None:
                # small sleep to avoid busy-looping when no frames
                time.sleep(0.005)
            else:
                total_frames += 1
                frames_since_last += 1
                total_update_time += update_dt

                cv2.imshow("GelSight Mini", frame)

            now = time.time()
            if now - t_last_report >= measure_interval:
                elapsed = now - t_last_report
                fps_instant = frames_since_last / elapsed if elapsed > 0 else 0.0
                elapsed_total = now - t_start
                avg_fps = total_frames / elapsed_total if elapsed_total > 0 else 0.0
                avg_update_time = (total_update_time / total_frames) if total_frames > 0 else 0.0

                print(
                    f"[FPS] instant={fps_instant:.2f}Hz  avg={avg_fps:.2f}Hz  "
                    f"frames_total={total_frames}  avg_update_time={avg_update_time*1000:.2f}ms"
                )

                frames_since_last = 0
                t_last_report = now

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

        # final summary
        t_end = time.time()
        total_time = t_end - t_start
        avg_fps_final = total_frames / total_time if total_time > 0 else 0.0
        avg_update_time_final = (total_update_time / total_frames) if total_frames > 0 else 0.0
        print("=== SUMMARY ===")
        print(f"Total frames: {total_frames}")
        print(f"Total time  : {total_time:.3f} s")
        print(f"Average FPS : {avg_fps_final:.2f} Hz")
        print(f"Avg update() time: {avg_update_time_final*1000:.2f} ms")

        cv2.destroyAllWindows()


    except Exception as e:
        print("❌ Non connesso al GelSight Mini")
        print(f"   {type(e).__name__}: {e}")

    finally:
        try:
            if hasattr(cam, "camera") and cam.camera is not None:
                cam.camera.release()
        except Exception:
            pass


if __name__ == "__main__":
    main()
