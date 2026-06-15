#!/usr/bin/env python3
import argparse
import logging
import sys
import time
from pathlib import Path

import pyzed.sl as sl

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

RESOLUTIONS = {
    "HD2K":   sl.RESOLUTION.HD2K,
    "HD1080": sl.RESOLUTION.HD1080,
    "SVGA":   sl.RESOLUTION.SVGA,
}

DEPTH_MODE = {
    "NEURAL_LIGHT": sl.DEPTH_MODE.NEURAL_LIGHT,
    "NEURAL": sl.DEPTH_MODE.NEURAL,
    "NEURAL_PLUS": sl.DEPTH_MODE.NEURAL_PLUS
}

def parse_args():
    parser = argparse.ArgumentParser(description="Record ZED camera to an SVO file.")
    parser.add_argument("--output", default="test_svo.svo2", help="Output .svo2 file name")
    parser.add_argument("--dir", default="/home/nvidia/ros2_ws/src/zedx_pure_pursuit/data/svo", help="Output directory")
    parser.add_argument("--fps", type=int, default=60, choices=[15, 30, 60, 120], help="Camera FPS (default: 60)")
    parser.add_argument("--resolution", default="SVGA", choices=RESOLUTIONS.keys(), help="Camera resolution (default: SVGA)")
    parser.add_argument("--depth_mode", default="NEURAL_LIGHT", choices=DEPTH_MODE.keys(), help="Camera resolution (default: SVGA)")
    return parser.parse_args()

def main():
    args = parse_args()
    out_dir = Path(args.dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = RESOLUTIONS[args.resolution]
    init_params.camera_fps = args.fps
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_params.depth_mode = DEPTH_MODE[args.depth_mode]

    log.info("Opening camera | resolution=%s fps=%d depth_mode=%s", args.resolution, args.fps, args.depth_mode)
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        log.error("Failed to open ZED camera")
        sys.exit(1)

    runtime_parameters = sl.RuntimeParameters()

    recording_params = sl.RecordingParameters()
    recording_params.compression_mode = sl.SVO_COMPRESSION_MODE.H264
    recording_params.video_filename = str(out_dir / args.output)

    if zed.enable_recording(recording_params) != sl.ERROR_CODE.SUCCESS:
        log.error("Failed to enable recording")
        zed.close()
        sys.exit(1)

    log.info("Recording to %s — press Ctrl+C to stop", args.output)
    try:
        frame_count = 0
        while True:
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                frame_count += 1
                if frame_count % 100 == 0:
                    log.info("Captured %d frames", frame_count)
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        log.info("Stopping...")

    zed.disable_recording()
    zed.close()
    log.info("Done — %d frames saved to %s", frame_count, args.output)

if __name__ == "__main__":
    main()
