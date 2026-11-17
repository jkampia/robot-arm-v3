"""Persistent board pickup poses. Coordinates use mm; pitch and roll use degrees."""

import json
import math
import os
import tempfile
import threading
from pathlib import Path


class ParameterManager:
    PICKUP_POINTS = (
        "top_right_pickup_point",
        "top_left_pickup_point",
        "bottom_right_pickup_point",
        "bottom_left_pickup_point",
    )

    def __init__(self, path=None):
        self.path = Path(path) if path is not None else Path(__file__).with_name("parameters.json")
        self._lock = threading.RLock()
        self._poses = dict.fromkeys(self.PICKUP_POINTS)
        if self.path.exists():
            data = json.loads(self.path.read_text())
            if not isinstance(data, dict) or set(data) - set(self.PICKUP_POINTS):
                raise ValueError("Invalid pickup point parameter file")
            for name, pose in data.items():
                if pose is not None:
                    self._poses[name] = self.validate_pose(pose)

    @staticmethod
    def validate_pose(pose):
        if not isinstance(pose, (list, tuple)) or len(pose) != 5:
            raise ValueError("A pickup pose requires X, Y, Z, P, R")
        if any(isinstance(value, bool) for value in pose):
            raise ValueError("Pose values must be numbers")
        values = [float(value) for value in pose]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Pose values must be finite")
        return values

    def to_dict(self):
        with self._lock:
            return {name: list(pose) if pose is not None else None
                    for name, pose in self._poses.items()}

    def get_pickup_point(self, name):
        with self._lock:
            if name not in self._poses:
                raise ValueError(f"Unknown pickup point: {name}")
            if self._poses[name] is None:
                raise ValueError(f"Pickup point has not been saved: {name}")
            return list(self._poses[name])

    def set_pickup_point(self, name, pose):
        values = self.validate_pose(pose)
        with self._lock:
            if name not in self._poses:
                raise ValueError(f"Unknown pickup point: {name}")
            updated = {**self._poses, name: values}
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(mode="w", dir=self.path.parent,
                                                 delete=False) as file:
                    temporary = Path(file.name)
                    json.dump(updated, file, indent=2, allow_nan=False)
                    file.write("\n")
                os.replace(temporary, self.path)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            self._poses = updated
