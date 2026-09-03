"""MediaPipe Hands 21-keypoint real-time tracker for LEAP Hand.

Wraps MediaPipe Tasks API (mp.tasks.vision.HandLandmarker) for per-frame use.
Provides:
  - detect(image) → list of 21 (x, y, z) landmarks per detected hand.
  - draw_landmarks(image, results) → annotated image.
  - run_demo(camera_id) → live webcam demo with OpenCV window.

Workstream: W1 手势映射 — .claude/workstreams/01-gesture-mapping.md
"""

import cv2
import time
import numpy as np
from pathlib import Path
from typing import List, Optional, Tuple

from mediapipe import Image as MpImage
from mediapipe import ImageFormat
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    HandLandmarker,
    HandLandmarkerOptions,
    HandLandmarkerResult,
    HandLandmarksConnections,
    RunningMode,
    drawing_utils,
)
from mediapipe.tasks.python.vision.hand_landmarker import HandLandmark

# ─── Model path ───────────────────────────────────────────────────
def _get_default_model_path() -> Path:
    candidates = [
        Path(__file__).resolve().parent / "models" / "hand_landmarker.task",
    ]
    for p in candidates:
        if p.exists():
            return p
    return candidates[0]



_DEFAULT_MODEL = _get_default_model_path()


# ─── Constants ────────────────────────────────────────────────────
_NUM_LANDMARKS = 21

# Alias our Landmark dict to the built-in HandLandmark enum values
Landmark = {
    "WRIST": HandLandmark.WRIST,
    "THUMB_CMC": HandLandmark.THUMB_CMC,
    "THUMB_MCP": HandLandmark.THUMB_MCP,
    "THUMB_IP": HandLandmark.THUMB_IP,
    "THUMB_TIP": HandLandmark.THUMB_TIP,
    "INDEX_MCP": HandLandmark.INDEX_FINGER_MCP,
    "INDEX_PIP": HandLandmark.INDEX_FINGER_PIP,
    "INDEX_DIP": HandLandmark.INDEX_FINGER_DIP,
    "INDEX_TIP": HandLandmark.INDEX_FINGER_TIP,
    "MIDDLE_MCP": HandLandmark.MIDDLE_FINGER_MCP,
    "MIDDLE_PIP": HandLandmark.MIDDLE_FINGER_PIP,
    "MIDDLE_DIP": HandLandmark.MIDDLE_FINGER_DIP,
    "MIDDLE_TIP": HandLandmark.MIDDLE_FINGER_TIP,
    "RING_MCP": HandLandmark.RING_FINGER_MCP,
    "RING_PIP": HandLandmark.RING_FINGER_PIP,
    "RING_DIP": HandLandmark.RING_FINGER_DIP,
    "RING_TIP": HandLandmark.RING_FINGER_TIP,
    "PINKY_MCP": HandLandmark.PINKY_MCP,
    "PINKY_PIP": HandLandmark.PINKY_PIP,
    "PINKY_DIP": HandLandmark.PINKY_DIP,
    "PINKY_TIP": HandLandmark.PINKY_TIP,
}

# Finger landmark ranges (start_idx, tip_idx)
FINGERS = {
    "thumb": (HandLandmark.THUMB_CMC, HandLandmark.THUMB_TIP),
    "index": (HandLandmark.INDEX_FINGER_MCP, HandLandmark.INDEX_FINGER_TIP),
    "middle": (HandLandmark.MIDDLE_FINGER_MCP, HandLandmark.MIDDLE_FINGER_TIP),
    "ring": (HandLandmark.RING_FINGER_MCP, HandLandmark.RING_FINGER_TIP),
    "pinky": (HandLandmark.PINKY_MCP, HandLandmark.PINKY_TIP),
}


class HandTracker:
    """MediaPipe Tasks HandLandmarker wrapper.

    Usage:
        tracker = HandTracker()
        results = tracker.detect(bgr_image)   # returns list[HandResult]
        if results:
            for hand in results:
                pts = tracker.landmark_xyz(hand)  # (21, 3) ndarray
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        max_num_hands: int = 2,
        min_detection_confidence: float = 0.35,
        min_presence_confidence: float = 0.30,
        min_tracking_confidence: float = 0.30,
    ):
        model = model_path or str(_DEFAULT_MODEL)
        if not Path(model).exists():
            raise FileNotFoundError(
                f"HandLandmarker model not found: {model}\n"
                f"Download from: https://developers.google.com/mediapipe/solutions/vision/hand_landmarker"
            )

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model),
            running_mode=RunningMode.VIDEO,
            num_hands=max_num_hands,
            min_hand_detection_confidence=min_detection_confidence,
            min_hand_presence_confidence=min_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._landmarker = HandLandmarker.create_from_options(options)

    # ─── Public API ───────────────────────────────────────────────

    def detect(self, image: np.ndarray) -> List["HandResult"]:
        """Run hand tracking on a BGR image (VIDEO mode: temporal prior).

        Args:
            image: BGR (H, W, 3) uint8 numpy array.

        Returns:
            List of HandResult, one per detected hand (empty list if none).
        """
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = MpImage(image_format=ImageFormat.SRGB, data=rgb)
        ts = int(time.monotonic() * 1000)  # monotonic ms → VIDEO mode prior
        mp_result = self._landmarker.detect_for_video(mp_image, ts)

        return self._parse_result(mp_result)

    def detect_raw(self, image: np.ndarray) -> HandLandmarkerResult:
        """Return raw HandLandmarkerResult (for advanced use)."""
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = MpImage(image_format=ImageFormat.SRGB, data=rgb)
        ts = int(time.monotonic() * 1000)
        return self._landmarker.detect_for_video(mp_image, ts)

    def draw_landmarks(
        self,
        image: np.ndarray,
        hand_results: List["HandResult"],
        draw_connections: bool = True,
    ) -> np.ndarray:
        """Draw 21 landmarks (and connections) onto a copy of the image.

        Args:
            image: BGR image.
            hand_results: Output from self.detect().
            draw_connections: If True, draw bone connections too.

        Returns:
            Annotated BGR image (new array, input not mutated).
        """
        annotated = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w = annotated.shape[:2]

        for hr in hand_results:
            if draw_connections:
                drawing_utils.draw_landmarks(
                    annotated,
                    list(hr.landmarks),
                    connections=HandLandmarksConnections.HAND_CONNECTIONS,
                    landmark_drawing_spec=drawing_utils.DrawingSpec(
                        color=(0, 255, 0), thickness=2, circle_radius=2
                    ),
                    connection_drawing_spec=drawing_utils.DrawingSpec(
                        color=(255, 255, 255), thickness=1, circle_radius=1
                    ),
                )
            else:
                for lm in hr.landmarks:
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(annotated, (cx, cy), 3, (0, 255, 0), -1)

            # Label handedness at wrist
            wrist = hr.landmarks[0]
            x, y = int(wrist.x * w), int(wrist.y * h)
            cv2.putText(
                annotated, hr.handedness,
                (x - 30, y - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )

        # Hand count overlay
        cv2.putText(
            annotated, f"Hands: {len(hand_results)}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2,
        )

        # Convert back to BGR for consistent output
        return cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)

    def landmark_xy(
        self,
        hand_result: "HandResult",
        image_shape: Tuple[int, int],
    ) -> np.ndarray:
        """Return (21, 2) pixel (x, y) array for one hand."""
        h, w = image_shape
        pts = np.zeros((_NUM_LANDMARKS, 2), dtype=np.float32)
        for i, lm in enumerate(hand_result.landmarks):
            pts[i] = (lm.x * w, lm.y * h)
        return pts

    def landmark_xyz(self, hand_result: "HandResult") -> np.ndarray:
        """Return (21, 3) normalized (x, y, z) array.

        x, y in [0, 1]; z is depth relative to wrist (unitless).
        """
        pts = np.zeros((_NUM_LANDMARKS, 3), dtype=np.float32)
        for i, lm in enumerate(hand_result.landmarks):
            pts[i] = (lm.x, lm.y, lm.z)
        return pts

    def close(self):
        """Release MediaPipe resources."""
        self._landmarker.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    # ─── Internal ─────────────────────────────────────────────────

    def _parse_result(self, mp_result: HandLandmarkerResult) -> List["HandResult"]:
        """Convert raw HandLandmarkerResult to a list of HandResult."""
        if not mp_result.hand_landmarks:
            return []

        results = []
        for i, landmarks in enumerate(mp_result.hand_landmarks):
            handedness = (
                mp_result.handedness[i][0].category_name
                if mp_result.handedness and i < len(mp_result.handedness)
                else "Unknown"
            )
            results.append(HandResult(
                landmarks=list(landmarks),
                handedness=handedness,
                world_landmarks=(
                    list(mp_result.hand_world_landmarks[i])
                    if mp_result.hand_world_landmarks
                    and i < len(mp_result.hand_world_landmarks)
                    else None
                ),
            ))
        return results


class HandResult:
    """Container for one detected hand."""

    def __init__(
        self,
        landmarks: list,
        handedness: str,
        world_landmarks: Optional[list] = None,
    ):
        self._landmarks = landmarks  # list[NormalizedLandmark]
        self.handedness = handedness  # "Left" or "Right"
        self.world_landmarks = world_landmarks  # list[Landmark] or None

    @property
    def landmarks(self):
        return self._landmarks

    @property
    def wrist(self):
        """Shorthand for wrist landmark (index 0)."""
        return self._landmarks[0]

    @property
    def num_landmarks(self) -> int:
        return len(self._landmarks)

    def __repr__(self):
        w = self.wrist
        return (
            f"HandResult(handedness={self.handedness}, "
            f"wrist=({w.x:.3f}, {w.y:.3f}, {w.z:.3f}))"
        )


# ─── Demo / Test ─────────────────────────────────────────────────

def run_demo(camera_id: int = 0):
    """Live MediaPipe Hands demo in an OpenCV window. Press 'q' to quit."""
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {camera_id}")
        return

    tracker = HandTracker(max_num_hands=2)
    print("[INFO] MediaPipe Hands demo running. Press 'q' to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            frame = cv2.flip(frame, 1)  # mirror
            results = tracker.detect(frame)
            annotated = tracker.draw_landmarks(frame, results)
            cv2.imshow("MediaPipe Hands — LEAP Hand Tracker", annotated)

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
    finally:
        tracker.close()
        cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Demo stopped.")


def test_with_image(image_path: str):
    """Run detection on a single image and print all 21 landmarks.

    Also saves an annotated copy as `<name>_tracked.jpg`.
    """
    image = cv2.imread(image_path)
    if image is None:
        print(f"[ERROR] Cannot read: {image_path}")
        return

    tracker = HandTracker(max_num_hands=2)
    results = tracker.detect(image)

    print(f"Detected {len(results)} hand(s):")
    for i, hr in enumerate(results):
        wrist = hr.wrist
        print(
            f"  Hand {i}: {hr.handedness}, "
            f"wrist=({wrist.x:.4f}, {wrist.y:.4f}, {wrist.z:.4f})"
        )
        for name, idx in Landmark.items():
            lm = hr.landmarks[idx]
            print(f"    {name:15s}  ({lm.x:.4f}, {lm.y:.4f}, {lm.z:.4f})")

    annotated = tracker.draw_landmarks(image, results)
    out_path = image_path.rsplit(".", 1)[0] + "_tracked.jpg"
    cv2.imwrite(out_path, annotated)
    print(f"\nAnnotated image saved to: {out_path}")

    tracker.close()


def _smoke_test():
    """Minimal inference test — no camera or image file needed."""
    print("[TEST] Smoke-testing MediaPipe Hands import + pipeline...")
    tracker = HandTracker(max_num_hands=1)
    dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    results = tracker.detect(dummy)
    print(f"  Hands detected on blank image: {len(results)} (expected 0)")
    tracker.close()
    print("[TEST] Smoke test PASSED. MediaPipe Hands is working.")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--image":
        if len(sys.argv) < 3:
            print("Usage: python hand_tracker.py --image <path>")
        else:
            test_with_image(sys.argv[2])
    elif len(sys.argv) > 1 and sys.argv[1] == "--demo":
        cam_id = int(sys.argv[2]) if len(sys.argv) > 2 else 0
        run_demo(camera_id=cam_id)
    else:
        _smoke_test()
