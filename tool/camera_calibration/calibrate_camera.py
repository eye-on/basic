#!/usr/bin/env python3
"""
Camera calibration utility for the football robot vision pipeline.

This script:
1. Loads chessboard photos from a directory.
2. Detects corners and runs OpenCV camera calibration.
3. Saves JSON results compatible with this repo's camera model fields.
4. Exports visualizations for detected corners and undistorted comparisons.

Example:
  python calibrate_camera.py ^
    --images "photos/*.jpg" ^
    --pattern-cols 9 ^
    --pattern-rows 6 ^
    --square-size-mm 20 ^
    --output-dir output
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import asdict, dataclass
from typing import Any

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
except Exception as exc:
    print(
        "Missing dependency. Install with: pip install opencv-python numpy\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    sys.exit(2)


SUPPORTED_IMAGE_EXTENSIONS = (
    ".bmp",
    ".dib",
    ".jpg",
    ".jpeg",
    ".png",
    ".tif",
    ".tiff",
)


@dataclass
class CalibrationSummary:
    image_count_total: int
    image_count_used: int
    image_width_px: int
    image_height_px: int
    pattern_cols: int
    pattern_rows: int
    square_size_mm: float
    rms_reprojection_error: float
    mean_reprojection_error_px: float
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float
    k2: float
    p1: float
    p2: float
    k3: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate a pinhole camera with 5-parameter distortion."
    )
    parser.add_argument(
        "--images",
        required=True,
        help=(
            "Glob pattern or directory for calibration images, "
            "e.g. photos/*.jpg or photos"
        ),
    )
    parser.add_argument(
        "--pattern-cols",
        type=int,
        required=True,
        help="Number of inner corners per chessboard row.",
    )
    parser.add_argument(
        "--pattern-rows",
        type=int,
        required=True,
        help="Number of inner corners per chessboard column.",
    )
    parser.add_argument(
        "--square-size-mm",
        type=float,
        required=True,
        help="Chessboard square size in mm.",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory to write JSON and preview images.",
    )
    parser.add_argument(
        "--preview-limit",
        type=int,
        default=12,
        help="Max number of undistort comparison images to export.",
    )
    parser.add_argument(
        "--fail-on-mixed-sizes",
        action="store_true",
        help="Fail if input images do not share one resolution.",
    )
    parser.add_argument(
        "--skip-undistort-previews",
        action="store_true",
        help="Skip exporting side-by-side undistort previews.",
    )
    return parser.parse_args()


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def collect_image_paths(images_arg: str) -> list[str]:
    if os.path.isdir(images_arg):
        collected: list[str] = []
        for entry in sorted(os.listdir(images_arg)):
            path = os.path.join(images_arg, entry)
            if not os.path.isfile(path):
                continue
            _, extension = os.path.splitext(entry)
            if extension.lower() in SUPPORTED_IMAGE_EXTENSIONS:
                collected.append(path)
        return collected

    if any(token in images_arg for token in ("*", "?", "[")):
        import glob

        return sorted(glob.glob(images_arg))

    if os.path.isfile(images_arg):
        return [images_arg]

    return []


def build_object_points(pattern_cols: int, pattern_rows: int, square_size_mm: float) -> Any:
    object_points = np.zeros((pattern_rows * pattern_cols, 3), np.float32)
    grid = np.mgrid[0:pattern_cols, 0:pattern_rows].T.reshape(-1, 2)
    object_points[:, :2] = grid
    object_points *= float(square_size_mm)
    return object_points


def detect_corners(
    image_path: str,
    pattern_size: tuple[int, int],
    subpix_criteria: Any,
) -> tuple[bool, Any, Any, tuple[int, int]]:
    image = cv2.imread(image_path)
    if image is None:
      raise RuntimeError(f"Failed to read image: {image_path}")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if found:
        corners = cv2.cornerSubPix(
            gray,
            corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=subpix_criteria,
        )
    return found, image, corners, (gray.shape[1], gray.shape[0])


def reprojection_error(
    object_points: list[Any],
    image_points: list[Any],
    rvecs: list[Any],
    tvecs: list[Any],
    camera_matrix: Any,
    dist_coeffs: Any,
) -> float:
    total_error = 0.0
    total_points = 0
    for obj_pts, img_pts, rvec, tvec in zip(
        object_points, image_points, rvecs, tvecs
    ):
        projected, _ = cv2.projectPoints(
            obj_pts, rvec, tvec, camera_matrix, dist_coeffs
        )
        error = cv2.norm(img_pts, projected, cv2.NORM_L2)
        total_error += float(error * error)
        total_points += len(obj_pts)
    if total_points == 0:
        return float("nan")
    return math.sqrt(total_error / total_points)


def save_json(path: str, payload: dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=True)


def write_detection_preview(
    output_dir: str,
    image_name: str,
    image: Any,
    pattern_size: tuple[int, int],
    corners: Any,
    found: bool,
) -> None:
    preview = image.copy()
    cv2.drawChessboardCorners(preview, pattern_size, corners, found)
    out_name = os.path.splitext(os.path.basename(image_name))[0] + "_corners.jpg"
    cv2.imwrite(os.path.join(output_dir, out_name), preview)


def write_undistort_preview(
    output_dir: str,
    image_name: str,
    image: Any,
    camera_matrix: Any,
    dist_coeffs: Any,
) -> None:
    undistorted = cv2.undistort(image, camera_matrix, dist_coeffs)
    side_by_side = np.hstack((image, undistorted))
    out_name = os.path.splitext(os.path.basename(image_name))[0] + "_undistort.jpg"
    cv2.imwrite(os.path.join(output_dir, out_name), side_by_side)


def main() -> int:
    args = parse_args()
    pattern_size = (args.pattern_cols, args.pattern_rows)

    image_paths = collect_image_paths(args.images)
    if not image_paths:
        print(
            "No calibration images matched. Supported formats: "
            + ", ".join(SUPPORTED_IMAGE_EXTENSIONS)
            + f"\nInput: {args.images}",
            file=sys.stderr,
        )
        return 1

    ensure_dir(args.output_dir)
    corners_dir = os.path.join(args.output_dir, "corners")
    previews_dir = os.path.join(args.output_dir, "undistort_previews")
    ensure_dir(corners_dir)
    if not args.skip_undistort_previews:
        ensure_dir(previews_dir)

    object_template = build_object_points(
        args.pattern_cols, args.pattern_rows, args.square_size_mm
    )
    subpix_criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    object_points: list[Any] = []
    image_points: list[Any] = []
    accepted_paths: list[str] = []
    rejected_paths: list[str] = []
    accepted_images: list[Any] = []
    image_size: tuple[int, int] | None = None
    mixed_sizes: list[tuple[str, tuple[int, int]]] = []

    for image_path in image_paths:
        found, image, corners, current_size = detect_corners(
            image_path, pattern_size, subpix_criteria
        )
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            mixed_sizes.append((image_path, current_size))

        if not found:
            rejected_paths.append(image_path)
            continue

        object_points.append(object_template.copy())
        image_points.append(corners)
        accepted_paths.append(image_path)
        accepted_images.append(image)
        write_detection_preview(
            corners_dir, image_path, image, pattern_size, corners, found
        )

    if image_size is None:
        print("No readable input images.", file=sys.stderr)
        return 1

    if mixed_sizes:
        lines = [
            f"Mixed image sizes detected. First size: {image_size[0]}x{image_size[1]}"
        ]
        lines.extend(
            f"  {path}: {size[0]}x{size[1]}" for path, size in mixed_sizes[:10]
        )
        message = "\n".join(lines)
        if args.fail_on_mixed_sizes:
            print(message, file=sys.stderr)
            return 1
        print(f"Warning:\n{message}", file=sys.stderr)

    if len(object_points) < 8:
        print(
            f"Only {len(object_points)} valid images were detected. "
            "Use at least 8-10, preferably 20+.",
            file=sys.stderr,
        )
        return 1

    calibration_flags = 0
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=calibration_flags,
    )

    dist = dist_coeffs.reshape(-1)
    k1 = float(dist[0]) if dist.size > 0 else 0.0
    k2 = float(dist[1]) if dist.size > 1 else 0.0
    p1 = float(dist[2]) if dist.size > 2 else 0.0
    p2 = float(dist[3]) if dist.size > 3 else 0.0
    k3 = float(dist[4]) if dist.size > 4 else 0.0

    mean_error = reprojection_error(
        object_points,
        image_points,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    summary = CalibrationSummary(
        image_count_total=len(image_paths),
        image_count_used=len(object_points),
        image_width_px=int(image_size[0]),
        image_height_px=int(image_size[1]),
        pattern_cols=args.pattern_cols,
        pattern_rows=args.pattern_rows,
        square_size_mm=float(args.square_size_mm),
        rms_reprojection_error=float(rms),
        mean_reprojection_error_px=float(mean_error),
        fx=float(camera_matrix[0, 0]),
        fy=float(camera_matrix[1, 1]),
        cx=float(camera_matrix[0, 2]),
        cy=float(camera_matrix[1, 2]),
        k1=k1,
        k2=k2,
        p1=p1,
        p2=p2,
        k3=k3,
    )

    result_payload = {
        "summary": asdict(summary),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.reshape(-1).tolist(),
        "repo_camera_model": {
            "image_width_px": summary.image_width_px,
            "image_height_px": summary.image_height_px,
            "fx": summary.fx,
            "fy": summary.fy,
            "cx": summary.cx,
            "cy": summary.cy,
            "k1": summary.k1,
            "k2": summary.k2,
            "p1": summary.p1,
            "p2": summary.p2,
            "k3": summary.k3,
        },
        "accepted_images": accepted_paths,
        "rejected_images": rejected_paths,
    }

    save_json(os.path.join(args.output_dir, "calibration_result.json"), result_payload)
    save_json(os.path.join(args.output_dir, "repo_camera_model.json"), result_payload["repo_camera_model"])

    snippet_lines = [
        f"config.image_width_px = {summary.image_width_px:.1f};",
        f"config.image_height_px = {summary.image_height_px:.1f};",
        f"config.camera.fx = {summary.fx:.6f};",
        f"config.camera.fy = {summary.fy:.6f};",
        f"config.camera.cx = {summary.cx:.6f};",
        f"config.camera.cy = {summary.cy:.6f};",
        f"config.camera.k1 = {summary.k1:.9f};",
        f"config.camera.k2 = {summary.k2:.9f};",
        f"config.camera.p1 = {summary.p1:.9f};",
        f"config.camera.p2 = {summary.p2:.9f};",
        f"config.camera.k3 = {summary.k3:.9f};",
    ]
    with open(
        os.path.join(args.output_dir, "repo_snippet.txt"),
        "w",
        encoding="utf-8",
    ) as handle:
        handle.write("\n".join(snippet_lines) + "\n")

    if not args.skip_undistort_previews:
        for image_path, image in list(zip(accepted_paths, accepted_images))[: args.preview_limit]:
            write_undistort_preview(
                previews_dir,
                image_path,
                image,
                camera_matrix,
                dist_coeffs,
            )

    print("Calibration completed.")
    print(f"Used images: {summary.image_count_used} / {summary.image_count_total}")
    print(f"Image size: {summary.image_width_px}x{summary.image_height_px}")
    print(f"RMS reprojection error: {summary.rms_reprojection_error:.6f}")
    print(f"Mean reprojection error: {summary.mean_reprojection_error_px:.6f} px")
    print("Repo camera model:")
    print(json.dumps(result_payload["repo_camera_model"], indent=2, ensure_ascii=True))
    print(f"Output directory: {os.path.abspath(args.output_dir)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
