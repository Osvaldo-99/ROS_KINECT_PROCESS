#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
process_point_cloud.py

Reproducible PLY point cloud processing pipeline.

This script provides:
1. Loading of .PLY point clouds.
2. Point cloud reduction using iterative voxelization with Open3D.
3. Exportation of reduced point clouds to .PLY.
4. Exportation of reduced point clouds to .DAE using cube-based representation.
5. Exportation of reduced points to .CSV format.
6. Generation of a representative robot sphere in .DAE format.
7. Floor and wall segmentation using RANSAC.
8. Exportation of segmented floor and walls to .DAE.
9. Convex Hull computation and exportation for floor contours.
10. Experimental summary exportation in .CSV and .JSON formats.

Example usage:

python3 src/process_point_cloud.py \
    --input "/home/osvaldo07/Data/original_cloud.ply" \
    --output-dir "data/output/experiment_26" \
    --prefix "Experiment_26" \
    --target-points 100000 \
    --tolerance 10000 \
    --voxel-start 0.015 \
    --voxel-max 0.05 \
    --voxel-step 0.001
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import trimesh
import open3d as o3d

from numpy.linalg import norm
from scipy.spatial import ConvexHull
from scipy.spatial import QhullError


# ============================================================
# TIME UTILITIES
# ============================================================

def format_time(seconds):
    """
    Converts seconds into a readable minutes-and-seconds format.
    """
    minutes = int(seconds // 60)
    sec = seconds % 60
    return f"{minutes} min {sec:.2f} s"


# ============================================================
# GEOMETRIC UTILITIES
# ============================================================

def plane_from_points(p1, p2, p3):
    """
    Computes the plane passing through three points.

    Plane equation:
        n . x + d = 0

    Returns:
        n: unit normal vector
        d: plane offset term
    """
    v1 = p2 - p1
    v2 = p3 - p1

    n = np.cross(v1, v2)
    n_norm = norm(n)

    if n_norm < 1e-12:
        return None, None

    n = n / n_norm
    d = -np.dot(n, p1)

    return n, d


def point_plane_dist(points, n, d):
    """
    Computes the algebraic distance from a set of points to a plane.
    """
    return points.dot(n) + d


def angle_deg(u, v):
    """
    Computes the angle in degrees between two vectors.
    """
    u = u / (norm(u) + 1e-12)
    v = v / (norm(v) + 1e-12)

    cosang = np.clip(np.dot(u, v), -1.0, 1.0)
    return np.degrees(np.arccos(cosang))


# ============================================================
# POINT CLOUD REDUCTION WITH OPEN3D
# ============================================================

def iterative_voxel_downsampling(
    input_ply_path,
    output_ply_path,
    target_points=100000,
    tolerance=10000,
    voxel_start=0.015,
    voxel_max=0.05,
    voxel_step=0.001
):
    """
    Reduces a point cloud using Open3D voxel_down_sample.

    The process tests different voxel sizes until it approximates
    the target number of points.

    Returns:
        points: NumPy array containing the reduced points
        metrics: dictionary containing reduction metrics
    """

    input_ply_path = Path(input_ply_path)
    output_ply_path = Path(output_ply_path)

    if not input_ply_path.exists():
        raise FileNotFoundError(f"File not found: {input_ply_path}")

    if input_ply_path.suffix.lower() != ".ply":
        raise ValueError("The input file must have a .ply extension")

    print("Loading original point cloud with Open3D...")
    cloud = o3d.io.read_point_cloud(str(input_ply_path))

    original_points = len(cloud.points)

    if original_points == 0:
        raise ValueError("The point cloud is empty.")

    print(f"🔢 Original points: {original_points}")

    current_voxel = voxel_start
    best_cloud = None
    best_error = float("inf")
    best_voxel = voxel_start
    best_total_points = 0

    voxel_history = []

    while current_voxel <= voxel_max + 1e-12:
        filtered_cloud = cloud.voxel_down_sample(voxel_size=current_voxel)
        total_points = len(filtered_cloud.points)
        error = abs(total_points - target_points)

        voxel_history.append({
            "voxel": round(current_voxel, 6),
            "points": int(total_points),
            "error": int(error)
        })

        print(f"🔍 Testing voxel={current_voxel:.3f} → {total_points} points")

        if error < best_error:
            best_error = error
            best_cloud = filtered_cloud
            best_voxel = current_voxel
            best_total_points = total_points

        if error <= tolerance:
            print("🎯 Tolerance reached. Stopping iteration.")
            break

        current_voxel += voxel_step

    if best_cloud is None:
        raise RuntimeError("Unable to reduce the point cloud.")

    output_ply_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(output_ply_path), best_cloud)

    final_points = len(best_cloud.points)
    reduction = 100 * (1 - final_points / original_points)

    print(f"\n✅ Reduced point cloud saved as: {output_ply_path}")
    print(f"📦 Optimal voxel: {best_voxel:.3f}")
    print(f"📉 Final points: {final_points}")
    print(f"📊 Reduction: {reduction:.2f}%")

    points = np.asarray(best_cloud.points)

    metrics = {
        "original_points": int(original_points),
        "final_points": int(final_points),
        "target_points": int(target_points),
        "tolerance": int(tolerance),
        "optimal_voxel": float(best_voxel),
        "voxel_start": float(voxel_start),
        "voxel_max": float(voxel_max),
        "voxel_step": float(voxel_step),
        "reduction_percentage": float(reduction),
        "best_error": int(best_error),
        "best_total_points": int(best_total_points),
        "voxel_history": voxel_history
    }

    return points, metrics


# ============================================================
# PLANE RANSAC
# ============================================================

def fit_plane_ransac(points, distance_threshold=0.02, max_iterations=2000, random_state=0):
    """
    Fits a plane using RANSAC.

    Returns:
        n_refined: refined plane normal
        d_refined: refined plane offset term
        inliers: boolean mask for plane inlier points
    """
    rng = np.random.default_rng(random_state)

    N = points.shape[0]
    best_inliers = None
    best_count = 0
    best_model = (None, None)

    if N < 3:
        return None, None, np.zeros(N, dtype=bool)

    idxs = np.arange(N)

    for _ in range(max_iterations):
        i1, i2, i3 = rng.choice(idxs, size=3, replace=False)

        p1 = points[i1]
        p2 = points[i2]
        p3 = points[i3]

        n, d = plane_from_points(p1, p2, p3)

        if n is None:
            continue

        dist = np.abs(point_plane_dist(points, n, d))
        inliers = dist <= distance_threshold

        count = np.count_nonzero(inliers)

        if count > best_count:
            best_count = count
            best_inliers = inliers
            best_model = (n, d)

    n, d = best_model

    if n is None or best_inliers is None:
        return None, None, np.zeros(N, dtype=bool)

    P = points[best_inliers]

    if P.shape[0] < 3:
        return None, None, np.zeros(N, dtype=bool)

    centroid = P.mean(axis=0)

    _, _, Vt = np.linalg.svd(P - centroid)

    n_refined = Vt[-1]
    n_refined = n_refined / (norm(n_refined) + 1e-12)

    d_refined = -np.dot(n_refined, centroid)

    dist = np.abs(point_plane_dist(points, n_refined, d_refined))
    inliers = dist <= distance_threshold

    return n_refined, d_refined, inliers


# ============================================================
# FLOOR AND WALL SEGMENTATION
# ============================================================

def segment_floor_and_walls(
    points,
    distance_threshold=0.02,
    max_iterations=2000,
    z_axis=np.array([0, 0, 1.0]),
    floor_angle_tol_deg=15.0,
    wall_angle_range_deg=(75.0, 105.0),
    min_inliers_plane=800,
    max_wall_planes=6,
    random_state=42
):
    """
    Segments floor and walls from the reduced point cloud.

    Criterion:
    - Floor: plane whose normal is approximately parallel to the Z axis.
    - Walls: planes whose normal forms approximately 90 degrees with the Z axis.
    """

    result = {
        "floor_points": np.empty((0, 3)),
        "floor_normal": None,
        "floor_d": None,
        "walls": [],
        "remaining_points": points.copy()
    }

    n, d, inliers = fit_plane_ransac(
        points,
        distance_threshold=distance_threshold,
        max_iterations=max_iterations,
        random_state=random_state
    )

    if n is not None:
        ang = angle_deg(n, z_axis)

        if ang <= floor_angle_tol_deg or (180 - ang) <= floor_angle_tol_deg:
            result["floor_points"] = points[inliers]
            result["floor_normal"] = n
            result["floor_d"] = d
            remaining = points[~inliers]
        else:
            remaining = points.copy()
    else:
        remaining = points.copy()

    current = remaining
    rng = np.random.default_rng(random_state + 123)

    for _ in range(max_wall_planes):
        if current.shape[0] < min_inliers_plane:
            break

        n, d, inliers = fit_plane_ransac(
            current,
            distance_threshold=distance_threshold,
            max_iterations=max_iterations,
            random_state=int(rng.integers(0, 1_000_000_000))
        )

        if n is None:
            break

        ang = angle_deg(n, z_axis)

        if wall_angle_range_deg[0] <= ang <= wall_angle_range_deg[1]:
            wall_pts = current[inliers]

            if wall_pts.shape[0] >= min_inliers_plane:
                result["walls"].append({
                    "points": wall_pts,
                    "normal": n,
                    "d": d
                })

                current = current[~inliers]
            else:
                break
        else:
            current = current[~inliers]

    result["remaining_points"] = current

    return result


# ============================================================
# FLOOR CONVEX HULL
# ============================================================

def convex_hull_on_plane(points_on_plane, plane_normal):
    """
    Computes the 2D Convex Hull of points projected onto the floor plane.
    """
    if points_on_plane.shape[0] < 3:
        return None, None, None

    n = plane_normal / (norm(plane_normal) + 1e-12)

    tmp = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])

    u = np.cross(n, tmp)
    u = u / (norm(u) + 1e-12)

    v = np.cross(n, u)
    v = v / (norm(v) + 1e-12)

    c = points_on_plane.mean(axis=0)

    X = points_on_plane - c
    x2 = np.c_[X.dot(u), X.dot(v)]

    try:
        hull2d = ConvexHull(x2)
    except QhullError:
        return None, None, None

    return hull2d, x2, (u, v, c)


def hull2d_vertices_3d(hull2d, basis):
    """
    Converts 2D Convex Hull vertices back to 3D coordinates.
    """
    u, v, c = basis

    order = hull2d.vertices
    order = np.r_[order, order[0]]

    verts2d = hull2d.points[order]

    verts3d = c + np.outer(verts2d[:, 0], u) + np.outer(verts2d[:, 1], v)

    return verts3d


# ============================================================
# DAE AND CSV EXPORTATION
# ============================================================

def export_points_to_collada(points, cube_size, output_collada_path, scal, color):
    """
    Exports points as a collection of cubes in DAE format.

    Note:
    This method can be computationally expensive if the cloud contains too many points.
    For lightweight visualization, use previously reduced point clouds.
    """
    output_collada_path = Path(output_collada_path)
    output_collada_path.parent.mkdir(parents=True, exist_ok=True)

    meshes = []

    for point in points:
        scaled_point = point * scal

        cube = trimesh.creation.box(
            extents=[cube_size * scal] * 3,
            transform=trimesh.transformations.translation_matrix(scaled_point)
        )

        cube.visual.face_colors = color
        meshes.append(cube)

    if len(meshes) == 0:
        cube = trimesh.creation.box(
            extents=[1e-6, 1e-6, 1e-6],
            transform=trimesh.transformations.translation_matrix([1e9, 1e9, 1e9])
        )

        cube.visual.face_colors = color
        meshes = [cube]

    combined = trimesh.util.concatenate(meshes)
    combined.export(str(output_collada_path), file_type="dae")


def export_points_to_csv(points, output_csv_path, scal):
    """
    Guarda los points en un archivo CSV con columnas x, y, z.
    """
    output_csv_path = Path(output_csv_path)
    output_csv_path.parent.mkdir(parents=True, exist_ok=True)

    scaled_points = points * scal

    df = pd.DataFrame(scaled_points, columns=["x", "y", "z"])
    df.to_csv(output_csv_path, index=False)


def export_floor_hull_wire(
    hull_vertices_3d,
    out_path,
    wire_radius=0.01,
    color=[0.0, 0.8, 1.0, 1.0]
):
    """
    Exports the floor contour as a wire-like DAE structure.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if hull_vertices_3d is None or hull_vertices_3d.shape[0] < 2:
        return False

    segments = []

    for i in range(len(hull_vertices_3d) - 1):
        p0 = hull_vertices_3d[i]
        p1 = hull_vertices_3d[i + 1]

        length = norm(p1 - p0)

        if length < 1e-12:
            continue

        seg = trimesh.creation.cylinder(
            radius=wire_radius,
            height=length,
            sections=12
        )

        direction = (p1 - p0) / (length + 1e-12)

        z_axis = np.array([0, 0, 1.0])

        v = np.cross(z_axis, direction)
        s = norm(v)
        c = np.dot(z_axis, direction)

        if s < 1e-12:
            R = np.eye(4)

            if c < 0:
                R[:3, :3] = -np.eye(3)
            else:
                R[:3, :3] = np.eye(3)
        else:
            vx = np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])

            R3 = np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2))

            R = np.eye(4)
            R[:3, :3] = R3

        T = np.eye(4)
        T[:3, 3] = (p0 + p1) / 2.0

        seg.apply_transform(T @ R)
        seg.visual.face_colors = color

        segments.append(seg)

    if len(segments) == 0:
        return False

    wire = trimesh.util.concatenate(segments)
    wire.export(str(out_path), file_type="dae")

    return True


# ============================================================
# COMMAND-LINE ARGUMENTS
# ============================================================

def parse_args():
    """
    Defines the command-line arguments required to run the script
    without manually modifying paths inside the code.
    """

    parser = argparse.ArgumentParser(
        description="Reproducible PLY point cloud processing: reduction, DAE, CSV, floor/wall segmentation, and Convex Hull."
    )

    parser.add_argument(
        "--input",
        required=True,
        help="Input path of the original point cloud in .ply format."
    )

    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory where all generated results will be saved."
    )

    parser.add_argument(
        "--prefix",
        default="processed_cloud",
        help="Prefix used to name output files."
    )

    parser.add_argument(
        "--target-points",
        type=int,
        default=100000,
        help="Target number of points after voxelization-based reduction."
    )

    parser.add_argument(
        "--tolerance",
        type=int,
        default=10000,
        help="Allowed tolerance with respect to the target number of points."
    )

    parser.add_argument(
        "--voxel-start",
        type=float,
        default=0.015,
        help="Initial voxel size."
    )

    parser.add_argument(
        "--voxel-max",
        type=float,
        default=0.05,
        help="Maximum voxel size."
    )

    parser.add_argument(
        "--voxel-step",
        type=float,
        default=0.001,
        help="Iterative voxel size increment."
    )

    parser.add_argument(
        "--cube-size",
        type=float,
        default=0.005,
        help="Cube size used to represent each point in DAE."
    )

    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Scale factor applied to exported points."
    )

    parser.add_argument(
        "--ransac-distance",
        type=float,
        default=0.02,
        help="Distance threshold for plane RANSAC."
    )

    parser.add_argument(
        "--ransac-iterations",
        type=int,
        default=2500,
        help="Maximum number of RANSAC iterations."
    )

    parser.add_argument(
        "--floor-angle-tol",
        type=float,
        default=15.0,
        help="Angular tolerance in degrees for floor detection."
    )

    parser.add_argument(
        "--wall-angle-min",
        type=float,
        default=75.0,
        help="Minimum angle in degrees for wall classification."
    )

    parser.add_argument(
        "--wall-angle-max",
        type=float,
        default=105.0,
        help="Maximum angle in degrees for wall classification."
    )

    parser.add_argument(
        "--min-inliers-plane",
        type=int,
        default=800,
        help="Minimum number of points required to accept a plane."
    )

    parser.add_argument(
        "--max-wall-planes",
        type=int,
        default=6,
        help="Maximum number of wall planes to detect."
    )

    parser.add_argument(
        "--random-state",
        type=int,
        default=42,
        help="Random seed for reproducibility."
    )

    parser.add_argument(
        "--no-dae",
        action="store_true",
        help="If enabled, DAE files are not exported. Useful for quick tests."
    )

    return parser.parse_args()


# ============================================================
# MAIN FUNCTION
# ============================================================

def main():
    args = parse_args()

    input_ply_path = Path(args.input)
    base_dir = Path(args.output_dir)
    base_dir.mkdir(parents=True, exist_ok=True)

    prefix = args.prefix

    output_reduced_ply = base_dir / f"{prefix}_reduced.ply"

    output_collada_path = base_dir / f"{prefix}_scene.dae"
    output_csv_path = base_dir / f"{prefix}_points.csv"
    output_sphere_collada_path = base_dir / f"{prefix}_robot.dae"

    output_floor_dae = base_dir / f"{prefix}_floor.dae"
    output_walls_dae = base_dir / f"{prefix}_walls.dae"
    output_floor_hull_dae = base_dir / f"{prefix}_floor_hull.dae"

    output_metrics_csv = base_dir / f"{prefix}_metrics.csv"
    output_summary_json = base_dir / f"{prefix}_summary.json"
    output_voxel_history_csv = base_dir / f"{prefix}_voxel_history.csv"

    target_points = args.target_points
    tolerance = args.tolerance
    voxel_start = args.voxel_start
    voxel_max = args.voxel_max
    voxel_step = args.voxel_step

    cube_size = args.cube_size
    scal = args.scale

    color_scene = [0.0, 1.0, 0.0, 1.0]
    robot_color = [1.0, 0.0, 0.0, 1.0]
    floor_color = [0.2, 0.9, 0.2, 1.0]
    wall_color = [0.9, 0.2, 0.2, 1.0]
    hull_color = [0.0, 0.8, 1.0, 1.0]

    total_start_time = time.perf_counter()

    timings = {
        "reduction": 0.0,
        "dae_scene": 0.0,
        "csv": 0.0,
        "robot": 0.0,
        "segmentation": 0.0,
        "floor": 0.0,
        "walls": 0.0,
        "hull": 0.0,
        "total": 0.0
    }

    generated_files = {
        "reduced_point_cloud_ply": str(output_reduced_ply),
        "full_scene_dae": None if args.no_dae else str(output_collada_path),
        "csv_points": str(output_csv_path),
        "robot_dae": None if args.no_dae else str(output_sphere_collada_path),
        "floor_dae": None if args.no_dae else str(output_floor_dae),
        "walls_dae": None if args.no_dae else str(output_walls_dae),
        "floor_hull_dae": None if args.no_dae else str(output_floor_hull_dae),
        "metrics_csv": str(output_metrics_csv),
        "summary_json": str(output_summary_json),
        "voxel_history_csv": str(output_voxel_history_csv)
    }

    try:
        print("\n========================================")
        print("REPRODUCIBLE 3D POINT CLOUD PROCESSING")
        print("========================================")

        print("\nInput file:")
        print(input_ply_path)

        print("\nOutput directory:")
        print(base_dir)

        print("\nExperiment prefix:")
        print(prefix)

        # ----------------------------
        # Stage 1: point cloud reduction
        # ----------------------------
        print("\n----------------------------------------")
        print("Stage 1: voxelization-based point cloud reduction")
        print("----------------------------------------")

        t0 = time.perf_counter()

        points, metrics_reduction = iterative_voxel_downsampling(
            input_ply_path=str(input_ply_path),
            output_ply_path=str(output_reduced_ply),
            target_points=target_points,
            tolerance=tolerance,
            voxel_start=voxel_start,
            voxel_max=voxel_max,
            voxel_step=voxel_step
        )

        timings["reduction"] = time.perf_counter() - t0
        print(f"\n⏱️ Point cloud reduction time: {format_time(timings['reduction'])}")

        # Save voxelization history
        pd.DataFrame(metrics_reduction["voxel_history"]).to_csv(
            output_voxel_history_csv,
            index=False
        )

        # ----------------------------
        # Stage 2: export scene DAE
        # ----------------------------
        if not args.no_dae:
            print("\n----------------------------------------")
            print("Stage 2: export reduced full scene to DAE")
            print("----------------------------------------")

            t0 = time.perf_counter()

            export_points_to_collada(
                points,
                cube_size,
                str(output_collada_path),
                scal,
                color_scene
            )

            timings["dae_scene"] = time.perf_counter() - t0
            print(f"⏱️ Scene DAE export time: {format_time(timings['dae_scene'])}")
        else:
            print("\n⏭️ Scene DAE export skipped due to --no-dae")

        # ----------------------------
        # Stage 3: save CSV
        # ----------------------------
        print("\n----------------------------------------")
        print("Stage 3: save reduced point cloud CSV")
        print("----------------------------------------")

        t0 = time.perf_counter()

        export_points_to_csv(
            points,
            str(output_csv_path),
            scal
        )

        timings["csv"] = time.perf_counter() - t0
        print(f"⏱️ CSV saving time: {format_time(timings['csv'])}")

        # ----------------------------
        # Stage 4: export robot
        # ----------------------------
        if not args.no_dae:
            print("\n----------------------------------------")
            print("Stage 4: export representative robot sphere")
            print("----------------------------------------")

            t0 = time.perf_counter()

            sphere = trimesh.creation.icosphere(radius=cube_size * scal)
            sphere.visual.face_colors = robot_color
            sphere.export(str(output_sphere_collada_path), file_type="dae")

            timings["robot"] = time.perf_counter() - t0
            print(f"⏱️ Robot DAE export time: {format_time(timings['robot'])}")
        else:
            print("\n⏭️ Robot DAE export skipped due to --no-dae")

        # ----------------------------
        # Stage 5: segmentation
        # ----------------------------
        print("\n----------------------------------------")
        print("Stage 5: floor and wall segmentation")
        print("----------------------------------------")

        t0 = time.perf_counter()

        seg = segment_floor_and_walls(
            points,
            distance_threshold=args.ransac_distance,
            max_iterations=args.ransac_iterations,
            floor_angle_tol_deg=args.floor_angle_tol,
            wall_angle_range_deg=(args.wall_angle_min, args.wall_angle_max),
            min_inliers_plane=args.min_inliers_plane,
            max_wall_planes=args.max_wall_planes,
            random_state=args.random_state
        )

        timings["segmentation"] = time.perf_counter() - t0
        print(f"⏱️ Floor/wall segmentation time: {format_time(timings['segmentation'])}")

        floor_points_count = int(seg["floor_points"].shape[0])
        wall_planes_count = int(len(seg["walls"]))
        wall_points_count = 0

        # ----------------------------
        # Stage 6: export floor
        # ----------------------------
        print("\n----------------------------------------")
        print("Stage 6: export floor")
        print("----------------------------------------")

        if seg["floor_points"].size > 0:
            print(f"✅ Detected floor points: {floor_points_count}")

            if not args.no_dae:
                t0 = time.perf_counter()

                export_points_to_collada(
                    seg["floor_points"],
                    cube_size,
                    str(output_floor_dae),
                    scal,
                    floor_color
                )

                timings["floor"] = time.perf_counter() - t0
                print(f"⏱️ Floor DAE export time: {format_time(timings['floor'])}")
            else:
                print("⏭️ Floor DAE export skipped due to --no-dae")
        else:
            print("⚠️ Not enough floor points were detected.")

        # ----------------------------
        # Stage 7: export walls
        # ----------------------------
        print("\n----------------------------------------")
        print("Stage 7: export walls")
        print("----------------------------------------")

        if len(seg["walls"]) > 0:
            all_walls = np.vstack([
                w["points"] for w in seg["walls"] if w["points"].size > 0
            ])

            wall_points_count = int(all_walls.shape[0])

            print(f"✅ Detected wall planes: {wall_planes_count}")
            print(f"✅ Detected wall points: {wall_points_count}")

            if not args.no_dae:
                t0 = time.perf_counter()

                export_points_to_collada(
                    all_walls,
                    cube_size,
                    str(output_walls_dae),
                    scal,
                    wall_color
                )

                timings["walls"] = time.perf_counter() - t0
                print(f"⏱️ Wall DAE export time: {format_time(timings['walls'])}")
            else:
                print("⏭️ Wall DAE export skipped due to --no-dae")
        else:
            print("⚠️ Not enough walls were detected.")

        # ----------------------------
        # Stage 8: floor Convex Hull
        # ----------------------------
        print("\n----------------------------------------")
        print("Stage 8: floor Convex Hull")
        print("----------------------------------------")

        hull_exported = False

        if seg["floor_points"].shape[0] >= 3 and seg["floor_normal"] is not None:
            print("Computing floor Convex Hull...")

            t0 = time.perf_counter()

            hull2d, _, basis = convex_hull_on_plane(
                seg["floor_points"],
                seg["floor_normal"]
            )

            if hull2d is not None:
                hull3d = hull2d_vertices_3d(hull2d, basis)

                if not args.no_dae:
                    hull_exported = export_floor_hull_wire(
                        hull3d,
                        str(output_floor_hull_dae),
                        wire_radius=0.01,
                        color=hull_color
                    )

                    if hull_exported:
                        print("✅ Floor contour exported.")
                    else:
                        print("⚠️ Unable to export the floor contour.")
                else:
                    print("⏭️ Convex Hull DAE export skipped due to --no-dae")
            else:
                print("⚠️ Unable to compute the floor Convex Hull.")

            timings["hull"] = time.perf_counter() - t0
            print(f"⏱️ Floor Convex Hull time: {format_time(timings['hull'])}")
        else:
            print("⚠️ Unable to compute the floor Convex Hull.")

        # ============================================================
        # FINAL SUMMARY
        # ============================================================

        timings["total"] = time.perf_counter() - total_start_time

        summary = {
            "input_ply": str(input_ply_path),
            "output_dir": str(base_dir),
            "prefix": prefix,
            "parameters": {
                "target_points": target_points,
                "tolerance": tolerance,
                "voxel_start": voxel_start,
                "voxel_max": voxel_max,
                "voxel_step": voxel_step,
                "cube_size": cube_size,
                "scale": scal,
                "ransac_distance": args.ransac_distance,
                "ransac_iterations": args.ransac_iterations,
                "floor_angle_tol": args.floor_angle_tol,
                "wall_angle_min": args.wall_angle_min,
                "wall_angle_max": args.wall_angle_max,
                "min_inliers_plane": args.min_inliers_plane,
                "max_wall_planes": args.max_wall_planes,
                "random_state": args.random_state,
                "no_dae": args.no_dae
            },
            "metrics_reduction": {
                "original_points": metrics_reduction["original_points"],
                "final_points": metrics_reduction["final_points"],
                "reduction_percentage": metrics_reduction["reduction_percentage"],
                "optimal_voxel": metrics_reduction["optimal_voxel"],
                "best_error": metrics_reduction["best_error"]
            },
            "segmentation_metrics": {
                "floor_points": floor_points_count,
                "wall_planes": wall_planes_count,
                "wall_points": wall_points_count,
                "floor_convex_hull_exported": hull_exported
            },
            "timings_seconds": timings,
            "timings_formatted": {
                key: format_time(value) for key, value in timings.items()
            },
            "generated_files": generated_files
        }

        # Save JSON summary
        with open(output_summary_json, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=4, ensure_ascii=False)

        # Save CSV metrics
        metrics_csv = {
            "input_ply": str(input_ply_path),
            "output_dir": str(base_dir),
            "prefix": prefix,
            "original_points": metrics_reduction["original_points"],
            "final_points": metrics_reduction["final_points"],
            "reduction_percentage": metrics_reduction["reduction_percentage"],
            "optimal_voxel": metrics_reduction["optimal_voxel"],
            "target_points": target_points,
            "tolerance": tolerance,
            "floor_points": floor_points_count,
            "wall_planes": wall_planes_count,
            "wall_points": wall_points_count,
            "reduction_time_s": timings["reduction"],
            "scene_dae_time_s": timings["dae_scene"],
            "csv_time_s": timings["csv"],
            "robot_time_s": timings["robot"],
            "segmentation_time_s": timings["segmentation"],
            "floor_time_s": timings["floor"],
            "walls_time_s": timings["walls"],
            "hull_time_s": timings["hull"],
            "total_time_s": timings["total"]
        }

        pd.DataFrame([metrics_csv]).to_csv(output_metrics_csv, index=False)

        print("\n========================================")
        print("✅ PROCESS COMPLETED")
        print("========================================")

        print("\n⏱️ TIME SUMMARY:")
        print(f"- Point cloud reduction:        {format_time(timings['reduction'])}")
        print(f"- Full scene DAE:      {format_time(timings['dae_scene'])}")
        print(f"- CSV saving:             {format_time(timings['csv'])}")
        print(f"- Robot DAE:                {format_time(timings['robot'])}")
        print(f"- Segmentation:             {format_time(timings['segmentation'])}")
        print(f"- Floor DAE:                 {format_time(timings['floor'])}")
        print(f"- Walls DAE:              {format_time(timings['walls'])}")
        print(f"- Floor Convex Hull:         {format_time(timings['hull'])}")
        print(f"- TOTAL TIME:             {format_time(timings['total'])}")

        print("\n📊 MAIN METRICS:")
        print(f"- Original points:        {metrics_reduction['original_points']}")
        print(f"- Final points:           {metrics_reduction['final_points']}")
        print(f"- Reduction:                {metrics_reduction['reduction_percentage']:.2f}%")
        print(f"- Optimal voxel:             {metrics_reduction['voxel_optimo']:.3f}")
        print(f"- Floor points:           {floor_points_count}")
        print(f"- Wall planes:          {wall_planes_count}")
        print(f"- Wall points:        {wall_points_count}")

        print("\n📁 Generated files:")
        for nombre, ruta in generated_files.items():
            if ruta is not None:
                print(f"- {nombre}: {ruta}")

    except Exception as e:
        print("\n❌ ERROR DURING PROCESSING")
        print(str(e))
        sys.exit(1)


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()