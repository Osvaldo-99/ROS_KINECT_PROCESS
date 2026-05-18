#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
procesar_nube_3d.py

Procesamiento reproducible de nubes de puntos PLY.

Este script permite:
1. Leer una nube de puntos .PLY.
2. Reducir la nube mediante voxelización iterativa con Open3D.
3. Exportar la nube reducida a .PLY.
4. Exportar la nube reducida a .DAE mediante cubos.
5. Exportar los puntos reducidos a .CSV.
6. Generar una esfera representativa del robot en .DAE.
7. Segmentar piso y paredes mediante RANSAC.
8. Exportar piso y paredes a .DAE.
9. Calcular y exportar el contorno del piso mediante Convex Hull.
10. Guardar un resumen experimental en .CSV y .JSON.

Ejemplo de uso:

python3 src/procesar_nube_3d.py \
    --input "/home/osvaldo07/Imágenes/Prueba_26_de_abril.ply" \
    --output-dir "data/output/prueba_26" \
    --prefix "Prueba_26" \
    --objetivo-puntos 100000 \
    --tolerancia 10000 \
    --voxel-inicial 0.015 \
    --voxel-maximo 0.05 \
    --paso 0.001
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
# FUNCIONES DE TIEMPO
# ============================================================

def formato_tiempo(segundos):
    """
    Convierte segundos a formato legible en minutos y segundos.
    """
    minutos = int(segundos // 60)
    seg = segundos % 60
    return f"{minutos} min {seg:.2f} s"


# ============================================================
# FUNCIONES GEOMÉTRICAS
# ============================================================

def plane_from_points(p1, p2, p3):
    """
    Calcula el plano que pasa por tres puntos.

    Ecuación del plano:
        n . x + d = 0

    Regresa:
        n: vector normal unitario
        d: término independiente del plano
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
    Calcula la distancia algebraica de un conjunto de puntos a un plano.
    """
    return points.dot(n) + d


def angle_deg(u, v):
    """
    Calcula el ángulo en grados entre dos vectores.
    """
    u = u / (norm(u) + 1e-12)
    v = v / (norm(v) + 1e-12)

    cosang = np.clip(np.dot(u, v), -1.0, 1.0)
    return np.degrees(np.arccos(cosang))


# ============================================================
# REDUCCIÓN DE NUBE CON OPEN3D
# ============================================================

def reduce_point_cloud_open3d(
    input_ply_path,
    output_ply_path,
    objetivo_puntos=100000,
    tolerancia=10000,
    voxel_inicial=0.015,
    voxel_maximo=0.05,
    paso=0.001
):
    """
    Reduce una nube de puntos usando voxel_down_sample de Open3D.

    El proceso prueba diferentes tamaños de voxel hasta aproximarse
    al número objetivo de puntos.

    Regresa:
        points: arreglo NumPy con los puntos reducidos
        metricas: diccionario con métricas de reducción
    """

    input_ply_path = Path(input_ply_path)
    output_ply_path = Path(output_ply_path)

    if not input_ply_path.exists():
        raise FileNotFoundError(f"Archivo no encontrado: {input_ply_path}")

    if input_ply_path.suffix.lower() != ".ply":
        raise ValueError("El archivo de entrada debe tener extensión .ply")

    print("Leyendo nube original con Open3D...")
    nube = o3d.io.read_point_cloud(str(input_ply_path))

    puntos_originales = len(nube.points)

    if puntos_originales == 0:
        raise ValueError("La nube de puntos está vacía.")

    print(f"🔢 Puntos originales: {puntos_originales}")

    voxel_actual = voxel_inicial
    mejor_nube = None
    mejor_error = float("inf")
    mejor_voxel = voxel_inicial
    mejor_total_puntos = 0

    historial_voxeles = []

    while voxel_actual <= voxel_maximo + 1e-12:
        nube_filtrada = nube.voxel_down_sample(voxel_size=voxel_actual)
        total_puntos = len(nube_filtrada.points)
        error = abs(total_puntos - objetivo_puntos)

        historial_voxeles.append({
            "voxel": round(voxel_actual, 6),
            "puntos": int(total_puntos),
            "error": int(error)
        })

        print(f"🔍 Probar voxel={voxel_actual:.3f} → {total_puntos} puntos")

        if error < mejor_error:
            mejor_error = error
            mejor_nube = nube_filtrada
            mejor_voxel = voxel_actual
            mejor_total_puntos = total_puntos

        if error <= tolerancia:
            print("🎯 Tolerancia alcanzada. Deteniendo iteración.")
            break

        voxel_actual += paso

    if mejor_nube is None:
        raise RuntimeError("No se pudo reducir la nube de puntos.")

    output_ply_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(output_ply_path), mejor_nube)

    puntos_finales = len(mejor_nube.points)
    reduccion = 100 * (1 - puntos_finales / puntos_originales)

    print(f"\n✅ Nube reducida guardada como: {output_ply_path}")
    print(f"📦 Voxel óptimo: {mejor_voxel:.3f}")
    print(f"📉 Puntos finales: {puntos_finales}")
    print(f"📊 Reducción: {reduccion:.2f}%")

    points = np.asarray(mejor_nube.points)

    metricas = {
        "puntos_originales": int(puntos_originales),
        "puntos_finales": int(puntos_finales),
        "objetivo_puntos": int(objetivo_puntos),
        "tolerancia": int(tolerancia),
        "voxel_optimo": float(mejor_voxel),
        "voxel_inicial": float(voxel_inicial),
        "voxel_maximo": float(voxel_maximo),
        "paso": float(paso),
        "reduccion_porcentaje": float(reduccion),
        "mejor_error": int(mejor_error),
        "mejor_total_puntos": int(mejor_total_puntos),
        "historial_voxeles": historial_voxeles
    }

    return points, metricas


# ============================================================
# RANSAC DE PLANO
# ============================================================

def ransac_plane(points, distance_threshold=0.02, max_iterations=2000, random_state=0):
    """
    Ajusta un plano mediante RANSAC.

    Regresa:
        n_refined: normal refinada del plano
        d_refined: término d refinado del plano
        inliers: máscara booleana de puntos pertenecientes al plano
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
# SEGMENTACIÓN DE PISO Y PAREDES
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
    Segmenta piso y paredes a partir de la nube reducida.

    Criterio:
    - Piso: plano cuya normal es aproximadamente paralela al eje Z.
    - Paredes: planos cuya normal forma aproximadamente 90 grados con el eje Z.
    """

    result = {
        "floor_points": np.empty((0, 3)),
        "floor_normal": None,
        "floor_d": None,
        "walls": [],
        "remaining_points": points.copy()
    }

    n, d, inliers = ransac_plane(
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

        n, d, inliers = ransac_plane(
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
# CONVEX HULL DEL PISO
# ============================================================

def convex_hull_on_plane(points_on_plane, plane_normal):
    """
    Calcula el Convex Hull 2D de puntos proyectados sobre el plano del piso.
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
    Convierte los vértices del Convex Hull 2D de regreso a coordenadas 3D.
    """
    u, v, c = basis

    order = hull2d.vertices
    order = np.r_[order, order[0]]

    verts2d = hull2d.points[order]

    verts3d = c + np.outer(verts2d[:, 0], u) + np.outer(verts2d[:, 1], v)

    return verts3d


# ============================================================
# EXPORTACIÓN A DAE Y CSV
# ============================================================

def points_to_collada(points, cube_size, output_collada_path, scal, color):
    """
    Exporta puntos como una colección de cubos en formato DAE.

    Nota:
    Este método puede ser pesado si la nube tiene demasiados puntos.
    Para visualización ligera, conviene usar nubes previamente reducidas.
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


def save_points_to_csv(points, output_csv_path, scal):
    """
    Guarda los puntos en un archivo CSV con columnas x, y, z.
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
    Exporta el contorno del piso como una estructura tipo alambre en DAE.
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
# ARGUMENTOS DE TERMINAL
# ============================================================

def parse_args():
    """
    Define los argumentos necesarios para ejecutar el script
    sin modificar manualmente rutas dentro del código.
    """

    parser = argparse.ArgumentParser(
        description="Procesamiento reproducible de nube PLY: reducción, DAE, CSV, segmentación de piso/paredes y Convex Hull."
    )

    parser.add_argument(
        "--input",
        required=True,
        help="Ruta de entrada de la nube original en formato .ply."
    )

    parser.add_argument(
        "--output-dir",
        required=True,
        help="Carpeta donde se guardarán todos los resultados generados."
    )

    parser.add_argument(
        "--prefix",
        default="nube_procesada",
        help="Prefijo para nombrar los archivos de salida."
    )

    parser.add_argument(
        "--objetivo-puntos",
        type=int,
        default=100000,
        help="Número objetivo de puntos después de la reducción por voxelización."
    )

    parser.add_argument(
        "--tolerancia",
        type=int,
        default=10000,
        help="Tolerancia permitida respecto al número objetivo de puntos."
    )

    parser.add_argument(
        "--voxel-inicial",
        type=float,
        default=0.015,
        help="Tamaño inicial del voxel."
    )

    parser.add_argument(
        "--voxel-maximo",
        type=float,
        default=0.05,
        help="Tamaño máximo del voxel."
    )

    parser.add_argument(
        "--paso",
        type=float,
        default=0.001,
        help="Incremento iterativo del tamaño de voxel."
    )

    parser.add_argument(
        "--cube-size",
        type=float,
        default=0.005,
        help="Tamaño del cubo utilizado para representar cada punto en DAE."
    )

    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Factor de escala aplicado a los puntos exportados."
    )

    parser.add_argument(
        "--ransac-distance",
        type=float,
        default=0.02,
        help="Umbral de distancia para RANSAC de planos."
    )

    parser.add_argument(
        "--ransac-iterations",
        type=int,
        default=2500,
        help="Número máximo de iteraciones de RANSAC."
    )

    parser.add_argument(
        "--floor-angle-tol",
        type=float,
        default=15.0,
        help="Tolerancia angular en grados para detectar piso."
    )

    parser.add_argument(
        "--wall-angle-min",
        type=float,
        default=75.0,
        help="Ángulo mínimo en grados para clasificar paredes."
    )

    parser.add_argument(
        "--wall-angle-max",
        type=float,
        default=105.0,
        help="Ángulo máximo en grados para clasificar paredes."
    )

    parser.add_argument(
        "--min-inliers-plane",
        type=int,
        default=800,
        help="Número mínimo de puntos para aceptar un plano."
    )

    parser.add_argument(
        "--max-wall-planes",
        type=int,
        default=6,
        help="Número máximo de planos de pared a detectar."
    )

    parser.add_argument(
        "--random-state",
        type=int,
        default=42,
        help="Semilla aleatoria para reproducibilidad."
    )

    parser.add_argument(
        "--no-dae",
        action="store_true",
        help="Si se activa, no exporta archivos DAE. Útil para pruebas rápidas."
    )

    return parser.parse_args()


# ============================================================
# FUNCIÓN PRINCIPAL
# ============================================================

def main():
    args = parse_args()

    input_ply_path = Path(args.input)
    base_dir = Path(args.output_dir)
    base_dir.mkdir(parents=True, exist_ok=True)

    prefix = args.prefix

    output_reduced_ply = base_dir / f"{prefix}_reducida.ply"

    output_collada_path = base_dir / f"{prefix}_escena.dae"
    output_csv_path = base_dir / f"{prefix}_puntos.csv"
    output_sphere_collada_path = base_dir / f"{prefix}_robot.dae"

    output_floor_dae = base_dir / f"{prefix}_floor.dae"
    output_walls_dae = base_dir / f"{prefix}_walls.dae"
    output_floor_hull_dae = base_dir / f"{prefix}_floor_hull.dae"

    output_metrics_csv = base_dir / f"{prefix}_metricas.csv"
    output_summary_json = base_dir / f"{prefix}_resumen.json"
    output_voxel_history_csv = base_dir / f"{prefix}_historial_voxeles.csv"

    objetivo_puntos = args.objetivo_puntos
    tolerancia = args.tolerancia
    voxel_inicial = args.voxel_inicial
    voxel_maximo = args.voxel_maximo
    paso = args.paso

    cube_size = args.cube_size
    scal = args.scale

    color_escena = [0.0, 1.0, 0.0, 1.0]
    color_robot = [1.0, 0.0, 0.0, 1.0]
    color_floor = [0.2, 0.9, 0.2, 1.0]
    color_walls = [0.9, 0.2, 0.2, 1.0]
    color_hull = [0.0, 0.8, 1.0, 1.0]

    tiempo_inicio_total = time.perf_counter()

    tiempos = {
        "reduccion": 0.0,
        "dae_escena": 0.0,
        "csv": 0.0,
        "robot": 0.0,
        "segmentacion": 0.0,
        "floor": 0.0,
        "walls": 0.0,
        "hull": 0.0,
        "total": 0.0
    }

    archivos_generados = {
        "nube_reducida_ply": str(output_reduced_ply),
        "escena_completa_dae": None if args.no_dae else str(output_collada_path),
        "csv_puntos": str(output_csv_path),
        "robot_dae": None if args.no_dae else str(output_sphere_collada_path),
        "floor_dae": None if args.no_dae else str(output_floor_dae),
        "walls_dae": None if args.no_dae else str(output_walls_dae),
        "floor_hull_dae": None if args.no_dae else str(output_floor_hull_dae),
        "metricas_csv": str(output_metrics_csv),
        "resumen_json": str(output_summary_json),
        "historial_voxeles_csv": str(output_voxel_history_csv)
    }

    try:
        print("\n========================================")
        print("PROCESAMIENTO REPRODUCIBLE DE NUBE 3D")
        print("========================================")

        print("\nArchivo de entrada:")
        print(input_ply_path)

        print("\nCarpeta de salida:")
        print(base_dir)

        print("\nPrefijo de experimento:")
        print(prefix)

        # ----------------------------
        # Etapa 1: reducción de nube
        # ----------------------------
        print("\n----------------------------------------")
        print("Etapa 1: reducción de nube por voxelización")
        print("----------------------------------------")

        t0 = time.perf_counter()

        points, metricas_reduccion = reduce_point_cloud_open3d(
            input_ply_path=str(input_ply_path),
            output_ply_path=str(output_reduced_ply),
            objetivo_puntos=objetivo_puntos,
            tolerancia=tolerancia,
            voxel_inicial=voxel_inicial,
            voxel_maximo=voxel_maximo,
            paso=paso
        )

        tiempos["reduccion"] = time.perf_counter() - t0
        print(f"\n⏱️ Tiempo reducción de nube: {formato_tiempo(tiempos['reduccion'])}")

        # Guardar historial de voxelización
        pd.DataFrame(metricas_reduccion["historial_voxeles"]).to_csv(
            output_voxel_history_csv,
            index=False
        )

        # ----------------------------
        # Etapa 2: exportar escena DAE
        # ----------------------------
        if not args.no_dae:
            print("\n----------------------------------------")
            print("Etapa 2: exportar escena completa reducida a DAE")
            print("----------------------------------------")

            t0 = time.perf_counter()

            points_to_collada(
                points,
                cube_size,
                str(output_collada_path),
                scal,
                color_escena
            )

            tiempos["dae_escena"] = time.perf_counter() - t0
            print(f"⏱️ Tiempo exportación escena DAE: {formato_tiempo(tiempos['dae_escena'])}")
        else:
            print("\n⏭️ Exportación DAE de escena omitida por --no-dae")

        # ----------------------------
        # Etapa 3: guardar CSV
        # ----------------------------
        print("\n----------------------------------------")
        print("Etapa 3: guardar CSV de nube reducida")
        print("----------------------------------------")

        t0 = time.perf_counter()

        save_points_to_csv(
            points,
            str(output_csv_path),
            scal
        )

        tiempos["csv"] = time.perf_counter() - t0
        print(f"⏱️ Tiempo guardado CSV: {formato_tiempo(tiempos['csv'])}")

        # ----------------------------
        # Etapa 4: exportar robot
        # ----------------------------
        if not args.no_dae:
            print("\n----------------------------------------")
            print("Etapa 4: exportar esfera representativa del robot")
            print("----------------------------------------")

            t0 = time.perf_counter()

            sphere = trimesh.creation.icosphere(radius=cube_size * scal)
            sphere.visual.face_colors = color_robot
            sphere.export(str(output_sphere_collada_path), file_type="dae")

            tiempos["robot"] = time.perf_counter() - t0
            print(f"⏱️ Tiempo exportación robot DAE: {formato_tiempo(tiempos['robot'])}")
        else:
            print("\n⏭️ Exportación DAE del robot omitida por --no-dae")

        # ----------------------------
        # Etapa 5: segmentación
        # ----------------------------
        print("\n----------------------------------------")
        print("Etapa 5: segmentación de piso y paredes")
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

        tiempos["segmentacion"] = time.perf_counter() - t0
        print(f"⏱️ Tiempo segmentación piso/paredes: {formato_tiempo(tiempos['segmentacion'])}")

        floor_points_count = int(seg["floor_points"].shape[0])
        wall_planes_count = int(len(seg["walls"]))
        wall_points_count = 0

        # ----------------------------
        # Etapa 6: exportar piso
        # ----------------------------
        print("\n----------------------------------------")
        print("Etapa 6: exportar piso")
        print("----------------------------------------")

        if seg["floor_points"].size > 0:
            print(f"✅ Puntos de piso detectados: {floor_points_count}")

            if not args.no_dae:
                t0 = time.perf_counter()

                points_to_collada(
                    seg["floor_points"],
                    cube_size,
                    str(output_floor_dae),
                    scal,
                    color_floor
                )

                tiempos["floor"] = time.perf_counter() - t0
                print(f"⏱️ Tiempo exportación piso DAE: {formato_tiempo(tiempos['floor'])}")
            else:
                print("⏭️ Exportación DAE de piso omitida por --no-dae")
        else:
            print("⚠️ No se detectó piso suficiente.")

        # ----------------------------
        # Etapa 7: exportar paredes
        # ----------------------------
        print("\n----------------------------------------")
        print("Etapa 7: exportar paredes")
        print("----------------------------------------")

        if len(seg["walls"]) > 0:
            all_walls = np.vstack([
                w["points"] for w in seg["walls"] if w["points"].size > 0
            ])

            wall_points_count = int(all_walls.shape[0])

            print(f"✅ Planos de pared detectados: {wall_planes_count}")
            print(f"✅ Puntos de paredes detectados: {wall_points_count}")

            if not args.no_dae:
                t0 = time.perf_counter()

                points_to_collada(
                    all_walls,
                    cube_size,
                    str(output_walls_dae),
                    scal,
                    color_walls
                )

                tiempos["walls"] = time.perf_counter() - t0
                print(f"⏱️ Tiempo exportación paredes DAE: {formato_tiempo(tiempos['walls'])}")
            else:
                print("⏭️ Exportación DAE de paredes omitida por --no-dae")
        else:
            print("⚠️ No se detectaron paredes suficientes.")

        # ----------------------------
        # Etapa 8: Convex Hull del piso
        # ----------------------------
        print("\n----------------------------------------")
        print("Etapa 8: Convex Hull del piso")
        print("----------------------------------------")

        hull_exportado = False

        if seg["floor_points"].shape[0] >= 3 and seg["floor_normal"] is not None:
            print("Calculando Convex Hull del piso...")

            t0 = time.perf_counter()

            hull2d, _, basis = convex_hull_on_plane(
                seg["floor_points"],
                seg["floor_normal"]
            )

            if hull2d is not None:
                hull3d = hull2d_vertices_3d(hull2d, basis)

                if not args.no_dae:
                    hull_exportado = export_floor_hull_wire(
                        hull3d,
                        str(output_floor_hull_dae),
                        wire_radius=0.01,
                        color=color_hull
                    )

                    if hull_exportado:
                        print("✅ Contorno del piso exportado.")
                    else:
                        print("⚠️ No se pudo exportar el contorno del piso.")
                else:
                    print("⏭️ Exportación DAE de Convex Hull omitida por --no-dae")
            else:
                print("⚠️ No fue posible calcular el Convex Hull del piso.")

            tiempos["hull"] = time.perf_counter() - t0
            print(f"⏱️ Tiempo Convex Hull piso: {formato_tiempo(tiempos['hull'])}")
        else:
            print("⚠️ No fue posible calcular el Convex Hull del piso.")

        # ============================================================
        # RESUMEN FINAL
        # ============================================================

        tiempos["total"] = time.perf_counter() - tiempo_inicio_total

        resumen = {
            "input_ply": str(input_ply_path),
            "output_dir": str(base_dir),
            "prefix": prefix,
            "parametros": {
                "objetivo_puntos": objetivo_puntos,
                "tolerancia": tolerancia,
                "voxel_inicial": voxel_inicial,
                "voxel_maximo": voxel_maximo,
                "paso": paso,
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
            "metricas_reduccion": {
                "puntos_originales": metricas_reduccion["puntos_originales"],
                "puntos_finales": metricas_reduccion["puntos_finales"],
                "reduccion_porcentaje": metricas_reduccion["reduccion_porcentaje"],
                "voxel_optimo": metricas_reduccion["voxel_optimo"],
                "mejor_error": metricas_reduccion["mejor_error"]
            },
            "metricas_segmentacion": {
                "puntos_piso": floor_points_count,
                "planos_pared": wall_planes_count,
                "puntos_paredes": wall_points_count,
                "convex_hull_piso_exportado": hull_exportado
            },
            "tiempos_segundos": tiempos,
            "tiempos_formato": {
                key: formato_tiempo(value) for key, value in tiempos.items()
            },
            "archivos_generados": archivos_generados
        }

        # Guardar resumen JSON
        with open(output_summary_json, "w", encoding="utf-8") as f:
            json.dump(resumen, f, indent=4, ensure_ascii=False)

        # Guardar métricas CSV
        metricas_csv = {
            "input_ply": str(input_ply_path),
            "output_dir": str(base_dir),
            "prefix": prefix,
            "puntos_originales": metricas_reduccion["puntos_originales"],
            "puntos_finales": metricas_reduccion["puntos_finales"],
            "reduccion_porcentaje": metricas_reduccion["reduccion_porcentaje"],
            "voxel_optimo": metricas_reduccion["voxel_optimo"],
            "objetivo_puntos": objetivo_puntos,
            "tolerancia": tolerancia,
            "puntos_piso": floor_points_count,
            "planos_pared": wall_planes_count,
            "puntos_paredes": wall_points_count,
            "tiempo_reduccion_s": tiempos["reduccion"],
            "tiempo_dae_escena_s": tiempos["dae_escena"],
            "tiempo_csv_s": tiempos["csv"],
            "tiempo_robot_s": tiempos["robot"],
            "tiempo_segmentacion_s": tiempos["segmentacion"],
            "tiempo_floor_s": tiempos["floor"],
            "tiempo_walls_s": tiempos["walls"],
            "tiempo_hull_s": tiempos["hull"],
            "tiempo_total_s": tiempos["total"]
        }

        pd.DataFrame([metricas_csv]).to_csv(output_metrics_csv, index=False)

        print("\n========================================")
        print("✅ PROCESO COMPLETADO")
        print("========================================")

        print("\n⏱️ RESUMEN DE TIEMPOS:")
        print(f"- Reducción de nube:        {formato_tiempo(tiempos['reduccion'])}")
        print(f"- Escena completa DAE:      {formato_tiempo(tiempos['dae_escena'])}")
        print(f"- Guardado CSV:             {formato_tiempo(tiempos['csv'])}")
        print(f"- Robot DAE:                {formato_tiempo(tiempos['robot'])}")
        print(f"- Segmentación:             {formato_tiempo(tiempos['segmentacion'])}")
        print(f"- Piso DAE:                 {formato_tiempo(tiempos['floor'])}")
        print(f"- Paredes DAE:              {formato_tiempo(tiempos['walls'])}")
        print(f"- Convex Hull piso:         {formato_tiempo(tiempos['hull'])}")
        print(f"- TIEMPO TOTAL:             {formato_tiempo(tiempos['total'])}")

        print("\n📊 MÉTRICAS PRINCIPALES:")
        print(f"- Puntos originales:        {metricas_reduccion['puntos_originales']}")
        print(f"- Puntos finales:           {metricas_reduccion['puntos_finales']}")
        print(f"- Reducción:                {metricas_reduccion['reduccion_porcentaje']:.2f}%")
        print(f"- Voxel óptimo:             {metricas_reduccion['voxel_optimo']:.3f}")
        print(f"- Puntos de piso:           {floor_points_count}")
        print(f"- Planos de pared:          {wall_planes_count}")
        print(f"- Puntos de paredes:        {wall_points_count}")

        print("\n📁 Archivos generados:")
        for nombre, ruta in archivos_generados.items():
            if ruta is not None:
                print(f"- {nombre}: {ruta}")

    except Exception as e:
        print("\n❌ ERROR DURANTE EL PROCESAMIENTO")
        print(str(e))
        sys.exit(1)


# ============================================================
# PUNTO DE ENTRADA
# ============================================================

if __name__ == "__main__":
    main()