import os
import sys
import json
import argparse
import numpy as np
import open3d as o3d


def rotate_points(points, angle_deg):
    """
    Rota los puntos en el plano X-Y alrededor del eje Z en grados.
    """
    theta = np.deg2rad(angle_deg)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    rotated_xy = points[:, :2] @ R.T
    # Conservar Z sin cambios
    rotated = np.hstack((rotated_xy, points[:, 2:3]))
    return rotated


def find_optimal_rotation(points, seg_groups, segment_to_point_indices, angle_step=5):
    """
    Busca el ángulo (en grados) que minimiza el área total de los bounding boxes
    en el plano X-Y para todos los objetos.
    Se explora en [0, 180) grados con el incremento 'angle_step'.
    """
    best_angle = None
    best_area = None
    angles = np.arange(0, 180, angle_step)
    for angle in angles:
        rotated = rotate_points(points, angle)
        total_area = 0.0
        # Para cada objeto se calcula el área del bounding box axis-aligned en X-Y.
        for group in seg_groups:
            segment_ids = group["segments"]
            point_indices = []
            for seg_id in segment_ids:
                point_indices.extend(segment_to_point_indices.get(seg_id, []))
            if not point_indices:
                continue
            obj_points = rotated[point_indices]
            # Solo considerar coordenadas X-Y
            x = obj_points[:, 0]
            y = obj_points[:, 1]
            area = (x.max() - x.min()) * (y.max() - y.min())
            total_area += area
        if best_area is None or total_area < best_area:
            best_area = total_area
            best_angle = angle
    return best_angle, best_area


def generate_semantic_ground_truth(mesh_path, segs_path, aggregation_path, optimal_angle=None):
    # Cargar la nube de puntos desde el archivo PLY
    pcd = o3d.io.read_point_cloud(mesh_path)
    points = np.asarray(pcd.points)

    # Si se especificó un ángulo óptimo, rotar la nube de puntos
    if optimal_angle is not None:
        points = rotate_points(points, optimal_angle)
        print(f"Aplicando rotación global de {optimal_angle} grados.")

    # Cargar segmentaciones
    with open(segs_path, 'r') as f:
        seg_data = json.load(f)
    # Lista con un índice de segmento por punto
    seg_indices = seg_data["segIndices"]

    # Crear mapeo: segment_id -> lista de índices de puntos
    segment_to_point_indices = {}
    for idx, seg_id in enumerate(seg_indices):
        segment_to_point_indices.setdefault(seg_id, []).append(idx)

    # Cargar agregación
    with open(aggregation_path, 'r') as f:
        agg_data = json.load(f)
    seg_groups = agg_data["segGroups"]

    # Construir el resultado
    result = {"instances": {}}
    for group in seg_groups:
        obj_id = group["objectId"]
        label = group["label"]
        segment_ids = group["segments"]

        # Reunir todos los índices de puntos asociados al objeto
        point_indices = []
        for seg_id in segment_ids:
            point_indices.extend(segment_to_point_indices.get(seg_id, []))
        if not point_indices:
            continue

        obj_points = points[point_indices]
        # Para el plano X-Y usamos la rotación global ya aplicada
        obj_xy = obj_points[:, :2]

        # Bounding box axis-aligned en X-Y
        xmin, xmax = np.min(obj_xy, axis=0)[0], np.max(obj_xy, axis=0)[0]
        ymin, ymax = np.min(obj_xy, axis=0)[1], np.max(obj_xy, axis=0)[1]
        center_xy = [(xmin + xmax) / 2, (ymin + ymax) / 2]
        size_xy = [xmax - xmin, ymax - ymin]

        # Para Z usamos bounding box axis-aligned (sin rotación)
        z_min = np.min(obj_points[:, 2])
        z_max = np.max(obj_points[:, 2])
        center_z = (z_min + z_max) / 2
        size_z = z_max - z_min

        center = center_xy + [center_z]
        size = size_xy + [size_z]

        result["instances"][f"obj{obj_id}"] = {
            "bbox": {
                "center": center,
                "size": size
            },
            "results": {
                label: 1.0
            }
        }
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compute object bounding boxes from mesh and segmentation data with global alignment (minimizing 2D area).")
    parser.add_argument("mesh", type=str, help="Path to the mesh .ply file")
    parser.add_argument(
        "segs", type=str, help="Path to the segmentation .json file")
    parser.add_argument("aggregation", type=str,
                        help="Path to the aggregation .json file")
    parser.add_argument("-o", "--output", type=str, default="semantic_ground_truth.json",
                        help="Output file name (default: semantic_ground_truth.json)")
    parser.add_argument("--visualize", action="store_true",
                        help="Visualize the mesh using Open3D")
    parser.add_argument("--angle-step", type=float, default=5,
                        help="Incremento en grados para la búsqueda de la rotación óptima (default: 5)")
    args = parser.parse_args()

    for f in [args.mesh, args.segs, args.aggregation]:
        if not os.path.isfile(f):
            print(f"Error: File not found - {f}")
            sys.exit(1)

    if args.visualize:
        mesh = o3d.io.read_triangle_mesh(args.mesh)
        o3d.visualization.draw_geometries([mesh])

    # Cargar la nube original para la búsqueda de la rotación óptima
    pcd = o3d.io.read_point_cloud(args.mesh)
    orig_points = np.asarray(pcd.points)

    # Cargar segmentaciones para construir el mapeo
    with open(args.segs, 'r') as f:
        seg_data = json.load(f)
    seg_indices = seg_data["segIndices"]
    segment_to_point_indices = {}
    for idx, seg_id in enumerate(seg_indices):
        segment_to_point_indices.setdefault(seg_id, []).append(idx)

    # Cargar grupos de la agregación
    with open(args.aggregation, 'r') as f:
        agg_data = json.load(f)
    seg_groups = agg_data["segGroups"]

    # Buscar la rotación óptima (entre 0 y 180 grados) que minimiza el área total 2D
    best_angle, best_area = find_optimal_rotation(
        orig_points, seg_groups, segment_to_point_indices, angle_step=args.angle_step)
    print(
        f"Rotación óptima encontrada: {best_angle}° (área total = {best_area:.4f})")

    # Generar el ground truth usando la rotación óptima global
    result = generate_semantic_ground_truth(
        args.mesh, args.segs, args.aggregation, optimal_angle=best_angle)
    print(result)

    # Guardar el resultado en JSON
    with open(args.output, "w") as f:
        json.dump(result, f, indent=4)
    print(f"Saved output to {args.output}")
