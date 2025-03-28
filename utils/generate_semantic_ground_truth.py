import os
import sys
import json
import argparse
import numpy as np
import open3d as o3d


def rotate_points(points, angle_deg):
    """
    Rotates the points in the X-Y plane around the Z-axis in degrees.
    """
    theta = np.deg2rad(angle_deg)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    rotated_xy = points[:, :2] @ R.T
    # Keep Z without changes
    rotated = np.hstack((rotated_xy, points[:, 2:3]))
    return rotated


def find_optimal_rotation(points, seg_groups, segment_to_point_indices, angle_step=5):
    """
    Finds the angle (in degrees) that minimizes the total area of the bounding boxes
    in the X-Y plane for all objects.
    It explores the range [0, 180) degrees with the increment 'angle_step'.
    """
    best_angle = None
    best_area = None
    angles = np.arange(0, 180, angle_step)
    for angle in angles:
        rotated = rotate_points(points, angle)
        total_area = 0.0
        # For each object, calculate the area of the axis-aligned bounding box in X-Y.
        for group in seg_groups:
            segment_ids = group["segments"]
            point_indices = []
            for seg_id in segment_ids:
                point_indices.extend(segment_to_point_indices.get(seg_id, []))
            if not point_indices:
                continue
            obj_points = rotated[point_indices]
            # Only consider X-Y coordinates
            x = obj_points[:, 0]
            y = obj_points[:, 1]
            area = (x.max() - x.min()) * (y.max() - y.min())
            total_area += area
        if best_area is None or total_area < best_area:
            best_area = total_area
            best_angle = angle
    return best_angle, best_area


def generate_semantic_ground_truth(mesh_path, segs_path, aggregation_path, optimal_angle=None):
    # Load the point cloud from the PLY file
    pcd = o3d.io.read_point_cloud(mesh_path)
    points = np.asarray(pcd.points)

    # If an optimal angle was specified, rotate the point cloud
    if optimal_angle is not None:
        points = rotate_points(points, optimal_angle)
        print(f"Applying global rotation of {optimal_angle} degrees.")

    # Load segmentations
    with open(segs_path, 'r') as f:
        seg_data = json.load(f)
    # List with a segment index per point
    seg_indices = seg_data["segIndices"]

    # Create mapping: segment_id -> list of point indices
    segment_to_point_indices = {}
    for idx, seg_id in enumerate(seg_indices):
        segment_to_point_indices.setdefault(seg_id, []).append(idx)

    # Load aggregation
    with open(aggregation_path, 'r') as f:
        agg_data = json.load(f)
    seg_groups = agg_data["segGroups"]

    # Build the result
    result = {"instances": {}}
    for group in seg_groups:
        obj_id = group["objectId"]
        label = group["label"]
        segment_ids = group["segments"]

        # Gather all point indices associated with the object
        point_indices = []
        for seg_id in segment_ids:
            point_indices.extend(segment_to_point_indices.get(seg_id, []))
        if not point_indices:
            continue

        obj_points = points[point_indices]
        # For the X-Y plane, use the globally applied rotation
        obj_xy = obj_points[:, :2]

        # Axis-aligned bounding box in X-Y
        xmin, xmax = np.min(obj_xy, axis=0)[0], np.max(obj_xy, axis=0)[0]
        ymin, ymax = np.min(obj_xy, axis=0)[1], np.max(obj_xy, axis=0)[1]
        center_xy = [(xmin + xmax) / 2, (ymin + ymax) / 2]
        size_xy = [xmax - xmin, ymax - ymin]

        # For Z, use the axis-aligned bounding box (without rotation)
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


def main(args):
    # Check if all required files exist
    for f in [args.mesh, args.segs, args.aggregation]:
        if not os.path.isfile(f):
            print(f"Error: File not found - {f}")
            sys.exit(1)

    if args.visualize:
        # Visualize the mesh using Open3D
        mesh = o3d.io.read_triangle_mesh(args.mesh)
        o3d.visualization.draw_geometries([mesh])

    # Load the original point cloud for optimal rotation search
    pcd = o3d.io.read_point_cloud(args.mesh)
    orig_points = np.asarray(pcd.points)

    # Load segmentations to build the mapping
    with open(args.segs, 'r') as f:
        seg_data = json.load(f)
    seg_indices = seg_data["segIndices"]
    segment_to_point_indices = {}
    for idx, seg_id in enumerate(seg_indices):
        segment_to_point_indices.setdefault(seg_id, []).append(idx)

    # Load groups from the aggregation
    with open(args.aggregation, 'r') as f:
        agg_data = json.load(f)
    seg_groups = agg_data["segGroups"]

    # Find the optimal rotation (between 0 and 180 degrees) that minimizes the total 2D area
    best_angle, best_area = find_optimal_rotation(
        orig_points, seg_groups, segment_to_point_indices, angle_step=args.angle_step)
    print(
        f"Optimal rotation found: {best_angle}° (total area = {best_area:.4f})")

    # Generate ground truth
    result = generate_semantic_ground_truth(
        args.mesh, args.segs, args.aggregation, optimal_angle=best_angle)
    print(result)

    # Save the result to a JSON file
    with open(args.output_path, "w") as f:
        json.dump(result, f, indent=4)
    print(f"Semantic ground truth saved to {args.output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compute object bounding boxes from mesh and segmentation data with global alignment (minimizing 2D area).")
    parser.add_argument("mesh", type=str, help="Path to the mesh .ply file")
    parser.add_argument(
        "segs", type=str, help="Path to the segmentation .json file")
    parser.add_argument("aggregation", type=str,
                        help="Path to the aggregation .json file")
    parser.add_argument("-o", "--output-path", type=str, default="semantic_ground_truth.json",
                        help="Output file name (default: semantic_ground_truth.json)")
    parser.add_argument("--visualize", action="store_true",
                        help="Visualize the mesh using Open3D")
    parser.add_argument("--angle-step", type=float, default=5,
                        help="Step size in degrees for the optimal rotation search (default: 5)")
    args = parser.parse_args()

    main(args)
