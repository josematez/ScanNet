import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse


if __name__ == "__main__":
    # Configure arguments
    parser = argparse.ArgumentParser(
        description="Visualize semantic ground truth map.")
    parser.add_argument("-i", "--input-path", type=str, required=True,
                        help="Path to the input JSON file.")
    parser.add_argument("-o", "--output-path", type=str, required=True,
                        help="Path to save the output visualization.")

    args = parser.parse_args()

    # Load the generated JSON
    with open(args.input_path, "r") as f:
        data = json.load(f)

    instances = data["instances"]

    plt.figure(figsize=(8, 8))
    ax = plt.gca()

    # Use a colormap to assign a distinct color to each object
    num_objs = len(instances)
    colors = plt.get_cmap("tab20", num_objs)

    # Initialize limits to adjust the visualization
    x_min, x_max = float('inf'), float('-inf')
    y_min, y_max = float('inf'), float('-inf')

    # Draw each object with its filled bounding box
    for idx, (obj_id, obj_data) in enumerate(instances.items()):
        center = obj_data["bbox"]["center"]
        size = obj_data["bbox"]["size"]
        # Extract X-Y coordinates
        x_center, y_center = center[0], center[1]
        w, h = size[0], size[1]
        half_w, half_h = w / 2, h / 2

        # Update visualization limits
        x_min = min(x_min, x_center - half_w)
        x_max = max(x_max, x_center + half_w)
        y_min = min(y_min, y_center - half_h)
        y_max = max(y_max, y_center + half_h)

        color = colors(idx)
        # Draw filled rectangle with transparency
        rect = patches.Rectangle((x_center - half_w, y_center - half_h), w, h,
                                 edgecolor=color, facecolor=color, alpha=0.5)
        ax.add_patch(rect)

        # Display label (object and class) at the center of the bounding box
        label = list(obj_data["results"].keys())[0]
        ax.text(x_center, y_center, f"{obj_id}-{label}",
                fontsize=8, ha="center", va="center", color="black")

    # Adjust margins and limits
    margin = 1
    plt.xlim(x_min - margin, x_max + margin)
    plt.ylim(y_min - margin, y_max + margin)
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.title(f"Visualization of the map for {args.input_path}")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(args.output_path)
    # plt.show()

    print(f"Visualization saved to \n{args.output_path}")
