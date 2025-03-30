##!/bin/sh
# Exit immediately if a command exits with a non-zero status.
set -e

# Check if the correct number of arguments is provided
if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    echo "Usage: $0 <scene_id> <sequence_id> [--evaluation-only]: (Hint: From ScanNet <scene_id>_<sequence_id>)"
    exit 1
fi

scene_id=$1
sequence_id=$2

# Check if the evaluation-only flag is set
EVALUATION_ONLY=false
if [ "$3" = "--evaluation-only" ]; then
    EVALUATION_ONLY=true
fi

scene_id=$(printf "%04d" "$scene_id")
sequence_id=$(printf "%02d" "$sequence_id")
scene="scene"$scene_id"_"$sequence_id

THIS_FILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAW_DATA_DIR="$THIS_FILE_DIR/raw_data/"
EVALUATION_DIR="$THIS_FILE_DIR/evaluation/"

SCANNET_URL="https://kaldir.vc.in.tum.de/scannet/v1/scans/"

echo "Processing scene: $scene"

if [ "$EVALUATION_ONLY" = false ]; then
    # RAW_DATA PART

    cd "$RAW_DATA_DIR" || exit

    mkdir -p "$scene"

    cd "$RAW_DATA_DIR/$scene" || exit

    echo "Downloading .sens file..."
    if [ ! -f "$scene.sens" ]; then
        wget "$SCANNET_URL$scene/$scene.sens"
    else
        echo "$scene.sens already exists. Skipping download."
    fi

    echo "Extracting data (RGB-D images, poses and intrinsics)..."
    python3 $THIS_FILE_DIR/utils/"reader.py" --filename $THIS_FILE_DIR/raw_data/$scene/$scene.sens --output_path $THIS_FILE_DIR/raw_data/$scene --export_depth_images --export_color_images --export_poses --export_intrinsics
fi

# EVALUATION PART

cd "$EVALUATION_DIR" || exit

mkdir -p "$scene"

cd "$EVALUATION_DIR/$scene" || exit

echo "Downloading .ply mesh file..."
if [ ! -f "${scene}_vh_clean_2.ply" ]; then
    wget "$SCANNET_URL${scene}/${scene}_vh_clean_2.ply"
else
    echo "${scene}_vh_clean_2.ply already exists. Skipping download."
fi

echo "Downloading aggregation.json file..."
if [ ! -f "${scene}.aggregation.json" ]; then
    wget "$SCANNET_URL${scene}/${scene}.aggregation.json"
else
    echo "${scene}.aggregation.json already exists. Skipping download."
fi

echo "Downloading segs.json file..."
if [ ! -f "${scene}_vh_clean_2.0.010000.segs.json" ]; then
    wget -O "${scene}_vh_clean_2.0.010000.segs.json" "$SCANNET_URL${scene}/${scene}_vh_clean_2.0.010000.segs.json"
else
    echo "${scene}_vh_clean_2.0.010000.segs.json already exists. Skipping download."
fi

echo "Generating semantic ground truth..."
python3 $THIS_FILE_DIR/utils/generate_semantic_ground_truth.py \
    $EVALUATION_DIR/$scene/${scene}_vh_clean_2.ply \
    $EVALUATION_DIR/$scene/${scene}_vh_clean_2.0.010000.segs.json \
    $EVALUATION_DIR/$scene/${scene}.aggregation.json \
    -o $EVALUATION_DIR/$scene/semantic_ground_truth.json \
    --angle-step 1

echo "Showing semantic ground truth..."
python3 $THIS_FILE_DIR/utils/show_semantic_ground_truth.py \
    -i $EVALUATION_DIR/$scene/semantic_ground_truth.json \
    -o $EVALUATION_DIR/$scene/semantic_ground_truth.png
