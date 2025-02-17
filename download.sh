##!/bin/sh
# Exit immediately if a command exits with a non-zero status.
set -e

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <scene_id> and $1 <sequence_id>: (Hint: From ScanNet <scene_id>_<sequence_id>)"
    exit 1
fi

scene_id=$1;
sequence_id=$2;

scene_id=$(printf "%04d" "$scene_id")
sequence_id=$(printf "%02d" "$sequence_id")
THIS_FILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


scene="scene"$scene_id"_"$sequence_id

echo $scene

ROOT_DIR="/home/josematez/Datasets/ScanNet/"
SCANNET_URL="https://kaldir.vc.in.tum.de/scannet/v1/scans/"

cd $THIS_FILE_DIR/raw_data

mkdir $scene

cd $THIS_FILE_DIR/raw_data/$scene

echo "Downloading .sens file..."
wget "$SCANNET_URL$scene/$scene.sens"

echo "Extracting data (RGB-D images, poses and intrinsics)..."
python3 $THIS_FILE_DIR/utils/"reader.py" --filename $THIS_FILE_DIR/raw_data/$scene/$scene.sens --output_path $THIS_FILE_DIR/raw_data/$scene --export_depth_images --export_color_images --export_poses --export_intrinsics
