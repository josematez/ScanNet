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

# Check if the scene directory exists
if [ -d "$THIS_FILE_DIR/raw_data/$scene" ]; then
    echo "Directory $scene_dir exists. Proceeding with the ROSbag generation..."
   
else
    echo "The raw data of $scene does not exist. Downloading it..."
    sh download.sh $1 $2
fi

python3 $THIS_FILE_DIR/utils/raw_to_rosbag.py --datapath "$THIS_FILE_DIR/raw_data/$scene"
echo "Converting ROS1 bag to ROS2 bag..."
rosbags-convert --src "$THIS_FILE_DIR/to_ros/ROS1_bags/$scene.bag" --dst "$THIS_FILE_DIR/to_ros/ROS2_bags/$scene"
echo "ROS2 bag saved in: $THIS_FILE_DIR/to_ros/ROS2_bags/$scene"
