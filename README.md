This repository aims to facilitate the automatic creation of bags for both, ROS1 and ROS2, of sequences of the [ScanNet](http://www.scan-net.org/) dataset. Preview of the available sequences [here](https://kaldir.vc.in.tum.de/scannet_browse/scans/scannet/grouped).

# Requeriments

To setup your environment, simply run:

```
pip install -r requirements.txt
```

# Installation

Simply, clone the repository:

```
git clone git@github.com:josematez/ScanNet.git
```

# Usage

To only download the raw data of a given sequence:

```
sh download.sh <scene_id> <sequence_id>
```
Note that, for example, if you want the sequence scene0000_01, the IDs are extracted as: scene<scene_id>_<sequence_id>. So, in that case, you should do ```sh download.sh 0 1```

To automatically download (if not already downloaded) and create both ROS1 and ROS2 bags:

```
sh create_rosbag.sh <scene_id> <sequence_id>
```

It will create in the directories to_ros/ROS1_bags and to_ros/ROS2_bags the respective bags files.
