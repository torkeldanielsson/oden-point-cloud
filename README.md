# Oden Point Cloud

An [Oden](https://docs.voysys.dev) plugin that streams lidar point clouds from the Streamer to the Player and draws them in the 3D scene, lined up with the camera video. It takes `sensor_msgs/PointCloud2` from ROS 2 through a small bridge, Draco-compresses the points and sends them beside the video. The plugin is one library with two entities:

- **Point Cloud Streamer** takes point clouds from one or more lidars on ROS 2 topics (through the bridge below), keeps the points within a range and height window, voxel-downsamples them and Draco-compresses them in chunks per azimuth sector. A chunk fits in one packet and is smaller on slower links, so that each chunk holds up the video only briefly. It shares the link with the video: its budget is its proportional share, Max Bitrate × the video's sent/configured bitrate, and it shares that budget between the lidars in proportion to their need. Each lidar's frame goes out over com channels at once, so that the Player draws it whole. When its share shrinks it first lowers the frame rate, down to the minimum frame rate (1 fps while at Min Bitrate), then makes the voxels coarser, up to 1 m, and then sends those coarse frames less often. It sends at least Min Bitrate (0.05 Mbit/s by default), so that the operator keeps a coarse cloud when the link carries almost nothing. With Min Bitrate 0 it stops sending when the video sends nothing, and the cloud disappears.
- **Point Cloud Player** decodes the chunks and draws the newest points of every lidar as small squares, coloured by height from 2 m below to 3 m above the cameras. Each chunk replaces the older chunks of its lidar that overlap its sectors, and a new frame drops what is left of the frames before the previous one, so a lost packet only loses its own sector for one frame and old points do not linger. When a lidar stops, its points disappear after two of its frame intervals, at least 1 s and at most 5 s.

## How much it sends

The plugin shares the link with the video. Its budget is its proportional share: Max Bitrate (1.5 Mbit/s by default) × the video's sent/configured bitrate, at most Max Bitrate and at least Min Bitrate (0.05 Mbit/s by default). The video's sent bitrate follows the Streamer's bandwidth regulator, so the point cloud ducks with the video when the link gets tight: a video that sends 1.6 of its configured 2 Mbit/s gives 80% of Max Bitrate, 1.2 Mbit/s. The share is 0 until the encoder streams, so only Min Bitrate goes out then. A scene so simple that the encoder sends below its bitrate lowers the share the same way. The point cloud's own bytes come on top of the video's, so on a constrained link it adds some queueing in front of the video: lower Max Bitrate when video latency matters more than the cloud.

## Oden versions

Works with the Oden Streamer and Player 2.49 and newer, which report the video's sent bitrate to plugins. The plugin is built against the plugin API of 2.51.3 ([`oden_plugin_rs`](https://github.com/voysys/oden_plugin_rs) v51.3). Tested with the 2.50.2 and 2.51.3 Streamer and Player release packages on Linux x86_64 (the Streamer in an Ubuntu 22.04 container), the 2.51.3 Streamer on a Jetson AGX Orin with JetPack 6.2 (the plugin builds natively and loads there), and the bridge on ROS 2 Humble.

## Building

Build the plugin on each computer that runs Oden: the robot's Streamer computer and the operator's computer. On Ubuntu 22.04, x86_64 or a Jetson with JetPack 6, it needs Rust 1.88 or newer (Ubuntu's own `rustc` is older, so use rustup) and a C toolchain:

```
sudo apt-get update
sudo apt-get install -y build-essential curl git
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
. "$HOME/.cargo/env"
git clone https://github.com/torkeldanielsson/oden-point-cloud
cd oden-point-cloud
cargo build --release
```

The library is `target/release/libpoint_cloud.so` on Linux, `point_cloud.dll` on Windows and `libpoint_cloud.dylib` on macOS.

## Trying it

`streamer.vproj` and `player.vproj` are a minimal pair with a test pattern as the camera. The Streamer project encodes with NVENC. They load the library from `target/release`, so start them from this folder on one computer:

```
cd oden-point-cloud
oden-streamer --project "$PWD/streamer.vproj" &
oden-vr --project "$PWD/player.vproj" &
```

Then feed it a lidar, live or from a bag, in a second terminal in the same folder. Replace `my_drive.mcap` and `/lidar_points` with your bag and your lidar topic. Humble needs the mcap storage plugin to play `.mcap` bags:

```
source /opt/ros/humble/setup.bash
sudo apt-get install -y ros-$ROS_DISTRO-rosbag2-storage-mcap
ros2 bag play my_drive.mcap --loop &
python3 bridge/point_cloud_bridge.py --ros-args -p 'topics:=[/lidar_points]'
```

## Setting up your projects

Load the library as a project plugin in both projects (`Plugins` > `Plugin Paths`, or put it next to the project file).

**Streamer:** add one `Point Cloud Streamer` as a child of the `Stitched Video` entity that holds the cameras. Set its position and rotation to the pose of the clouds' frame in the camera rig, in Oden axes: the lidar's frame, or the bridge's `target_frame` when there are several lidars. The plugin converts ROS axes (x forward, y left, z up) to Oden's and applies the rig's global calibration.

**Player:** add `Point Cloud Player` as a child of the `Remote Stitched Video` (or the stitched video entity that shows the cameras). Leave its transform at the default of zero, since the points already arrive in the stitched video's frame. With the view at the cameras, the points sit on the right pixels of the video. Away from the cameras, points off the video's projection surface show parallax.

## ROS 2 bridge

`bridge/point_cloud_bridge.py` subscribes to `sensor_msgs/PointCloud2` topics and forwards x, y and z to the Streamer, which listens on TCP 127.0.0.1:29571. It drops points without a return (non-finite or all zero). Run it on the Streamer computer in a sourced ROS 2 (Humble or newer) environment with `sensor_msgs_py` and `tf2_ros`. One lidar, in its own frame:

```
python3 bridge/point_cloud_bridge.py --ros-args -p 'topics:=[/lidar_points]'
```

Several lidars, merged into one cloud in one frame:

```
python3 bridge/point_cloud_bridge.py --ros-args \
    -p 'topics:=[/top/points,/front/points,/rear/points]' -p target_frame:=base_link
```

With `target_frame` and several topics, the bridge keeps the newest sweep of every topic and sends one merged cloud, as a single source, whenever the first topic delivers a sweep. List the lidar with the most frequent sweeps first. The lidars then update together and the Player draws them as one frame.

The sweeps are not captured at the same moment, so on a moving robot they would be shifted against each other. The bridge moves each sweep to where `target_frame` is at the newest sweep's stamp: it looks the sweep's frame up at the sweep's stamp and `target_frame` at the merged stamp, with tf's time travel through `fixed_frame`. That frame must be fixed in the world and have a transform to `target_frame`, such as `odom`. If its newest transform is older than the merged stamp, the sweeps are moved to that transform's time instead. Without a transform through `fixed_frame`, it uses the transform to `target_frame` at the sweep's stamp, or the latest one.

The bridge never queues old data: a merged sweep stays only until a newer one replaces it, and a sweep more than `max_sweep_age_s` older than the newest is left out.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `topics` | `[/lidar_points]` | `PointCloud2` topics. The first one sets the rate of the merged cloud |
| `target_frame` | none | Frame of the cloud the bridge sends. Without it each topic is forwarded as its own source in its own frame |
| `fixed_frame` | `odom` | World-fixed frame that the motion between sweeps is measured in |
| `max_sweep_age_s` | 0.5 | Sweeps older than this, relative to the newest sweep, are not merged |

A merged cloud is about three times the points of one lidar, so the Streamer voxel-downsamples one larger source instead of three, with the same bitrate budget.

Then set the `Point Cloud Streamer` pose to `base_link` (the `target_frame`) in the camera rig (with one lidar, to the lidar's frame). `ros2 run tf2_ros tf2_echo <camera frame> base_link` prints it in ROS axes, for a camera's body frame (not its optical frame). From its output, set these in the entity in the Streamer project file:

- `position : [ -y, z, -x ];` from `Translation: [x, y, z]`
- `pre_rotation : [ w, -y, z, -x ];` from `Rotation: in Quaternion (xyzw) [x, y, z, w]`, a quaternion in Oden's order w, x, y, z
- `pre_rotation_order : "z_y_x";`

Keep `pre_rotation_order : "z_y_x"`: the Streamer's GUI shows a quaternion rotation as Euler angles in that order, and with the default order the rotation changes as soon as the entity is selected. Oden's Euler angles cannot hold ROS roll, pitch and yaw as they are, so use the quaternion.

The bridge keeps only the newest cloud per topic and reconnects when the Streamer restarts.

## Streamer settings

| Setting | Default | Meaning |
| --- | --- | --- |
| Max Range M | 50 | Points further away than this from the origin of the clouds' frame are dropped |
| Min Height M | −1000 | Points lower than this are dropped. Height is z in the frame of the clouds the bridge sends: `target_frame` if set, otherwise each lidar's own frame |
| Max Height M | 1000 | Points higher than this are dropped |
| Min Voxel Size M | 0.05 | Finest voxel size the plugin uses |
| Min Fps | 10 | Frame rate kept before the voxels get coarser, so that each lidar sweep goes out while the budget allows; 1 or less while at Min Bitrate |
| Min Bitrate Mbps | 0.05 | Least the point cloud sends, also when the video sends little; 0 sends nothing then |
| Max Bitrate Mbps | 1.5 | Budget while the video sends its configured bitrate |

The Streamer entity shows the budget, smoothed with a 0.25 s time constant when it decreases and 3 s when it increases, the video's share and whether it is at Min Bitrate. It also shows, per lidar, the sent bitrate and frame rate and the size of the last frame.

At the default Min Bitrate, each of three 128-beam lidars gets about 13–21 kbit/s and sends about one frame per second of 800–1500 points at 0.3–0.6 m voxels. A frame is at most 64 chunks of 1.3 kB; a lidar whose frame would need more gets coarser voxels.

## Player settings

| Setting | Default | Meaning |
| --- | --- | --- |
| Point Size Px | 3 | Width of each point's square on screen, in pixels at the window's scale |

## License

MIT, see `LICENSE`. The plugin links [`draco-core`](https://crates.io/crates/draco-core) (Apache-2.0) and [`oden_plugin_rs`](https://github.com/voysys/oden_plugin_rs) (0BSD).
