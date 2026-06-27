# HesaiLidar_SDK_2.0

[👉 Chinese version](README_CN.md)

## 1 Check Compatibility

### 1.1 Lidar Models

| Pandar       | OT           | QT           | XT           | AT           | FT           | JT           |
|:------------|:------------|:------------|:------------|:------------|:------------|:------------|
| Pandar40P    | OT128        | PandarQT     | PandarXT     | AT128E2X     | FT120        | JT128        |
| Pandar40M    | OT128_40     | QT128C2X     | PandarXT-16  | AT128P       | FTX          | JT64P        |
| Pandar64     | -            | -            | XT32M2X      | ATX          | -            | JT16         |
| Pandar128E3X | -            | -            | -            | -            | -            | -            |
| Pandar90E3X  | -            | -            | -            | -            | -            | -            |

## AMZ Custom Modifications

These changes extend the upstream SDK to support depth and intensity image generation inside the parsers. The ROS2 driver then publishes these as `sensor_msgs/Image` topics.

### `CMakeLists.txt` / `libhesai/CMakeLists.txt` — OpenCV dependency

OpenCV (`opencv2/core.hpp`) added as a required dependency to support `cv::Mat` image buffers inside `LidarDecodedFrame`. The ROS2 driver already had OpenCV; this extends that requirement into the SDK itself.

### `libhesai/lidar_types.h` — image buffers in `LidarDecodedFrame`

Two `cv::Mat` members added to `LidarDecodedFrame`:

```cpp
cv::Mat depth_img;      // CV_32FC1 — distance in metres per pixel
cv::Mat intensity_img;  // CV_8UC1  — reflectivity per pixel
```

These are allocated in `DecodePacket` (once per frame) and filled in `ComputeXYZI` (once per point). The ROS2 driver reads them via `frame.depth_img` / `frame.intensity_img`.

### `driver_param.h` — new `InputParam` fields

Four fields added to `InputParam` to support image publishing from the ROS2 layer:

| Field | Type | Description |
|:------|:-----|:------------|
| `send_depth_image_ros` | `bool` | Enable depth image publishing |
| `send_intensity_image_ros` | `bool` | Enable intensity image publishing |
| `ros_send_depth_image_topic` | `std::string` | ROS topic for depth image (default `hesai_depth_image`) |
| `ros_send_intensity_image_topic` | `std::string` | ROS topic for intensity image (default `hesai_intensity_image`) |

### `general_parser.h` — FOV-aware azimuth column count

When `remake_config.max_azi_scan` is left unset (−1) and a partial azimuth FOV is configured (`fov_start` / `fov_end`), the column count is now derived from the actual azimuth range instead of always defaulting to the full-circle value. This prevents the image from having large empty column regions on the right when only a sector of the sweep is used:

```cpp
// Before: always used default_remake_config.max_azi_scan
// After: if FOV differs from defaults, derive from configured range
rq.max_azi_scan = round((rq.max_azi - rq.min_azi) / rq.ring_azi_resolution);
```

### `udp4_7_parser.h` — ATX

**Problem:** ATX sends 116 channels in two elevation-sorted groups (normal Ch.0–63, super-resolution Ch.64–115), both covering the same FOV. Using `channel_index` as the image row stacked both groups vertically, duplicating the scene.

**Fix — row assignment:** Applied the RingID interleaving formula from ATX User Manual §A.1.2 to map both groups into a single 116-row image:
- Ch 0–11 → ring_id = channel_index (normal only, above super-res FOV)
- Ch 12–63 → ring_id = `2 * channel_index - 11` (odd rows)
- Ch 64–115 → ring_id = `2 * (channel_index - 64) + 12` (even rows)

`set_ring` and `DoRemake` both use ring_id (not channel_index), keeping the point cloud ring field and depth image row consistent.

### `udp4_3_parser.h` — AT128

Image buffer allocation added to `DecodePacket` (height = `laser_num`, width = `max_azi_scan`). In `ComputeXYZI`, `row = channel_index` (AT128 channels are already ordered top-to-bottom by elevation) and `col` is derived from `DoRemake`'s azimuth bin output.

### `udp1_4_parser.h` — OT128 / PandarN / JT128

#### OT128

**Problem:** OT128 has a strongly non-uniform channel distribution — 64 channels (Ch.25–88) span only 7.8° near the horizon at 0.125°/channel, while 8 channels (Ch.1–8) span 7.2° at the top edge at ~0.9°/channel. Placing rows by `channel_index` stretches the centre of the image and compresses the edges, producing a geometrically distorted result.

**Fix — row assignment:** Elevation-angle-based geometric rectification using the calibrated elevation from the correction file:

```
row = round((max_elev - elevation_deg) / ring_elev_resolution)
     = round((15.0 - elevation_deg) / 0.125)
```

This maps the full FOV (−25° to +15°) uniformly across 320 rows at 0.125°/row. `DoRemake` receives `row` (not `channel_index`) as its ring argument so that `frame.points[col * 320 + row]` aligns with `depth_img[row][col]`. `set_ring` still stores the physical channel index (0–127). Rows at elevation angles where no laser fires are left as zero.

**Fix — image height:** Changed from `max_elev_scan` default (320, coincidentally correct for OT128 rectification) — kept at 320 for OT128, set to `laser_num` for all other sensors.


### 1.2 Operating Systems

- Ubuntu 16/18/20/22.04 
- Windows 10

### 1.3 Compiler Versions

Ubuntu
- Cmake 3.8.0 and above
- G++ 7.5 and above

Windows
- Cmake 3.8.0 and above
- MSVC 2019 and above

### 1.4 Dependencies

- If using point cloud visualization features, `PCL` installation is required
- If parsing PCAP files, `libpcap` installation is required
- If using TLS/mTLS-based Ptcs communication (supported by some lidars), `openssl` installation is required

<!-- - If parsing lidar point cloud correction files, `libyaml` installation is required  // Required for parsing config.yaml files in ROS driver -->

## 2 Getting Started

### 2.1 Clone

```bash
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git
```

> On Windows systems, downloading the repository as a ZIP file is not recommended as it may cause compilation errors due to symbolic link issues.

### 2.2 Compilation

#### 2.2.1 Compilation Instructions for Ubuntu
```bash
# 0. Install dependencies
sudo apt update && sudo apt install -y libpcl-dev libpcap-dev libyaml-cpp-dev openssl

# 1. Navigate to source directory
cd HesaiLidar_SDK_2.0

# 2. Create build directory and navigate to build directory
mkdir -p build && cd build

# 3. Configure project with Cmake
#    - Add -DCMAKE_BUILD_TYPE=Release for optimized compilation
cmake -DCMAKE_BUILD_TYPE=Release ..

# 4. Compile SDK
#    - Use -j$(nproc) to utilize all CPU cores
make -j$(nproc)
```

#### 2.2.2 Compilation Instructions for Windows
Please refer to **[How to Compile SDK on Windows](docs/compile_on_windows.md)**.

#### 2.2.3 Remove dependency on openssl library (not using PTCS communication)
Please refer to the operations in **[Compile Macro Control](docs/compile_macro_control_description.md)** to configure the macro `WITH_PTCS_USE` to be inactive.

## 3 Application Guide

### 3.1 Parse Lidar Data Online
Please refer to **[How to Parse Lidar Data Online](docs/parsing_lidar_data_online.md)**.

### 3.2 Parse PCAP File Data Offline
Please refer to **[How to Parse PCAP File Data Offline](docs/parsing_pcap_file_data_offline.md)**.

### 3.3 Point Cloud Data Visualization
Please refer to **[How to Visualize Point Cloud Data](docs/visualization_of_point_cloud_data.md)**.

### 3.4 Coordinate Transformation
Please refer to **[How to Perform Coordinate Transformation](docs/coordinate_transformation.md)**.

### 3.5 Save Point Cloud Data as PCD Files
Please refer to **[How to Save Point Cloud Data as PCD Files](docs/save_point_cloud_data_as_a_pcd_file.md)**.

### 3.6 Use GPU Acceleration
Please refer to **[How to Use GPU Acceleration for Performance Optimization](docs/use_gpu_acceleration.md)**.

### 3.7 Invoke SDK API Command Interface (PTC Communication)
Please refer to **[How to Invoke SDK API Command Interface](docs/invoke_sdk_api_command_interface.md)**.

### 3.8 Common Troubleshooting (WARNING)
Please refer to **[Common Troubleshooting (WARNING)](docs/common_error_codes.md)**.

### 3.9 Packet Loss Statistics
Please refer to **[How to Perform Packet Loss Statistics](docs/packet_loss_analysis.md)**.

### 3.10 Use Multi-threading to Accelerate Parsing
Please refer to the `thread_num` configuration in **[Functional Parameter Reference](docs/parameter_introduction.md)** and configure it to a value >1
> Note: The maximum allowed thread count is [CPU maximum cores - 2]. If configured beyond this, it will be modified to this number. Multi-threading will consume more CPU resources, please configure appropriately.

### 3.11 Parse Multiple Lidar Data Online
Navigate to [multi_test.cc](./test/multi_test.cc)

For parsing parameter configuration reference, see **[How to Parse Lidar Data Online](docs/parsing_lidar_data_online.md)**

> The basic principle is to use multi-threading to start two SDKs to parse data

### 3.12 Filter and Parse Specified Lidar Data from PCAP or Real-time Reception Containing Multi-lidar Data

Please refer to the descriptions of `device_udp_src_port` and `device_fault_port` in **[Functional Parameter Reference](docs/parameter_introduction.md)**

Enable point cloud packet filtering by configuring `device_udp_src_port` (point cloud packet source port number) and `device_ip_address` (point cloud packet source IP), parsing only point cloud packets from this source IP + source port number.

Enable fault message filtering by configuring `device_fault_port` (fault message source port number) and `device_ip_address` (fault message source IP), parsing only fault messages from this source IP + source port number.

### 3.13 Get Specific Timestamp for Each Point in Pandar Series, OT128, XT Series, QT Series Lidars (Point Cloud Packet Timestamp + Firing Channel Time Correction)

Please use the LidarPointXYZICRTT structure to declare HesaiLidarSdk, where uint64_t timeSecond is the seconds time part and uint32_t timeNanosecond is the nanoseconds time part. For example: `HesaiLidarSdk<LidarPointXYZICRTT> sample;`

### 3.14 Set SDK Point Cloud Reception Timeout and PTC Timeout During Initialization

1. Set SDK point cloud reception timeout

    Please refer to `recv_point_cloud_timeout` in **[Functional Parameter Reference](docs/parameter_introduction.md)**. This parameter defaults to `-1`, meaning that during initialization, if no valid point cloud is received, it will block and wait indefinitely. When this parameter is configured to >= 0, the SDK will wait for a period of time before initialization fails and exits.

2. Set PTC timeout
    
    Please refer to `ptc_connect_timeout` in **[Functional Parameter Reference](docs/parameter_introduction.md)**. This parameter defaults to `-1`, meaning that during initialization, if in `DATA_FROM_LIDAR` mode, it will block and wait for PTC connection indefinitely. When this parameter is >= 0, the SDK will wait for a period of time before reporting a connection timeout error and continuing initialization.

### 3.15 Point Cloud Rearrangement Based on Horizontal and Vertical Angles
Please refer to **[Point Cloud Rearrangement Function](docs/point_cloud_rearrangement_function.md)**

## 4 Functional Parameter Reference
Please refer to **[Functional Parameter Reference](docs/parameter_introduction.md)**.