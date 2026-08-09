# Divitor ROS 2 Workspace

This workspace contains the driver, perception, and bringup packages for the **Divitor** project—an aerial target detection system utilizing a Raspberry Pi. It integrates both thermal sensing (via the MLX90640 sensor) and RGB vision (via an IMX219 camera) to execute real-time human detection and streaming.

---

## Workspace Structure

The workspace contains the following core ROS 2 packages under `src/`:

```
src/
├── divitor_bringup/      # Python package for launching and configuring nodes
├── divitor_driver/       # C++ package for camera and thermal sensor drivers
├── divitor_perception/   # C++ package for preprocessors and detectors (YOLO / Thermal)
└── ros-gst-bridge/       # Submodule providing ROS 2 <-> GStreamer bridge elements
```

---

## Architecture & Data Flow

Below is the node data flow and topic layout when running the full system. Composable container launches can optimize this flow using zero-copy intra-process communication.

```mermaid
graph TD
    %% Drivers
    subgraph Drivers [Hardware & Sensor Drivers]
        IMX[imx219_camera / gscam]
        MLX[mlx90640_camera]
    end

    %% Perception Nodes
    subgraph Perception [Perception Pipeline]
        PRE[thermal_preprocessor]
        DET[adaptive_thermal_detector]
        YOLO[yolo_detector_node]
        TDET[thermal_detector_node]
    end

    %% Topics
    IMX -- "/rgb_camera/image_raw" --> YOLO
    MLX -- "/thermal_camera/image_raw" --> PRE
    PRE -- "/thermal_camera/image_preprocessed" --> DET
    
    %% Outputs
    DET -- "/thermal_camera/detections" --> D_OUT["vision_msgs/Detection2DArray"]
    DET -- "/thermal_camera/debug_mask" --> M_OUT["sensor_msgs/Image (Debug Mask)"]
    YOLO -- "/rgb_detection/visualization" --> Y_OUT["sensor_msgs/Image (YOLO Vis)"]
    
    %% Simple Thermal
    MLX -. "alternative subscription" .-> TDET
    TDET -- "thermal_detection/visualization" --> T_OUT["sensor_msgs/Image (Thermal Vis)"]
```

---

## Prerequisites & Dependencies

### System Requirements
* **OS**: Linux (typically Ubuntu 22.04 LTS for ROS 2 Humble)
* **ROS 2 Distro**: Humble Hawksbill

### Hardware Requirements
* **Processor**: Raspberry Pi 4 / 5 or equivalent board
* **Cameras**: 
  * Raspberry Pi Camera Module V2 (IMX219)
  * MLX90640 Thermal Sensor connected via I2C (`/dev/i2c-1`)

### Dependencies
Ensure the following packages and libraries are installed:
```bash
sudo apt update && sudo apt install -y \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools \
  libopencv-dev \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-sensor-msgs \
  ros-humble-gscam
```

---

## Installation & Compilation

1. **Clone the repository and submodules**:
   ```bash
   git clone --recursive <repository-url>
   cd ros2_ws
   ```
2. **Build the workspace**:
   Use `colcon build` to compile all packages:
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
3. **Source the setup script**:
   ```bash
   source install/setup.bash
   ```

---

## Packages & Nodes Detail

### 1. `divitor_driver`

Handles communication with physical sensors.

* **`mlx90640_camera`**:
  * Reads the 32x24 raw thermal matrix from the MLX90640 sensor over I2C.
  * **Published Topics**:
    * `thermal_camera/image_raw` (`sensor_msgs/msg/Image` in `32FC1` format)
  * **Parameters**:
    * `i2c_address` (default: `0x33`): I2C address of the thermal camera.
    * `power_of_2_refresh_rate` (default: `5` -> $2^5 = 32\text{ Hz}$): Configures refresh rate of the sensor.

* **`imx219_camera_node`** (Optional wrapper; `gscam` is typically used instead):
  * Captures frames via GStreamer `libcamerasrc` and publishes as ROS Images.
  * **Published Topics**:
    * `/rgb_camera/image_raw` (`sensor_msgs/msg/Image`)

---

### 2. `divitor_perception`

Processes camera frames and performs target extraction.

* **`thermal_preprocessor`**:
  * Applies spatial upsampling (Bicubic), temporal Exponential Moving Average (EMA) filtering to reduce noise, and Gaussian Blur.
  * **Subscribed Topics**:
    * `/raw_temperature` (remapped from `/thermal_camera/image_raw`)
  * **Published Topics**:
    * `/preprocessed_temperature` (remapped to `/thermal_camera/image_preprocessed`)
  * **Parameters**:
    * `target_width` (default: `128`): Spatial upscaling target width.
    * `target_height` (default: `96`): Spatial upscaling target height.
    * `enable_temporal_filter` (default: `true`): Toggle temporal noise EMA smoothing.
    * `ema_alpha` (default: `0.6`): Alpha constant for temporal smoothing.

* **`adaptive_thermal_detector`**:
  * Dynamic target detector with specialized lighting/solar rejection modes.
  * **Subscribed Topics**:
    * `/raw_temperature` (remapped to `/thermal_camera/image_preprocessed`)
  * **Published Topics**:
    * `/detections` (`vision_msgs/msg/Detection2DArray`): Detection bounding boxes.
    * `/debug_mask` (`sensor_msgs/msg/Image`): Cleaned binary detection mask.
  * **Parameters**:
    * `detection_mode` (default: `"NIGHT"`): `"NIGHT"` (global dynamic thresholding) or `"MIDDAY"` (local variance/box-filtered thresholding).
    * `k_sigma` (default: `1.5`): Multiplier for standard deviation thresholds.
    * `temp_min` (default: `24.0`): Physiological minimum human temperature limit.
    * `temp_max` (default: `38.0`): Physiological maximum human temperature limit.
    * `solar_reject_temp` (default: `38.0`): Excludes sun-baked land/metal targets in Midday mode.
    * `min_aspect_ratio` (default: `0.4`), `max_aspect_ratio` (default: `1.2`): Target aspect ratio filter for aerial targets.

* **`yolo_detector_node`**:
  * RGB camera human/object detector using ONNX Runtime.
  * **Subscribed Topics**:
    * `/rgb_camera/image_raw` (`sensor_msgs/msg/Image`)
  * **Published Topics**:
    * `/rgb_detection/visualization` (`sensor_msgs/msg/Image` with bounding boxes drawn)
  * **Parameters**:
    * `model_path`: Location of the `.onnx` model (defaults to package `models/yolo26n.onnx`).
    * `labels_path`: Path to COCO class labels (defaults to package `models/coco.names`).

---

### 3. `divitor_bringup`

Contains launch files to initiate various combinations of drivers and perception nodes.

* **`driver_n_perception.launch.py`**:
  * Launches standalone processes for `gscam` (IMX219), `mlx90640_camera`, `thermal_detector_node`, and `yolo_detector_node`.
  * Allows setting camera parameters (width, height, framerate, format) via arguments.
* **`divitor_components.launch.py`**:
  * High-performance composition launch utilizing Composable Components.
  * Launches `gscam`, `mlx90640_camera`, `thermal_preprocessor`, `adaptive_thermal_detector`, and `yolo_detector_node` in a shared memory container (`divitor_components_container`) with intra-process communication enabled to achieve zero-copy speedups.

---

## Running the Workspace

### 1. Launch Composable Components (Recommended)
This runs the full processing pipeline in a single container with optimized zero-copy message transfers:
```bash
ros2 launch divitor_bringup divitor_components.launch.py
```

### 2. Launch Standalone Nodes
To run nodes in separate OS processes:
```bash
ros2 launch divitor_bringup driver_n_perception.launch.py rgb_width:=640 rgb_height:=480 rgb_framerate:=30
```
