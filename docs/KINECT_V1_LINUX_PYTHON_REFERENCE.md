# Xbox 360 Kinect (Kinect v1) on Linux with Python - Complete Technical Reference

## Table of Contents
1. [Hardware Specifications](#1-hardware-specifications)
2. [Drivers and Libraries](#2-drivers-and-libraries)
3. [Installation on Linux](#3-installation-on-linux)
4. [Python Bindings and API](#4-python-bindings-and-api)
5. [Capturing RGB, Depth, and IR Streams](#5-capturing-rgb-depth-and-ir-streams)
6. [Camera Intrinsics and Calibration](#6-camera-intrinsics-and-calibration)
7. [Depth to 3D Point Cloud Conversion](#7-depth-to-3d-point-cloud-conversion)
8. [Point Cloud Accumulation for 3D Scanning](#8-point-cloud-accumulation-for-3d-scanning)
9. [Mesh Reconstruction and Export](#9-mesh-reconstruction-and-export)
10. [Complete 3D Scanning Pipeline](#10-complete-3d-scanning-pipeline)
11. [Sources and References](#11-sources-and-references)

---

## 1. Hardware Specifications

The Xbox 360 Kinect (Model 1414) uses **structured light** technology:

| Component          | Specification                                    |
|--------------------|--------------------------------------------------|
| RGB Camera         | 640x480 @ 30fps (up to 1280x1024 at lower fps)  |
| Depth Sensor       | 640x480 @ 30fps, 11-bit depth (2048 levels)      |
| IR Sensor          | 640x480 (shared with depth sensor)                |
| Depth Range        | 0.7m - 6.0m (usable), 1.2m - 3.5m (optimal)     |
| Depth Technology   | IR structured light pattern projection             |
| Field of View      | 57 degrees horizontal, 43 degrees vertical        |
| Tilt Motor         | -30 to +30 degrees                                |
| USB                | USB 2.0 (requires external power via adapter)     |
| Accelerometer      | 3-axis                                            |
| LED                | Multi-color status LED                            |

**Important**: The Kinect v1 requires a USB + power adapter cable. The original Xbox 360 cable
has a proprietary connector; you need either the official AC adapter accessory or a third-party
USB + 12V power splitter cable.

---

## 2. Drivers and Libraries

### Primary Option: libfreenect (OpenKinect)

**libfreenect** is the open-source userspace driver for the Xbox 360 Kinect. It is the most
mature and widely-used option for Kinect v1 on Linux.

- Repository: https://github.com/OpenKinect/libfreenect
- License: Dual Apache v2 / GPL v2
- Supports: Linux, macOS, Windows
- Provides: RGB, Depth, IR capture; motor tilt; LED control; accelerometer

### Alternative: OpenNI / OpenNI2

OpenNI was an open-source framework for natural interaction. It supported Kinect v1 through
the SensorKinect driver. However:
- OpenNI 1.x is effectively abandoned
- OpenNI2 has limited Kinect v1 support
- The primary maintainer (PrimeSense) was acquired by Apple
- **Recommendation: Use libfreenect instead**

### Do NOT confuse with:
- **libfreenect2** - This is for Kinect v2 (Xbox One Kinect), NOT the Xbox 360 Kinect
- **Azure Kinect SDK** - This is for the Azure Kinect DK, a completely different device

---

## 3. Installation on Linux

### 3.1 Install System Dependencies

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    git cmake build-essential \
    libusb-1.0-0-dev \
    freeglut3-dev libxmu-dev libxi-dev \
    python3-dev python3-numpy cython3

# Fedora
sudo dnf install cmake gcc-c++ libusb1-devel \
    freeglut-devel libXmu-devel libXi-devel \
    python3-devel python3-numpy python3-Cython

# Arch Linux
# There is a libfreenect PKGBUILD in the AUR:
#   yay -S libfreenect
#   yay -S libfreenect-git   (latest development version)
```

### 3.2 Build libfreenect from Source

```bash
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build && cd build

# Configure with Python 3 bindings
cmake .. \
    -DBUILD_PYTHON3=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_FAKENECT=ON

make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib64/
```

### 3.3 Install Python Wrapper

**Option A: From the build (recommended)**

The cmake build with `-DBUILD_PYTHON3=ON` installs the wrapper automatically.

**Option B: Manual installation from source**

```bash
cd /path/to/libfreenect/wrappers/python
sudo python3 setup.py install
# OR for local/venv install:
python3 setup.py build_ext --inplace
pip install -e .
```

**Option C: From PyPI**

```bash
pip install freenect
```

Note: The PyPI package may lag behind the latest source. Building from source is recommended
for the most reliable experience.

### 3.4 Set Up udev Rules (Required for Non-Root Access)

```bash
# Copy the udev rules file
sudo cp /path/to/libfreenect/platform/linux/udev/51-kinect.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add your user to the required groups
sudo adduser $USER video
sudo adduser $USER plugdev

# Log out and back in for group changes to take effect
```

The `51-kinect.rules` file grants access to the Kinect's USB interfaces for users in the
`plugdev` group. Without this, you would need to run all Kinect programs as root.

### 3.5 Install Python Scientific Stack

```bash
pip install numpy opencv-python open3d scipy matplotlib trimesh
```

### 3.6 Verify Installation

```bash
# Test that the Kinect is detected (plug in the Kinect first)
freenect-glview

# Test from Python
python3 -c "import freenect; print('freenect imported successfully')"
```

---

## 4. Python Bindings and API

### 4.1 freenect Module - Complete API Reference

The Python wrapper exposes the following constants and functions:

#### Video Format Constants
```python
import freenect

freenect.VIDEO_RGB             # Decompressed RGB (640x480x3, uint8)
freenect.VIDEO_BAYER           # Raw Bayer pattern
freenect.VIDEO_IR_8BIT         # 8-bit infrared (640x488, uint8)
freenect.VIDEO_IR_10BIT        # 10-bit infrared (640x488, uint16)
freenect.VIDEO_IR_10BIT_PACKED # 10-bit packed IR
freenect.VIDEO_YUV_RGB         # YUV decoded to RGB
freenect.VIDEO_YUV_RAW         # Raw YUV
```

#### Depth Format Constants
```python
freenect.DEPTH_11BIT           # Raw 11-bit disparity (640x480, uint16, 0-2047)
freenect.DEPTH_10BIT           # Raw 10-bit disparity (640x480, uint16)
freenect.DEPTH_11BIT_PACKED    # 11-bit packed
freenect.DEPTH_10BIT_PACKED    # 10-bit packed
freenect.DEPTH_REGISTERED      # Depth in mm, registered/aligned to RGB image (640x480, uint16)
freenect.DEPTH_MM              # Depth in mm, NOT aligned to RGB (640x480, uint16)
```

**Key distinction:**
- `DEPTH_11BIT` gives raw disparity values (0-2047), NOT distances. You must convert.
- `DEPTH_MM` gives depth in millimeters but in the depth camera's coordinate frame.
- `DEPTH_REGISTERED` gives depth in millimeters AND aligned to the RGB camera frame.
  This is typically what you want for colored point clouds.

#### LED Constants
```python
freenect.LED_OFF
freenect.LED_GREEN
freenect.LED_RED
freenect.LED_YELLOW
freenect.LED_BLINK_GREEN
freenect.LED_BLINK_RED_YELLOW
```

#### Resolution Constants
```python
freenect.RESOLUTION_LOW        # QVGA (320x240)
freenect.RESOLUTION_MEDIUM     # VGA  (640x480) - default
freenect.RESOLUTION_HIGH       # SXGA (1280x1024) - RGB only
```

#### Core Functions

```python
# --- Synchronous API (simplest, recommended for scanning) ---

# Get a single depth frame (blocking)
# Returns: (numpy_array, timestamp)
# array shape: (480, 640) dtype: uint16
depth, timestamp = freenect.sync_get_depth(index=0, format=freenect.DEPTH_MM)

# Get a single video frame (blocking)
# Returns: (numpy_array, timestamp)
# RGB array shape: (480, 640, 3) dtype: uint8
# IR array shape: (480, 488) dtype: uint8 or uint16
rgb, timestamp = freenect.sync_get_video(index=0, format=freenect.VIDEO_RGB)

# Stop the sync runloop (call when done)
freenect.sync_stop()


# --- Asynchronous API (callback-based, for real-time applications) ---

def depth_callback(dev, depth, timestamp):
    # depth is a numpy array (480, 640) uint16
    pass

def video_callback(dev, video, timestamp):
    # video is a numpy array (480, 640, 3) uint8
    pass

def body_callback(dev, ctx):
    # Called each iteration; raise freenect.Kill to stop
    pass

# Start the event loop with callbacks
freenect.runloop(
    depth=depth_callback,
    video=video_callback,
    body=body_callback
)


# --- Device Control ---

# These require a device handle from the async API

# Motor tilt control (-30 to +30 degrees)
freenect.set_tilt_degs(dev, angle)
freenect.update_tilt_state(dev)
state = freenect.get_tilt_state(dev)
current_angle = freenect.get_tilt_degs(state)

# Accelerometer
ax, ay, az = freenect.get_mks_accel(state)  # in m/s^2
# or shorthand:
ax, ay, az = freenect.get_accel(dev)

# LED control
freenect.set_led(dev, freenect.LED_GREEN)

# Device enumeration
ctx = freenect.init()
num = freenect.num_devices(ctx)
```

---

## 5. Capturing RGB, Depth, and IR Streams

### 5.1 Basic RGB + Depth Viewer

```python
import freenect
import cv2
import numpy as np

def capture_and_display():
    """Capture and display RGB and depth streams."""
    while True:
        # Get RGB frame
        rgb, _ = freenect.sync_get_video(0, freenect.VIDEO_RGB)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Get depth frame in millimeters
        depth_mm, _ = freenect.sync_get_depth(0, freenect.DEPTH_MM)

        # Normalize depth for visualization (0-255)
        # Valid depth range is roughly 400-10000mm
        depth_display = np.clip(depth_mm, 0, 8000)
        depth_display = (depth_display / 8000.0 * 255).astype(np.uint8)
        depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

        cv2.imshow('RGB', bgr)
        cv2.imshow('Depth', depth_colormap)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

    freenect.sync_stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    capture_and_display()
```

### 5.2 Capturing IR Stream

```python
import freenect
import cv2
import numpy as np

def capture_ir():
    """Capture and display IR stream.

    IMPORTANT: You cannot capture RGB and IR simultaneously on Kinect v1.
    The IR sensor and RGB camera share the video pipeline.
    Switch between them as needed.
    """
    while True:
        # Get IR frame (8-bit version for simplicity)
        ir, _ = freenect.sync_get_video(0, freenect.VIDEO_IR_8BIT)

        # IR frame is 640x488 (slightly taller than 640x480)
        cv2.imshow('IR', ir)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    freenect.sync_stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    capture_ir()
```

### 5.3 Capturing Registered Depth (Aligned to RGB)

```python
import freenect
import numpy as np

def get_registered_rgbd():
    """
    Get RGB and depth frames that are aligned to each other.
    DEPTH_REGISTERED aligns the depth map to the RGB camera's coordinate frame.
    This means pixel (x, y) in the depth image corresponds to pixel (x, y) in the RGB image.
    """
    rgb, rgb_ts = freenect.sync_get_video(0, freenect.VIDEO_RGB)
    depth_mm, depth_ts = freenect.sync_get_depth(0, freenect.DEPTH_REGISTERED)

    # depth_mm is now in millimeters AND aligned to the RGB camera
    # Both are 640x480
    return rgb, depth_mm

rgb, depth = get_registered_rgbd()
print(f"RGB shape: {rgb.shape}, dtype: {rgb.dtype}")       # (480, 640, 3) uint8
print(f"Depth shape: {depth.shape}, dtype: {depth.dtype}")  # (480, 640) uint16
print(f"Depth range: {depth[depth > 0].min()} - {depth[depth > 0].max()} mm")
```

### 5.4 Async Capture with Motor and LED Control

```python
import freenect
import cv2
import numpy as np
import threading

# Global frame storage
current_depth = None
current_rgb = None
lock = threading.Lock()

def depth_cb(dev, data, timestamp):
    global current_depth
    with lock:
        current_depth = data.copy()

def rgb_cb(dev, data, timestamp):
    global current_rgb
    with lock:
        current_rgb = data.copy()

def body_cb(dev, ctx):
    # Control the LED
    freenect.set_led(dev, freenect.LED_GREEN)

    # Set tilt to 0 degrees (level)
    freenect.set_tilt_degs(dev, 0)

    # Check accelerometer
    freenect.update_tilt_state(dev)
    state = freenect.get_tilt_state(dev)
    ax, ay, az = freenect.get_mks_accel(state)

# Start the runloop in a background thread
kinect_thread = threading.Thread(
    target=freenect.runloop,
    kwargs={
        'depth': depth_cb,
        'video': rgb_cb,
        'body': body_cb,
    },
    daemon=True
)
kinect_thread.start()
```

---

## 6. Camera Intrinsics and Calibration

### 6.1 Default Intrinsic Parameters (Kinect v1)

These are well-known approximate intrinsic parameters for the Kinect v1. Each individual
Kinect may vary slightly, but these work reasonably well as defaults.

#### Depth Camera Intrinsics (640x480)
```python
# From Nicolas Burrus / RGBDemo calibration
fx_d = 594.21434211923247
fy_d = 591.04053696870778
cx_d = 339.30780975300314
cy_d = 242.73913761751615
```

#### RGB Camera Intrinsics (640x480)
```python
fx_rgb = 529.21508098293293
fy_rgb = 525.56393630057437
cx_rgb = 328.94272028759258
cy_rgb = 267.48068171871557
```

#### RGB Camera Distortion Coefficients
```python
k1_rgb = 0.26451622
k2_rgb = -0.83990749
p1_rgb = -0.0019922302
p2_rgb = 0.0014371996
k3_rgb = 0.91192465
```

#### Depth Camera Distortion Coefficients
```python
k1_d = -0.26386490
k2_d = 0.99966832
p1_d = -0.00076275862
p2_d = 0.0050350940
k3_d = -1.3053628
```

#### Depth-to-RGB Extrinsic Transform
```python
import numpy as np

# Rotation (approximately identity - the cameras are nearly co-planar)
R = np.array([
    [ 9.9984628826577793e-01, 1.2635359098409581e-03, -1.7487233004436643e-02],
    [-1.4779096108364480e-03, 9.9992385683542895e-01, -1.2251380107679535e-02],
    [ 1.7470421412464927e-02, 1.2275341476520762e-02,  9.9977202419716948e-01]
])

# Translation (meters)
T = np.array([1.9985242312092553e-02, -7.4423738761617583e-04, -1.0916736334336222e-02])
```

### 6.2 Open3D Intrinsic Objects

Open3D provides a built-in `PrimeSenseDefault` intrinsic that is a close approximation:

```python
import open3d as o3d

# Built-in PrimeSense/Kinect v1 default intrinsics
# fx=525.0, fy=525.0, cx=319.5, cy=239.5 at 640x480
intrinsic_default = o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

# Or create custom intrinsics with the calibrated values:
intrinsic_calibrated = o3d.camera.PinholeCameraIntrinsic(
    width=640, height=480,
    fx=594.21434211923247,
    fy=591.04053696870778,
    cx=339.30780975300314,
    cy=242.73913761751615
)
```

### 6.3 Raw Disparity to Distance Conversion

If you use `DEPTH_11BIT` format, you get raw disparity values (0-2047). Convert to meters:

```python
def raw_disparity_to_meters(raw_depth):
    """
    Convert raw 11-bit Kinect disparity to distance in meters.
    raw_depth: integer 0-2047
    Returns: distance in meters (0 means invalid/no reading)
    """
    # From Nicolas Burrus calibration
    # Values of 2047 or 0 indicate invalid depth
    if raw_depth >= 2047 or raw_depth <= 0:
        return 0.0
    return 1.0 / (raw_depth * -0.0030711016 + 3.3309495161)
```

Vectorized version:
```python
import numpy as np

def disparity_to_meters_array(raw_depth_array):
    """Convert an entire 640x480 raw disparity frame to meters."""
    depth_m = np.zeros_like(raw_depth_array, dtype=np.float64)
    valid = (raw_depth_array > 0) & (raw_depth_array < 2047)
    depth_m[valid] = 1.0 / (raw_depth_array[valid] * -0.0030711016 + 3.3309495161)
    return depth_m
```

**Recommendation**: Use `DEPTH_MM` or `DEPTH_REGISTERED` format instead, which gives you
millimeter values directly, using libfreenect's internal calibration (more accurate).

### 6.4 Custom Calibration with OpenCV

For the best accuracy, calibrate your specific Kinect:

```python
import cv2
import numpy as np

# Print a checkerboard pattern and capture multiple images
# from the Kinect's RGB camera at different angles

checkerboard_size = (9, 6)  # inner corners
square_size = 0.025  # 25mm squares

# Collect frames
objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# For each captured frame:
# gray = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2GRAY)
# ret, corners = cv2.findChessboardCorners(gray, checkerboard_size)
# if ret:
#     corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
#     imgpoints.append(corners2)
#     objpoints.append(objp)

# Calibrate
# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
#     objpoints, imgpoints, (640, 480), None, None)
# fx, fy = mtx[0,0], mtx[1,1]
# cx, cy = mtx[0,2], mtx[1,2]
```

---

## 7. Depth to 3D Point Cloud Conversion

### 7.1 The Projection Model

A depth camera captures a 2D depth image. Each pixel (u, v) has a depth value Z.
To convert to 3D coordinates (X, Y, Z) in camera space:

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = Z (the depth value)
```

Where (fx, fy) are focal lengths and (cx, cy) is the principal point.

### 7.2 Pure NumPy Point Cloud Generation

```python
import numpy as np
import freenect

# Kinect v1 depth camera intrinsics
FX_D = 594.21434211923247
FY_D = 591.04053696870778
CX_D = 339.30780975300314
CY_D = 242.73913761751615

def depth_to_point_cloud(depth_mm, fx=FX_D, fy=FY_D, cx=CX_D, cy=CY_D):
    """
    Convert a 640x480 depth image (in mm) to a 3D point cloud.

    Parameters:
        depth_mm: numpy array (480, 640) of uint16, depth in millimeters
        fx, fy: focal lengths in pixels
        cx, cy: principal point in pixels

    Returns:
        points: numpy array (N, 3) of float64, XYZ in meters
        valid_mask: boolean array (480, 640) indicating valid depth pixels
    """
    rows, cols = depth_mm.shape

    # Create coordinate grids
    u = np.arange(cols)  # 0..639
    v = np.arange(rows)  # 0..479
    u, v = np.meshgrid(u, v)

    # Convert depth to meters
    z = depth_mm.astype(np.float64) / 1000.0

    # Mask invalid depths (0 means no reading)
    valid = z > 0

    # Back-project to 3D
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Stack into (H, W, 3) then reshape to (N, 3)
    points = np.stack([x, y, z], axis=-1)

    # Return only valid points
    return points[valid], valid


def depth_to_colored_point_cloud(depth_mm, rgb, fx=FX_D, fy=FY_D, cx=CX_D, cy=CY_D):
    """
    Convert registered depth + RGB to a colored point cloud.

    IMPORTANT: Use DEPTH_REGISTERED format so depth is aligned to RGB.

    Parameters:
        depth_mm: (480, 640) uint16 depth in mm (DEPTH_REGISTERED)
        rgb: (480, 640, 3) uint8 RGB image

    Returns:
        points: (N, 3) float64, XYZ in meters
        colors: (N, 3) float64, RGB normalized to [0, 1]
    """
    points, valid = depth_to_point_cloud(depth_mm, fx, fy, cx, cy)
    colors = rgb[valid].astype(np.float64) / 255.0
    return points, colors


# --- Usage ---
depth_mm, _ = freenect.sync_get_depth(0, freenect.DEPTH_REGISTERED)
rgb, _ = freenect.sync_get_video(0, freenect.VIDEO_RGB)

points, colors = depth_to_colored_point_cloud(depth_mm, rgb)
print(f"Point cloud: {points.shape[0]} points")
print(f"X range: {points[:, 0].min():.3f} to {points[:, 0].max():.3f} m")
print(f"Y range: {points[:, 1].min():.3f} to {points[:, 1].max():.3f} m")
print(f"Z range: {points[:, 2].min():.3f} to {points[:, 2].max():.3f} m")
```

### 7.3 Open3D Point Cloud from Depth

```python
import open3d as o3d
import numpy as np
import freenect

def capture_open3d_point_cloud():
    """Capture a single colored point cloud using Open3D."""

    # Capture registered depth and RGB
    depth_mm, _ = freenect.sync_get_depth(0, freenect.DEPTH_REGISTERED)
    rgb, _ = freenect.sync_get_video(0, freenect.VIDEO_RGB)

    # Create Open3D images
    depth_o3d = o3d.geometry.Image(depth_mm.astype(np.uint16))
    color_o3d = o3d.geometry.Image(rgb.astype(np.uint8))

    # Create RGBD image
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1000.0,     # depth is in mm, convert to meters
        depth_trunc=4.0,        # truncate at 4 meters
        convert_rgb_to_intensity=False
    )

    # Camera intrinsics
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width=640, height=480,
        fx=594.21434211923247,
        fy=591.04053696870778,
        cx=339.30780975300314,
        cy=242.73913761751615
    )

    # Generate point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # Flip (Open3D convention: camera looks along +Z, but
    # the resulting cloud may appear flipped)
    pcd.transform([[1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])

    return pcd

pcd = capture_open3d_point_cloud()
print(f"Point cloud has {len(pcd.points)} points")
o3d.visualization.draw_geometries([pcd])
```

### 7.4 Filtering and Cleaning Point Clouds

```python
import open3d as o3d
import numpy as np

def clean_point_cloud(pcd, voxel_size=0.005, nb_neighbors=20, std_ratio=2.0):
    """
    Clean a raw point cloud:
    1. Voxel downsampling to reduce density
    2. Statistical outlier removal to eliminate noise
    3. Estimate normals for downstream processing
    """
    # Voxel downsampling (5mm voxels by default)
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"After voxel downsampling: {len(pcd_down.points)} points")

    # Statistical outlier removal
    pcd_clean, ind = pcd_down.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )
    print(f"After outlier removal: {len(pcd_clean.points)} points")

    # Estimate normals
    pcd_clean.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.02,   # 2cm search radius
            max_nn=30       # max 30 neighbors
        )
    )

    # Orient normals consistently (toward camera at origin)
    pcd_clean.orient_normals_towards_camera_location(
        camera_location=np.array([0.0, 0.0, 0.0])
    )

    return pcd_clean
```

---

## 8. Point Cloud Accumulation for 3D Scanning

### 8.1 Approach Overview

To build a complete 3D model, you need to:
1. Capture multiple point clouds from different viewpoints
2. Register (align) them into a common coordinate frame
3. Merge them into a single unified point cloud or volume
4. Reconstruct a mesh surface

There are two main strategies:
- **Turntable scanning**: Object rotates on a turntable, camera is fixed
- **Freehand scanning**: Camera moves around the object, using ICP/TSDF for registration

### 8.2 Turntable Scanning (Simplest Approach)

```python
import open3d as o3d
import numpy as np
import freenect
import time

def turntable_scan(num_captures=12, pause_seconds=5.0,
                    voxel_size=0.005, depth_trunc=1.5):
    """
    Capture multiple point clouds from a turntable setup.

    Place the object on a turntable. The Kinect is stationary.
    Rotate the turntable by (360/num_captures) degrees between each capture.

    Parameters:
        num_captures: number of captures around 360 degrees
        pause_seconds: time to wait between captures (for manual rotation)
        voxel_size: downsampling voxel size in meters
        depth_trunc: max depth in meters (clip background)
    """
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width=640, height=480,
        fx=594.21, fy=591.04, cx=339.31, cy=242.74
    )

    point_clouds = []
    angle_step = 360.0 / num_captures

    for i in range(num_captures):
        input(f"Position {i+1}/{num_captures} "
              f"(rotate ~{angle_step:.0f} deg). Press Enter to capture...")

        # Capture
        depth_mm, _ = freenect.sync_get_depth(0, freenect.DEPTH_REGISTERED)
        rgb, _ = freenect.sync_get_video(0, freenect.VIDEO_RGB)

        # Create RGBD
        depth_o3d = o3d.geometry.Image(depth_mm.astype(np.uint16))
        color_o3d = o3d.geometry.Image(rgb.astype(np.uint8))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d,
            depth_scale=1000.0,
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False
        )

        # Generate point cloud
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

        # Clean
        pcd = pcd.voxel_down_sample(voxel_size)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # Apply known rotation (turntable angle)
        angle_rad = np.radians(i * angle_step)
        R = np.array([
            [np.cos(angle_rad), 0, np.sin(angle_rad), 0],
            [0, 1, 0, 0],
            [-np.sin(angle_rad), 0, np.cos(angle_rad), 0],
            [0, 0, 0, 1]
        ])
        pcd.transform(R)

        point_clouds.append(pcd)
        print(f"  Captured {len(pcd.points)} points")

    freenect.sync_stop()

    # Merge all point clouds
    merged = o3d.geometry.PointCloud()
    for pcd in point_clouds:
        merged += pcd

    # Final cleanup
    merged = merged.voxel_down_sample(voxel_size)
    merged.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30)
    )

    return merged, point_clouds
```

### 8.3 Pairwise ICP Registration

For freehand scanning where poses are unknown, use ICP to align consecutive frames:

```python
import open3d as o3d
import numpy as np

def pairwise_icp(source, target, voxel_size=0.005, max_correspondence=0.02):
    """
    Align source point cloud to target using ICP.

    Returns:
        transformation: 4x4 numpy array
        information: information matrix
    """
    # Estimate normals if not present
    for pcd in [source, target]:
        if not pcd.has_normals():
            pcd.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=voxel_size * 2, max_nn=30))

    # Point-to-plane ICP
    result = o3d.pipelines.registration.registration_icp(
        source, target,
        max_correspondence_distance=max_correspondence,
        init=np.identity(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=100)
    )

    print(f"ICP fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.6f}")
    return result.transformation, result.fitness


def colored_icp_registration(source, target, voxel_size=0.005):
    """
    Multi-scale colored ICP for more accurate registration.
    Uses both geometry AND color for alignment.
    """
    voxel_radius = [voxel_size * 4, voxel_size * 2, voxel_size]
    max_iter = [50, 30, 14]
    current_transformation = np.identity(4)

    for scale in range(3):
        radius = voxel_radius[scale]
        iter_count = max_iter[scale]

        # Downsample
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        # Estimate normals
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        # Colored ICP
        result = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down,
            radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=iter_count)
        )
        current_transformation = result.transformation

    print(f"Colored ICP fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.6f}")
    return current_transformation, result.fitness
```

### 8.4 Multiway Registration (Global Optimization)

When you have many overlapping point clouds, pairwise ICP can accumulate drift.
Open3D's multiway registration corrects this:

```python
import open3d as o3d
import numpy as np

def multiway_registration(pcds, voxel_size=0.005, max_correspondence=0.02):
    """
    Globally optimize poses for a list of point clouds.

    Parameters:
        pcds: list of open3d.geometry.PointCloud
        voxel_size: downsampling voxel size
        max_correspondence: ICP max correspondence distance

    Returns:
        list of 4x4 transformation matrices (one per point cloud)
    """
    # Build pose graph
    pose_graph = o3d.pipelines.registration.PoseGraph()

    # Add first node (identity, reference frame)
    pose_graph.nodes.append(
        o3d.pipelines.registration.PoseGraphNode(np.identity(4)))

    n = len(pcds)
    cumulative_transform = np.identity(4)

    for i in range(n):
        for j in range(i + 1, n):
            # Pairwise registration
            source_down = pcds[i].voxel_down_sample(voxel_size)
            target_down = pcds[j].voxel_down_sample(voxel_size)

            source_down.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=voxel_size * 2, max_nn=30))
            target_down.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=voxel_size * 2, max_nn=30))

            result = o3d.pipelines.registration.registration_icp(
                source_down, target_down,
                max_correspondence,
                np.identity(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )

            # Determine edge type
            if j == i + 1:
                # Odometry edge (consecutive frames - reliable)
                uncertain = False
                cumulative_transform = result.transformation @ cumulative_transform
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(cumulative_transform)))
            else:
                # Loop closure edge (non-consecutive - less reliable)
                uncertain = True

            if result.fitness > 0.3:  # Only add good edges
                info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                    source_down, target_down, max_correspondence, result.transformation)

                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        i, j, result.transformation, info, uncertain=uncertain))

    # Global optimization
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence,
        edge_prune_threshold=0.25,
        reference_node=0
    )

    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option
    )

    # Extract optimized transforms
    transforms = [pose_graph.nodes[i].pose for i in range(n)]
    return transforms


def merge_with_transforms(pcds, transforms, voxel_size=0.003):
    """Apply transforms and merge point clouds."""
    merged = o3d.geometry.PointCloud()
    for pcd, transform in zip(pcds, transforms):
        tmp = pcd.voxel_down_sample(voxel_size)
        tmp.transform(transform)
        merged += tmp

    merged = merged.voxel_down_sample(voxel_size)
    return merged
```

### 8.5 TSDF Volume Integration (Best Quality)

TSDF (Truncated Signed Distance Function) integration is the gold standard for
RGB-D reconstruction. It produces smoother, more complete results than point cloud merging.

```python
import open3d as o3d
import numpy as np
import freenect

def tsdf_scanning_session(num_frames=50, voxel_length=0.004, sdf_trunc=0.02):
    """
    Real-time TSDF integration from Kinect frames.

    This is a simplified version assuming small camera motion
    between consecutive frames (uses ICP for pose tracking).
    """
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width=640, height=480,
        fx=594.21, fy=591.04, cx=339.31, cy=242.74
    )

    # Create TSDF volume
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,      # 4mm voxels
        sdf_trunc=sdf_trunc,            # 20mm truncation
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )

    # Track camera pose
    cumulative_pose = np.identity(4)
    prev_pcd = None

    for i in range(num_frames):
        # Capture
        depth_mm, _ = freenect.sync_get_depth(0, freenect.DEPTH_REGISTERED)
        rgb, _ = freenect.sync_get_video(0, freenect.VIDEO_RGB)

        # Create RGBD image
        depth_o3d = o3d.geometry.Image(depth_mm.astype(np.uint16))
        color_o3d = o3d.geometry.Image(rgb.astype(np.uint8))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d,
            depth_scale=1000.0,
            depth_trunc=3.0,
            convert_rgb_to_intensity=False
        )

        # Generate point cloud for ICP tracking
        curr_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
        curr_pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

        if prev_pcd is not None:
            # Track pose with ICP
            result = o3d.pipelines.registration.registration_icp(
                curr_pcd, prev_pcd,
                max_correspondence_distance=0.05,
                init=np.identity(4),
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )
            cumulative_pose = cumulative_pose @ result.transformation

        # Integrate into TSDF volume
        volume.integrate(
            rgbd,
            intrinsic,
            np.linalg.inv(cumulative_pose)
        )

        prev_pcd = curr_pcd
        print(f"Frame {i+1}/{num_frames} integrated (fitness: "
              f"{result.fitness:.3f if prev_pcd is not None and i > 0 else 'N/A'})")

    freenect.sync_stop()

    # Extract mesh
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    # Also extract point cloud if needed
    pcd = volume.extract_point_cloud()

    return mesh, pcd
```

---

## 9. Mesh Reconstruction and Export

### 9.1 Poisson Surface Reconstruction

```python
import open3d as o3d
import numpy as np

def poisson_reconstruction(pcd, depth=9, density_threshold_quantile=0.01):
    """
    Reconstruct a triangle mesh from a point cloud using Poisson reconstruction.

    Parameters:
        pcd: open3d.geometry.PointCloud (must have normals!)
        depth: octree depth (higher = more detail, 8-10 typical)
        density_threshold_quantile: remove low-density vertices (0.01 = bottom 1%)

    Returns:
        mesh: open3d.geometry.TriangleMesh
    """
    # Ensure normals exist
    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.02, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(k=15)

    # Poisson reconstruction
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, width=0, scale=1.1, linear_fit=False)

    print(f"Poisson mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")

    # Remove low-density vertices (noisy extrapolations)
    densities = np.asarray(densities)
    density_threshold = np.quantile(densities, density_threshold_quantile)
    vertices_to_remove = densities < density_threshold
    mesh.remove_vertices_by_mask(vertices_to_remove)

    print(f"After density filter: {len(mesh.vertices)} vertices, "
          f"{len(mesh.triangles)} triangles")

    return mesh
```

### 9.2 Ball Pivoting Algorithm (Alternative)

```python
def ball_pivoting_reconstruction(pcd, radii=None):
    """
    Reconstruct mesh using Ball Pivoting Algorithm.
    Works well for uniformly-sampled point clouds.
    """
    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.02, max_nn=30))

    if radii is None:
        # Estimate good radii from point cloud density
        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radii = [avg_dist * 1.0, avg_dist * 2.0, avg_dist * 4.0]

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii))

    print(f"Ball pivoting mesh: {len(mesh.vertices)} vertices, "
          f"{len(mesh.triangles)} triangles")

    return mesh
```

### 9.3 Export to PLY Format

PLY (Polygon File Format) is widely supported by Blender, MeshLab, CloudCompare, etc.

```python
import open3d as o3d

# Export point cloud as PLY
o3d.io.write_point_cloud("scan_pointcloud.ply", pcd)

# Export triangle mesh as PLY
o3d.io.write_triangle_mesh("scan_mesh.ply", mesh)

# PLY supports:
# - Vertex positions (x, y, z)
# - Vertex colors (r, g, b)
# - Vertex normals (nx, ny, nz)
# - Face indices
# Both ASCII and binary formats
```

### 9.4 Export to OBJ Format

OBJ is a text-based format supported by virtually all 3D software.

```python
# Export mesh as OBJ
o3d.io.write_triangle_mesh("scan_mesh.obj", mesh)

# Note: OBJ files can also have .mtl material files
# Open3D will generate one if the mesh has vertex colors
```

### 9.5 Export to Other Formats

```python
# STL (for 3D printing)
o3d.io.write_triangle_mesh("scan_mesh.stl", mesh)

# GLTF/GLB (for web and game engines)
o3d.io.write_triangle_mesh("scan_mesh.glb", mesh)

# XYZ (simple point cloud text format)
o3d.io.write_point_cloud("scan_points.xyz", pcd)

# PCD (Point Cloud Data - PCL format)
o3d.io.write_point_cloud("scan_points.pcd", pcd)
```

### 9.6 Using Trimesh for Additional Export Options

```python
import trimesh
import numpy as np

def open3d_to_trimesh(o3d_mesh):
    """Convert Open3D mesh to trimesh for additional export options."""
    vertices = np.asarray(o3d_mesh.vertices)
    faces = np.asarray(o3d_mesh.triangles)

    # Handle vertex colors
    if o3d_mesh.has_vertex_colors():
        colors = (np.asarray(o3d_mesh.vertex_colors) * 255).astype(np.uint8)
        # Add alpha channel
        colors = np.column_stack([colors, np.full(len(colors), 255, dtype=np.uint8)])
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces,
                                vertex_colors=colors)
    else:
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces)

    return mesh

# Convert and export
tri_mesh = open3d_to_trimesh(mesh)

# Export with trimesh (supports many formats)
tri_mesh.export("scan.ply")
tri_mesh.export("scan.obj")
tri_mesh.export("scan.stl")
tri_mesh.export("scan.glb")       # GLTF binary
tri_mesh.export("scan.dae")       # Collada
tri_mesh.export("scan.off")       # Object File Format
```

### 9.7 Importing into Blender

```
In Blender:
1. File > Import > Stanford (.ply) or Wavefront (.obj)
2. Select your exported file
3. For PLY with vertex colors:
   - Switch to Material Preview or Rendered view
   - In Shader Editor, add "Attribute" node
   - Set attribute name to "Col" (vertex color)
   - Connect Color output to Base Color of Principled BSDF
4. For further cleanup:
   - Edit Mode > Select All > Mesh > Clean Up > Decimate Geometry
   - Or use the Remesh modifier for a cleaner topology
```

---

## 10. Complete 3D Scanning Pipeline

Here is a complete, ready-to-use scanning application:

```python
#!/usr/bin/env python3
"""
Kinect v1 3D Scanner
Complete pipeline: capture -> point cloud -> registration -> mesh -> export

Requirements:
    pip install numpy opencv-python open3d

    Plus libfreenect with Python bindings (see installation section).
"""

import freenect
import numpy as np
import open3d as o3d
import cv2
import time
import os
from datetime import datetime


# ============================================================
# Configuration
# ============================================================

class KinectConfig:
    # Depth camera intrinsics (Kinect v1 defaults)
    FX = 594.21434211923247
    FY = 591.04053696870778
    CX = 339.30780975300314
    CY = 242.73913761751615
    WIDTH = 640
    HEIGHT = 480

    # Scanning parameters
    DEPTH_SCALE = 1000.0        # mm to meters
    DEPTH_TRUNC = 3.0           # max depth in meters
    VOXEL_SIZE = 0.005          # 5mm downsampling
    ICP_MAX_CORRESPONDENCE = 0.05  # 5cm ICP threshold

    # TSDF parameters
    TSDF_VOXEL_LENGTH = 0.004   # 4mm voxels
    TSDF_SDF_TRUNC = 0.02       # 20mm truncation

    # Mesh reconstruction
    POISSON_DEPTH = 9
    DENSITY_QUANTILE = 0.01

    @classmethod
    def get_intrinsic(cls):
        return o3d.camera.PinholeCameraIntrinsic(
            cls.WIDTH, cls.HEIGHT, cls.FX, cls.FY, cls.CX, cls.CY)


# ============================================================
# Kinect Capture
# ============================================================

class KinectCapture:
    """Handles capturing frames from the Kinect."""

    def __init__(self, device_index=0):
        self.device_index = device_index

    def get_rgbd(self):
        """Capture a single RGBD frame pair."""
        depth_mm, _ = freenect.sync_get_depth(
            self.device_index, freenect.DEPTH_REGISTERED)
        rgb, _ = freenect.sync_get_video(
            self.device_index, freenect.VIDEO_RGB)
        return rgb, depth_mm

    def get_open3d_rgbd(self, depth_trunc=None):
        """Capture and return as Open3D RGBDImage."""
        if depth_trunc is None:
            depth_trunc = KinectConfig.DEPTH_TRUNC

        rgb, depth_mm = self.get_rgbd()

        color_o3d = o3d.geometry.Image(rgb.astype(np.uint8))
        depth_o3d = o3d.geometry.Image(depth_mm.astype(np.uint16))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d,
            depth_scale=KinectConfig.DEPTH_SCALE,
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False
        )
        return rgbd, rgb, depth_mm

    def get_point_cloud(self, depth_trunc=None):
        """Capture and return as Open3D PointCloud."""
        rgbd, rgb, depth_mm = self.get_open3d_rgbd(depth_trunc)
        intrinsic = KinectConfig.get_intrinsic()
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
        return pcd, rgbd

    def stop(self):
        """Release the Kinect."""
        freenect.sync_stop()


# ============================================================
# Point Cloud Processing
# ============================================================

class PointCloudProcessor:
    """Utilities for point cloud cleaning and registration."""

    @staticmethod
    def clean(pcd, voxel_size=None, nb_neighbors=20, std_ratio=2.0):
        """Downsample and remove outliers."""
        if voxel_size is None:
            voxel_size = KinectConfig.VOXEL_SIZE

        pcd_down = pcd.voxel_down_sample(voxel_size)
        pcd_clean, _ = pcd_down.remove_statistical_outlier(
            nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        return pcd_clean

    @staticmethod
    def estimate_normals(pcd, radius=0.02, max_nn=30):
        """Estimate and orient surface normals."""
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius, max_nn=max_nn))
        pcd.orient_normals_towards_camera_location(
            camera_location=np.array([0.0, 0.0, 0.0]))
        return pcd

    @staticmethod
    def register_icp(source, target, max_dist=None, init_transform=None):
        """Align source to target using point-to-plane ICP."""
        if max_dist is None:
            max_dist = KinectConfig.ICP_MAX_CORRESPONDENCE
        if init_transform is None:
            init_transform = np.identity(4)

        # Ensure normals
        for pcd in [source, target]:
            if not pcd.has_normals():
                PointCloudProcessor.estimate_normals(pcd)

        result = o3d.pipelines.registration.registration_icp(
            source, target, max_dist, init_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
        )
        return result.transformation, result.fitness, result.inlier_rmse

    @staticmethod
    def register_colored_icp(source, target, voxel_size=None):
        """Multi-scale colored ICP registration."""
        if voxel_size is None:
            voxel_size = KinectConfig.VOXEL_SIZE

        voxel_radius = [voxel_size * 4, voxel_size * 2, voxel_size]
        max_iter = [50, 30, 14]
        transform = np.identity(4)

        for scale in range(3):
            radius = voxel_radius[scale]
            src_down = source.voxel_down_sample(radius)
            tgt_down = target.voxel_down_sample(radius)

            src_down.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
            tgt_down.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

            result = o3d.pipelines.registration.registration_colored_icp(
                src_down, tgt_down, radius, transform,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_fitness=1e-6, relative_rmse=1e-6,
                    max_iteration=max_iter[scale])
            )
            transform = result.transformation

        return transform, result.fitness, result.inlier_rmse


# ============================================================
# 3D Scanner
# ============================================================

class KinectScanner:
    """Complete 3D scanning pipeline."""

    def __init__(self):
        self.capture = KinectCapture()
        self.processor = PointCloudProcessor()
        self.frames = []        # List of (rgbd, pcd) tuples
        self.transforms = []    # Camera pose for each frame
        self.volume = None      # TSDF volume

    def initialize_tsdf(self):
        """Create a fresh TSDF volume."""
        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=KinectConfig.TSDF_VOXEL_LENGTH,
            sdf_trunc=KinectConfig.TSDF_SDF_TRUNC,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )

    def capture_frame(self):
        """Capture a single frame and track pose."""
        pcd, rgbd = self.capture.get_point_cloud()
        pcd_clean = self.processor.clean(pcd)
        self.processor.estimate_normals(pcd_clean)

        if len(self.frames) == 0:
            # First frame: identity pose
            transform = np.identity(4)
        else:
            # Register against previous frame
            prev_pcd = self.frames[-1][1]
            transform, fitness, rmse = self.processor.register_colored_icp(
                pcd_clean, prev_pcd)

            if fitness < 0.3:
                print(f"WARNING: Low registration fitness ({fitness:.3f}). "
                      f"Frame may be poorly aligned.")

            # Accumulate pose
            transform = self.transforms[-1] @ transform

        self.frames.append((rgbd, pcd_clean))
        self.transforms.append(transform)

        # Integrate into TSDF if available
        if self.volume is not None:
            intrinsic = KinectConfig.get_intrinsic()
            self.volume.integrate(
                rgbd, intrinsic, np.linalg.inv(transform))

        return pcd_clean, transform

    def get_merged_point_cloud(self):
        """Merge all captured point clouds."""
        merged = o3d.geometry.PointCloud()
        for (rgbd, pcd), transform in zip(self.frames, self.transforms):
            tmp = o3d.geometry.PointCloud(pcd)
            tmp.transform(transform)
            merged += tmp

        merged = merged.voxel_down_sample(KinectConfig.VOXEL_SIZE)
        self.processor.estimate_normals(merged)
        return merged

    def get_tsdf_mesh(self):
        """Extract mesh from TSDF volume."""
        if self.volume is None:
            raise RuntimeError("TSDF volume not initialized. Call initialize_tsdf() first.")
        mesh = self.volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        return mesh

    def get_poisson_mesh(self, pcd=None):
        """Reconstruct mesh using Poisson surface reconstruction."""
        if pcd is None:
            pcd = self.get_merged_point_cloud()

        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=KinectConfig.POISSON_DEPTH)

        # Remove low-density vertices
        densities = np.asarray(densities)
        threshold = np.quantile(densities, KinectConfig.DENSITY_QUANTILE)
        mesh.remove_vertices_by_mask(densities < threshold)
        mesh.compute_vertex_normals()

        return mesh

    def export(self, filename, obj=None):
        """
        Export mesh or point cloud.

        Parameters:
            filename: output path (extension determines format: .ply, .obj, .stl, .glb)
            obj: Open3D geometry to export (TriangleMesh or PointCloud).
                 If None, exports TSDF mesh if available, else Poisson mesh.
        """
        if obj is None:
            if self.volume is not None:
                obj = self.get_tsdf_mesh()
            else:
                obj = self.get_poisson_mesh()

        ext = os.path.splitext(filename)[1].lower()

        if ext in ['.ply', '.obj', '.stl', '.glb', '.gltf', '.off']:
            if isinstance(obj, o3d.geometry.PointCloud):
                o3d.io.write_point_cloud(filename, obj)
            else:
                o3d.io.write_triangle_mesh(filename, obj)
            print(f"Exported to {filename}")
        else:
            raise ValueError(f"Unsupported format: {ext}")

    def cleanup(self):
        """Release resources."""
        self.capture.stop()


# ============================================================
# Interactive Scanner Application
# ============================================================

def run_interactive_scanner():
    """Run an interactive 3D scanning session."""

    scanner = KinectScanner()
    scanner.initialize_tsdf()

    print("=" * 60)
    print("Kinect v1 3D Scanner")
    print("=" * 60)
    print("Commands:")
    print("  SPACE  - Capture frame")
    print("  v      - View current point cloud")
    print("  m      - View current mesh (TSDF)")
    print("  p      - View Poisson mesh")
    print("  s      - Save/export")
    print("  q/ESC  - Quit")
    print("=" * 60)

    frame_count = 0

    try:
        while True:
            # Show live preview
            rgb, depth_mm = scanner.capture.get_rgbd()
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # Depth visualization
            depth_vis = np.clip(depth_mm.astype(np.float32) / 4000.0, 0, 1)
            depth_vis = (depth_vis * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

            # Overlay frame count
            cv2.putText(bgr, f"Frames: {frame_count}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('RGB Preview', bgr)
            cv2.imshow('Depth Preview', depth_color)

            key = cv2.waitKey(30) & 0xFF

            if key == ord(' '):
                # Capture frame
                pcd, transform = scanner.capture_frame()
                frame_count += 1
                print(f"Frame {frame_count}: {len(pcd.points)} points")

            elif key == ord('v') and frame_count > 0:
                # Visualize merged point cloud
                merged = scanner.get_merged_point_cloud()
                o3d.visualization.draw_geometries([merged],
                    window_name="Merged Point Cloud")

            elif key == ord('m') and frame_count > 0:
                # Visualize TSDF mesh
                mesh = scanner.get_tsdf_mesh()
                o3d.visualization.draw_geometries([mesh],
                    window_name="TSDF Mesh")

            elif key == ord('p') and frame_count > 0:
                # Visualize Poisson mesh
                mesh = scanner.get_poisson_mesh()
                o3d.visualization.draw_geometries([mesh],
                    window_name="Poisson Mesh")

            elif key == ord('s') and frame_count > 0:
                # Export
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

                # Export point cloud
                pcd = scanner.get_merged_point_cloud()
                scanner.export(f"scan_{timestamp}_cloud.ply", pcd)

                # Export TSDF mesh
                mesh = scanner.get_tsdf_mesh()
                scanner.export(f"scan_{timestamp}_mesh.ply", mesh)
                scanner.export(f"scan_{timestamp}_mesh.obj", mesh)

                print(f"Saved scan_{timestamp}_*.ply and .obj")

            elif key == ord('q') or key == 27:
                break

    finally:
        scanner.cleanup()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    run_interactive_scanner()
```

---

## 11. Sources and References

### Official Repositories and Documentation
- [OpenKinect/libfreenect](https://github.com/OpenKinect/libfreenect) - Official Kinect v1 open-source driver
- [libfreenect Python Wrapper](https://github.com/OpenKinect/libfreenect/tree/master/wrappers/python) - Python bindings source code
- [freenect on PyPI](https://pypi.org/project/freenect/) - Python 3 pip-installable package
- [libfreenect udev rules](https://github.com/OpenKinect/libfreenect/blob/master/platform/linux/udev/51-kinect.rules) - Linux device permission rules

### Open3D Documentation
- [Surface Reconstruction (Poisson, Ball Pivoting)](https://www.open3d.org/docs/release/tutorial/geometry/surface_reconstruction.html)
- [ICP Registration](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html)
- [Colored Point Cloud Registration](https://www.open3d.org/docs/release/tutorial/pipelines/colored_pointcloud_registration.html)
- [Multiway Registration](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html)
- [RGBD Integration (TSDF)](https://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html)
- [TSDF Volume Integration](https://www.open3d.org/docs/release/tutorial/t_reconstruction_system/integration.html)

### Calibration and Technical References
- [Kinect Calibration - Nicolas Burrus / RGBDemo](https://nicolas.burrus.name/oldstuff/kinect_calibration/) - Definitive Kinect v1 calibration parameters
- [Kinect Calibration Study (GMU)](https://cs.gmu.edu/~xzhou10/doc/kinect-study.pdf) - Academic study of calibration methods
- [Getting Kinect Calibration Parameters (ROS Answers)](https://answers.ros.org/question/9331/getting-the-calibration-parameters-of-the-kinect-camera/)
- [Stanford Kinect Sensor Programming](https://graphics.stanford.edu/~mdfisher/Kinect.html)
- [Kinect Wikipedia](https://en.wikipedia.org/wiki/Kinect) - Hardware specifications

### 3D Scanning Projects and Tutorials
- [kinect_3d_dev](https://github.com/janbijster/kinect_3d_dev) - Kinect v1 3D scanning scripts (point clouds, meshes, voxels)
- [kinect_point_cloud](https://github.com/fvilmos/kinect_point_cloud) - Real-time 3D point cloud with PLY export
- [KinectUtil](https://github.com/JasonZhu1313/KinectUtil) - RGB-depth registration and calibration utilities
- [3D Point Cloud Registration (GitHub)](https://github.com/LeafarCoder/3D-point-cloud-registration) - ICP registration with Kinect
- [libfreenect Python Depth Image Tutorial](https://jonnoftw.github.io/2017/01/27/libfreenect-python-depth-image)
- [Install Guide Gist](https://gist.github.com/Collin-Emerson-Miller/8b4630c767aeb4a0b324ea4070c3db9d)

### Additional Resources
- [Kinect v1 vs v2 Depth Data Comparison](http://www.bryancook.net/2014/02/comparing-kinect-v1-and-v2-depth-data.html)
- [Kinect v1 vs v2 Field of View Comparison](https://smeenk.com/kinect-field-of-view-comparison/)
- [RTAB-Map with Xbox 360 Kinect](https://newscrewdriver.com/2019/01/26/xbox-360-kinect-and-rtab-map-handheld-3d-environment-scanning/)
- [Kinect 3D Scanner Tutorial (All3DP)](https://all3dp.com/2/kinect-3d-scanner-easy-beginner-tutorial/)
- [libfreenect depth format options (Issue #515)](https://github.com/OpenKinect/libfreenect/issues/515)
- [Depth registration (Issue #513)](https://github.com/OpenKinect/libfreenect/issues/513)
