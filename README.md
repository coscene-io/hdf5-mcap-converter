# HDF5 ↔ MCAP Converter

Convert robotics datasets between HDF5 and MCAP formats with a single configuration file.
Perfect for ROS2 workflows and robot learning datasets.

## Quick Start

### 1. HDF5 → MCAP Conversion

```bash
./script/run_converter.sh hdf5-to-mcap \
    -c config_aloha_unified.yaml \
    -f episode_0_aloha.hdf5 \
    -u dual_vx300s.urdf \
    -m output.mcap
```

### 2. MCAP → HDF5 Conversion

```bash
./script/run_converter.sh mcap-to-hdf5 \
    -m output.mcap \
    -c config_aloha_unified.yaml \
    -u dual_vx300s.urdf \
    -f converted.hdf5
```

## What You Get

- **Joint States**: Robot joint positions, velocities, efforts → `/joint_states` topic
- **Transforms**: Complete robot TF tree → `/tf` topic
- **Camera Data**: Images with proper timestamps → `/camera/*/image_compressed`
- **Custom Data**: Your dataset-specific data → custom topics
- **Robot Model**: URDF embedded in MCAP → `/robot_description`

## Configuration

Create a single YAML file that works for both conversion directions:

```yaml
# config_my_robot.yaml
sampling_rate: 20.0  # Hz

# Map HDF5 joint data to URDF joints
joint_states:
  hdf5_paths:
    position: "/observations/qpos"
    velocity: "/observations/qvel"
  mcap_topic: "/joint_states"
  
  joint_mappings:
    "arm_joint_1": 0  # URDF joint name → HDF5 column index
    "arm_joint_2": 1
    "arm_joint_3": 2
    # ... add all your joints

# Camera data
cameras:
  - hdf5_path: "/observations/images/front_camera"
    mcap_topic: "/camera/front/image_compressed"
    frame_id: "camera_link"
    format: "jpeg"

# Gripper with value mapping
gripper:
  - urdf_joint_name: "gripper_finger"
    hdf5_path: "/observations/qpos"
    hdf5_index: 6
    value_mapping:
      hdf5_range: [0.0, 1.0]      # Your HDF5 values
      urdf_range: [0.0, 0.04]     # URDF joint limits

# Custom data (optional)
custom_topics:
  - mcap_topic: "/robot_actions"
    hdf5_path: "/actions"
    handler_name: "my_action_handler"
    message_type: "std_msgs/msg/Float64MultiArray"  # See supported types below
```

## Custom Handlers

For dataset-specific data, create Python handlers.

### Supported Message Types

- `std_msgs/msg/Float64MultiArray` - Multi-dimensional float64 arrays (default)
- `std_msgs/msg/Float32MultiArray` - Multi-dimensional float32 arrays
- `sensor_msgs/msg/PointCloud2` - 3D point cloud data
- `geometry_msgs/msg/Twist` - Linear and angular velocity
- `geometry_msgs/msg/PoseStamped` - Position and orientation with timestamp

### 1. Create Handler File

```python
# custom_extractors/my_action_handler.py
import numpy as np
from typing import Dict, List, Any, Tuple
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

def extract_to_mcap(dataset, timestamp: float, frame_idx: int, config: Dict) -> List[Any]:
    """Convert HDF5 data to ROS messages."""
    if frame_idx >= dataset.shape[0]:
        return []
    
    # Get your data
    action_data = dataset[frame_idx]
    
    # Create ROS message
    msg = Float64MultiArray()
    msg.layout = MultiArrayLayout()
    msg.layout.data_offset = 0
    
    dim = MultiArrayDimension()
    dim.label = "actions"
    dim.size = len(action_data)
    dim.stride = len(action_data)
    msg.layout.dim = [dim]
    
    msg.data = [float(x) for x in action_data]
    return [msg]

def extract_to_hdf5(messages: List[Tuple[float, Any]], config: Dict) -> Dict[str, np.ndarray]:
    """Convert ROS messages back to HDF5 data."""
    if not messages:
        return {}
    
    hdf5_path = config.get('hdf5_path', '/actions')
    action_arrays = []
    
    for timestamp, msg in messages:
        if hasattr(msg, 'data'):
            action_arrays.append(np.array(msg.data))
        else:
            action_arrays.append(np.zeros(7))  # Your action dimension
    
    if action_arrays:
        return {hdf5_path: np.stack(action_arrays)}
    return {}
```

### 2. Add Custom Parameters (Optional)

```yaml
custom_topics:
  - mcap_topic: "/robot_actions"
    hdf5_path: "/actions"
    handler_name: "my_action_handler"
    # Custom parameters for your handler
    scaling_factor: 0.1
    apply_smoothing: true
    max_velocity: 2.0
```

Access in your handler:

```python
def extract_to_mcap(dataset, timestamp, frame_idx, config):
    scaling = config.get('scaling_factor', 1.0)
    max_vel = config.get('max_velocity', 1.0)
    # Use your parameters...
```

## File Organization

```
your_project/
├── config/
│   └── config_my_robot.yaml     # Your configuration
├── custom_extractors/            # Your custom handlers (optional)
│   └── my_action_handler.py
├── hdf5/
│   └── my_dataset.hdf5          # Your HDF5 files
├── urdf/
│   └── my_robot.urdf            # Your robot model
└── output/
    └── converted.mcap           # Generated MCAP files
```

## Command Options

The converter uses a unified entry point script that automatically selects the appropriate conversion tool:

```bash
./script/run_converter.sh <direction> [OPTIONS]
```

### Conversion Directions

- `hdf5-to-mcap` - Convert HDF5 files to MCAP format
- `mcap-to-hdf5` - Convert MCAP files to HDF5 format

### Execution Modes

- **Default (Docker)**: Runs in pre-orchestrated containers for workflow integration
- **Standalone**: Builds and runs Docker containers locally for development

```bash
# Docker mode (default) - for workflow integration
./script/run_converter.sh hdf5-to-mcap -c config.yaml -f data.hdf5 -u robot.urdf -m output.mcap

# Standalone mode - for development and testing
./script/run_converter.sh hdf5-to-mcap --standalone -c config.yaml -f data.hdf5 -u robot.urdf -m output.mcap
```

### HDF5 → MCAP Options

```bash
./script/run_converter.sh hdf5-to-mcap [OPTIONS]

Required:
  -c, --config <file>      Configuration YAML file
  -f, --hdf5 <file>        Input HDF5 file(s)
  -u, --urdf <file>        Robot URDF file
  -m, --output-file <file> Output MCAP filename

Optional:
  -i, --input-dir <dir>    Directory containing input files
  -o, --output-dir <dir>   Directory for output files
  --custom-handlers        Specify custom handlers
  --standalone             Use standalone Docker mode
  --debug                  Enable debug output
  --log-to-file           Save logs to file
```

### MCAP → HDF5 Options

```bash
./script/run_converter.sh mcap-to-hdf5 [OPTIONS]

Required:
  -m, --mcap-file <file>   Input MCAP file
  -c, --config <file>      Configuration YAML file
  -u, --urdf <file>        Robot URDF file
  -f, --hdf5-file <file>   Output HDF5 filename

Optional:
  -i, --input-dir <dir>    Directory containing input files
  -o, --output-dir <dir>   Directory for output files
  --custom-handlers        Specify custom handlers
  --standalone             Use standalone Docker mode
  --debug                  Enable debug output
  --log-to-file           Save logs to file
```

## Examples Included

This repository includes working examples:

- **Configuration**: `config/config_aloha_unified.yaml` - Dual-arm robot setup
- **Dataset**: `hdf5/episode_0_aloha.hdf5` - Real robot data (1500 frames)
- **Robot Model**: `urdf/dual_vx300s.urdf` - Dual ViperX 300s arms
- **Custom Handlers**: `custom_extractors/aloha_*_handler.py` - Action and sensor data

Try them:

```bash
# Convert included example
./script/run_converter.sh hdf5-to-mcap \
    -c config/config_aloha_unified.yaml \
    -f hdf5/episode_0_aloha.hdf5 \
    -u urdf/dual_vx300s.urdf \
    -m test_output.mcap

# Convert back
./script/run_converter.sh mcap-to-hdf5 \
    -m test_output.mcap \
    -c config/config_aloha_unified.yaml \
    -u urdf/dual_vx300s.urdf \
    -f test_output.hdf5
```

## Requirements

- **Docker**: All conversions run in Docker containers
- **Linux/macOS**: Tested on Ubuntu 20.04+ and macOS

No Python installation needed on host - everything runs in Docker!

## Troubleshooting

### Common Issues

1. **"Handler not found"**: Make sure your handler file is in `custom_extractors/`
   or use `--input-dir`
2. **"Dataset not found"**: Check your `hdf5_path` in the configuration
3. **"Joint mapping error"**: Verify joint names match your URDF exactly
4. **"Docker build failed"**: Ensure Docker is running and you have internet access

### Debug Mode

```bash
# Add --debug for detailed output
./script/run_converter.sh hdf5-to-mcap --debug [other options]
./script/run_converter.sh mcap-to-hdf5 --debug [other options]
```

### Log Files

```bash
# Add --log-to-file to save logs
./script/run_converter.sh hdf5-to-mcap --log-to-file [other options]
./script/run_converter.sh mcap-to-hdf5 --log-to-file [other options]
```

## Advanced Usage

### Multiple Files

```bash
# Process multiple HDF5 files
-f "episode_*.hdf5"              # Glob pattern
-f "file1.hdf5,file2.hdf5"       # Comma-separated
```

### Custom Directories

```bash
# Use custom input/output directories
-i /path/to/inputs -o /path/to/outputs
```

### Different Sampling Rates

```yaml
# In your config file
sampling_rate: 20.0  # Default

cameras:
  - hdf5_path: "/images/camera1"
    sampling_rate: 30.0  # Override for this camera
```

## Getting Help

```bash
# Show general help
./script/run_converter.sh --help

# Show direction-specific help
./script/run_converter.sh hdf5-to-mcap --help
./script/run_converter.sh mcap-to-hdf5 --help
```

For detailed custom handler documentation, see [`CUSTOM_HANDLERS_GUIDE.md`](CUSTOM_HANDLERS_GUIDE.md).