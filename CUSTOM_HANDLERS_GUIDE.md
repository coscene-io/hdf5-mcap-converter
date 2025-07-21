# Custom Handlers Guide

This guide explains how to create custom handlers for processing data between HDF5 and MCAP formats.

## Overview

Custom handlers allow you to process data that doesn't fit the standard joint states,
transforms, or camera patterns. They provide a flexible way to convert custom data types
between HDF5 datasets and ROS messages.

## Handler Interface

### Function Signatures

Custom handlers must implement at least one of the two functions:

#### HDF5 → MCAP Direction

```python
def extract_to_mcap(dataset, timestamp: float, frame_idx: int, config: Dict) -> List[Any]:
    """
    Extract data from HDF5 dataset and convert to MCAP messages.
    
    Args:
        dataset: HDF5 dataset object (from config['hdf5_path'])
        timestamp: Current timestamp in seconds
        frame_idx: Current frame index
        config: Configuration dictionary with custom parameters
    
    Returns:
        List of ROS messages to write to MCAP
    """
```

#### MCAP → HDF5 Direction

```python
def extract_to_hdf5(messages: List[Tuple[float, Any]], config: Dict) -> Dict[str, np.ndarray]:
    """
    Extract data from MCAP messages and convert to HDF5 format.
    
    Args:
        messages: List of (timestamp, message) tuples
        config: Configuration dictionary with custom parameters
    
    Returns:
        Dictionary mapping HDF5 paths to numpy arrays
    """
```

## Key Features

### 1. Direct Dataset Access

Handlers receive the specific dataset directly from `config['hdf5_path']`, providing a clean interface without the need for path lookup in the handler.

### 2. Custom Configuration Parameters

Handlers can now access any custom parameters defined in the configuration:

```yaml
custom_topics:
  - mcap_topic: "/sensor/force_torque"
    hdf5_path: "/observations/force_torque"
    handler_name: "force_torque_handler"
    # Custom parameters
    scaling_factor: 0.001  # Convert mN to N
    offset: 0.0
    filter_outliers: true
    outlier_threshold: 2.5
    sensor_type: "force_torque"
```

Access these in your handler:

```python
def extract_to_mcap(dataset, timestamp, frame_idx, config):
    scaling_factor = config.get('scaling_factor', 1.0)
    offset = config.get('offset', 0.0)
    filter_outliers = config.get('filter_outliers', False)
    # ... use parameters
```

## Configuration

### Basic Configuration

```yaml
custom_topics:
  - mcap_topic: "/my_topic"
    hdf5_path: "/observations/my_data"
    handler_name: "my_handler"  # Will look for my_handler.py
    message_type: "std_msgs/msg/Float64MultiArray"
```

### With Custom Parameters

```yaml
custom_topics:
  - mcap_topic: "/action"
    hdf5_path: "/action"
    handler_name: "aloha_action_handler"
    message_type: "std_msgs/msg/Float64MultiArray"
    # Custom parameters for this handler
    data_scaling: 1.0
    apply_smoothing: true
    smoothing_window: 5
```

### Supported Message Types

The converter supports these ROS message types for custom topics:

- **`std_msgs/msg/Float64MultiArray`** - Multi-dimensional float64 arrays (default)
- **`std_msgs/msg/Float32MultiArray`** - Multi-dimensional float32 arrays  
- **`sensor_msgs/msg/PointCloud2`** - 3D point cloud data with fields
- **`geometry_msgs/msg/Twist`** - Linear and angular velocity commands
- **`geometry_msgs/msg/PoseStamped`** - Position and orientation with timestamp

Your handler must return the appropriate ROS message type based on your configuration.

### Handler Location

Handlers are resolved in this order:

1. If `script_path` is provided: use that exact path
2. If `--input-dir` is specified: look for `{handler_name}.py` in that directory
3. Otherwise: look in `custom_extractors/{handler_name}.py`

## Example Handler

Here's a complete example handler with custom parameters:

```python
#!/usr/bin/env python3
"""
Force-Torque Sensor Handler
Processes 6-DOF force-torque sensor data with custom scaling and filtering.
"""

import numpy as np
from typing import Dict, List, Any, Tuple
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

def extract_to_mcap(dataset, timestamp: float, frame_idx: int, config: Dict) -> List[Any]:
    """Extract force-torque data from HDF5 dataset."""
    try:
        if frame_idx >= dataset.shape[0]:
            return []
        
        # Get 6-DOF force-torque data [fx, fy, fz, tx, ty, tz]
        ft_data = dataset[frame_idx]
        
        # Apply custom parameters
        force_scaling = config.get('force_scaling', 1.0)
        torque_scaling = config.get('torque_scaling', 1.0)
        force_offset = config.get('force_offset', [0.0, 0.0, 0.0])
        torque_offset = config.get('torque_offset', [0.0, 0.0, 0.0])
        
        # Scale and offset forces (first 3 elements)
        ft_data[:3] = ft_data[:3] * force_scaling + force_offset
        # Scale and offset torques (last 3 elements)
        ft_data[3:] = ft_data[3:] * torque_scaling + torque_offset
        
        # Create ROS message
        msg = Float64MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0
        
        dim = MultiArrayDimension()
        dim.label = "force_torque_6dof"
        dim.size = 6
        dim.stride = 6
        msg.layout.dim = [dim]
        
        msg.data = [float(x) for x in ft_data]
        
        return [msg]
        
    except Exception as e:
        print(f"Error extracting force-torque data at frame {frame_idx}: {e}")
        return []

def extract_to_hdf5(messages: List[Tuple[float, Any]], config: Dict) -> Dict[str, np.ndarray]:
    """Extract force-torque data from MCAP messages."""
    if not messages:
        return {}
    
    try:
        hdf5_path = config.get('hdf5_path', '/observations/force_torque')
        
        # Get reverse transformation parameters
        force_scaling = config.get('force_scaling', 1.0)
        torque_scaling = config.get('torque_scaling', 1.0)
        force_offset = config.get('force_offset', [0.0, 0.0, 0.0])
        torque_offset = config.get('torque_offset', [0.0, 0.0, 0.0])
        
        ft_arrays = []
        
        for timestamp, msg in messages:
            if hasattr(msg, 'data'):
                ft_data = np.array(msg.data)
                
                # Apply reverse transformation
                ft_data[:3] = (ft_data[:3] - force_offset) / force_scaling
                ft_data[3:] = (ft_data[3:] - torque_offset) / torque_scaling
                
                ft_arrays.append(ft_data)
            else:
                ft_arrays.append(np.zeros(6))
        
        if ft_arrays:
            result_array = np.stack(ft_arrays)
            return {hdf5_path: result_array}
        
        return {}
        
    except Exception as e:
        print(f"Error converting force-torque messages to HDF5: {e}")
        return {}
```

## Best Practices

### 1. Error Handling

Always wrap your handler logic in try-catch blocks and return empty results on error:

```python
def extract_to_mcap(dataset, timestamp, frame_idx, config):
    try:
        # Your logic here
        return [msg]
    except Exception as e:
        print(f"Error in handler at frame {frame_idx}: {e}")
        return []
```

### 2. Parameter Validation

Validate and provide defaults for custom parameters:

```python
scaling_factor = config.get('scaling_factor', 1.0)
if scaling_factor <= 0:
    print("Warning: Invalid scaling_factor, using 1.0")
    scaling_factor = 1.0
```

### 3. Frame Index Validation

Always check if the frame index is valid:

```python
if frame_idx >= dataset.shape[0]:
    return []
```

### 4. Bidirectional Consistency

Ensure your HDF5→MCAP and MCAP→HDF5 functions are inverses of each other when using
the same parameters.

## Testing Your Handler

1. Create a test configuration with your handler
2. Run a round-trip conversion: HDF5 → MCAP → HDF5
3. Compare the original and final HDF5 files
4. Test with various custom parameter values
