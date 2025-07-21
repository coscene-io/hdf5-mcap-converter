#!/usr/bin/env python3
"""
ALOHA Base Action Data Handler
Handles conversion of base action data between HDF5 and MCAP formats.
"""

import numpy as np
from typing import Dict, List, Any, Tuple
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

def extract_to_mcap(dataset, timestamp: float, frame_idx: int, config: Dict) -> List[Any]:
    """
    Extract base action data from HDF5 dataset and convert to MCAP messages.

    Args:
        dataset: HDF5 dataset object (from config['hdf5_path'])
        timestamp: Current timestamp in seconds
        frame_idx: Current frame index
        config: Configuration dictionary with custom parameters

    Returns:
        List of ROS messages to write to MCAP
    """
    try:
        # Check if frame index is valid
        if frame_idx >= dataset.shape[0]:
            return []

        # Get base action data for this frame
        base_action_data = dataset[frame_idx]  # Shape: (2,) - typically [linear_vel, angular_vel]

        # Support custom configuration parameters
        velocity_limit = config.get('velocity_limit')  # Example custom parameter
        if velocity_limit is not None:
            # Clamp velocities to limit
            base_action_data = np.clip(base_action_data, -velocity_limit, velocity_limit)

        # Create Float64MultiArray message
        msg = Float64MultiArray()

        # Set up the layout
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0

        # Define dimensions
        dim = MultiArrayDimension()
        dim.label = "base_action_values"
        dim.size = len(base_action_data)
        dim.stride = len(base_action_data)
        msg.layout.dim = [dim]

        # Set the data
        msg.data = [float(x) for x in base_action_data]

        return [msg]

    except Exception as e:
        print(f"Error extracting base action data at frame {frame_idx}: {e}")
        return []

def extract_to_hdf5(messages: List[Tuple[float, Any]], config: Dict) -> Dict[str, np.ndarray]:
    """
    Extract base action data from MCAP messages and convert to HDF5 format.

    Args:
        messages: List of (timestamp, message) tuples
        config: Configuration dictionary

    Returns:
        Dictionary mapping HDF5 paths to numpy arrays
    """
    if not messages:
        return {}

    try:
        hdf5_path = config.get('hdf5_path', '/base_action')

        base_action_arrays = []

        for timestamp, msg in messages:
            if hasattr(msg, 'data') and hasattr(msg, 'layout'):
                # Float64MultiArray message
                base_action_data = np.array(msg.data)
                base_action_arrays.append(base_action_data)
            else:
                # Fallback: create zero array
                base_action_arrays.append(np.zeros(2))

        if base_action_arrays:
            # Stack into 2D array: (frames, 2)
            result_array = np.stack(base_action_arrays)
            return {hdf5_path: result_array}

        return {}

    except Exception as e:
        print(f"Error converting base action messages to HDF5: {e}")
        return {}

# Helper function for debugging
def get_base_action_info():
    """Return information about the base action data structure"""
    return {
        'description': 'ALOHA robot base action data',
        'hdf5_shape': '(frames, 2)',
        'data_type': 'float64',
        'content': 'Base movement commands [linear_velocity, angular_velocity]',
        'mcap_message_type': 'std_msgs/msg/Float64MultiArray'
    }
