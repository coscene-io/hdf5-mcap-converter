#!/usr/bin/env python3
"""
ALOHA Action Data Handler
Handles conversion of action data between HDF5 and MCAP formats.
"""

import numpy as np
from typing import Dict, List, Any, Tuple
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

def extract_to_mcap(dataset, timestamp: float, frame_idx: int, config: Dict) -> List[Any]:
    """
    Extract action data from HDF5 dataset and convert to MCAP messages.

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

        # Get action data for this frame
        action_data = dataset[frame_idx]  # Shape: (14,)

        # Support custom configuration parameters
        data_scaling = config.get('data_scaling', 1.0)  # Example custom parameter
        if data_scaling != 1.0:
            action_data = action_data * data_scaling

        # Create Float64MultiArray message
        msg = Float64MultiArray()

        # Set up the layout
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0

        # Define dimensions
        dim = MultiArrayDimension()
        dim.label = "action_values"
        dim.size = len(action_data)
        dim.stride = len(action_data)
        msg.layout.dim = [dim]

        # Set the data
        msg.data = [float(x) for x in action_data]

        return [msg]

    except Exception as e:
        print(f"Error extracting action data at frame {frame_idx}: {e}")
        return []

def extract_to_hdf5(messages: List[Tuple[float, Any]], config: Dict) -> Dict[str, np.ndarray]:
    """
    Extract action data from MCAP messages and convert to HDF5 format.

    Args:
        messages: List of (timestamp, message) tuples
        config: Configuration dictionary

    Returns:
        Dictionary mapping HDF5 paths to numpy arrays
    """
    if not messages:
        return {}

    try:
        hdf5_path = config.get('hdf5_path', '/action')

        action_arrays = []

        for timestamp, msg in messages:
            if hasattr(msg, 'data') and hasattr(msg, 'layout'):
                # Float64MultiArray message
                action_data = np.array(msg.data)
                action_arrays.append(action_data)
            else:
                # Fallback: create zero array
                action_arrays.append(np.zeros(14))

        if action_arrays:
            # Stack into 2D array: (frames, 14)
            result_array = np.stack(action_arrays)
            return {hdf5_path: result_array}

        return {}

    except Exception as e:
        print(f"Error converting action messages to HDF5: {e}")
        return {}

# Helper function for debugging
def get_action_info():
    """Return information about the action data structure"""
    return {
        'description': 'ALOHA robot action data',
        'hdf5_shape': '(frames, 14)',
        'data_type': 'float64',
        'content': 'Robot action commands for dual arm system',
        'mcap_message_type': 'std_msgs/msg/Float64MultiArray'
    }
