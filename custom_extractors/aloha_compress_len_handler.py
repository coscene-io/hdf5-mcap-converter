#!/usr/bin/env python3
"""
ALOHA Compress Length Data Handler
Handles conversion of compress_len data between HDF5 and MCAP formats.
"""

import numpy as np
from typing import Dict, List, Any, Tuple
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

def extract_to_mcap(dataset, timestamp: float, frame_idx: int, config: Dict) -> List[Any]:
    """
    Extract compress_len data from HDF5 dataset and convert to MCAP messages.

    Args:
        dataset: HDF5 dataset object (from config['hdf5_path'])
        timestamp: Current timestamp in seconds
        frame_idx: Current frame index
        config: Configuration dictionary with custom parameters

    Returns:
        List of ROS messages to write to MCAP
    """
    try:
        # The compress_len dataset has shape (3, 1500) - 3 compression values per frame
        # Check if frame index is valid for the second dimension
        if frame_idx >= dataset.shape[1]:
            return []

        # Get compress_len data for this frame (all 3 values)
        # Extract column frame_idx from the (3, 1500) array
        compress_len_data = dataset[:, frame_idx]  # Shape: (3,)

        # Support custom configuration parameters
        min_value = config.get('min_compression_length', 0.0)

        # Apply minimum value constraint to all values
        compress_len_data = np.maximum(compress_len_data, min_value)

        # Create Float64MultiArray message
        msg = Float64MultiArray()

        # Set up the layout
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0

        # Define dimensions
        dim = MultiArrayDimension()
        dim.label = "compress_len_values"
        dim.size = len(compress_len_data)
        dim.stride = len(compress_len_data)
        msg.layout.dim = [dim]

        # Set the data
        msg.data = [float(x) for x in compress_len_data]

        return [msg]

    except Exception as e:
        print(f"Error extracting compress_len data at frame {frame_idx}: {e}")
        return []

def extract_to_hdf5(messages: List[Tuple[float, Any]], config: Dict) -> Dict[str, np.ndarray]:
    """
    Extract compress_len data from MCAP messages and convert to HDF5 format.

    Args:
        messages: List of (timestamp, message) tuples
        config: Configuration dictionary

    Returns:
        Dictionary mapping HDF5 paths to numpy arrays
    """
    if not messages:
        return {}

    try:
        hdf5_path = config.get('hdf5_path', '/compress_len')

        compress_len_arrays = []

        for timestamp, msg in messages:
            if hasattr(msg, 'data') and hasattr(msg, 'layout'):
                # Float64MultiArray message containing 3 compression values
                compress_len_data = np.array(msg.data)
                compress_len_arrays.append(compress_len_data)
            else:
                # Fallback: create zero values for 3 cameras
                compress_len_arrays.append(np.array([0.0, 0.0, 0.0]))

        if compress_len_arrays:
            # Stack arrays and transpose to get shape (3, num_frames)
            # Each message contains [val1, val2, val3] for the 3 cameras
            result_array = np.array(compress_len_arrays)  # Shape: (num_frames, 3)
            result_array = result_array.T  # Transpose to (3, num_frames)
            return {hdf5_path: result_array}

        return {}

    except Exception as e:
        print(f"Error converting compress_len messages to HDF5: {e}")
        return {}

# Helper function for debugging
def get_compress_len_info():
    """Return information about the compress_len data structure"""
    return {
        'description': 'ALOHA compression length data',
        'hdf5_shape': '(3, frames)',
        'data_type': 'float64',
        'content': 'Compression length values for 3 cameras per frame',
        'mcap_message_type': 'std_msgs/msg/Float64MultiArray'
    }
