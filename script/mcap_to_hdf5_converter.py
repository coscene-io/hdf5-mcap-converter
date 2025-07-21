#!/usr/bin/env python3

import os
import h5py
import numpy as np
import time
from datetime import datetime
import argparse
import yaml
import json
import importlib.util
import sys
from typing import Dict, List, Any, Optional, Callable, Tuple
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState, Image, CompressedImage, PointCloud2
from std_msgs.msg import Header, Float64MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist, PoseStamped
import traceback
from io import BytesIO
from PIL import Image as PILImage
from config_utils import load_unified_config, UnifiedConfig, ConfigurationError

# Parse arguments
parser = argparse.ArgumentParser(description='Convert MCAP files to HDF5 format')
parser.add_argument('--mcap-file', type=str, required=True, help='Path to input MCAP file')
parser.add_argument('--hdf5-file', type=str, required=True, help='Output HDF5 file path')
parser.add_argument('--config-file', type=str, required=True, help='Path to YAML configuration file')
parser.add_argument('--urdf-file', type=str, required=True, help='Path to robot URDF file')
parser.add_argument('--debug', action='store_true', help='Enable debug output')
parser.add_argument('--input-dir', type=str, help='Base input directory (inside container, e.g., /mnt_input)')
parser.add_argument('--output-dir', type=str, help='Output directory for HDF5 files (inside container, e.g., /mnt_output)')
parser.add_argument('--custom-handlers', type=str, help='Custom handler scripts (comma-separated list or glob pattern, e.g., "handler1.py,handler2.py" or "*.py")')
args = parser.parse_args()

# Global set to track which handler warnings have been printed
_warned_missing_handlers = set()

# Create output directory if it doesn't exist
output_dir = os.path.dirname(args.hdf5_file)
if output_dir:  # Only create directory if there is a directory path
    os.makedirs(output_dir, exist_ok=True)

# Initialize ROS2
rclpy.init()

def resolve_custom_handler_path(handler_name: str = None, script_path: str = None, input_dir: str = None, custom_handlers: str = None) -> Optional[str]:
    """Resolve the path to a custom handler script"""
    import glob

    # Priority 1: If script_path is provided (backward compatibility), use it
    if script_path:
        if os.path.exists(script_path):
            return script_path
        return None

    # Priority 2: If handler_name is provided, resolve using custom_handlers or defaults
    if handler_name:
        # Ensure handler_name has .py extension
        if not handler_name.endswith('.py'):
            handler_name = f"{handler_name}.py"

        # If custom_handlers is specified, search in those files
        if custom_handlers:
            handler_files = []

            # Parse custom_handlers (comma-separated or glob)
            if ',' in custom_handlers:
                # Comma-separated list
                handler_specs = [spec.strip() for spec in custom_handlers.split(',')]
            else:
                # Single spec (could be glob)
                handler_specs = [custom_handlers.strip()]

            # Resolve each spec
            for spec in handler_specs:
                # Only join with input_dir if spec doesn't already start with ../ or /
                if input_dir and not os.path.isabs(spec) and not spec.startswith('../'):
                    # Make relative to input_dir
                    spec = os.path.join(input_dir, spec)

                # Handle glob patterns
                if '*' in spec or '?' in spec:
                    expanded_files = glob.glob(spec)
                    handler_files.extend(expanded_files)
                else:
                    if os.path.exists(spec):
                        handler_files.append(spec)

            # Look for our specific handler in the resolved files
            for handler_file in handler_files:
                if os.path.basename(handler_file) == handler_name:
                    return handler_file

            return None

        # If no custom_handlers, use default locations
        search_paths = []

        if input_dir:
            # Default: look in input_dir/custom_extractors/
            search_paths.append(os.path.join(input_dir, "custom_extractors", handler_name))

        # Legacy fallback
        search_paths.append(os.path.join("custom_extractors", handler_name))

        for handler_path in search_paths:
            if os.path.exists(handler_path):
                return handler_path

        return None

    return None

def load_custom_schema_handler(script_path: str, handler_name: str) -> Optional[Callable]:
    """Load a custom schema handler from a Python script"""
    try:
        spec = importlib.util.spec_from_file_location("custom_handler", script_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return getattr(module, handler_name)
    except Exception as e:
        print(f"Error loading custom schema handler from {script_path}: {e}")
        return None

def ros_time_to_seconds(ros_time: RosTime) -> float:
    """Convert ROS Time to seconds as float"""
    return ros_time.sec + ros_time.nanosec / 1_000_000_000.0

def extract_joint_states_data(messages: List[Tuple[float, JointState]], config: Dict, unified_config: Optional[UnifiedConfig] = None) -> Dict[str, np.ndarray]:
    """Extract joint state data and organize according to configuration"""
    if not messages:
        return {}

    # Get joint ordering from unified config
    if unified_config:
        try:
            joint_order = unified_config.get_joint_order()
        except Exception:
            # Fallback to legacy method
            joint_order = config.get('joint_order', [])
            if not joint_order:
                # Get all unique joint names across all messages
                all_joint_names = set()
                for _, msg in messages:
                    all_joint_names.update(msg.name)
                joint_order = sorted(list(all_joint_names))
    else:
        # Legacy method
        joint_order = config.get('joint_order', [])
        if not joint_order:
            # Get all unique joint names across all messages
            all_joint_names = set()
            for _, msg in messages:
                all_joint_names.update(msg.name)
            joint_order = sorted(list(all_joint_names))

    # Insert gripper joints at their correct positions if they're configured
    if unified_config:
        gripper_mappers = unified_config.get_gripper_mappers()
        gripper_configs = unified_config.config.get('gripper', [])

        # Sort gripper configs by hdf5_index to insert in correct order
        sorted_gripper_configs = sorted(gripper_configs, key=lambda x: x.get('hdf5_index', 999))

        for gripper_config in sorted_gripper_configs:
            joint_name = gripper_config.get('urdf_joint_name')
            hdf5_index = gripper_config.get('hdf5_index')

            if joint_name and hdf5_index is not None:
                if joint_name not in joint_order:
                    # Insert at the correct position
                    if hdf5_index < len(joint_order):
                        joint_order.insert(hdf5_index, joint_name)
                    else:
                        joint_order.append(joint_name)
                    print(f"Inserted gripper joint '{joint_name}' at position {hdf5_index}")
                else:
                    # Move to correct position if already exists
                    current_idx = joint_order.index(joint_name)
                    if current_idx != hdf5_index:
                        joint_order.pop(current_idx)
                        if hdf5_index < len(joint_order):
                            joint_order.insert(hdf5_index, joint_name)
                        else:
                            joint_order.append(joint_name)
                        print(f"Moved gripper joint '{joint_name}' from position {current_idx} to {hdf5_index}")

    print(f"Joint order for HDF5 output: {joint_order}")
    print(f"Total joints: {len(joint_order)}")

    num_frames = len(messages)
    num_joints = len(joint_order)

    # Initialize arrays
    positions = np.zeros((num_frames, num_joints))
    velocities = np.zeros((num_frames, num_joints))
    efforts = np.zeros((num_frames, num_joints))

    # Fill arrays according to joint order
    for frame_idx, (timestamp, msg) in enumerate(messages):
        for joint_idx, joint_name in enumerate(joint_order):
            if joint_name in msg.name:
                msg_joint_idx = msg.name.index(joint_name)

                # Position
                if msg_joint_idx < len(msg.position):
                    positions[frame_idx, joint_idx] = msg.position[msg_joint_idx]

                # Velocity
                if msg_joint_idx < len(msg.velocity):
                    velocities[frame_idx, joint_idx] = msg.velocity[msg_joint_idx]

                # Effort
                if msg_joint_idx < len(msg.effort):
                    efforts[frame_idx, joint_idx] = msg.effort[msg_joint_idx]

    # Apply gripper transformations using unified configuration
    gripper_transformations = []  # Initialize for legacy compatibility check
    if unified_config:
        # Use unified gripper mappers
        gripper_mappers = unified_config.get_gripper_mappers()
        for joint_name, (value_mapper, gripper_config) in gripper_mappers.items():
            if joint_name in joint_order:
                joint_idx = joint_order.index(joint_name)

                # Apply URDF → HDF5 conversion for all frames
                for frame_idx in range(num_frames):
                    # Transform position
                    urdf_value = positions[frame_idx, joint_idx]
                    hdf5_value = value_mapper.urdf_to_hdf5(urdf_value)
                    positions[frame_idx, joint_idx] = hdf5_value





                print(f"Applied unified gripper transformation for {joint_name} (position only; velocity and effort unchanged)")
    else:
        # Legacy gripper transformations for backward compatibility
        gripper_transformations = config.get('gripper_transformations', [])
        for gripper_config in gripper_transformations:
            joint_name = gripper_config.get('joint_name')
            column_index = gripper_config.get('column_index')
            value_type = gripper_config.get('value_type', 'direct')

            if joint_name and column_index is not None and joint_name in joint_order:
                joint_idx = joint_order.index(joint_name)

                if value_type == "reverse_normalized":
                    # Convert URDF angle back to original HDF5 range
                    urdf_min = gripper_config.get('urdf_min_angle', 0.021)
                    urdf_max = gripper_config.get('urdf_max_angle', 0.057)
                    hdf5_min = gripper_config.get('hdf5_min_value', -1.0)
                    hdf5_max = gripper_config.get('hdf5_max_value', 1.0)

                    for frame_idx in range(num_frames):
                        # Transform position
                        urdf_angle = positions[frame_idx, joint_idx]

                        # Normalize URDF angle to 0-1 range
                        if urdf_max != urdf_min:
                            normalized = (urdf_angle - urdf_min) / (urdf_max - urdf_min)
                        else:
                            normalized = 0.0
                        normalized = max(0.0, min(1.0, normalized))

                        # Scale to original HDF5 range
                        hdf5_value = hdf5_min + normalized * (hdf5_max - hdf5_min)
                        positions[frame_idx, joint_idx] = hdf5_value



                    print(f"Applied reverse gripper transformation for {joint_name} at column {column_index} (position only; velocity and effort unchanged)")
                elif value_type != "direct":
                    print(f"Warning: Unknown gripper transformation type '{value_type}' for {joint_name}")

    # Ensure positions array matches expected column order
    if gripper_transformations:
        print(f"Joint order: {joint_order}")
        print(f"Position array shape: {positions.shape}")
        if args.debug:
            print(f"Sample gripper values after transformation:")
            for gripper_config in gripper_transformations:
                joint_name = gripper_config.get('joint_name')
                if joint_name in joint_order:
                    joint_idx = joint_order.index(joint_name)
                    print(f"  {joint_name} (col {joint_idx}): {positions[0, joint_idx]:.6f}")

    result = {}

    # Map according to configuration
    joint_mapping = config.get('joint_mapping', {})
    if joint_mapping:
        # Use explicit mapping (for complex configurations)
        for hdf5_path, joint_spec in joint_mapping.items():
            if isinstance(joint_spec, str):
                # Simple joint name mapping
                if joint_spec in joint_order:
                    joint_idx = joint_order.index(joint_spec)
                    result[hdf5_path] = positions[:, joint_idx:joint_idx+1]
            elif isinstance(joint_spec, dict):
                # Complex mapping with multiple joints or data types
                joint_names = joint_spec.get('joints', [])
                data_type = joint_spec.get('data_type', 'position')

                if joint_names:
                    indices = [joint_order.index(name) for name in joint_names if name in joint_order]
                    if indices:
                        if data_type == 'position':
                            result[hdf5_path] = positions[:, indices]
                        elif data_type == 'velocity':
                            result[hdf5_path] = velocities[:, indices]
                        elif data_type == 'effort':
                            result[hdf5_path] = efforts[:, indices]
    else:
        # Simple mapping using hdf5_paths (mirrors existing converter format)
        hdf5_paths = config.get('hdf5_paths', {})
        if 'position' in hdf5_paths:
            result[hdf5_paths['position']] = positions
        if 'velocity' in hdf5_paths:
            result[hdf5_paths['velocity']] = velocities
        if 'effort' in hdf5_paths:
            result[hdf5_paths['effort']] = efforts

    return result

def extract_tf_data(messages: List[Tuple[float, TFMessage]], config: Dict) -> Dict[str, np.ndarray]:
    """Extract transform data and organize according to configuration"""
    if not messages:
        return {}

    result = {}

    # Handle both old format (tf_mapping) and new format (extractions)
    extractions = config.get('extractions', [])
    if not extractions:
        # Fallback to old format for compatibility
        tf_mapping = config.get('tf_mapping', {})
        extractions = [{'hdf5_path': path, **spec} for path, spec in tf_mapping.items()]

    for extraction in extractions:
        hdf5_path = extraction.get('hdf5_path')
        parent_frame = extraction.get('parent_frame')
        child_frame = extraction.get('child_frame')
        data_type = extraction.get('data_type', 'pose')  # pose, translation, rotation, etc.

        if not hdf5_path or not parent_frame or not child_frame:
            continue

        # Extract transforms for this frame pair
        transforms_data = []
        for timestamp, tf_msg in messages:
            found_transform = None
            for transform in tf_msg.transforms:
                if (transform.header.frame_id == parent_frame and
                    transform.child_frame_id == child_frame):
                    found_transform = transform
                    break

            if found_transform:
                if data_type == 'pose':
                    # 7D pose: [x, y, z, qx, qy, qz, qw]
                    pose = [
                        found_transform.transform.translation.x,
                        found_transform.transform.translation.y,
                        found_transform.transform.translation.z,
                        found_transform.transform.rotation.x,
                        found_transform.transform.rotation.y,
                        found_transform.transform.rotation.z,
                        found_transform.transform.rotation.w
                    ]
                    transforms_data.append(pose)
                elif data_type == 'translation':
                    # 3D translation: [x, y, z]
                    translation = [
                        found_transform.transform.translation.x,
                        found_transform.transform.translation.y,
                        found_transform.transform.translation.z
                    ]
                    transforms_data.append(translation)
                elif data_type == 'rotation':
                    # 4D quaternion: [x, y, z, w]
                    rotation = [
                        found_transform.transform.rotation.x,
                        found_transform.transform.rotation.y,
                        found_transform.transform.rotation.z,
                        found_transform.transform.rotation.w
                    ]
                    transforms_data.append(rotation)
            else:
                # No transform found, use zeros
                if data_type == 'pose':
                    transforms_data.append([0.0] * 7)
                elif data_type == 'translation':
                    transforms_data.append([0.0] * 3)
                elif data_type == 'rotation':
                    transforms_data.append([0.0, 0.0, 0.0, 1.0])  # Identity quaternion

        if transforms_data:
            result[hdf5_path] = np.array(transforms_data)

    return result

def extract_image_data(messages: List[Tuple[float, Any]], config: Dict) -> Dict[str, np.ndarray]:
    """Extract image data and organize according to configuration"""
    if not messages:
        return {}

    result = {}
    hdf5_path = config.get('hdf5_path')
    format_type = config.get('format', 'jpeg')  # Default to jpeg for ALOHA

    if not hdf5_path:
        return result

    images_data = []
    successful_extractions = 0

    for timestamp, msg in messages:
        try:
            if hasattr(msg, 'data') and hasattr(msg, 'format'):
                # CompressedImage (this is what we have from MCAP)
                if format_type == 'jpeg' or format_type == 'compressed':
                    # For ALOHA format, store compressed JPEG bytes to match original HDF5 structure
                    compressed_bytes = np.frombuffer(msg.data, dtype=np.uint8)
                    images_data.append(compressed_bytes)
                    successful_extractions += 1
                elif format_type == 'raw_compressed':
                    # Store compressed bytes directly
                    compressed_bytes = np.frombuffer(msg.data, dtype=np.uint8)
                    images_data.append(compressed_bytes)
                    successful_extractions += 1
            elif hasattr(msg, 'data') and hasattr(msg, 'encoding'):
                # Raw Image
                height, width = msg.height, msg.width
                if msg.encoding == 'rgb8':
                    rgb_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                    images_data.append(rgb_array)
                    successful_extractions += 1
                elif msg.encoding == 'bgr8':
                    bgr_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                    rgb_array = bgr_array[:, :, ::-1]  # BGR to RGB
                    images_data.append(rgb_array)
                    successful_extractions += 1
        except Exception as e:
            print(f"Error processing image at timestamp {timestamp}: {e}")
            if args.debug:
                traceback.print_exc()

    if images_data:
        if format_type in ['jpeg', 'compressed']:
            # For JPEG/compressed format, handle variable-length arrays like original HDF5
            max_length = max(len(img) for img in images_data) if images_data else 0
            if max_length > 0:
                # Pad all images to the same length (like original HDF5 format)
                padded_images = []
                for img in images_data:
                    if len(img) < max_length:
                        padded = np.pad(img, (0, max_length - len(img)), mode='constant', constant_values=0)
                        padded_images.append(padded)
                    else:
                        padded_images.append(img)
                result[hdf5_path] = np.stack(padded_images)
            print(f"    Successfully extracted {successful_extractions}/{len(messages)} images as compressed bytes")
        elif format_type in ['rgb', 'bgr']:
            # Stack as 4D array: (frames, height, width, channels)
            result[hdf5_path] = np.stack(images_data)
            print(f"    Successfully extracted {successful_extractions}/{len(messages)} images as RGB arrays")
        elif format_type == 'raw_compressed':
            # For compressed images, handle variable-length arrays
            max_length = max(len(img) for img in images_data) if images_data else 0
            if max_length > 0:
                # Pad all images to the same length
                padded_images = []
                for img in images_data:
                    if len(img) < max_length:
                        padded = np.pad(img, (0, max_length - len(img)), mode='constant', constant_values=0)
                        padded_images.append(padded)
                    else:
                        padded_images.append(img)
                result[hdf5_path] = np.stack(padded_images)
            print(f"    Successfully extracted {successful_extractions}/{len(messages)} images as compressed bytes")
        else:
            # Store as list of variable-length arrays
            result[hdf5_path] = images_data
            print(f"    Successfully extracted {successful_extractions}/{len(messages)} images as variable arrays")
    else:
        print(f"    Warning: No images could be extracted from {len(messages)} messages")

    return result

def extract_gripper_data(joint_messages: List[Tuple[float, JointState]], config: Dict) -> Dict[str, np.ndarray]:
    """Extract gripper data from joint states and transform according to configuration"""
    if not joint_messages:
        return {}

    result = {}
    gripper_configs = config if isinstance(config, list) else [config]

    for gripper_config in gripper_configs:
        hdf5_path = gripper_config.get('hdf5_path')
        urdf_joint_name = gripper_config.get('urdf_joint_name')
        value_type = gripper_config.get('value_type', 'direct')

        if not hdf5_path or not urdf_joint_name:
            continue

        gripper_values = []

        for timestamp, msg in joint_messages:
            if urdf_joint_name in msg.name:
                joint_idx = msg.name.index(urdf_joint_name)
                if joint_idx < len(msg.position):
                    raw_value = msg.position[joint_idx]

                    # Transform value according to configuration
                    if value_type == "angle_to_normalized":
                        # Convert URDF angle to normalized value (reverse of HDF5→MCAP)
                        input_min = gripper_config.get('input_min_angle_radians', 0.021)
                        input_max = gripper_config.get('input_max_angle_radians', 0.057)
                        output_min = gripper_config.get('output_min_value', 0.0)
                        output_max = gripper_config.get('output_max_value', 1.0)

                        # Normalize input angle
                        if input_max != input_min:
                            normalized = (raw_value - input_min) / (input_max - input_min)
                        else:
                            normalized = 0.0
                        normalized = max(0.0, min(1.0, normalized))

                        # Scale to output range
                        transformed_value = output_min + normalized * (output_max - output_min)
                        gripper_values.append(transformed_value)
                    elif value_type == "normalized_to_angle":
                        # Convert normalized value back to URDF angle (for testing)
                        input_min = gripper_config.get('input_min_value', 0.0)
                        input_max = gripper_config.get('input_max_value', 1.0)
                        output_min = gripper_config.get('output_min_angle_radians', 0.021)
                        output_max = gripper_config.get('output_max_angle_radians', 0.057)

                        # Normalize input
                        if input_max != input_min:
                            normalized = (raw_value - input_min) / (input_max - input_min)
                        else:
                            normalized = 0.0
                        normalized = max(0.0, min(1.0, normalized))

                        # Scale to angle range
                        transformed_value = output_min + normalized * (output_max - output_min)
                        gripper_values.append(transformed_value)
                    elif value_type == "direct":
                        gripper_values.append(raw_value)
                    else:
                        print(f"Warning: Unknown gripper value_type '{value_type}', using direct value")
                        gripper_values.append(raw_value)
                else:
                    gripper_values.append(0.0)
            else:
                gripper_values.append(0.0)

        if gripper_values:
            result[hdf5_path] = np.array(gripper_values).reshape(-1, 1)

    return result

def extract_custom_data(messages: List[Tuple[float, Any]], config: Dict, unified_config: Optional[UnifiedConfig] = None, input_dir: str = None, custom_handlers: str = None) -> Dict[str, np.ndarray]:
    """Extract custom topic data using user-provided handler"""
    if not messages:
        return {}

    # Support both old format (schema_script) and new format (script_path)
    script_path = config.get('script_path') or config.get('schema_script')
    handler_name_config = config.get('handler_name')
    topic = config.get('mcap_topic', 'unknown')

    # Use unified config to auto-detect handler function if available
    if unified_config:
        handler_function = unified_config.get_custom_topic_handler(config, 'mcap_to_hdf5')
    else:
        handler_function = config.get('handler_function', 'extract_to_hdf5')

    # Resolve handler path using new logic
    resolved_script_path = resolve_custom_handler_path(
        handler_name=handler_name_config,
        script_path=script_path,
        input_dir=input_dir,
        custom_handlers=custom_handlers
    )

    if not resolved_script_path:
        # Only warn once per missing handler
        handler_key = f"{topic}:{handler_name_config or script_path}"
        if handler_key not in _warned_missing_handlers:
            _warned_missing_handlers.add(handler_key)
            if handler_name_config:
                print(f"Warning: Handler '{handler_name_config}' not found for custom topic '{topic}'")
            else:
                print(f"Warning: Could not resolve script path for custom topic '{topic}'")
        return {}

    handler = load_custom_schema_handler(resolved_script_path, handler_function)
    if not handler:
        return {}

    try:
        return handler(messages, config)
    except Exception as e:
        print(f"Error in custom data extraction: {e}")
        if args.debug:
            traceback.print_exc()
        return {}

def convert_mcap_to_hdf5():
    print(f"Processing MCAP file: {args.mcap_file}")
    print(f"Using configuration file: {args.config_file}")
    print(f"Output will be saved to: {args.hdf5_file}")

    # Load configuration using unified config system
    try:
        unified_config = load_unified_config(args.config_file)

        # Handle legacy configuration migration
        if unified_config.is_legacy_format():
            print("Detected legacy configuration format. Using compatibility mode.")
            config = unified_config.config  # Use original config for legacy support
        else:
            print("Using unified configuration format.")
            config = unified_config.config

        print("Successfully loaded configuration.")
        if args.debug:
            print("--- Configuration ---")
            print(yaml.dump(config, default_flow_style=False))
            print("---------------------")
    except ConfigurationError as e:
        print(f"Configuration error: {e}")
        return
    except Exception as e:
        print(f"An unexpected error occurred while loading config: {e}")
        return

    # Read MCAP file and collect messages by topic
    topic_messages = {}
    topic_schemas = {}

    print("\n--- READING MCAP FILE ---")
    try:
        with open(args.mcap_file, 'rb') as f:
            reader = make_reader(f)

            # Collect schema information and messages
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic

                if topic not in topic_messages:
                    topic_messages[topic] = []

                # Deserialize message based on schema
                try:
                    if schema.name == "sensor_msgs/msg/JointState":
                        msg = deserialize_message(message.data, JointState)
                    elif schema.name == "tf2_msgs/msg/TFMessage":
                        msg = deserialize_message(message.data, TFMessage)
                    elif schema.name == "sensor_msgs/msg/CompressedImage":
                        msg = deserialize_message(message.data, CompressedImage)
                    elif schema.name == "sensor_msgs/msg/Image":
                        msg = deserialize_message(message.data, Image)
                    elif schema.name == "std_msgs/msg/Float64MultiArray":
                        msg = deserialize_message(message.data, Float64MultiArray)
                    elif schema.name == "std_msgs/msg/Float32MultiArray":
                        msg = deserialize_message(message.data, Float32MultiArray)
                    elif schema.name == "sensor_msgs/msg/PointCloud2":
                        msg = deserialize_message(message.data, PointCloud2)
                    elif schema.name == "geometry_msgs/msg/Twist":
                        msg = deserialize_message(message.data, Twist)
                    elif schema.name == "geometry_msgs/msg/PoseStamped":
                        msg = deserialize_message(message.data, PoseStamped)
                    else:
                        # For custom messages, store raw data
                        msg = message.data

                    timestamp = message.log_time / 1_000_000_000.0  # Convert to seconds
                    topic_messages[topic].append((timestamp, msg))
                except Exception as e:
                    print(f"Error deserializing message for topic {topic}: {e}")
                    if args.debug:
                        traceback.print_exc()

    except Exception as e:
        print(f"Error reading MCAP file: {e}")
        if args.debug:
            traceback.print_exc()
        return

    print(f"Found {len(topic_messages)} topics with messages:")
    for topic, messages in topic_messages.items():
        print(f"  {topic}: {len(messages)} messages")

    # Process data according to configuration
    hdf5_datasets = {}

    print("\n--- PROCESSING DATA ---")

    # Process joint states
    joint_state_messages = None
    if 'joint_states' in config:
        js_config = config['joint_states']
        topic = js_config.get('mcap_topic', '/joint_states')

        if topic in topic_messages:
            print(f"Processing joint states from topic: {topic}")
            joint_state_messages = topic_messages[topic]
            js_data = extract_joint_states_data(joint_state_messages, js_config, unified_config)
            hdf5_datasets.update(js_data)
        else:
            print(f"Warning: Joint states topic '{topic}' not found in MCAP file")

    # Process transforms
    if 'transforms' in config:
        tf_config = config['transforms']
        topic = tf_config.get('mcap_topic', '/tf')

        if topic in topic_messages:
            print(f"Processing transforms from topic: {topic}")
            tf_data = extract_tf_data(topic_messages[topic], tf_config)
            hdf5_datasets.update(tf_data)
        else:
            print(f"Warning: Transform topic '{topic}' not found in MCAP file")

    # Process cameras
    if 'cameras' in config:
        for cam_config in config['cameras']:
            topic = cam_config.get('mcap_topic')
            if not topic:
                continue

            if topic in topic_messages:
                print(f"Processing camera from topic: {topic}")
                img_data = extract_image_data(topic_messages[topic], cam_config)
                hdf5_datasets.update(img_data)
                print(f"  Extracted {len(topic_messages[topic])} images to {cam_config.get('hdf5_path')}")
            else:
                print(f"Warning: Camera topic '{topic}' not found in MCAP file")

    # Note: Gripper data is now integrated into joint states via gripper_transformations

    # Process custom topics
    if 'custom_topics' in config:
        for custom_config in config['custom_topics']:
            topic = custom_config.get('mcap_topic')
            if not topic:
                continue

            if topic in topic_messages:
                print(f"Processing custom topic: {topic}")
                custom_data = extract_custom_data(topic_messages[topic], custom_config, unified_config, args.input_dir, args.custom_handlers)
                hdf5_datasets.update(custom_data)
            else:
                print(f"Warning: Custom topic '{topic}' not found in MCAP file")

    # Write HDF5 file
    print("\n--- WRITING HDF5 FILE ---")
    try:
        with h5py.File(args.hdf5_file, 'w') as hdf5_file:
            for hdf5_path, data in hdf5_datasets.items():
                print(f"Writing dataset: {hdf5_path} with shape {data.shape if hasattr(data, 'shape') else 'variable'}")

                # Create groups if necessary
                group_path = '/'.join(hdf5_path.split('/')[:-1])
                if group_path and group_path not in hdf5_file:
                    hdf5_file.create_group(group_path)

                # Write dataset
                if isinstance(data, np.ndarray):
                    hdf5_file.create_dataset(hdf5_path, data=data)
                elif isinstance(data, list):
                    # Variable length data
                    dt = h5py.special_dtype(vlen=np.uint8)
                    hdf5_file.create_dataset(hdf5_path, (len(data),), dtype=dt, data=data)

        print(f"Successfully wrote HDF5 file: {args.hdf5_file}")

    except Exception as e:
        print(f"Error writing HDF5 file: {e}")
        if args.debug:
            traceback.print_exc()
        return

if __name__ == "__main__":
    try:
        convert_mcap_to_hdf5()
    except Exception as e:
        print(f"Error during conversion: {e}")
        if args.debug:
            traceback.print_exc()
    finally:
        rclpy.shutdown()
