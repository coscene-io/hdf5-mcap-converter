#!/usr/bin/env python3

import os
import h5py
import numpy as np
import time
from datetime import datetime
import xml.etree.ElementTree as ET
import argparse
import yaml
from io import BytesIO
from PIL import Image as PILImage
from mcap.writer import Writer
from mcap.well_known import SchemaEncoding, MessageEncoding
from rclpy.serialization import serialize_message
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState, Image, CompressedImage, PointCloud2, PointField
from std_msgs.msg import Header, Float64MultiArray, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from std_msgs.msg import String as StringMsg
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, Vector3
import traceback
import sys
import importlib.util
from typing import Optional, Callable, List, Dict, Any
from config_utils import load_unified_config, UnifiedConfig, ConfigurationError

# Exit codes for different error types
EXIT_SUCCESS = 0
EXIT_CONFIG_ERROR = 1      # Configuration file errors
EXIT_VALIDATION_ERROR = 2  # Data validation errors (missing datasets, invalid values)
EXIT_IO_ERROR = 3          # File I/O errors (can't write output, etc.)
EXIT_RUNTIME_ERROR = 4     # Other runtime errors

# Parse arguments
parser = argparse.ArgumentParser(description='Convert HDF5 and URDF files to MCAP')
parser.add_argument('--hdf5-file', type=str, required=True, help='Path to HDF5 file (inside container, e.g., /mnt/input.hdf5)')
parser.add_argument('--urdf-file', type=str, required=True, help='Path to URDF file (inside container, e.g., /mnt/robot.urdf)')
parser.add_argument('--output-file', type=str, required=True, help='Output MCAP file path (inside container, e.g., /mnt_out/output.mcap)')
parser.add_argument('--debug', action='store_true', help='Enable debug output')
parser.add_argument('--config-file', type=str, required=True, help='Path to YAML configuration file (inside container, e.g., /mnt/config.yaml)')
parser.add_argument('--input-dir', type=str, help='Base input directory (inside container, e.g., /mnt_input)')
parser.add_argument('--custom-handlers', type=str, help='Custom handler scripts (comma-separated list or glob pattern, e.g., "handler1.py,handler2.py" or "*.py")')
args = parser.parse_args()

# Global set to track which handler warnings have been printed
_warned_missing_handlers = set()

# Create output directory if it doesn't exist
output_dir = os.path.dirname(args.output_file)
os.makedirs(output_dir, exist_ok=True)

# Initialize ROS2
rclpy.init()

# Define schema definitions directly since we can't access them through ROS utilities
HEADER_SCHEMA = b"""
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp
string frame_id
"""

TIME_SCHEMA = b"""
# Time with seconds and nanoseconds
int32 sec
uint32 nanosec
"""

TF_MSG_SCHEMA = b"""
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id at the time of header.stamp
#
# This message contains a vector of transform stamped objects
# All transforms are valid at the time of header.stamp

geometry_msgs/TransformStamped[] transforms

# The standard ROS message definition continues, but for MCAP we need to include all
# referenced message definitions:

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
# Time with seconds and nanoseconds
int32 sec
uint32 nanosec

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.
float64 x
float64 y
float64 z
float64 w
"""

JOINT_STATE_SCHEMA = b"""
# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
# Time with seconds and nanoseconds
int32 sec
uint32 nanosec
"""

# CompressedImage schema definition
COMPRESSED_IMAGE_SCHEMA = b"""
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                            # Header frame_id should be optical frame of camera
                            # origin of frame should be optical center of camera
                            # +x should point to the right in the image
                            # +y should point down in the image
                            # +z should point into to plane of the image
                            # If the frame_id here and the frame_id of the CameraInfo
                            # message associated with the image conflict
                            # the behavior is undefined

string format        # Specifies the format of the data
                     #   Acceptable values:
                     #     jpeg, png, tiff
uint8[] data         # Compressed image buffer

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
# Time with seconds and nanoseconds
int32 sec
uint32 nanosec
"""

IMAGE_SCHEMA = b"""
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

std_msgs/Header header # Header timestamp should be acquisition time of image
                            # Header frame_id should be optical frame of camera
                            # origin of frame should be optical center of camera
                            # +x should point to the right in the image
                            # +y should point down in the image
                            # +z should point into to plane of the image
                            # If the frame_id here and the frame_id of the CameraInfo
                            # message associated with the image conflict
                            # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
# Time with seconds and nanoseconds
int32 sec
uint32 nanosec
"""

STRING_SCHEMA = b"""
string data
"""

# Float64MultiArray schema for custom topics
FLOAT64_MULTIARRAY_SCHEMA = b"""
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) = data[k + 1920*j + 921600*i]

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
"""

# Additional schemas for custom topics
FLOAT32_MULTIARRAY_SCHEMA = b"""
MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data

================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
"""

POINT_CLOUD2_SCHEMA = b"""
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its format described by the
# contents of the "fields" array.

std_msgs/Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
sensor_msgs/PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
"""

TWIST_SCHEMA = b"""
# This expresses velocity in free space broken into its linear and angular parts.
geometry_msgs/Vector3  linear
geometry_msgs/Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.
float64 x
float64 y
float64 z
"""

POSE_STAMPED_SCHEMA = b"""
# A Pose with reference coordinate frame and timestamp
std_msgs/Header header
geometry_msgs/Pose pose

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec

================================================================================
MSG: geometry_msgs/Pose
# A representation of position and orientation in free space
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.
float64 x
float64 y
float64 z
float64 w
"""

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

def load_custom_handler(script_path: str, handler_name: str) -> Optional[Callable]:
    """Load a custom handler from a Python script"""
    try:
        spec = importlib.util.spec_from_file_location("custom_handler", script_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return getattr(module, handler_name)
    except Exception as e:
        print(f"Error loading custom handler from {script_path}: {e}")
        if args.debug:
            traceback.print_exc()
        return None

def parse_urdf(urdf_file):
    """Extract joint information from URDF file"""
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    joints = {}
    print("--- Parsing URDF Joints ---")
    for joint in root.findall('.//joint'):
        joint_name = joint.get('name')
        joint_type = joint.get('type')
        # Only print joint info in debug mode
        if args.debug:
            print(f"  Found: {joint_name} (Type: {joint_type})")

        joints[joint_name] = {
            'type': joint_type,
            'parent': joint.find('parent').get('link'),
            'child': joint.find('child').get('link')
        }
        origin = joint.find('origin')
        if origin is not None:
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
            rpy = [float(r) for r in origin.get('rpy', '0 0 0').split()]
            joints[joint_name]['origin'] = {'xyz': xyz, 'rpy': rpy}
        else:
            joints[joint_name]['origin'] = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}

        # Parse axis (required for non-fixed joints)
        axis_element = joint.find('axis')
        if axis_element is not None:
            axis_xyz = [float(a) for a in axis_element.get('xyz', '1 0 0').split()]
            joints[joint_name]['axis'] = axis_xyz
        elif joint_type != 'fixed':
            print(f"Warning: Joint '{joint_name}' is type '{joint_type}' but missing <axis> tag. Defaulting to X-axis.")
            joints[joint_name]['axis'] = [1.0, 0.0, 0.0]
        else:
             joints[joint_name]['axis'] = [1.0, 0.0, 0.0] # Default for fixed, though not used

        # Parse mimic tag if present
        mimic_element = joint.find('mimic')
        if mimic_element is not None:
            try:
                mimicked_joint_name = mimic_element.get('joint')
                multiplier = float(mimic_element.get('multiplier', '1.0'))
                offset = float(mimic_element.get('offset', '0.0'))
                if not mimicked_joint_name:
                    raise ValueError("Mimic tag missing required 'joint' attribute")
                joints[joint_name]['mimic'] = {'joint': mimicked_joint_name, 'multiplier': multiplier, 'offset': offset}
                print(f"    -> Found mimic: mimics '{mimicked_joint_name}', mult={multiplier}, offset={offset}")
            except Exception as e:
                print(f"Warning: Error parsing mimic tag for joint '{joint_name}': {e}")

    print("--- Finished Parsing URDF Joints ---")
    return joints

def rpy_to_quaternion(r, p, y):
    """Convert roll, pitch, yaw to quaternion"""
    cy = np.cos(y * 0.5)
    sy = np.sin(y * 0.5)
    cp = np.cos(p * 0.5)
    sp = np.sin(p * 0.5)
    cr = np.cos(r * 0.5)
    sr = np.sin(r * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz

# --- Quaternion Math Helpers ---
def quaternion_multiply(q1, q0):
    """Multiply two quaternions q1*q0. Input/output: [w, x, y, z] list/tuple."""
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return [
        w1*w0 - x1*x0 - y1*y0 - z1*z0,
        w1*x0 + x1*w0 + y1*z0 - z1*y0,
        w1*y0 - x1*z0 + y1*w0 + z1*x0,
        w1*z0 + x1*y0 - y1*x0 + z1*w0
    ]

def quaternion_from_axis_angle(axis, angle):
    """Create quaternion from axis and angle. Input axis: [x,y,z], angle (rad). Output: [w, x, y, z]."""
    angle = float(angle)
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-8:
        # Default to identity if axis is zero vector
        return [1.0, 0.0, 0.0, 0.0]
    axis = axis / norm
    half_angle = angle * 0.5
    sin_half = np.sin(half_angle)
    cos_half = np.cos(half_angle)
    qx = axis[0] * sin_half
    qy = axis[1] * sin_half
    qz = axis[2] * sin_half
    qw = cos_half
    return [qw, qx, qy, qz]

def rotate_vector_by_quaternion(v, q):
    """Rotate vector v by quaternion q. Input v=[x,y,z], q=[w,x,y,z]. Output: [x',y',z']"""
    # Create pure quaternion for vector
    p = [0.0] + v
    # Conjugate of rotation quaternion
    q_conj = [q[0], -q[1], -q[2], -q[3]]
    # Rotate: p' = q * p * q_conj
    p_prime = quaternion_multiply(quaternion_multiply(q, p), q_conj)
    return p_prime[1:] # Return vector part
# --- End Quaternion Math Helpers ---

def create_transform_message(parent_frame, child_frame, translation, rotation, timestamp):
    """Create a TransformStamped message"""
    msg = TransformStamped()
    msg.header.stamp = timestamp
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame

    msg.transform.translation.x = float(translation[0])
    msg.transform.translation.y = float(translation[1])
    msg.transform.translation.z = float(translation[2])

    msg.transform.rotation.w = float(rotation[0])
    msg.transform.rotation.x = float(rotation[1])
    msg.transform.rotation.y = float(rotation[2])
    msg.transform.rotation.z = float(rotation[3])

    # Validate data types to ensure proper serialization
    assert isinstance(msg.header.frame_id, str), f"frame_id must be string, got {type(msg.header.frame_id)}"
    assert isinstance(msg.child_frame_id, str), f"child_frame_id must be string, got {type(msg.child_frame_id)}"
    assert isinstance(msg.transform.translation.x, float), f"translation.x must be float, got {type(msg.transform.translation.x)}"
    assert isinstance(msg.transform.rotation.w, float), f"rotation.w must be float, got {type(msg.transform.rotation.w)}"

    return msg

def create_joint_state_message(joint_names, positions, velocities, efforts, timestamp):
    """Create a JointState message"""
    msg = JointState()
    msg.header.stamp = timestamp
    msg.name = joint_names
    msg.position = [float(p) for p in positions]
    if velocities is not None:
        msg.velocity = [float(v) for v in velocities]
    if efforts is not None:
        msg.effort = [float(e) for e in efforts]
    return msg

def ros_time_to_timestamp_ns(time_msg):
    """Convert ROS Time to nanoseconds timestamp"""
    return time_msg.sec * 1_000_000_000 + time_msg.nanosec

def is_dataset(hdf5_obj):
    """Check if an HDF5 object is a Dataset"""
    return isinstance(hdf5_obj, h5py.Dataset)

def debug_print_hdf5_structure(f, path=""):
    """Print the full structure of an HDF5 file for debugging"""
    if not args.debug:
        return

    for key in f.keys():
        if isinstance(f[key], h5py.Group):
            print(f"{path}/{key} (Group)")
            debug_print_hdf5_structure(f[key], f"{path}/{key}")
        else:
            print(f"{path}/{key} (Dataset: {f[key].shape}, {f[key].dtype})")
            if len(f[key].shape) == 0 or f[key].shape[0] < 5:
                print(f"  Value: {f[key][...]}")
            else:
                print(f"  First few values: {f[key][:5]}")

def find_dataset_recursively(group, target_name=None):
    """Recursively search for a dataset in an HDF5 file or group"""
    if target_name is None:
        # Find the first dataset if no specific name is provided
        for key, item in group.items():
            if isinstance(item, h5py.Dataset) and len(item.shape) > 0:
                return item
            elif isinstance(item, h5py.Group):
                result = find_dataset_recursively(item)
                if result is not None:
                    return result
    else:
        # Find a dataset with a specific name
        if target_name in group:
            item = group[target_name]
            if isinstance(item, h5py.Dataset) and len(item.shape) > 0:
                return item
            elif isinstance(item, h5py.Group):
                return find_dataset_recursively(item, target_name)

        # If not found directly, search recursively in all groups
        for key, item in group.items():
            if isinstance(item, h5py.Group):
                result = find_dataset_recursively(item, target_name)
                if result is not None:
                    return result

    return None

def debug_object(obj, name, max_items=5):
    """Print detailed debug information about an object"""
    if not args.debug:
        return

    print(f"Debug info for {name}:")
    print(f"  Type: {type(obj)}")

    if hasattr(obj, 'shape'):
        print(f"  Shape: {obj.shape}")

    if hasattr(obj, 'dtype'):
        print(f"  Dtype: {obj.dtype}")

    if hasattr(obj, 'keys'):
        print(f"  Keys: {list(obj.keys())}")

    if hasattr(obj, '__len__'):
        print(f"  Length: {len(obj)}")
        if len(obj) > 0:
            sample = obj[0]
            print(f"  First item type: {type(sample)}")
            if hasattr(sample, 'shape'):
                print(f"  First item shape: {sample.shape}")
            if hasattr(sample, 'dtype'):
                print(f"  First item dtype: {sample.dtype}")
            if hasattr(sample, '__len__'):
                print(f"  First item length: {len(sample)}")
                if len(sample) > 0 and max_items > 0:
                    print(f"  First {min(max_items, len(sample))} elements: {sample[:max_items]}")

    print()  # Add a blank line for readability

def convert_hdf5_to_mcap():
    print(f"Processing HDF5 file: {args.hdf5_file}")
    print(f"Using URDF file: {args.urdf_file}")
    print(f"Using configuration file: {args.config_file}")
    print(f"Output will be saved to: {args.output_file}")

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
        sys.exit(EXIT_CONFIG_ERROR)
    except Exception as e:
        print(f"An unexpected error occurred while loading config: {e}")
        sys.exit(EXIT_RUNTIME_ERROR)

    # Validate basic config structure
    if 'joint_states' not in config or 'hdf5_paths' not in config['joint_states'] or 'position' not in config['joint_states']['hdf5_paths']:
        print("ERROR: Config missing required 'joint_states.hdf5_paths.position'")
        sys.exit(EXIT_VALIDATION_ERROR)
    if 'cameras' not in config:
        print("Warning: No 'cameras' section found in config. No image data will be processed.")
        config['cameras'] = [] # Ensure cameras list exists even if empty
    if 'joint_mappings' not in config['joint_states']:
        print("ERROR: Config missing required 'joint_states.joint_mappings' for explicit mapping.")
        sys.exit(EXIT_VALIDATION_ERROR)
    if 'gripper' in config and not isinstance(config['gripper'], list):
         print("ERROR: 'gripper' section must be a list of configurations.")
         sys.exit(EXIT_VALIDATION_ERROR)

    # Validate and set sampling rates
    global_sampling_rate = config.get('sampling_rate', 20.0)  # Default 20 Hz
    print(f"Global sampling rate: {global_sampling_rate} Hz")

    # Set sampling rates for joint states
    joint_states_sampling_rate = config['joint_states'].get('sampling_rate', global_sampling_rate)
    print(f"Joint states sampling rate: {joint_states_sampling_rate} Hz")

    # Validate sampling rates are positive
    if global_sampling_rate <= 0:
        print("ERROR: Global sampling rate must be positive")
        sys.exit(EXIT_VALIDATION_ERROR)
    if joint_states_sampling_rate <= 0:
        print("ERROR: Joint states sampling rate must be positive")
        sys.exit(EXIT_VALIDATION_ERROR)

    # Parse URDF to get joint information
    joints = parse_urdf(args.urdf_file)
    num_urdf_joints = len(joints)
    joint_names = list(joints.keys())

    # Read URDF file content for /robot_description
    urdf_content_str = None
    try:
        with open(args.urdf_file, 'r') as f:
            urdf_content_str = f.read()
    except Exception as e:
        print(f"Warning: Could not read URDF file content from '{args.urdf_file}' for /robot_description: {e}")

    # Open HDF5 file
    with h5py.File(args.hdf5_file, 'r') as f:
        # Check what data is available in the HDF5 file
        print("Available keys in HDF5 file:", list(f.keys()))

        # Debug: print the HDF5 structure if debug is enabled
        if args.debug:
            print("\nHDF5 file structure:")
            debug_print_hdf5_structure(f)

        # Determine frame counts from actual datasets
        print("\n--- DETERMINING FRAME COUNTS FROM DATASETS ---")

        # Get joint states frame count (primary reference)
        joint_pos_path = config['joint_states']['hdf5_paths']['position']
        if joint_pos_path not in f:
            print(f"ERROR: Joint position dataset '{joint_pos_path}' not found. Cannot proceed.")
            sys.exit(EXIT_VALIDATION_ERROR)

        joint_states_frames = f[joint_pos_path].shape[0]
        joint_states_duration = joint_states_frames / joint_states_sampling_rate
        print(f"Joint states: {joint_states_frames} frames at {joint_states_sampling_rate} Hz = {joint_states_duration:.2f}s")

        # Validate other joint datasets have matching frame counts
        vel_path = config['joint_states']['hdf5_paths'].get('velocity')
        eff_path = config['joint_states']['hdf5_paths'].get('effort')

        if vel_path and vel_path in f:
            vel_frames = f[vel_path].shape[0]
            if vel_frames != joint_states_frames:
                print(f"WARNING: Velocity dataset has {vel_frames} frames, expected {joint_states_frames}")

        if eff_path and eff_path in f:
            eff_frames = f[eff_path].shape[0]
            if eff_frames != joint_states_frames:
                print(f"WARNING: Effort dataset has {eff_frames} frames, expected {joint_states_frames}")

        # Check camera frame counts and sampling rates
        camera_info = []
        for i, camera_config in enumerate(config['cameras']):
            cam_path = camera_config.get('hdf5_path')
            cam_topic = camera_config.get('mcap_topic', f'camera_{i}')
            cam_sampling_rate = camera_config.get('sampling_rate', global_sampling_rate)

            if cam_path and cam_path in f:
                cam_frames = f[cam_path].shape[0]
                cam_duration = cam_frames / cam_sampling_rate
                expected_frames = int(joint_states_duration * cam_sampling_rate)

                camera_info.append({
                    'topic': cam_topic,
                    'path': cam_path,
                    'frames': cam_frames,
                    'sampling_rate': cam_sampling_rate,
                    'duration': cam_duration,
                    'expected_frames': expected_frames
                })

                print(f"Camera '{cam_topic}': {cam_frames} frames at {cam_sampling_rate} Hz = {cam_duration:.2f}s")

                if abs(cam_duration - joint_states_duration) > 0.1:  # 100ms tolerance
                    print(f"  WARNING: Duration mismatch with joint states ({joint_states_duration:.2f}s)")
            else:
                print(f"WARNING: Camera dataset '{cam_path}' not found")

        # Use joint states as the primary timeline
        num_frames = joint_states_frames
        primary_sampling_rate = joint_states_sampling_rate
        print(f"\nUsing joint states as primary timeline: {num_frames} frames at {primary_sampling_rate} Hz")

        # --- Data Loading and Mapping ---
        print("\n--- LOADING AND MAPPING JOINT DATA (from config) ---")
        joint_positions = np.zeros((num_frames, num_urdf_joints))
        joint_velocities = np.zeros((num_frames, num_urdf_joints))
        joint_efforts = np.zeros((num_frames, num_urdf_joints))

        js_config = config['joint_states']
        pos_path = js_config['hdf5_paths'].get('position')
        vel_path = js_config['hdf5_paths'].get('velocity')
        eff_path = js_config['hdf5_paths'].get('effort')
        joint_mappings = js_config.get('joint_mappings', {})

        hdf5_pos_data = None
        if pos_path and pos_path in f:
            hdf5_pos_data = f[pos_path][:]
            print(f"  Loaded raw position data from '{pos_path}' with shape {hdf5_pos_data.shape}")
        else:
            print(f"ERROR: Position dataset '{pos_path}' not found. Cannot proceed.")
            sys.exit(EXIT_VALIDATION_ERROR)

        hdf5_vel_data = None
        if vel_path and vel_path in f:
            hdf5_vel_data = f[vel_path][:]
            print(f"  Loaded raw velocity data from '{vel_path}' with shape {hdf5_vel_data.shape}")
            if hdf5_vel_data.shape[0] != num_frames:
                print(f"  Warning: Velocity data frames ({hdf5_vel_data.shape[0]}) mismatch ({num_frames}). Velocities may be incorrect.")
                hdf5_vel_data = None
        else:
            print("  Info: Velocity data path not specified or not found. Velocities will be zero.")

        hdf5_eff_data = None
        if eff_path and eff_path in f:
            hdf5_eff_data = f[eff_path][:]
            print(f"  Loaded raw effort data from '{eff_path}' with shape {hdf5_eff_data.shape}")
            if hdf5_eff_data.shape[0] != num_frames:
                print(f"  Warning: Effort data frames ({hdf5_eff_data.shape[0]}) mismatch ({num_frames}). Efforts may be incorrect.")
                hdf5_eff_data = None
        else:
            print("  Info: Effort data path not specified or not found. Efforts will be zero.")

        print("\nMapping arm joints based on 'joint_mappings':")
        mapped_urdf_indices = set()
        for urdf_joint_name, hdf5_col_index in joint_mappings.items():
            if urdf_joint_name not in joint_names:
                print(f"  Warning: Joint '{urdf_joint_name}' from config not found in URDF. Skipping.")
                continue

            # Check if hdf5_col_index is an int (for single column) or a dict for mapping
            if isinstance(hdf5_col_index, int):
                # Simple column index mapping
                if hdf5_col_index < 0 or hdf5_col_index >= hdf5_pos_data.shape[1]:
                    print(f"  Warning: HDF5 column index {hdf5_col_index} for URDF joint '{urdf_joint_name}' is out of bounds. Skipping.")
                    continue
                urdf_joint_idx = joint_names.index(urdf_joint_name)
                joint_positions[:, urdf_joint_idx] = hdf5_pos_data[:, hdf5_col_index]
                mapped_urdf_indices.add(urdf_joint_idx)
                print(f"  Mapped URDF '{urdf_joint_name}' (idx {urdf_joint_idx}) <- HDF5 Col {hdf5_col_index}")
                if hdf5_vel_data is not None and hdf5_col_index < hdf5_vel_data.shape[1]:
                    joint_velocities[:, urdf_joint_idx] = hdf5_vel_data[:, hdf5_col_index]
                if hdf5_eff_data is not None and hdf5_col_index < hdf5_eff_data.shape[1]:
                    joint_efforts[:, urdf_joint_idx] = hdf5_eff_data[:, hdf5_col_index]
            elif isinstance(hdf5_col_index, dict): # Per-joint mapping parameters
                # ... (Existing logic for per-joint mapping - to be reviewed/kept if it was added)
                # This part should contain linear_map_to_position or similar if we had that
                # For now, we just assume direct index mapping for arm joints.
                # If per-joint scaling/offset is needed for arm joints, this block would expand.
                print(f"  Info: Per-joint mapping for '{urdf_joint_name}' is not fully implemented beyond simple index. Using direct value if possible.")
                # Fallback to trying a simple index if 'hdf5_index' is present
                actual_hdf5_idx = hdf5_col_index.get('hdf5_index')
                if actual_hdf5_idx is not None and isinstance(actual_hdf5_idx, int):
                    if actual_hdf5_idx < 0 or actual_hdf5_idx >= hdf5_pos_data.shape[1]:
                        print(f"  Warning: HDF5 hdf5_index {actual_hdf5_idx} for URDF joint '{urdf_joint_name}' is out of bounds. Skipping.")
                        continue
                    urdf_joint_idx = joint_names.index(urdf_joint_name)
                    joint_positions[:, urdf_joint_idx] = hdf5_pos_data[:, actual_hdf5_idx]
                    mapped_urdf_indices.add(urdf_joint_idx)
                    print(f"  Mapped URDF '{urdf_joint_name}' (idx {urdf_joint_idx}) <- HDF5 Col {actual_hdf5_idx} (from detailed map)")
                    if hdf5_vel_data is not None and actual_hdf5_idx < hdf5_vel_data.shape[1]:
                       joint_velocities[:, urdf_joint_idx] = hdf5_vel_data[:, actual_hdf5_idx]
                    if hdf5_eff_data is not None and actual_hdf5_idx < hdf5_eff_data.shape[1]:
                       joint_efforts[:, urdf_joint_idx] = hdf5_eff_data[:, actual_hdf5_idx]
                else:
                    print(f"  Warning: Invalid or missing 'hdf5_index' in detailed mapping for '{urdf_joint_name}'. Skipping.")
            else:
                print(f"  Warning: Invalid mapping for '{urdf_joint_name}'. Expected int or dict. Skipping.")



        # Process gripper data using unified configuration
        gripper_mappers = unified_config.get_gripper_mappers()

        if gripper_mappers:
            print("\n--- PROCESSING GRIPPER DATA (unified format) ---")
            for joint_name, (value_mapper, gripper_config) in gripper_mappers.items():
                print(f"Processing gripper for joint '{joint_name}'...")
                try:
                    gripper_hdf5_path = gripper_config.get('hdf5_path')
                    gripper_hdf5_index = gripper_config.get('hdf5_index')

                    # --- Initial validation for this gripper config ---
                    if not gripper_hdf5_path or not joint_name:
                        print("  Warning: Config missing 'hdf5_path' or 'urdf_joint_name'. Skipping.")
                        continue
                    if gripper_hdf5_path not in f:
                        print(f"  Warning: HDF5 path '{gripper_hdf5_path}' not found. Skipping.")
                        continue
                    hdf5_gripper_dataset = f[gripper_hdf5_path]
                    if len(hdf5_gripper_dataset.shape) < 1 or hdf5_gripper_dataset.shape[0] != num_frames:
                        print(f"  Warning: Data shape {hdf5_gripper_dataset.shape} incompatible with num_frames {num_frames}. Skipping.")
                        continue
                    if joint_name not in joint_names:
                        print(f"  Warning: URDF joint '{joint_name}' not found. Skipping.")
                        continue
                    if gripper_hdf5_index is not None and (gripper_hdf5_index < 0 or gripper_hdf5_index >= hdf5_gripper_dataset.shape[1]):
                         print(f"  Warning: hdf5_index {gripper_hdf5_index} out of bounds for shape {hdf5_gripper_dataset.shape}. Skipping.")
                         continue
                    if gripper_hdf5_index is None and len(hdf5_gripper_dataset.shape) > 1 and hdf5_gripper_dataset.shape[1] != 1:
                         print(f"  Warning: Dataset '{gripper_hdf5_path}' has shape {hdf5_gripper_dataset.shape} but no hdf5_index specified. Skipping.")
                         continue
                    # --- End Initial validation ---

                    gripper_joint_idx = joint_names.index(joint_name)
                    print_idx_info = f" at index {gripper_hdf5_index}" if gripper_hdf5_index is not None else ""
                    print(f"  Mapping '{gripper_hdf5_path}'{print_idx_info} -> '{joint_name}' (idx {gripper_joint_idx}) using unified value mapping.")

                    # Value mapper is already validated during creation

                    # Process frames for this valid gripper config
                    for frame_idx in range(num_frames):
                        raw_gripper_value = 0.0
                        try:
                            if gripper_hdf5_index is not None:
                                raw_gripper_value = hdf5_gripper_dataset[frame_idx, gripper_hdf5_index]
                            elif len(hdf5_gripper_dataset.shape) == 1:
                                raw_gripper_value = hdf5_gripper_dataset[frame_idx]
                            else: # Shape is (N, 1)
                                raw_gripper_value = hdf5_gripper_dataset[frame_idx, 0]
                        except IndexError:
                             if frame_idx == 0: print(f"  Warning: IndexError reading frame {frame_idx}. Using 0.0.")
                             raw_gripper_value = 0.0

                        # Use unified value mapper for HDF5 → URDF conversion
                        gripper_output_value = value_mapper.hdf5_to_urdf(raw_gripper_value)

                        if joint_positions is None:
                            joint_positions = np.zeros((num_frames, num_urdf_joints))
                        joint_positions[frame_idx, gripper_joint_idx] = gripper_output_value

                        # Process gripper velocities if available
                        if hdf5_vel_data is not None and gripper_hdf5_index is not None and gripper_hdf5_index < hdf5_vel_data.shape[1]:
                            try:
                                raw_gripper_velocity = hdf5_vel_data[frame_idx, gripper_hdf5_index]
                                if joint_velocities is None:
                                    joint_velocities = np.zeros((num_frames, num_urdf_joints))
                                joint_velocities[frame_idx, gripper_joint_idx] = raw_gripper_velocity
                            except IndexError:
                                if frame_idx == 0: print(f"  Warning: IndexError reading velocity for frame {frame_idx}. Using 0.0.")

                        # Process gripper efforts if available
                        if hdf5_eff_data is not None and gripper_hdf5_index is not None and gripper_hdf5_index < hdf5_eff_data.shape[1]:
                            try:
                                raw_gripper_effort = hdf5_eff_data[frame_idx, gripper_hdf5_index]
                                if joint_efforts is None:
                                    joint_efforts = np.zeros((num_frames, num_urdf_joints))
                                joint_efforts[frame_idx, gripper_joint_idx] = raw_gripper_effort
                            except IndexError:
                                if frame_idx == 0: print(f"  Warning: IndexError reading effort for frame {frame_idx}. Using 0.0.")
                    # End frame loop

                    print(f"  Finished processing gripper for joint '{joint_name}' (position only; velocity and effort unchanged).")
                    mapped_urdf_indices.add(gripper_joint_idx)

                except KeyError as e:
                    print(f"  Warning: Gripper config for '{joint_name}' missing required mapping key: {e}. Skipping this gripper.")
                except Exception as e:
                    print(f"  Error processing gripper for joint '{joint_name}': {e}. Skipping this gripper.")
                    if args.debug: traceback.print_exc()
            # End gripper processing loop

        # Report unmapped non-fixed joints
        print("\nChecking for unmapped non-fixed joints in URDF:")
        for idx, name in enumerate(joint_names):
            if joints[name]['type'] != 'fixed' and idx not in mapped_urdf_indices:
                # Also check if it's a mimic joint - they are handled by TF calculation
                if not joints[name].get('mimic'):
                    print(f"  Warning: Non-fixed, non-mimic URDF joint '{name}' (index {idx}) was not mapped via 'joint_mappings' or 'gripper' config. Its state will remain zero.")
                # else: It's a mimic joint, its state is derived, so no warning needed for it not being in direct mappings.

        # Create timestamps for joint states (primary timeline)
        # Use arange to ensure exact sampling rate intervals
        joint_timestamps = np.arange(num_frames) / primary_sampling_rate
        print(f"Generated {len(joint_timestamps)} timestamps for joint states timeline")
        print(f"Timestamp range: {joint_timestamps[0]:.6f}s to {joint_timestamps[-1]:.6f}s")
        if len(joint_timestamps) > 1:
            interval = joint_timestamps[1] - joint_timestamps[0]
            actual_rate = 1.0 / interval
            print(f"Actual sampling interval: {interval:.6f}s ({actual_rate:.2f} Hz)")

        # Create MCAP writer and process data
        print("\n--- WRITING MCAP FILE ---")
        mcap_file = None
        try:
            mcap_file = open(args.output_file, 'wb')
            writer = Writer(mcap_file)
            writer.start()

            # Register schemas with our hardcoded schema definitions
            # TF Schema
            tf_schema_id = writer.register_schema(
                name="tf2_msgs/msg/TFMessage",
                encoding=SchemaEncoding.ROS2,
                data=TF_MSG_SCHEMA
            )

            # JointState Schema
            joint_state_schema_id = writer.register_schema(
                name="sensor_msgs/msg/JointState",
                encoding=SchemaEncoding.ROS2,
                data=JOINT_STATE_SCHEMA
            )

            # CompressedImage Schema (used by cameras defined as 'jpeg' format)
            compressed_image_schema_id = writer.register_schema(
                name="sensor_msgs/msg/CompressedImage",
                encoding=SchemaEncoding.ROS2,
                data=COMPRESSED_IMAGE_SCHEMA
            )

            # Custom topic schemas
            float64_multiarray_schema_id = writer.register_schema(
                name="std_msgs/msg/Float64MultiArray",
                encoding=SchemaEncoding.ROS2,
                data=FLOAT64_MULTIARRAY_SCHEMA
            )

            float32_multiarray_schema_id = writer.register_schema(
                name="std_msgs/msg/Float32MultiArray",
                encoding=SchemaEncoding.ROS2,
                data=FLOAT32_MULTIARRAY_SCHEMA
            )

            point_cloud2_schema_id = writer.register_schema(
                name="sensor_msgs/msg/PointCloud2",
                encoding=SchemaEncoding.ROS2,
                data=POINT_CLOUD2_SCHEMA
            )

            twist_schema_id = writer.register_schema(
                name="geometry_msgs/msg/Twist",
                encoding=SchemaEncoding.ROS2,
                data=TWIST_SCHEMA
            )

            pose_stamped_schema_id = writer.register_schema(
                name="geometry_msgs/msg/PoseStamped",
                encoding=SchemaEncoding.ROS2,
                data=POSE_STAMPED_SCHEMA
            )

            # Register /robot_description schema and channel if URDF content is available
            if urdf_content_str is not None:
                try:
                    robot_description_schema_id = writer.register_schema(
                        name="std_msgs/msg/String",
                        encoding=SchemaEncoding.ROS2,
                        data=STRING_SCHEMA
                    )
                    robot_description_channel_id = writer.register_channel(
                        topic="/robot_description",
                        message_encoding=MessageEncoding.CDR,
                        schema_id=robot_description_schema_id
                    )
                    robot_description_msg = StringMsg()
                    robot_description_msg.data = urdf_content_str
                    timestamp_zero_ns = 0
                    serialized_robot_desc = serialize_message(robot_description_msg)
                    writer.add_message(
                        channel_id=robot_description_channel_id,
                        log_time=timestamp_zero_ns,
                        data=serialized_robot_desc,
                        publish_time=timestamp_zero_ns
                    )
                    print("Successfully wrote URDF to /robot_description topic.")
                except Exception as e:
                    print(f"Error preparing or writing /robot_description message: {e}")
                    if args.debug:
                        traceback.print_exc()

            # Register channels
            tf_topic = config.get('tf_topic', '/tf') # Make TF topic configurable
            tf_channel_id = writer.register_channel(
                topic=tf_topic,
                message_encoding=MessageEncoding.CDR,
                schema_id=tf_schema_id
            )

            # JointState Channel (topic from config)
            joint_state_topic = config['joint_states'].get('mcap_topic', '/joint_states') # Default if not in config
            joint_state_channel_id = writer.register_channel(
                topic=joint_state_topic,
                message_encoding=MessageEncoding.CDR,
                schema_id=joint_state_schema_id
            )

            # Register camera channels dynamically
            camera_channel_ids = {}
            camera_schema_ids = {} # To store schema ID per format if needed later
            print("Registering camera channels based on config:")
            for camera_config in config['cameras']:
                cam_topic = camera_config.get('mcap_topic')
                cam_format = camera_config.get('format', 'jpeg').lower() # Default to jpeg
                cam_hdf5_path = camera_config.get('hdf5_path')

                if not cam_topic or not cam_hdf5_path:
                     print(f"Warning: Skipping camera entry due to missing 'mcap_topic' or 'hdf5_path': {camera_config}")
                     continue

                # Select schema based on format
                if cam_format == 'jpeg':
                    schema_id = compressed_image_schema_id
                    schema_name = "sensor_msgs/msg/CompressedImage"
                else:
                    print(f"Warning: Skipping camera '{cam_topic}' due to unsupported format '{cam_format}'. Only 'jpeg' is supported currently.")
                    continue

                # Register channel
                try:
                    channel_id = writer.register_channel(
                        topic=cam_topic,
                    message_encoding=MessageEncoding.CDR,
                        schema_id=schema_id
                    )
                    camera_channel_ids[cam_topic] = channel_id
                    print(f"  Registered channel for '{cam_topic}' (Schema: {schema_name}, ID: {channel_id})")
                except Exception as e:
                    print(f"Error registering channel for camera '{cam_topic}': {e}")

            # Register custom topic channels
            custom_channel_ids = {}
            custom_topics = config.get('custom_topics', [])
            print("Registering custom topic channels:")
            for custom_config in custom_topics:
                topic = custom_config.get('mcap_topic')
                message_type = custom_config.get('message_type', 'std_msgs/msg/Float64MultiArray')

                if not topic:
                    print(f"Warning: Skipping custom topic due to missing 'mcap_topic': {custom_config}")
                    continue

                # Select schema based on message type
                if message_type == 'std_msgs/msg/Float64MultiArray':
                    schema_id = float64_multiarray_schema_id
                    schema_name = "std_msgs/msg/Float64MultiArray"
                elif message_type == 'std_msgs/msg/Float32MultiArray':
                    schema_id = float32_multiarray_schema_id
                    schema_name = "std_msgs/msg/Float32MultiArray"
                elif message_type == 'sensor_msgs/msg/PointCloud2':
                    schema_id = point_cloud2_schema_id
                    schema_name = "sensor_msgs/msg/PointCloud2"
                elif message_type == 'geometry_msgs/msg/Twist':
                    schema_id = twist_schema_id
                    schema_name = "geometry_msgs/msg/Twist"
                elif message_type == 'geometry_msgs/msg/PoseStamped':
                    schema_id = pose_stamped_schema_id
                    schema_name = "geometry_msgs/msg/PoseStamped"
                else:
                    print(f"Warning: Unsupported message type '{message_type}' for custom topic '{topic}'. Using Float64MultiArray.")
                    schema_id = float64_multiarray_schema_id
                    schema_name = "std_msgs/msg/Float64MultiArray"

                # Register channel
                try:
                    channel_id = writer.register_channel(
                        topic=topic,
                        message_encoding=MessageEncoding.CDR,
                        schema_id=schema_id
                    )
                    custom_channel_ids[topic] = channel_id
                    print(f"  Registered channel for '{topic}' (Schema: {schema_name}, ID: {channel_id})")
                except Exception as e:
                    print(f"Error registering channel for custom topic '{topic}': {e}")

            # Process each timestamp
            start_time = datetime.now()
            total_images_processed = 0
            total_image_errors = 0
            camera_stats = {cam['mcap_topic']: {'processed': 0, 'errors': 0} for cam in config['cameras'] if 'mcap_topic' in cam}

            for i, timestamp in enumerate(joint_timestamps):
                # Create ROS Time message
                ros_time = Time()
                ros_time.sec = int(timestamp)
                ros_time.nanosec = int((timestamp - int(timestamp)) * 1_000_000_000)

                # Process joint states
                if joint_positions is not None:
                    positions = joint_positions[i]
                    velocities = None
                    if joint_velocities is not None:
                        velocities = joint_velocities[i]
                    efforts = None
                    if joint_efforts is not None:
                        efforts = joint_efforts[i]

                    joint_state_msg = create_joint_state_message(
                        joint_names,
                        positions,
                        velocities,
                        efforts,
                        ros_time
                    )

                    # Write joint state message
                    try:
                        writer.add_message(
                            channel_id=joint_state_channel_id,
                            log_time=ros_time_to_timestamp_ns(ros_time),
                            data=serialize_message(joint_state_msg),
                            publish_time=ros_time_to_timestamp_ns(ros_time)
                        )
                    except Exception as e:
                         print(f"Error serializing/writing JointState message at frame {i}: {e}")
                         if args.debug: traceback.print_exc()


                # Create transforms for each joint
                transforms = []
                for j, (joint_name, joint_info) in enumerate(joints.items()):
                    joint_pos = 0.0
                    mimic_info = joint_info.get('mimic')

                    # Determine joint position: use mimic calculation or direct value
                    if mimic_info is not None:
                        # This is a mimic joint - calculate its position
                        mimicked_joint_name = mimic_info['joint']
                        multiplier = mimic_info['multiplier']
                        offset = mimic_info['offset']
                        try:
                            mimicked_joint_idx = joint_names.index(mimicked_joint_name)
                            mimicked_pos = 0.0
                            if joint_positions is not None and mimicked_joint_idx < joint_positions.shape[1]:
                                mimicked_pos = joint_positions[i, mimicked_joint_idx]
                            else:
                                # This case might happen if the mimicked joint itself is beyond the HDF5 data range
                                # Usually, the mimicked joint (like gripper_controller) *is* included or calculated
                                print(f"Warning: Mimicked joint '{mimicked_joint_name}' (index {mimicked_joint_idx}) not found in position data for frame {i}. Using 0.0 for mimic source.")

                            joint_pos = mimicked_pos * multiplier + offset
                        except ValueError:
                            print(f"Error: Joint '{joint_name}' mimics '{mimicked_joint_name}', which was not found in URDF joint list. Skipping TF for this joint.")
                            continue # Skip this joint
                        except Exception as e:
                             print(f"Error calculating mimic position for '{joint_name}': {e}")
                             continue # Skip this joint
                    else:
                        # Not a mimic joint - get position directly if available
                        if joint_positions is not None and j < joint_positions.shape[1]:
                            joint_pos = joint_positions[i, j]
                        # Otherwise, joint_pos remains 0.0 (default)

                    origin = joint_info['origin']
                    axis = joint_info.get('axis', [1.0, 0.0, 0.0]) # Default to X axis if missing

                    # Calculate static transform from origin tag
                    origin_translation = origin['xyz']
                    origin_rotation_q = rpy_to_quaternion(origin['rpy'][0], origin['rpy'][1], origin['rpy'][2])

                    # Calculate motion transform based on joint type and position
                    motion_translation = [0.0, 0.0, 0.0]
                    motion_rotation_q = [1.0, 0.0, 0.0, 0.0] # Identity quaternion

                    if joint_info['type'] == 'revolute' or joint_info['type'] == 'continuous':
                        motion_rotation_q = quaternion_from_axis_angle(axis, joint_pos)
                    elif joint_info['type'] == 'prismatic':
                        # Translation occurs along the specified axis
                        motion_translation[0] = axis[0] * joint_pos
                        motion_translation[1] = axis[1] * joint_pos
                        motion_translation[2] = axis[2] * joint_pos
                    # No motion for 'fixed' joints

                    # Combine transforms: T_final = T_origin * T_motion
                    # Final rotation: q_final = q_origin * q_motion
                    final_rotation_q = quaternion_multiply(origin_rotation_q, motion_rotation_q)

                    # Final translation: t_final = t_origin + rotate_by_q_origin(t_motion)
                    # Rotate motion translation by origin rotation before adding
                    rotated_motion_translation = rotate_vector_by_quaternion(motion_translation, origin_rotation_q)
                    final_translation = [
                        origin_translation[0] + rotated_motion_translation[0],
                        origin_translation[1] + rotated_motion_translation[1],
                        origin_translation[2] + rotated_motion_translation[2]
                    ]

                    transform = create_transform_message(
                        parent_frame=joint_info['parent'],
                        child_frame=joint_info['child'],
                        translation=final_translation,
                        rotation=final_rotation_q,
                        timestamp=ros_time
                    )
                    # --- DEBUG PRINT ADDED ---
                    if args.debug and i < 3: # Print only for first few frames
                        print(f"DEBUG_TF Frame {i}: Adding transform {transform.header.frame_id} -> {transform.child_frame_id}")
                    # --- END DEBUG PRINT ---
                    transforms.append(transform)

                # Write transform messages
                if transforms:
                    tf_msg = TFMessage()
                    tf_msg.transforms = transforms

                    try:
                        serialized_data = serialize_message(tf_msg)
                        writer.add_message(
                            channel_id=tf_channel_id,
                            log_time=ros_time_to_timestamp_ns(ros_time),
                            data=serialized_data,
                            publish_time=ros_time_to_timestamp_ns(ros_time)
                        )
                    except Exception as e:
                        print(f"Error serializing/writing TF message at frame {i}: {e}")
                        if args.debug:
                            traceback.print_exc()

                # Process images based on config - handle different sampling rates
                for camera_config in config['cameras']:
                    cam_topic = camera_config.get('mcap_topic')
                    hdf5_path = camera_config.get('hdf5_path')
                    cam_format = camera_config.get('format', 'jpeg').lower()
                    frame_id = camera_config.get('frame_id', 'camera_link') # Default frame_id
                    jpeg_quality = camera_config.get('quality', 95) # Default quality
                    cam_sampling_rate = camera_config.get('sampling_rate', global_sampling_rate)

                    # Skip if channel wasn't registered (e.g., bad config entry)
                    if not cam_topic or cam_topic not in camera_channel_ids:
                        continue

                    # Skip if format is not supported (already warned during registration)
                    if cam_format != 'jpeg':
                         continue

                    channel_id = camera_channel_ids[cam_topic]
                    img_data = None
                    error_occurred = False
                    cam_frame_idx = 0  # Initialize for error reporting

                    try:
                        if hdf5_path not in f:
                            print(f"Warning: HDF5 path '{hdf5_path}' for camera '{cam_topic}' not found at frame {i}.")
                            error_occurred = True
                        else:
                            img_dataset = f[hdf5_path]

                            # Calculate camera frame index based on timestamp and camera sampling rate
                            cam_frame_idx = int(timestamp * cam_sampling_rate)

                            if cam_frame_idx < img_dataset.shape[0]:
                                img_data = img_dataset[cam_frame_idx]
                            else:
                                # Skip this frame if camera doesn't have data at this timestamp
                                continue

                        if img_data is not None:
                            jpeg_bytes = None
                            # Process based on format (currently only jpeg)
                            if isinstance(img_data, np.ndarray) and len(img_data.shape) == 3 and img_data.shape[2] == 3:
                                # Assume RGB, convert to JPEG
                                try:
                                    pil_img = PILImage.fromarray(img_data)
                                    buffer = BytesIO()
                                    pil_img.save(buffer, format="JPEG", quality=jpeg_quality)
                                    jpeg_bytes = buffer.getvalue()
                                except Exception as e:
                                    print(f"Error converting RGB to JPEG for '{cam_topic}' at frame {i}: {e}")
                                    error_occurred = True
                            elif isinstance(img_data, np.ndarray) and len(img_data.shape) == 1:
                                # Assume pre-compressed bytes (e.g., JPEG)
                                # Basic check for JPEG header
                                if img_data.shape[0] >= 3 and img_data[0] == 255 and img_data[1] == 216 and img_data[2] == 255:
                                     jpeg_bytes = img_data.tobytes()
                                else:
                                     print(f"Warning: Non-JPEG byte array found for '{cam_topic}' at frame {i}, skipping.")
                                     error_occurred = True # Treat as error if we expect JPEG
                            elif isinstance(img_data, bytes): # Already bytes
                                if len(img_data) >= 3 and img_data[0] == 255 and img_data[1] == 216 and img_data[2] == 255:
                                    jpeg_bytes = img_data
                                else:
                                    print(f"Warning: Non-JPEG bytes found for '{cam_topic}' at frame {i}, skipping.")
                                    error_occurred = True # Treat as error if we expect JPEG
                            else:
                                print(f"Warning: Unexpected data type '{type(img_data)}' for camera '{cam_topic}' at frame {i}, skipping.")
                                error_occurred = True

                            # Create and write message if we have JPEG bytes
                            if jpeg_bytes is not None:
                                try:
                                    compressed_msg = CompressedImage()
                                    compressed_msg.header = Header()
                                    compressed_msg.header.stamp = ros_time
                                    compressed_msg.header.frame_id = frame_id
                                    compressed_msg.format = "jpeg"
                                    compressed_msg.data = jpeg_bytes # Assign bytes directly

                                    serialized_compressed = serialize_message(compressed_msg)
                                    writer.add_message(
                                        channel_id=channel_id,
                                        log_time=ros_time_to_timestamp_ns(ros_time),
                                        data=serialized_compressed,
                                        publish_time=ros_time_to_timestamp_ns(ros_time)
                                    )
                                    # Increment success counts
                                    camera_stats[cam_topic]['processed'] += 1
                                    total_images_processed += 1
                                except Exception as e:
                                    print(f"Error serializing/writing image for '{cam_topic}' at joint frame {i} (cam frame {cam_frame_idx}): {e}")
                                    error_occurred = True
                                    if args.debug: traceback.print_exc()

                    except Exception as e:
                        print(f"General error processing camera '{cam_topic}' at joint frame {i} (cam frame {cam_frame_idx}): {e}")
                        error_occurred = True
                        if args.debug: traceback.print_exc()

                    if error_occurred:
                        camera_stats[cam_topic]['errors'] += 1
                        total_image_errors += 1

                # Process custom topics
                for custom_config in custom_topics:
                    topic = custom_config.get('mcap_topic')
                    script_path = custom_config.get('script_path')
                    handler_name = custom_config.get('handler_name')
                    # Use unified config to auto-detect handler function
                    handler_function = unified_config.get_custom_topic_handler(custom_config, 'hdf5_to_mcap')

                    if not topic or topic not in custom_channel_ids:
                        continue

                    # Create a unique key for this handler configuration
                    handler_key = f"{topic}:{handler_name or script_path}"

                    # Resolve handler path using new logic
                    resolved_script_path = resolve_custom_handler_path(
                        handler_name=handler_name,
                        script_path=script_path,
                        input_dir=args.input_dir,
                        custom_handlers=args.custom_handlers
                    )

                    if not resolved_script_path:
                        # Only warn once per missing handler
                        if handler_key not in _warned_missing_handlers:
                            _warned_missing_handlers.add(handler_key)
                            if handler_name:
                                print(f"Warning: Handler '{handler_name}' not found for custom topic '{topic}'")
                            else:
                                print(f"Warning: Could not resolve script path for custom topic '{topic}'")
                        continue

                    try:
                        # Load custom handler
                        handler = load_custom_handler(resolved_script_path, handler_function)
                        if not handler:
                            continue

                        # Get the specific dataset from HDF5 file
                        hdf5_path = custom_config.get('hdf5_path')
                        if not hdf5_path or hdf5_path not in f:
                            # Only warn once per missing dataset
                            dataset_key = f"{topic}:dataset:{hdf5_path}"
                            if dataset_key not in _warned_missing_handlers:
                                _warned_missing_handlers.add(dataset_key)
                                print(f"Warning: HDF5 path '{hdf5_path}' not found for custom topic '{topic}'")
                            continue

                        dataset = f[hdf5_path]

                        # Call handler with dataset instead of entire file
                        messages = handler(dataset, timestamp, i, custom_config)

                        if messages:
                            channel_id = custom_channel_ids[topic]
                            for msg in messages:
                                try:
                                    writer.add_message(
                                        channel_id=channel_id,
                                        log_time=ros_time_to_timestamp_ns(ros_time),
                                        data=serialize_message(msg),
                                        publish_time=ros_time_to_timestamp_ns(ros_time)
                                    )
                                except Exception as e:
                                    print(f"Error writing custom message for topic '{topic}' at frame {i}: {e}")
                                    if args.debug:
                                        traceback.print_exc()

                    except Exception as e:
                        print(f"Error processing custom topic '{topic}' at frame {i}: {e}")
                        if args.debug:
                            traceback.print_exc()

                # Print progress
                if i % 100 == 0 or i == num_frames - 1:
                    print(f"Processed {i+1}/{num_frames} frames")

            # Finalize MCAP file
            try:
                writer.finish()

                # Explicitly flush and close file
                mcap_file.flush()
                os.fsync(mcap_file.fileno())
                mcap_file.close()
                mcap_file = None

                # Verify file exists and has size
                if os.path.exists(args.output_file):
                    file_size = os.path.getsize(args.output_file)
                    if file_size == 0:
                        print(f"ERROR: Output file exists but has zero size!")
                        sys.exit(EXIT_IO_ERROR)
                    else:
                        print(f"Successfully created MCAP file with size: {file_size} bytes")
                else:
                    print(f"ERROR: Output file does not exist after writing: {args.output_file}")
                    sys.exit(EXIT_IO_ERROR)
            except Exception as e:
                print(f"ERROR: Failed to finish/flush MCAP writer: {e}")
                if args.debug:
                    traceback.print_exc()
                if mcap_file and not mcap_file.closed:
                    mcap_file.close()
                raise

            end_time = datetime.now()
            print(f"\nConversion completed in {(end_time - start_time).total_seconds():.2f} seconds")
            print(f"MCAP file saved to: {args.output_file}")
        except Exception as e:
            print(f"ERROR during file writing: {e}")
            if args.debug:
                traceback.print_exc()
            raise


if __name__ == "__main__":
    try:
        # Log basic info about conversion
        print(f"Processing HDF5 file: {args.hdf5_file}")
        print(f"Using URDF file: {args.urdf_file}")
        print(f"Using configuration file: {args.config_file}")
        print(f"Output will be saved to: {args.output_file}")

        convert_hdf5_to_mcap()
    except Exception as e:
        print(f"Error during conversion: {e}")
        if args.debug:
            traceback.print_exc()
        sys.exit(EXIT_RUNTIME_ERROR)
    finally:
        rclpy.shutdown()
