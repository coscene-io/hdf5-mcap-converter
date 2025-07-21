#!/usr/bin/env python3
"""
Example Multi-Type Handler
Demonstrates using different ROS message types in custom handlers.
"""

import numpy as np
from typing import Dict, List, Any, Tuple
from std_msgs.msg import Float64MultiArray, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

def extract_to_mcap(dataset, timestamp: float, frame_idx: int, config: Dict) -> List[Any]:
    """
    Extract data from HDF5 dataset and convert to various ROS message types.

    Args:
        dataset: HDF5 dataset object (from config['hdf5_path'])
        timestamp: Current timestamp in seconds
        frame_idx: Current frame index
        config: Configuration dictionary with custom parameters

    Returns:
        List of ROS messages to write to MCAP
    """
    try:
        if frame_idx >= dataset.shape[0]:
            return []

        # Get data for this frame
        data = dataset[frame_idx]

        # Get message type from config
        message_type = config.get('message_type', 'std_msgs/msg/Float64MultiArray')

        if message_type == 'std_msgs/msg/Float64MultiArray':
            return create_float64_multiarray(data, config)
        elif message_type == 'std_msgs/msg/Float32MultiArray':
            return create_float32_multiarray(data, config)
        elif message_type == 'geometry_msgs/msg/Twist':
            return create_twist_message(data, config)
        elif message_type == 'geometry_msgs/msg/PoseStamped':
            return create_pose_stamped_message(data, config, timestamp)
        elif message_type == 'sensor_msgs/msg/PointCloud2':
            return create_point_cloud2_message(data, config, timestamp)
        else:
            print(f"Unsupported message type: {message_type}")
            return []

    except Exception as e:
        print(f"Error extracting data at frame {frame_idx}: {e}")
        return []

def create_float64_multiarray(data, config: Dict) -> List[Any]:
    """Create Float64MultiArray message"""
    msg = Float64MultiArray()
    msg.layout = MultiArrayLayout()
    msg.layout.data_offset = 0

    # Ensure data is 1D
    if data.ndim > 1:
        data = data.flatten()

    dim = MultiArrayDimension()
    dim.label = config.get('array_label', 'data')
    dim.size = len(data)
    dim.stride = len(data)
    msg.layout.dim = [dim]

    msg.data = [float(x) for x in data]
    return [msg]

def create_float32_multiarray(data, config: Dict) -> List[Any]:
    """Create Float32MultiArray message"""
    msg = Float32MultiArray()
    msg.layout = MultiArrayLayout()
    msg.layout.data_offset = 0

    # Ensure data is 1D
    if data.ndim > 1:
        data = data.flatten()

    dim = MultiArrayDimension()
    dim.label = config.get('array_label', 'data')
    dim.size = len(data)
    dim.stride = len(data)
    msg.layout.dim = [dim]

    # Convert to float32
    msg.data = [float(np.float32(x)) for x in data]
    return [msg]

def create_twist_message(data, config: Dict) -> List[Any]:
    """Create Twist message from 6D data [vx, vy, vz, wx, wy, wz]"""
    if len(data) < 6:
        print(f"Warning: Twist requires 6 values, got {len(data)}. Padding with zeros.")
        data = np.pad(data, (0, max(0, 6 - len(data))), mode='constant')

    msg = Twist()

    # Linear velocity
    msg.linear = Vector3()
    msg.linear.x = float(data[0])
    msg.linear.y = float(data[1])
    msg.linear.z = float(data[2])

    # Angular velocity
    msg.angular = Vector3()
    msg.angular.x = float(data[3])
    msg.angular.y = float(data[4])
    msg.angular.z = float(data[5])

    return [msg]

def create_pose_stamped_message(data, config: Dict, timestamp: float) -> List[Any]:
    """Create PoseStamped message from 7D data [x, y, z, qx, qy, qz, qw]"""
    if len(data) < 7:
        print(f"Warning: PoseStamped requires 7 values, got {len(data)}. Padding with zeros.")
        data = np.pad(data, (0, max(0, 7 - len(data))), mode='constant')
        if len(data) >= 7 and data[6] == 0:  # Set default quaternion w=1
            data[6] = 1.0

    msg = PoseStamped()

    # Header
    msg.header = Header()
    msg.header.stamp = Time()
    msg.header.stamp.sec = int(timestamp)
    msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1_000_000_000)
    msg.header.frame_id = config.get('frame_id', 'base_link')

    # Pose
    msg.pose = Pose()

    # Position
    msg.pose.position = Point()
    msg.pose.position.x = float(data[0])
    msg.pose.position.y = float(data[1])
    msg.pose.position.z = float(data[2])

    # Orientation (quaternion)
    msg.pose.orientation = Quaternion()
    msg.pose.orientation.x = float(data[3])
    msg.pose.orientation.y = float(data[4])
    msg.pose.orientation.z = float(data[5])
    msg.pose.orientation.w = float(data[6])

    return [msg]

def create_point_cloud2_message(data, config: Dict, timestamp: float) -> List[Any]:
    """Create PointCloud2 message from 3D point data"""
    # Reshape data to Nx3 if needed
    if data.ndim == 1:
        if len(data) % 3 != 0:
            print(f"Warning: Point cloud data length {len(data)} not divisible by 3")
            return []
        data = data.reshape(-1, 3)
    elif data.ndim == 2 and data.shape[1] != 3:
        print(f"Warning: Point cloud data shape {data.shape} not Nx3")
        return []

    msg = PointCloud2()

    # Header
    msg.header = Header()
    msg.header.stamp = Time()
    msg.header.stamp.sec = int(timestamp)
    msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1_000_000_000)
    msg.header.frame_id = config.get('frame_id', 'base_link')

    # Point cloud structure
    msg.height = 1  # Unorganized point cloud
    msg.width = data.shape[0]
    msg.is_bigendian = False
    msg.point_step = 12  # 3 floats * 4 bytes each
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    # Define fields (x, y, z)
    fields = []
    for i, name in enumerate(['x', 'y', 'z']):
        field = PointField()
        field.name = name
        field.offset = i * 4  # 4 bytes per float32
        field.datatype = PointField.FLOAT32
        field.count = 1
        fields.append(field)
    msg.fields = fields

    # Convert data to bytes
    point_data = data.astype(np.float32).tobytes()
    msg.data = list(point_data)

    return [msg]

def extract_to_hdf5(messages: List[Tuple[float, Any]], config: Dict) -> Dict[str, np.ndarray]:
    """
    Extract data from various ROS message types and convert to HDF5 format.

    Args:
        messages: List of (timestamp, message) tuples
        config: Configuration dictionary

    Returns:
        Dictionary mapping HDF5 paths to numpy arrays
    """
    if not messages:
        return {}

    try:
        hdf5_path = config.get('hdf5_path', '/observations/custom_data')
        message_type = config.get('message_type', 'std_msgs/msg/Float64MultiArray')

        data_arrays = []

        for timestamp, msg in messages:
            if message_type == 'std_msgs/msg/Float64MultiArray':
                if hasattr(msg, 'data'):
                    data_arrays.append(np.array(msg.data))
                else:
                    data_arrays.append(np.array([0.0]))

            elif message_type == 'std_msgs/msg/Float32MultiArray':
                if hasattr(msg, 'data'):
                    data_arrays.append(np.array(msg.data, dtype=np.float32))
                else:
                    data_arrays.append(np.array([0.0], dtype=np.float32))

            elif message_type == 'geometry_msgs/msg/Twist':
                if hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                    twist_data = [
                        msg.linear.x, msg.linear.y, msg.linear.z,
                        msg.angular.x, msg.angular.y, msg.angular.z
                    ]
                    data_arrays.append(np.array(twist_data))
                else:
                    data_arrays.append(np.zeros(6))

            elif message_type == 'geometry_msgs/msg/PoseStamped':
                if hasattr(msg, 'pose'):
                    pose_data = [
                        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                        msg.pose.orientation.x, msg.pose.orientation.y,
                        msg.pose.orientation.z, msg.pose.orientation.w
                    ]
                    data_arrays.append(np.array(pose_data))
                else:
                    data_arrays.append(np.array([0, 0, 0, 0, 0, 0, 1]))  # Identity pose

            elif message_type == 'sensor_msgs/msg/PointCloud2':
                if hasattr(msg, 'data') and hasattr(msg, 'width'):
                    # Convert point cloud back to numpy array
                    point_data = np.frombuffer(bytes(msg.data), dtype=np.float32)
                    points = point_data.reshape(-1, 3)  # Assuming xyz points
                    data_arrays.append(points.flatten())  # Flatten for storage
                else:
                    data_arrays.append(np.array([0, 0, 0]))
            else:
                print(f"Unsupported message type for HDF5 conversion: {message_type}")
                data_arrays.append(np.array([0.0]))

        if data_arrays:
            # Stack arrays - handle variable lengths by padding if needed
            max_len = max(len(arr) for arr in data_arrays)
            padded_arrays = []
            for arr in data_arrays:
                if len(arr) < max_len:
                    padded = np.pad(arr, (0, max_len - len(arr)), mode='constant')
                    padded_arrays.append(padded)
                else:
                    padded_arrays.append(arr)

            result_array = np.stack(padded_arrays)
            return {hdf5_path: result_array}

        return {}

    except Exception as e:
        print(f"Error converting messages to HDF5: {e}")
        return {}

# Helper function for debugging
def get_handler_info():
    """Return information about supported message types and usage"""
    return {
        'description': 'Multi-type handler supporting various ROS message types',
        'supported_message_types': {
            'std_msgs/msg/Float64MultiArray': {
                'description': 'Multi-dimensional float64 arrays',
                'data_format': 'Any dimensional array, flattened to 1D',
                'config_params': ['array_label']
            },
            'std_msgs/msg/Float32MultiArray': {
                'description': 'Multi-dimensional float32 arrays',
                'data_format': 'Any dimensional array, flattened to 1D',
                'config_params': ['array_label']
            },
            'geometry_msgs/msg/Twist': {
                'description': 'Linear and angular velocity',
                'data_format': '6D array [vx, vy, vz, wx, wy, wz]',
                'config_params': []
            },
            'geometry_msgs/msg/PoseStamped': {
                'description': 'Position and orientation with timestamp',
                'data_format': '7D array [x, y, z, qx, qy, qz, qw]',
                'config_params': ['frame_id']
            },
            'sensor_msgs/msg/PointCloud2': {
                'description': '3D point cloud data',
                'data_format': 'Nx3 array or flattened 3N array [x1,y1,z1,x2,y2,z2,...]',
                'config_params': ['frame_id']
            }
        },
        'example_configs': [
            {
                'mcap_topic': '/robot/velocity',
                'hdf5_path': '/observations/velocity',
                'handler_name': 'example_multi_type_handler',
                'message_type': 'geometry_msgs/msg/Twist'
            },
            {
                'mcap_topic': '/robot/pose',
                'hdf5_path': '/observations/pose',
                'handler_name': 'example_multi_type_handler',
                'message_type': 'geometry_msgs/msg/PoseStamped',
                'frame_id': 'base_link'
            },
            {
                'mcap_topic': '/sensor/points',
                'hdf5_path': '/observations/point_cloud',
                'handler_name': 'example_multi_type_handler',
                'message_type': 'sensor_msgs/msg/PointCloud2',
                'frame_id': 'sensor_frame'
            }
        ]
    }
