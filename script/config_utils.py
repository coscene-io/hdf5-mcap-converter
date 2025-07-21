#!/usr/bin/env python3
"""
Configuration utilities for unified bidirectional HDF5 ↔ MCAP conversion.
Handles parsing of unified configuration format and value mapping operations.
"""

import yaml
from typing import Dict, List, Any, Optional, Tuple
import numpy as np

class ConfigurationError(Exception):
    """Exception raised for configuration-related errors"""
    pass

class ValueMapper:
    """Handles bidirectional value mapping between HDF5 and URDF ranges"""

    def __init__(self, hdf5_range: List[float], urdf_range: List[float]):
        """
        Initialize value mapper with HDF5 and URDF value ranges.

        Args:
            hdf5_range: [min, max] values in HDF5 format
            urdf_range: [min, max] values in URDF format
        """
        if len(hdf5_range) != 2 or len(urdf_range) != 2:
            raise ConfigurationError("Value ranges must contain exactly 2 elements [min, max]")

        self.hdf5_min, self.hdf5_max = float(hdf5_range[0]), float(hdf5_range[1])
        self.urdf_min, self.urdf_max = float(urdf_range[0]), float(urdf_range[1])

        # Calculate range spans
        self.hdf5_span = self.hdf5_max - self.hdf5_min
        self.urdf_span = self.urdf_max - self.urdf_min

        # Validate ranges
        if abs(self.hdf5_span) < 1e-10:
            raise ConfigurationError(f"HDF5 range span too small: {hdf5_range}")
        if abs(self.urdf_span) < 1e-10:
            raise ConfigurationError(f"URDF range span too small: {urdf_range}")

    def hdf5_to_urdf(self, hdf5_value: float) -> float:
        """Convert HDF5 value to URDF value"""
        # Normalize HDF5 value to 0-1 range
        normalized = (hdf5_value - self.hdf5_min) / self.hdf5_span
        normalized = max(0.0, min(1.0, normalized))  # Clamp to [0, 1]

        # Scale to URDF range
        urdf_value = self.urdf_min + normalized * self.urdf_span
        return float(urdf_value)

    def urdf_to_hdf5(self, urdf_value: float) -> float:
        """Convert URDF value to HDF5 value"""
        # Normalize URDF value to 0-1 range
        normalized = (urdf_value - self.urdf_min) / self.urdf_span
        normalized = max(0.0, min(1.0, normalized))  # Clamp to [0, 1]

        # Scale to HDF5 range
        hdf5_value = self.hdf5_min + normalized * self.hdf5_span
        return float(hdf5_value)

class UnifiedConfig:
    """Handles unified configuration parsing and provides conversion-direction-agnostic access"""

    def __init__(self, config_path: str):
        """Load and parse unified configuration file"""
        self.config_path = config_path
        self.config = self._load_config()
        self.config_version = self.config.get('_config_version', '1.0')
        self._validate_config()

    def _load_config(self) -> Dict[str, Any]:
        """Load YAML configuration file"""
        try:
            with open(self.config_path, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            raise ConfigurationError(f"Configuration file not found: {self.config_path}")
        except yaml.YAMLError as e:
            raise ConfigurationError(f"Error parsing YAML configuration: {e}")

    def _validate_config(self):
        """Validate configuration structure"""
        required_sections = ['joint_states']
        for section in required_sections:
            if section not in self.config:
                raise ConfigurationError(f"Missing required configuration section: {section}")

        # Validate joint_states section
        js_config = self.config['joint_states']
        if 'hdf5_paths' not in js_config or 'position' not in js_config['hdf5_paths']:
            raise ConfigurationError("Missing required joint_states.hdf5_paths.position")

    def is_legacy_format(self) -> bool:
        """Check if this is a legacy configuration format"""
        # Legacy indicators
        if 'gripper_transformations' in self.config.get('joint_states', {}):
            return True
        if any('value_type' in gripper and gripper['value_type'] == 'reverse_normalized'
               for gripper in self.config.get('gripper', [])):
            return True
        if any('handler_function' in topic
               for topic in self.config.get('custom_topics', [])):
            return True
        return False

    def get_joint_order(self) -> List[str]:
        """Get joint order for HDF5 output, deriving from joint_mappings if needed"""
        js_config = self.config['joint_states']

        # Use explicit joint_order if provided
        if 'joint_order' in js_config:
            return js_config['joint_order']

        # Derive from joint_mappings
        if 'joint_mappings' in js_config:
            # Sort by HDF5 column index
            mappings = js_config['joint_mappings']
            sorted_joints = sorted(mappings.items(), key=lambda x: x[1])
            return [joint_name for joint_name, _ in sorted_joints]

        raise ConfigurationError("No joint_order or joint_mappings found in configuration")

    def get_gripper_mappers(self) -> Dict[str, Tuple[ValueMapper, Dict[str, Any]]]:
        """Get value mappers for gripper configurations"""
        mappers = {}

        for gripper_config in self.config.get('gripper', []):
            joint_name = gripper_config.get('urdf_joint_name')
            if not joint_name:
                continue

            # Handle unified format
            if 'value_mapping' in gripper_config:
                mapping = gripper_config['value_mapping']
                hdf5_range = mapping.get('hdf5_range')
                urdf_range = mapping.get('urdf_range')

                if hdf5_range and urdf_range:
                    mapper = ValueMapper(hdf5_range, urdf_range)
                    mappers[joint_name] = (mapper, gripper_config)

            # Handle legacy format for backward compatibility
            elif gripper_config.get('value_type') == 'normalized':
                output_min = gripper_config.get('output_min_position', 0.021)
                output_max = gripper_config.get('output_max_position', 0.057)
                # Assume legacy normalized format uses 0-1 range
                mapper = ValueMapper([0.0, 1.0], [output_min, output_max])
                mappers[joint_name] = (mapper, gripper_config)

        return mappers

    def get_custom_topic_handler(self, topic_config: Dict[str, Any], direction: str) -> str:
        """Get appropriate handler function name for custom topic based on direction"""
        # Check for explicit handler_function (legacy format)
        if 'handler_function' in topic_config:
            return topic_config['handler_function']

        # Auto-detect based on direction
        if direction == 'hdf5_to_mcap':
            return 'extract_to_mcap'
        elif direction == 'mcap_to_hdf5':
            return 'extract_to_hdf5'
        else:
            raise ConfigurationError(f"Unknown conversion direction: {direction}")

    def migrate_to_unified_format(self) -> Dict[str, Any]:
        """Migrate legacy configuration to unified format"""
        if not self.is_legacy_format():
            return self.config

        print("Detected legacy configuration format, migrating to unified format...")

        # Start with current config
        unified_config = self.config.copy()

        # Migrate gripper transformations
        js_config = unified_config.get('joint_states', {})
        if 'gripper_transformations' in js_config:
            gripper_transforms = js_config.pop('gripper_transformations')

            # Convert to unified gripper format
            if 'gripper' not in unified_config:
                unified_config['gripper'] = []

            for transform in gripper_transforms:
                if transform.get('value_type') == 'reverse_normalized':
                    # Convert reverse_normalized to unified value_mapping
                    gripper_config = {
                        'urdf_joint_name': transform.get('joint_name'),
                        'hdf5_path': '/observations/qpos',  # Assume default
                        'hdf5_index': transform.get('column_index'),
                        'value_mapping': {
                            'hdf5_range': [
                                transform.get('hdf5_min_value', 0.0),
                                transform.get('hdf5_max_value', 1.0)
                            ],
                            'urdf_range': [
                                transform.get('urdf_min_angle', 0.021),
                                transform.get('urdf_max_angle', 0.057)
                            ]
                        }
                    }
                    unified_config['gripper'].append(gripper_config)

        # Migrate custom topics
        for topic in unified_config.get('custom_topics', []):
            # Remove direction-specific handler_function
            if 'handler_function' in topic:
                topic.pop('handler_function')

        # Add metadata
        unified_config['_config_version'] = '2.0'
        unified_config['_migrated_from'] = 'legacy'

        return unified_config

    def __getitem__(self, key):
        """Allow dict-like access to config"""
        return self.config[key]

    def get(self, key, default=None):
        """Allow dict-like access to config with default"""
        return self.config.get(key, default)

def load_unified_config(config_path: str) -> UnifiedConfig:
    """Load and return unified configuration"""
    return UnifiedConfig(config_path)

def detect_config_format(config_path: str) -> str:
    """Detect configuration format version"""
    try:
        config = UnifiedConfig(config_path)
        if config.is_legacy_format():
            return "legacy"
        else:
            return "unified"
    except Exception:
        return "unknown"
