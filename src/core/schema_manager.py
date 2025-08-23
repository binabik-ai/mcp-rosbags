#!/usr/bin/env python3
"""
Schema-based message field extraction and transformation.
"""

import yaml
import numpy as np
import math
from pathlib import Path
from typing import Any, Dict, Optional, List
import logging

logger = logging.getLogger(__name__)


class SchemaManager:
    """Manages message schemas and field extraction."""
    
    def __init__(self, schema_file: Path, max_array_length: int = 100):
        self.schemas = {}
        self.max_array_length = max_array_length
        
        if schema_file.exists():
            with open(schema_file, 'r') as f:
                config = yaml.safe_load(f)
                self.schemas = config.get('schemas', {})
                logger.info(f"Loaded {len(self.schemas)} message schemas from {schema_file}")
        else:
            logger.warning(f"Schema file not found: {schema_file}")
    
    def extract_fields(self, msg_dict: Dict[str, Any], msg_type: str) -> Dict[str, Any]:
        """Extract fields based on schema or return processed full message."""
        # Normalize message type (handle both / and . notation)
        normalized_type = msg_type.replace('.', '/')
        
        if normalized_type not in self.schemas:
            # No schema defined, return full message with array processing
            return self._process_arrays(msg_dict)
        
        schema = self.schemas[normalized_type]
        result = {}
        
        for output_field, path in schema.items():
            value = self._extract_field(msg_dict, path)
            
            # Special processing for orientation -> euler conversion
            if output_field == 'orientation' and isinstance(value, dict):
                value = self._add_euler_angles(value)
            
            # Process arrays (downsample if needed)
            if isinstance(value, (list, np.ndarray)):
                value = self._downsample_array(value)
            
            result[output_field] = value
        
        return result
    
    def _extract_field(self, data: Dict[str, Any], path: str) -> Any:
        """Extract a field using dot notation path."""
        try:
            parts = path.split('.')
            result = data
            
            for part in parts:
                # Handle array indexing
                if '[' in part and ']' in part:
                    field_name = part[:part.index('[')]
                    index = int(part[part.index('[')+1:part.index(']')])
                    result = result[field_name][index]
                elif isinstance(result, dict):
                    result = result.get(part)
                else:
                    return None
            
            return result
        except Exception as e:
            logger.debug(f"Failed to extract {path}: {e}")
            return None
    
    def _add_euler_angles(self, quaternion: Dict[str, float]) -> Dict[str, Any]:
        """Add euler angles to quaternion dict."""
        try:
            x = quaternion.get('x', 0)
            y = quaternion.get('y', 0)
            z = quaternion.get('z', 0)
            w = quaternion.get('w', 1)
            
            # Roll (x-axis rotation)
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            
            # Pitch (y-axis rotation)
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)
            
            # Yaw (z-axis rotation)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return {
                'quaternion': quaternion,
                'euler': {
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw,
                    'roll_deg': math.degrees(roll),
                    'pitch_deg': math.degrees(pitch),
                    'yaw_deg': math.degrees(yaw)
                }
            }
        except Exception as e:
            logger.error(f"Failed to convert quaternion to euler: {e}")
            return {'quaternion': quaternion}
    
    def _downsample_array(self, array: list) -> list:
        """Downsample array to max_array_length if needed."""
        if len(array) <= self.max_array_length:
            return array
        
        # Simple uniform downsampling
        indices = np.linspace(0, len(array) - 1, self.max_array_length, dtype=int)
        return [array[i] for i in indices]
    
    def _process_arrays(self, data: Any) -> Any:
        """Recursively process arrays in a message dict."""
        if isinstance(data, dict):
            return {k: self._process_arrays(v) for k, v in data.items()}
        elif isinstance(data, list) and len(data) > self.max_array_length:
            return self._downsample_array(data)
        else:
            return data
    
    def get_position_field(self, msg_type: str) -> Optional[str]:
        """Get the position field path for a message type."""
        normalized_type = msg_type.replace('.', '/')
        
        # Common position extractors
        position_paths = {
            "nav_msgs/msg/Odometry": "pose.pose.position",
            "geometry_msgs/msg/PoseStamped": "pose.position",
            "geometry_msgs/msg/PoseWithCovarianceStamped": "pose.pose.position",
            "geometry_msgs/msg/Pose": "position",
            "geometry_msgs/msg/PoseWithCovariance": "pose.position",
            "geometry_msgs/msg/TransformStamped": "transform.translation",
        }
        
        return position_paths.get(normalized_type)
    
    def get_velocity_field(self, msg_type: str) -> Optional[str]:
        """Get the velocity field path for a message type."""
        normalized_type = msg_type.replace('.', '/')
        
        # Common velocity extractors
        velocity_paths = {
            "nav_msgs/msg/Odometry": "twist.twist",
            "geometry_msgs/msg/TwistStamped": "twist",
            "geometry_msgs/msg/TwistWithCovarianceStamped": "twist.twist",
            "geometry_msgs/msg/Twist": "",  # The message itself is the twist
        }
        
        return velocity_paths.get(normalized_type)