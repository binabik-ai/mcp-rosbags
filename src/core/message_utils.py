#!/usr/bin/env python3
"""
Utilities for ROS message conversion and field extraction.
"""

import numpy as np
from typing import Any, Dict, List
import logging

logger = logging.getLogger(__name__)

def msg_to_dict(msg: Any, msg_type: str = None) -> Dict[str, Any]:
    """Convert a ROS message to dictionary, handling numpy arrays."""
    # Try to import ROS 2 for better message handling
    try:
        from rosidl_runtime_py import message_to_dict
        if msg_type:
            try:
                return message_to_dict(msg)
            except:
                pass
    except ImportError:
        pass
    
    # Fallback to manual conversion
    if hasattr(msg, '__dataclass_fields__'):
        result = {}
        for field in msg.__dataclass_fields__:
            value = getattr(msg, field)
            # Handle numpy arrays
            if isinstance(value, np.ndarray):
                result[field] = value.tolist()
            # Recursively handle nested messages
            elif hasattr(value, '__dataclass_fields__'):
                result[field] = msg_to_dict(value)
            # Handle lists of messages
            elif isinstance(value, (list, tuple)):
                result[field] = [msg_to_dict(item) if hasattr(item, '__dataclass_fields__') else item for item in value]
            else:
                result[field] = value
        return result
    elif hasattr(msg, 'data'):
        return {"data": msg.data}
    else:
        # For complex messages, convert to string representation
        return {"raw": str(msg)}

def extract_field(msg_dict: Dict[str, Any], field_path: str) -> Any:
    """
    Extract a field from a message dictionary using dot notation.
    
    Examples:
        extract_field(msg, "pose.pose.position.x")
        extract_field(msg, "ranges[360]")  # Array indexing
    """
    try:
        parts = field_path.split('.')
        result = msg_dict
        
        for part in parts:
            # Check for array indexing
            if '[' in part and ']' in part:
                field_name = part[:part.index('[')]
                index = int(part[part.index('[')+1:part.index(']')])
                result = result[field_name][index]
            else:
                result = result[part]
        
        return result
    except (KeyError, IndexError, TypeError) as e:
        logger.debug(f"Failed to extract field {field_path}: {e}")
        return None

def quaternion_to_euler(q: Dict[str, float]) -> Dict[str, float]:
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    try:
        # Extract quaternion components
        x = q.get('x', 0)
        y = q.get('y', 0)
        z = q.get('z', 0)
        w = q.get('w', 1)
        
        # Convert to Euler angles
        import math
        
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
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw)
        }
    except Exception as e:
        logger.error(f"Failed to convert quaternion to euler: {e}")
        return None

def calculate_distance_2d(p1: Dict[str, float], p2: Dict[str, float]) -> float:
    """Calculate 2D Euclidean distance between two points."""
    dx = p2.get('x', 0) - p1.get('x', 0)
    dy = p2.get('y', 0) - p1.get('y', 0)
    return (dx**2 + dy**2) ** 0.5

def calculate_distance_3d(p1: Dict[str, float], p2: Dict[str, float]) -> float:
    """Calculate 3D Euclidean distance between two points."""
    dx = p2.get('x', 0) - p1.get('x', 0)
    dy = p2.get('y', 0) - p1.get('y', 0)
    dz = p2.get('z', 0) - p1.get('z', 0)
    return (dx**2 + dy**2 + dz**2) ** 0.5

def downsample_trajectory(points: List[Dict[str, Any]], max_points: int = 100) -> List[Dict[str, Any]]:
    """
    Downsample a trajectory to a maximum number of points while preserving key features.
    Uses a simple distance-based algorithm.
    """
    if len(points) <= max_points:
        return points
    
    # Always keep first and last points
    result = [points[0]]
    
    # Calculate total distance
    distances = []
    total_distance = 0
    for i in range(1, len(points)):
        d = calculate_distance_3d(points[i-1].get('position', {}), points[i].get('position', {}))
        distances.append(d)
        total_distance += d
    
    # Sample points based on cumulative distance
    target_spacing = total_distance / (max_points - 1)
    accumulated = 0
    
    for i, d in enumerate(distances):
        accumulated += d
        if accumulated >= target_spacing:
            result.append(points[i + 1])
            accumulated = 0
    
    # Always add the last point
    if result[-1] != points[-1]:
        result.append(points[-1])
    
    return result