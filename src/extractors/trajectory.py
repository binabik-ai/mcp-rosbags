#!/usr/bin/env python3
"""
Trajectory analysis tool for comprehensive path analysis.
"""

import numpy as np
from typing import Dict, Any, List, Optional, Callable
import logging
from pathlib import Path

from rosbags.rosbag2 import Reader

logger = logging.getLogger(__name__)


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


def detect_waypoints(
    points: List[Dict[str, Any]],
    angle_threshold: float = 15.0,  # degrees
    min_distance: float = 0.5  # minimum distance between waypoints
) -> List[Dict[str, Any]]:
    """Detect significant waypoints where direction changes significantly."""
    if len(points) < 3:
        return points
    
    waypoints = [points[0]]  # Start position
    last_waypoint = points[0]
    
    for i in range(1, len(points) - 1):
        # Check distance from last waypoint
        curr_pos = points[i].get("position", points[i])
        distance_from_last = calculate_distance_2d(
            last_waypoint.get("position", last_waypoint),
            curr_pos
        )
        
        if distance_from_last < min_distance:
            continue
        
        # Calculate direction vectors
        prev_pos = points[i-1].get("position", points[i-1])
        next_pos = points[i+1].get("position", points[i+1])
        
        dx1 = curr_pos.get('x', 0) - prev_pos.get('x', 0)
        dy1 = curr_pos.get('y', 0) - prev_pos.get('y', 0)
        dx2 = next_pos.get('x', 0) - curr_pos.get('x', 0)
        dy2 = next_pos.get('y', 0) - curr_pos.get('y', 0)
        
        # Normalize vectors
        mag1 = (dx1**2 + dy1**2) ** 0.5
        mag2 = (dx2**2 + dy2**2) ** 0.5
        
        if mag1 > 0.01 and mag2 > 0.01:
            dx1, dy1 = dx1/mag1, dy1/mag1
            dx2, dy2 = dx2/mag2, dy2/mag2
            
            # Calculate angle change
            dot_product = dx1*dx2 + dy1*dy2
            angle_change = abs(np.arccos(np.clip(dot_product, -1, 1)))
            
            if np.degrees(angle_change) > angle_threshold:
                waypoints.append(points[i])
                last_waypoint = points[i]
    
    # Always add end position
    waypoints.append(points[-1])
    
    return waypoints


def extract_position(msg_data: Dict[str, Any]) -> Optional[Dict[str, float]]:
    """Extract position from various message types."""
    if "position" in msg_data:
        return msg_data["position"]
    elif "pose" in msg_data and isinstance(msg_data["pose"], dict):
        if "position" in msg_data["pose"]:
            return msg_data["pose"]["position"]
        elif "pose" in msg_data["pose"] and "position" in msg_data["pose"]["pose"]:
            return msg_data["pose"]["pose"]["position"]
    elif "transform" in msg_data and isinstance(msg_data["transform"], dict):
        if "translation" in msg_data["transform"]:
            trans = msg_data["transform"]["translation"]
            return {"x": trans.get("x", 0), "y": trans.get("y", 0), "z": trans.get("z", 0)}
    return None


async def analyze_trajectory(
    pose_topic: str,
    start_time: float,
    end_time: float,
    include_waypoints: bool = True,
    max_waypoints: int = 20,
    bag_path: Optional[str] = None,
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Analyze trajectory with comprehensive statistics and waypoint detection.
    
    Args:
        pose_topic: Topic containing pose/odometry data
        start_time: Start unix timestamp in seconds
        end_time: End unix timestamp in seconds
        include_waypoints: Whether to detect and include waypoints
        max_waypoints: Maximum number of waypoints to return
        bag_path: Optional specific bag file or directory
    """
    logger.info(f"Analyzing trajectory for {pose_topic} from {start_time} to {end_time}")
    
    if not _get_bag_files_fn:
        return {"error": "Bag files function not provided"}
    
    bags = _get_bag_files_fn(bag_path)
    if not bags:
        return {"error": "No bag files found"}
    
    start_ns = int(start_time * 1e9)
    end_ns = int(end_time * 1e9)
    
    # Collect all positions
    positions = []
    
    for bag_path in bags:
        try:
            with Reader(bag_path) as reader:
                # Check if topic exists
                if pose_topic not in [conn.topic for conn in reader.connections]:
                    continue
                
                for conn, timestamp, rawdata in reader.messages():
                    if conn.topic != pose_topic:
                        continue
                    
                    if start_ns <= timestamp <= end_ns:
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        position = extract_position(msg_dict)
                        
                        if position:
                            positions.append({
                                "timestamp": timestamp / 1e9,
                                "position": position
                            })
                        
        except Exception as e:
            logger.error(f"Error reading {bag_path}: {e}")
            continue
    
    if len(positions) < 2:
        return {"error": "Insufficient position data for trajectory analysis"}
    
    # Sort by timestamp
    positions.sort(key=lambda p: p["timestamp"])
    
    # Calculate statistics
    total_distance_2d = 0.0
    total_distance_3d = 0.0
    max_speed_2d = 0.0
    max_speed_3d = 0.0
    speeds = []
    
    # Track motion segments
    stationary_time = 0.0
    moving_time = 0.0
    stationary_threshold = 0.1  # m/s
    
    for i in range(1, len(positions)):
        # Calculate distances
        dist_2d = calculate_distance_2d(positions[i-1]["position"], positions[i]["position"])
        dist_3d = calculate_distance_3d(positions[i-1]["position"], positions[i]["position"])
        total_distance_2d += dist_2d
        total_distance_3d += dist_3d
        
        # Calculate speeds
        dt = positions[i]["timestamp"] - positions[i-1]["timestamp"]
        if dt > 0:
            speed_2d = dist_2d / dt
            speed_3d = dist_3d / dt
            speeds.append(speed_2d)
            max_speed_2d = max(max_speed_2d, speed_2d)
            max_speed_3d = max(max_speed_3d, speed_3d)
            
            # Track motion segments
            if speed_2d < stationary_threshold:
                stationary_time += dt
            else:
                moving_time += dt
    
    # Calculate duration and average speed
    duration = positions[-1]["timestamp"] - positions[0]["timestamp"]
    avg_speed_2d = total_distance_2d / duration if duration > 0 else 0
    avg_speed_3d = total_distance_3d / duration if duration > 0 else 0
    
    # Calculate displacement (straight-line distance from start to end)
    displacement_2d = calculate_distance_2d(positions[0]["position"], positions[-1]["position"])
    displacement_3d = calculate_distance_3d(positions[0]["position"], positions[-1]["position"])
    
    # Calculate path efficiency (displacement / total distance)
    path_efficiency = displacement_2d / total_distance_2d if total_distance_2d > 0 else 0
    
    # Calculate bounding box
    x_coords = [p["position"]["x"] for p in positions]
    y_coords = [p["position"]["y"] for p in positions]
    z_coords = [p["position"].get("z", 0) for p in positions]
    
    bounding_box = {
        "min": {"x": min(x_coords), "y": min(y_coords), "z": min(z_coords)},
        "max": {"x": max(x_coords), "y": max(y_coords), "z": max(z_coords)},
        "size": {
            "x": max(x_coords) - min(x_coords),
            "y": max(y_coords) - min(y_coords),
            "z": max(z_coords) - min(z_coords)
        }
    }
    
    result = {
        "pose_topic": pose_topic,
        "start_time": start_time,
        "end_time": end_time,
        "duration": duration,
        "total_points": len(positions),
        "distance_metrics": {
            "total_distance_2d": round(total_distance_2d, 3),
            "total_distance_3d": round(total_distance_3d, 3),
            "displacement_2d": round(displacement_2d, 3),
            "displacement_3d": round(displacement_3d, 3),
            "path_efficiency": round(path_efficiency, 3)
        },
        "speed_metrics": {
            "average_speed_2d": round(avg_speed_2d, 3),
            "average_speed_3d": round(avg_speed_3d, 3),
            "max_speed_2d": round(max_speed_2d, 3),
            "max_speed_3d": round(max_speed_3d, 3),
            "median_speed": round(np.median(speeds), 3) if speeds else 0
        },
        "motion_analysis": {
            "moving_time": round(moving_time, 2),
            "stationary_time": round(stationary_time, 2),
            "moving_percentage": round(100 * moving_time / duration, 1) if duration > 0 else 0
        },
        "start_position": {
            "x": round(positions[0]["position"]["x"], 3),
            "y": round(positions[0]["position"]["y"], 3),
            "z": round(positions[0]["position"].get("z", 0), 3)
        },
        "end_position": {
            "x": round(positions[-1]["position"]["x"], 3),
            "y": round(positions[-1]["position"]["y"], 3),
            "z": round(positions[-1]["position"].get("z", 0), 3)
        },
        "bounding_box": bounding_box
    }
    
    # Add waypoints if requested
    if include_waypoints:
        # Downsample first if we have too many points
        if len(positions) > 500:
            step = len(positions) // 500
            downsampled = positions[::step]
            if downsampled[-1] != positions[-1]:
                downsampled.append(positions[-1])
        else:
            downsampled = positions
        
        # Detect waypoints
        waypoints = detect_waypoints(downsampled)
        
        # Limit waypoints if needed
        if len(waypoints) > max_waypoints:
            # Keep start, end, and evenly distributed waypoints
            indices = np.linspace(0, len(waypoints)-1, max_waypoints, dtype=int)
            waypoints = [waypoints[i] for i in indices]
        
        result["waypoints"] = [
            {
                "timestamp": w["timestamp"],
                "position": {
                    "x": round(w["position"]["x"], 3),
                    "y": round(w["position"]["y"], 3),
                    "z": round(w["position"].get("z", 0), 3)
                }
            }
            for w in waypoints
        ]
        result["waypoint_count"] = len(waypoints)
    
    logger.info(f"Trajectory analysis complete: {total_distance_2d:.2f}m in {duration:.2f}s")
    
    return result


def register_trajectory_tools(server, get_bag_files_fn, deserialize_message_fn, config):
    """Register trajectory analysis tools with the MCP server."""
    from mcp.types import Tool
    
    tools = [
        Tool(
            name="analyze_trajectory",
            description="Analyze robot/vehicle trajectory with comprehensive metrics including distance, speed, efficiency, and waypoints",
            inputSchema={
                "type": "object",
                "properties": {
                    "pose_topic": {"type": "string", "description": "Pose/odometry topic (e.g., /odom, /robot_pose)"},
                    "start_time": {"type": "number", "description": "Start unix timestamp in seconds"},
                    "end_time": {"type": "number", "description": "End unix timestamp in seconds"},
                    "include_waypoints": {"type": "boolean", "description": "Include waypoint detection (default: true)"},
                    "max_waypoints": {"type": "integer", "description": "Maximum waypoints to return (default: 20)"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory"}
                },
                "required": ["pose_topic", "start_time", "end_time"]
            }
        )
    ]
    
    # Create handler
    async def handle_analyze_trajectory(args):
        return await analyze_trajectory(
            args["pose_topic"],
            args["start_time"],
            args["end_time"],
            args.get("include_waypoints", True),
            args.get("max_waypoints", 20),
            args.get("bag_path"),
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    return tools, {
        "analyze_trajectory": handle_analyze_trajectory
    }