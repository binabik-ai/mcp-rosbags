#!/usr/bin/env python3
"""
Unified search tool for finding messages based on conditions.
"""
import json
import re
from typing import Dict, Any, List, Optional, Callable
import logging
from pathlib import Path
from rosbags.rosbag2 import Reader

logger = logging.getLogger(__name__)

def _check_condition(msg_data: Dict[str, Any], condition_type: str, value: Any, config: Dict[str, Any], field_path: str = None) -> bool:
    """Check if a message matches a condition."""
    if condition_type == "contains":
        # For string messages - check if value is contained
        msg_str = _get_string_representation(msg_data)
        return str(value).lower() in msg_str.lower()
    
    elif condition_type == "equals":
        if field_path:
            msg_value = _extract_field_value(msg_data, field_path)
        else:
            msg_value = _get_scalar_value(msg_data)
        return msg_value == value
    
    elif condition_type == "greater_than":
        if field_path:
            msg_value = _extract_field_value(msg_data, field_path)
        else:
            msg_value = _get_scalar_value(msg_data)
        
        if msg_value is not None and isinstance(msg_value, (int, float)):
            try:
                return msg_value > float(value)
            except (ValueError, TypeError):
                return False
        return False
    
    elif condition_type == "less_than":
        if field_path:
            msg_value = _extract_field_value(msg_data, field_path)
        else:
            msg_value = _get_scalar_value(msg_data)
        
        if msg_value is not None and isinstance(msg_value, (int, float)):
            try:
                return msg_value < float(value)
            except (ValueError, TypeError):
                return False
        return False
    
    elif condition_type == "regex":
        # Regular expression matching
        msg_str = _get_string_representation(msg_data)
        try:
            pattern = re.compile(str(value))
            return pattern.search(msg_str) is not None
        except re.error:
            logger.error(f"Invalid regex pattern: {value}")
            return False
    
    elif condition_type == "near_position":
        # For pose messages - check if near a position
        position = _extract_position(msg_data)
        if position and isinstance(value, dict) and "x" in value and "y" in value:
            dx = position["x"] - value["x"]
            dy = position["y"] - value["y"]
            distance = (dx**2 + dy**2) ** 0.5
            tolerance = value.get("tolerance", config['data']['position_tolerance'])
            return distance <= tolerance
    
    elif condition_type == "field_equals":
        # Check if a specific field equals a value
        if isinstance(value, dict) and "field" in value and "value" in value:
            field_value = _extract_field_value(msg_data, value["field"])
            return field_value == value["value"]
    
    elif condition_type == "field_exists":
        # Check if a field exists
        return _extract_field_value(msg_data, str(value)) is not None
    
    return False

def _extract_matched_value(msg_data: Dict[str, Any], condition_type: str, field_path: str = None) -> Any:
    """Extract the actual value that matched the condition."""
    if condition_type in ["contains", "regex"]:
        return _get_string_representation(msg_data)
    elif condition_type in ["equals", "greater_than", "less_than"]:
        if field_path:
            return _extract_field_value(msg_data, field_path)
        else:
            return _get_scalar_value(msg_data)
    elif condition_type == "near_position":
        return _extract_position(msg_data)
    elif condition_type == "field_equals":
        return msg_data
    elif condition_type == "field_exists":
        return msg_data
    return None

def _get_string_representation(msg_data: Dict[str, Any]) -> str:
    """Get string representation of message data."""
    # Check common string fields
    if "data" in msg_data and isinstance(msg_data["data"], str):
        return msg_data["data"]
    elif "text" in msg_data:
        return str(msg_data["text"])
    elif "message" in msg_data:
        return str(msg_data["message"])
    elif "msg" in msg_data:
        return str(msg_data["msg"])
    elif "objects" in msg_data:
        return str(msg_data["objects"])
    else:
        # Convert entire message to string
        return json.dumps(msg_data)

def _get_scalar_value(msg_data: Dict[str, Any]) -> Any:
    """Get scalar value from message."""
    # Check common scalar fields
    if "data" in msg_data and isinstance(msg_data["data"], (bool, int, float)):
        return msg_data["data"]
    elif "value" in msg_data:
        return msg_data["value"]
    elif "state" in msg_data:
        return msg_data["state"]
    elif "level" in msg_data:
        return msg_data["level"]
    
    # Enhanced: Try to find any numeric value in the message
    def find_first_numeric(data):
        if isinstance(data, (int, float)):
            return data
        elif isinstance(data, dict):
            for key, value in data.items():
                result = find_first_numeric(value)
                if result is not None:
                    return result
        elif isinstance(data, list) and len(data) > 0:
            return find_first_numeric(data[0])
        return None
    
    return find_first_numeric(msg_data)

def _extract_position(msg_data: Dict[str, Any]) -> Optional[Dict[str, float]]:
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

def _extract_field_value(msg_data: Dict[str, Any], field_path: str) -> Any:
    """Extract a field value using dot notation (e.g., 'linear.x', 'pose.position.x')."""
    try:
        parts = field_path.split('.')
        result = msg_data
        for part in parts:
            if isinstance(result, dict):
                result = result.get(part)
            else:
                return None
        return result
    except Exception:
        return None

async def search_messages(
    topic: str,
    condition_type: str,
    value: Any,
    direction: str = "forward",
    limit: int = 100,
    start_time: Optional[float] = None,
    end_time: Optional[float] = None,
    correlate_with_topic: Optional[str] = None,
    correlation_tolerance: float = 0.5,
    bag_path: Optional[str] = None,
    field: Optional[str] = None,  # NEW: field path parameter
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Unified search for messages matching conditions.
    
    Args:
        topic: ROS topic to search
        condition_type: Type of condition (contains, equals, greater_than, less_than, regex, near_position, field_equals, field_exists)
        value: Value to match against
        direction: Search direction ("forward" or "backward")
        limit: Maximum number of matches to return
        start_time: Optional start time for search range
        end_time: Optional end time for search range
        correlate_with_topic: Optional topic to get correlated values from
        correlation_tolerance: Time tolerance for correlation in seconds
        bag_path: Optional specific bag file or directory
        field: Optional field path for numeric comparisons (e.g., 'linear.x')
    """
    logger.info(f"Searching {topic} for {condition_type}={value}, field={field}, direction={direction}, limit={limit}")
    
    if not _get_bag_files_fn:
        return {"error": "Bag files function not provided"}
    
    bags = _get_bag_files_fn(bag_path)
    if not bags:
        return {"error": "No bag files found"}
    
    # Process bags in appropriate order
    if direction == "backward":
        bags = list(reversed(bags))
    
    matches = []
    correlations = {} if correlate_with_topic else None
    
    for bag_path in bags:
        try:
            with Reader(bag_path) as reader:
                # Check if topic exists
                if topic not in [conn.topic for conn in reader.connections]:
                    continue
                
                # Collect messages (and optionally correlation messages)
                messages = []
                correlation_messages = {} if correlate_with_topic else None
                
                for conn, timestamp, rawdata in reader.messages():
                    time_sec = timestamp / 1e9
                    
                    # Check time range if specified
                    if start_time and time_sec < start_time:
                        continue
                    if end_time and time_sec > end_time:
                        continue
                    
                    if conn.topic == topic:
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        messages.append((time_sec, msg_dict, str(bag_path.name)))
                    elif correlate_with_topic and conn.topic == correlate_with_topic:
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        correlation_messages[time_sec] = msg_dict
                
                # Process messages in appropriate order
                if direction == "backward":
                    messages = list(reversed(messages))
                
                # Check conditions and collect matches
                for time_sec, msg_data, bag_name in messages:
                    if _check_condition(msg_data, condition_type, value, config, field):
                        match = {
                            "timestamp": time_sec,
                            "matched_value": _extract_matched_value(msg_data, condition_type, field),
                            "bag_file": bag_name
                        }
                        
                        # Add the full message data for context
                        match["message_data"] = msg_data
                        
                        # Add correlation if requested
                        if correlate_with_topic and correlation_messages:
                            # Find closest message within tolerance
                            closest_time = None
                            min_diff = float('inf')
                            for corr_time in correlation_messages.keys():
                                diff = abs(corr_time - time_sec)
                                if diff <= correlation_tolerance and diff < min_diff:
                                    min_diff = diff
                                    closest_time = corr_time
                            
                            if closest_time is not None:
                                match["correlated_data"] = {
                                    "topic": correlate_with_topic,
                                    "timestamp": closest_time,
                                    "time_diff": min_diff,
                                    "data": correlation_messages[closest_time]
                                }
                        
                        matches.append(match)
                        if len(matches) >= limit:
                            break
                
                if len(matches) >= limit:
                    break
                    
        except Exception as e:
            logger.error(f"Error reading {bag_path}: {e}")
            continue
    
    logger.info(f"Found {len(matches)} matches")
    
    result = {
        "topic": topic,
        "condition_type": condition_type,
        "condition_value": value,
        "field": field,
        "direction": direction,
        "match_count": len(matches),
        "matches": matches,
        "truncated": len(matches) >= limit
    }
    
    if correlate_with_topic:
        result["correlation_topic"] = correlate_with_topic
        result["correlation_tolerance"] = correlation_tolerance
    
    return result

def register_search_tools(server, get_bag_files_fn, deserialize_message_fn, config):
    """Register search tools with the MCP server."""
    from mcp.types import Tool
    
    tools = [
        Tool(
            name="search_messages",
            description="Search for messages matching conditions. Supports: contains, equals, greater_than, less_than, regex, near_position, field_equals, field_exists",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "ROS topic to search"},
                    "condition_type": {"type": "string", "description": "Type of condition to match"},
                    "value": {"type": "string", "description": "Value to match (format depends on condition_type)"},
                    "field": {"type": "string", "description": "Optional: field path for numeric comparisons (e.g., 'linear.x', 'pose.position.x')"},
                    "direction": {"type": "string", "description": "Search direction (default: forward)"},
                    "limit": {"type": "integer", "description": "Maximum matches to return (default: 100)"},
                    "start_time": {"type": "number", "description": "Optional: start unix timestamp"},
                    "end_time": {"type": "number", "description": "Optional: end unix timestamp"},
                    "correlate_with_topic": {"type": "string", "description": "Optional: get correlated values from another topic"},
                    "correlation_tolerance": {"type": "number", "description": "Time tolerance for correlation in seconds (default: 0.5)"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory"}
                },
                "required": ["topic", "condition_type", "value"]
            }
        )
    ]
    
    # Create handler
    async def handle_search_messages(args):
        return await search_messages(
            args["topic"],
            args["condition_type"],
            args["value"],
            args.get("direction", "forward"),
            args.get("limit", 100),
            args.get("start_time"),
            args.get("end_time"),
            args.get("correlate_with_topic"),
            args.get("correlation_tolerance", 0.5),
            args.get("bag_path"),
            args.get("field"),  # NEW: pass field parameter
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    return tools, {
        "search_messages": handle_search_messages
    }
