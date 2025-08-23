#!/usr/bin/env python3
"""
Unified ROS log analysis tool for /rosout messages.
"""

from typing import Dict, Any, List, Optional, Callable
import logging
from pathlib import Path
from collections import defaultdict

from rosbags.rosbag2 import Reader

logger = logging.getLogger(__name__)

# ROS log levels
LOG_LEVELS = {
    "DEBUG": 1,
    "INFO": 2,
    "WARN": 4,
    "ERROR": 8,
    "FATAL": 16
}

LEVEL_NAMES = {v: k for k, v in LOG_LEVELS.items()}


async def analyze_logs(
    level_filter: Optional[str] = None,
    node_filter: Optional[str] = None,
    return_summary: bool = False,
    start_time: Optional[float] = None,
    end_time: Optional[float] = None,
    max_logs: int = 100,
    bag_path: Optional[str] = None,
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Unified log analysis tool.
    
    Args:
        level_filter: Filter by log level (DEBUG, INFO, WARN, ERROR, FATAL)
        node_filter: Filter by node name (partial match supported)
        return_summary: If True, return statistics instead of individual logs
        start_time: Optional start unix timestamp
        end_time: Optional end unix timestamp
        max_logs: Maximum logs to return when not summarizing
        bag_path: Optional specific bag file or directory
    """
    logger.info(f"Analyzing logs: level={level_filter}, node={node_filter}, summary={return_summary}")
    
    if not _get_bag_files_fn:
        return {"error": "Bag files function not provided"}
    
    bags = _get_bag_files_fn(bag_path)
    if not bags:
        return {"error": "No bag files found"}
    
    # Validate level filter if provided
    if level_filter and level_filter.upper() not in LOG_LEVELS:
        return {"error": f"Invalid log level: {level_filter}. Valid levels: {list(LOG_LEVELS.keys())}"}
    
    target_level = LOG_LEVELS.get(level_filter.upper()) if level_filter else None
    
    # Collect logs and statistics
    logs = []
    level_counts = defaultdict(int)
    node_counts = defaultdict(int)
    node_level_counts = defaultdict(lambda: defaultdict(int))
    errors = []
    warnings = []
    fatals = []
    first_timestamp = None
    last_timestamp = None
    
    for bag_path in bags:
        try:
            with Reader(bag_path) as reader:
                # Check if /rosout exists
                if "/rosout" not in [conn.topic for conn in reader.connections]:
                    continue
                
                for conn, timestamp, rawdata in reader.messages():
                    if conn.topic != "/rosout":
                        continue
                    
                    time_sec = timestamp / 1e9
                    
                    # Check time range
                    if start_time and time_sec < start_time:
                        continue
                    if end_time and time_sec > end_time:
                        continue
                    
                    # Update timestamps for summary
                    if first_timestamp is None or time_sec < first_timestamp:
                        first_timestamp = time_sec
                    if last_timestamp is None or time_sec > last_timestamp:
                        last_timestamp = time_sec
                    
                    _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                    
                    msg_level = msg_dict.get("level", 0)
                    msg_node = msg_dict.get("name", "unknown")
                    
                    # Apply filters
                    if target_level and msg_level != target_level:
                        continue
                    
                    if node_filter and node_filter.lower() not in msg_node.lower():
                        continue
                    
                    # Update statistics
                    level_name = LEVEL_NAMES.get(msg_level, f"UNKNOWN_{msg_level}")
                    level_counts[level_name] += 1
                    node_counts[msg_node] += 1
                    node_level_counts[msg_node][level_name] += 1
                    
                    # Create log entry
                    log_entry = {
                        "timestamp": time_sec,
                        "level": level_name,
                        "name": msg_node,
                        "msg": msg_dict.get("msg", ""),
                        "file": msg_dict.get("file", ""),
                        "function": msg_dict.get("function", ""),
                        "line": msg_dict.get("line", 0),
                        "topics": msg_dict.get("topics", [])
                    }
                    
                    # Collect special logs for summary
                    if msg_level == LOG_LEVELS["ERROR"]:
                        errors.append(log_entry)
                    elif msg_level == LOG_LEVELS["WARN"]:
                        warnings.append(log_entry)
                    elif msg_level == LOG_LEVELS["FATAL"]:
                        fatals.append(log_entry)
                    
                    # Add to main logs if not summarizing
                    if not return_summary:
                        logs.append(log_entry)
                        if len(logs) >= max_logs:
                            break
                
                if not return_summary and len(logs) >= max_logs:
                    break
                    
        except Exception as e:
            logger.error(f"Error reading {bag_path}: {e}")
            continue
    
    # Return appropriate format
    if return_summary:
        # Calculate derived statistics
        duration = (last_timestamp - first_timestamp) if first_timestamp and last_timestamp else 0
        total_logs = sum(level_counts.values())
        
        # Sort nodes by message count
        top_nodes = sorted(node_counts.items(), key=lambda x: x[1], reverse=True)[:10]
        
        # Find nodes with most errors
        nodes_with_errors = []
        for node, levels in node_level_counts.items():
            error_count = levels.get("ERROR", 0) + levels.get("FATAL", 0)
            if error_count > 0:
                nodes_with_errors.append((node, error_count))
        nodes_with_errors.sort(key=lambda x: x[1], reverse=True)
        
        return {
            "type": "summary",
            "statistics": {
                "total_logs": total_logs,
                "duration": duration,
                "logs_per_second": total_logs / duration if duration > 0 else 0,
                "unique_nodes": len(node_counts),
                "level_counts": dict(level_counts),
                "error_count": len(errors),
                "warning_count": len(warnings),
                "fatal_count": len(fatals)
            },
            "top_nodes": [{"name": name, "count": count} for name, count in top_nodes],
            "nodes_with_errors": [{"name": name, "error_count": count} 
                                 for name, count in nodes_with_errors[:10]],
            "recent_errors": errors[-5:] if errors else [],
            "recent_warnings": warnings[-5:] if warnings else [],
            "all_fatals": fatals,  # Always show all fatals
            "time_range": {
                "start": first_timestamp,
                "end": last_timestamp
            }
        }
    else:
        # Return filtered logs
        logger.info(f"Found {len(logs)} logs matching filters")
        
        return {
            "type": "logs",
            "filters": {
                "level": level_filter,
                "node": node_filter,
                "time_range": {
                    "start": start_time,
                    "end": end_time
                }
            },
            "log_count": len(logs),
            "logs": logs,
            "truncated": len(logs) >= max_logs
        }


def register_logging_tools(server, get_bag_files_fn, deserialize_message_fn, config):
    """Register logging analysis tools with the MCP server."""
    from mcp.types import Tool
    
    tools = [
        Tool(
            name="analyze_logs",
            description="Analyze ROS log messages from /rosout. Can filter by level/node or return summary statistics.",
            inputSchema={
                "type": "object",
                "properties": {
                    "level_filter": {"type": "string", 
                                   "enum": ["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
                                   "description": "Optional: filter by log level"},
                    "node_filter": {"type": "string", 
                                  "description": "Optional: filter by node name (partial match)"},
                    "return_summary": {"type": "boolean", 
                                     "description": "Return statistics instead of individual logs (default: false)"},
                    "start_time": {"type": "number", "description": "Optional: start unix timestamp"},
                    "end_time": {"type": "number", "description": "Optional: end unix timestamp"},
                    "max_logs": {"type": "integer", "description": "Maximum logs to return (default: 100)"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory"}
                }
            }
        )
    ]
    
    # Create handler
    async def handle_analyze_logs(args):
        return await analyze_logs(
            args.get("level_filter"),
            args.get("node_filter"),
            args.get("return_summary", False),
            args.get("start_time"),
            args.get("end_time"),
            args.get("max_logs", config.get('logging', {}).get('default_limit', 100)),
            args.get("bag_path"),
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    return tools, {
        "analyze_logs": handle_analyze_logs
    }