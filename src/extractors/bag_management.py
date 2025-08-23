#!/usr/bin/env python3
"""
Bag management tools for filtering and manipulating ROS bags.
"""

import os
import shutil
from typing import Dict, Any, List, Optional, Callable
import logging
from pathlib import Path
from datetime import datetime

from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import serialize_cdr
import sqlite3

logger = logging.getLogger(__name__)


async def filter_bag(
    source_bag: str,
    output_name: str,
    time_filter: Optional[Dict] = None,
    topics: Optional[Dict] = None,
    downsample: Optional[Dict] = None,
    bag_path: Optional[str] = None,
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Create a filtered copy of a bag file.
    
    Args:
        source_bag: Path to source bag file or directory
        output_name: Name for the output bag (will be created in same directory)
        time_filter: Time filtering options
            - {"first_seconds": 30} - Keep only first 30 seconds
            - {"start": t1, "end": t2} - Keep specific time range
        topics: Topic filtering
            - {"include": ["/odom", "/scan"]} - Only include these topics
            - {"exclude": ["/camera/image_raw"]} - Exclude these topics
        downsample: Downsampling options
            - {"topic": "/scan", "rate": 10} - Limit topic to 10 Hz
    """
    logger.info(f"Filtering bag {source_bag} to {output_name}")
    
    source_path = Path(source_bag)
    if not source_path.exists():
        return {"error": f"Source bag does not exist: {source_bag}"}
    
    # Determine output path
    if source_path.is_file():
        output_dir = source_path.parent
    else:
        output_dir = source_path.parent
    
    output_path = output_dir / output_name
    
    # Check if output already exists
    if output_path.exists():
        return {"error": f"Output already exists: {output_path}"}
    
    # Statistics
    stats = {
        "input_messages": 0,
        "output_messages": 0,
        "input_topics": set(),
        "output_topics": set(),
        "input_duration": 0,
        "output_duration": 0,
        "filtered_by_time": 0,
        "filtered_by_topic": 0,
        "downsampled": 0
    }
    
    # Track last message time for downsampling
    last_message_time = {}
    
    try:
        # Open source bag
        with Reader(source_path) as reader:
            # Get metadata
            connections = list(reader.connections)
            
            # Determine time range
            first_timestamp = None
            last_timestamp = None
            
            # First pass: get time range
            for conn, timestamp, _ in reader.messages():
                if first_timestamp is None:
                    first_timestamp = timestamp
                last_timestamp = timestamp
                stats["input_messages"] += 1
                stats["input_topics"].add(conn.topic)
            
            if first_timestamp is None:
                return {"error": "Source bag is empty"}
            
            stats["input_duration"] = (last_timestamp - first_timestamp) / 1e9
            
            # Determine filtering time range
            filter_start = first_timestamp
            filter_end = last_timestamp
            
            if time_filter:
                if "first_seconds" in time_filter:
                    filter_end = first_timestamp + int(time_filter["first_seconds"] * 1e9)
                elif "start" in time_filter and "end" in time_filter:
                    filter_start = int(time_filter["start"] * 1e9)
                    filter_end = int(time_filter["end"] * 1e9)
            
            # Determine topics to include
            include_topics = None
            exclude_topics = []
            
            if topics:
                if "include" in topics:
                    include_topics = topics["include"]
                if "exclude" in topics:
                    exclude_topics = topics["exclude"]
            
            # Create output bag
            with Writer(output_path) as writer:
                # Add connections for topics we want to keep
                topic_map = {}
                for conn in connections:
                    # Check topic filter
                    if include_topics and conn.topic not in include_topics:
                        continue
                    if conn.topic in exclude_topics:
                        continue
                    
                    # Add connection (rosbags uses different API)
                    topic_map[conn.topic] = writer.add_connection(
                        conn.topic,
                        conn.msgtype
                    )
                
                # Second pass: write filtered messages
                for conn, timestamp, rawdata in reader.messages():
                    # Check if topic is included
                    if conn.topic not in topic_map:
                        stats["filtered_by_topic"] += 1
                        continue
                    
                    # Check time filter
                    if timestamp < filter_start or timestamp > filter_end:
                        stats["filtered_by_time"] += 1
                        continue
                    
                    # Check downsampling
                    if downsample and downsample.get("topic") == conn.topic:
                        target_rate = downsample.get("rate", 10)
                        min_interval = 1e9 / target_rate  # In nanoseconds
                        
                        if conn.topic in last_message_time:
                            if timestamp - last_message_time[conn.topic] < min_interval:
                                stats["downsampled"] += 1
                                continue
                        
                        last_message_time[conn.topic] = timestamp
                    
                    # Write message
                    writer.write(topic_map[conn.topic], timestamp, rawdata)
                    stats["output_messages"] += 1
                    stats["output_topics"].add(conn.topic)
        
        # Calculate output duration
        if stats["output_messages"] > 0:
            with Reader(output_path) as reader:
                first_out = None
                last_out = None
                for _, timestamp, _ in reader.messages():
                    if first_out is None:
                        first_out = timestamp
                    last_out = timestamp
                
                if first_out and last_out:
                    stats["output_duration"] = (last_out - first_out) / 1e9
        
        # Get output file size
        if output_path.is_file():
            output_size = output_path.stat().st_size / (1024 * 1024)  # MB
        else:
            # For directory-based bags
            output_size = sum(f.stat().st_size for f in output_path.rglob("*")) / (1024 * 1024)
        
        logger.info(f"Successfully created filtered bag: {output_path}")
        
        return {
            "success": True,
            "output_path": str(output_path),
            "output_size_mb": round(output_size, 2),
            "statistics": {
                "input_messages": stats["input_messages"],
                "output_messages": stats["output_messages"],
                "compression_ratio": round(stats["output_messages"] / stats["input_messages"], 3) 
                                    if stats["input_messages"] > 0 else 0,
                "input_topics": len(stats["input_topics"]),
                "output_topics": len(stats["output_topics"]),
                "topics_kept": list(stats["output_topics"]),
                "input_duration": round(stats["input_duration"], 2),
                "output_duration": round(stats["output_duration"], 2),
                "messages_filtered": {
                    "by_time": stats["filtered_by_time"],
                    "by_topic": stats["filtered_by_topic"],
                    "by_downsampling": stats["downsampled"]
                }
            },
            "applied_filters": {
                "time_filter": time_filter,
                "topic_filter": topics,
                "downsample": downsample
            }
        }
        
    except Exception as e:
        logger.error(f"Error filtering bag: {e}", exc_info=True)
        
        # Clean up partial output if it exists
        if output_path.exists():
            try:
                if output_path.is_file():
                    output_path.unlink()
                else:
                    shutil.rmtree(output_path)
            except:
                pass
        
        return {"error": f"Failed to filter bag: {str(e)}"}


def register_bag_management_tools(server, get_bag_files_fn, deserialize_message_fn, config):
    """Register bag management tools with the MCP server."""
    from mcp.types import Tool
    
    tools = [
        Tool(
            name="filter_bag",
            description="Create a filtered copy of a bag file with time, topic, and rate filtering",
            inputSchema={
                "type": "object",
                "properties": {
                    "source_bag": {"type": "string", "description": "Path to source bag file or directory"},
                    "output_name": {"type": "string", "description": "Name for output bag (created in same directory)"},
                    "time_filter": {
                        "type": "object",
                        "description": "Time filtering: {first_seconds: 30} or {start: t1, end: t2}"
                    },
                    "topics": {
                        "type": "object",
                        "description": "Topic filtering: {include: [topics]} or {exclude: [topics]}"
                    },
                    "downsample": {
                        "type": "object",
                        "description": "Downsampling: {topic: '/scan', rate: 10}"
                    },
                    "bag_path": {"type": "string", "description": "Optional: base path for bags"}
                },
                "required": ["source_bag", "output_name"]
            }
        )
    ]
    
    # Create handler
    async def handle_filter_bag(args):
        return await filter_bag(
            args["source_bag"],
            args["output_name"],
            args.get("time_filter"),
            args.get("topics"),
            args.get("downsample"),
            args.get("bag_path"),
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    return tools, {
        "filter_bag": handle_filter_bag
    }