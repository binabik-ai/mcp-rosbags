#!/usr/bin/env python3
"""
MCP server for rosbag memory capabilities with simplified tools.
"""

import asyncio
import json
import os
import yaml
import time
import base64
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
from io import BytesIO

from mcp.server import Server
from mcp.types import Tool, TextContent
import mcp.server.stdio

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# Import core utilities
from core.message_utils import msg_to_dict
from core.schema_manager import SchemaManager
from core.cache_manager import CacheManager

# Import simplified extractors
from extractors.search import register_search_tools
from extractors.logging import register_logging_tools
from extractors.trajectory import register_trajectory_tools
from extractors.visualization import register_visualization_tools
from extractors.bag_management import register_bag_management_tools
from extractors.lidar import register_lidar_tools
from extractors.tf_tree import register_tf_tools
from extractors.image import register_image_tools

import logging

# Setup logging
log_file = f"/tmp/mcp_rosbag_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Global configuration
CONFIG_DIR = Path(os.environ.get("MCP_ROSBAG_CONFIG", "./config"))
BAG_DIRECTORY = Path(os.environ.get("MCP_ROSBAG_DIR", "./rosbags"))
CURRENT_BAG_PATH = None  # Can be a single file or directory

# Create MCP server
server = Server("rosbag-memory")

# Global components
schema_manager: Optional[SchemaManager] = None
cache_manager: Optional[CacheManager] = None
config: Dict[str, Any] = {}

# Store tool handlers
tool_handlers = {}

def initialize_components():
    """Initialize all components from config."""
    global schema_manager, cache_manager, config
    
    # Load configuration
    config_file = CONFIG_DIR / "server_config.yaml"
    if config_file.exists():
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            logger.info(f"Loaded config from {config_file}")
    else:
        # Default configuration
        config = {
            "data": {
                "time_tolerance": 0.1,
                "max_range_messages": 100,
                "max_array_length": 100,
                "position_tolerance": 1.0
            },
            "cache": {
                "enabled": True,
                "max_size": 100,
                "ttl_seconds": 300
            },
            "logging": {
                "default_limit": 100
            }
        }
        logger.warning(f"No config file found at {config_file}, using defaults")
    
    # Initialize schema manager
    schema_file = CONFIG_DIR / "message_schemas.yaml"
    schema_manager = SchemaManager(schema_file, config['data'].get('max_array_length', 100))
    
    # Initialize cache manager (but don't expose clear_cache to LLM)
    if config['cache']['enabled']:
        cache_manager = CacheManager(
            max_size=config['cache']['max_size'],
            ttl_seconds=config['cache']['ttl_seconds']
        )
        logger.info("Cache manager initialized")

def _is_ros2_bag_dir(path: Path) -> bool:
    """Check if a directory is a ROS 2 bag directory."""
    return (path / 'metadata.yaml').exists()

def _get_bag_files(custom_path: Optional[str] = None) -> List[Path]:
    """Get all rosbag files in the directory or from custom path."""
    bags = []
    
    # Determine the search path
    if custom_path:
        search_path = Path(custom_path)
    else:
        search_path = CURRENT_BAG_PATH or BAG_DIRECTORY
    
    if not search_path or not search_path.exists():
        logger.warning(f"Search path does not exist: {search_path}")
        return []
    
    # Handle single file
    if search_path.is_file() and search_path.suffix in ['.mcap', '.db3']:
        logger.debug(f"Found single bag file: {search_path}")
        return [search_path]
    
    # Handle directory cases
    if search_path.is_dir():
        # Check if this is a ROS 2 bag directory itself
        if _is_ros2_bag_dir(search_path):
            logger.debug(f"Found ROS 2 bag directory: {search_path}")
            return [search_path]
        
        # Otherwise look for bags inside
        for item in search_path.iterdir():
            if item.is_dir() and _is_ros2_bag_dir(item):
                # ROS 2 bag directory
                bags.append(item)
            elif item.is_file() and item.suffix in ['.mcap', '.db3']:
                # Individual bag files
                bags.append(item)
        
        # Also check subdirectories one level deep
        for subdir in search_path.iterdir():
            if subdir.is_dir() and not _is_ros2_bag_dir(subdir):
                for item in subdir.iterdir():
                    if item.is_dir() and _is_ros2_bag_dir(item):
                        bags.append(item)
    
    logger.debug(f"Found {len(bags)} bag files/directories")
    return sorted(bags)

def _deserialize_message(rawdata: bytes, msg_type: str) -> Tuple[Any, Dict[str, Any]]:
    """Deserialize message data and apply schema extraction."""
    msg = deserialize_cdr(rawdata, msg_type)
    msg_dict = msg_to_dict(msg, msg_type)
    
    # Apply schema extraction if available
    if schema_manager:
        extracted = schema_manager.extract_fields(msg_dict, msg_type)
        return msg, extracted
    else:
        return msg, msg_dict

@server.list_tools()
async def list_tools() -> List[Tool]:
    """List available MCP tools."""
    # Core tools - simplified set
    core_tools = [
        Tool(
            name="set_bag_path",
            description="Set the path to a rosbag file or directory containing rosbags",
            inputSchema={
                "type": "object",
                "properties": {
                    "path": {"type": "string", "description": "Path to rosbag file or directory"}
                },
                "required": ["path"]
            }
        ),
        Tool(
            name="list_bags",
            description="List all available rosbag files in the current or specified directory",
            inputSchema={
                "type": "object",
                "properties": {
                    "path": {"type": "string", "description": "Optional: directory to search for bags (defaults to current bag path)"}
                }
            }
        ),
        Tool(
            name="bag_info",
            description="Get detailed information about a rosbag including all topics, message counts, and duration",
            inputSchema={
                "type": "object",
                "properties": {
                    "bag_path": {"type": "string", "description": "Path to rosbag file or directory"}
                },
                "required": ["bag_path"]
            }
        ),
        Tool(
            name="get_message_at_time",
            description="Get the message from a topic at a specific time",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "ROS topic name"},
                    "timestamp": {"type": "number", "description": "Unix timestamp in seconds"},
                    "tolerance": {"type": "number", "description": "Time tolerance in seconds (default: 0.1)"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory to search"}
                },
                "required": ["topic", "timestamp"]
            }
        ),
        Tool(
            name="get_messages_in_range",
            description="Get all messages from a topic within a time range",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "ROS topic name"},
                    "start_time": {"type": "number", "description": "Start unix timestamp in seconds"},
                    "end_time": {"type": "number", "description": "End unix timestamp in seconds"},
                    "max_messages": {"type": "integer", "description": "Maximum messages to return (default: 100)"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory to search"}
                },
                "required": ["topic", "start_time", "end_time"]
            }
        )
    ]
    
    # Register simplified extractors and get their tools
    search_tools, search_handlers = register_search_tools(server, _get_bag_files, _deserialize_message, config)
    logging_tools, logging_handlers = register_logging_tools(server, _get_bag_files, _deserialize_message, config)
    trajectory_tools, trajectory_handlers = register_trajectory_tools(server, _get_bag_files, _deserialize_message, config)
    visualization_tools, visualization_handlers = register_visualization_tools(server, _get_bag_files, _deserialize_message, config)
    bag_management_tools, bag_management_handlers = register_bag_management_tools(server, _get_bag_files, _deserialize_message, config)
    lidar_tools, lidar_handlers = register_lidar_tools(server, _get_bag_files, _deserialize_message, config)
    tf_tools, tf_handlers = register_tf_tools(server, _get_bag_files, _deserialize_message, config)
    image_tools, image_handlers = register_image_tools(server, _get_bag_files, _deserialize_message, config)
    
    # Store handlers globally
    global tool_handlers
    tool_handlers.update(search_handlers)
    tool_handlers.update(logging_handlers)
    tool_handlers.update(trajectory_handlers)
    tool_handlers.update(visualization_handlers)
    tool_handlers.update(bag_management_handlers)
    tool_handlers.update(lidar_handlers)
    tool_handlers.update(tf_handlers)
    tool_handlers.update(image_handlers)
    
    # Combine all tools
    return core_tools + search_tools + logging_tools + trajectory_tools + visualization_tools + bag_management_tools + lidar_tools + tf_tools + image_tools

@server.call_tool()
async def call_tool(name: str, arguments: Dict[str, Any]) -> List[TextContent]:
    """Handle tool calls."""
    logger.info(f"Tool called: {name} with arguments: {arguments}")
    
    # Check cache for certain tools
    cache_key = None
    if cache_manager and name in ["get_message_at_time", "get_messages_in_range"]:
        cache_key = cache_manager.get_key(name, arguments)
        cached_result = cache_manager.get(cache_key)
        if cached_result is not None:
            logger.debug(f"Cache hit for {name}")
            return [TextContent(type="text", text=json.dumps(cached_result, indent=2))]
    
    try:
        # Check if it's a core tool
        if name == "set_bag_path":
            result = await set_bag_path(arguments["path"])
        elif name == "get_message_at_time":
            result = await get_message_at_time(
                arguments["topic"],
                arguments["timestamp"],
                arguments.get("tolerance", config['data']['time_tolerance']),
                arguments.get("bag_path")
            )
        elif name == "get_messages_in_range":
            result = await get_messages_in_range(
                arguments["topic"],
                arguments["start_time"],
                arguments["end_time"],
                arguments.get("max_messages", config['data']['max_range_messages']),
                arguments.get("bag_path")
            )
        elif name == "list_bags":
            result = await list_bags(arguments.get("path"))
        elif name == "bag_info":
            result = await bag_info(arguments["bag_path"])
        # Check if it's an extractor tool
        elif name in tool_handlers:
            result = await tool_handlers[name](arguments)
        else:
            result = {"error": f"Unknown tool: {name}"}
            logger.error(f"Unknown tool called: {name}")
        
        # Cache the result if applicable
        if cache_manager and cache_key:
            cache_manager.set(cache_key, result)
        
        logger.debug(f"Tool {name} completed successfully")
        return [TextContent(type="text", text=json.dumps(result, indent=2))]
    
    except Exception as e:
        logger.error(f"Error in tool {name}: {e}", exc_info=True)
        return [TextContent(type="text", text=json.dumps({"error": str(e)}))]

async def set_bag_path(path: str) -> Dict[str, Any]:
    """Set the current bag path to a file or directory."""
    global CURRENT_BAG_PATH
    logger.info(f"Setting bag path to: {path}")
    
    path_obj = Path(path)
    if not path_obj.exists():
        logger.error(f"Path does not exist: {path}")
        return {"error": f"Path does not exist: {path}"}
    
    if path_obj.is_file() and path_obj.suffix not in ['.mcap', '.db3']:
        logger.error(f"Invalid bag file: {path}")
        return {"error": f"File is not a rosbag: {path}"}
    
    CURRENT_BAG_PATH = path_obj
    
    # Clear cache when changing bag path
    if cache_manager:
        cache_manager.clear()
    
    logger.info(f"Successfully set bag path to: {path}")
    
    # Return info about what was set
    if path_obj.is_file():
        return {
            "success": True,
            "type": "file",
            "path": str(path_obj),
            "size_mb": path_obj.stat().st_size / (1024 * 1024)
        }
    elif _is_ros2_bag_dir(path_obj):
        # This is a ROS 2 bag directory
        db3_files = list(path_obj.glob("*.db3"))
        mcap_files = list(path_obj.glob("*.mcap"))
        total_size = sum(f.stat().st_size for f in db3_files + mcap_files)
        return {
            "success": True,
            "type": "ros2_bag_directory",
            "path": str(path_obj),
            "db3_files": len(db3_files),
            "mcap_files": len(mcap_files),
            "total_size_mb": total_size / (1024 * 1024)
        }
    else:
        # Regular directory containing bags
        bag_files = _get_bag_files()
        return {
            "success": True,
            "type": "directory",
            "path": str(path_obj),
            "bag_count": len(bag_files),
            "bags": [str(f.name) for f in bag_files]
        }

async def get_message_at_time(topic: str, timestamp: float, tolerance: float, bag_path: Optional[str] = None) -> Dict[str, Any]:
    """Find the closest message to a given timestamp."""
    logger.info(f"Searching for message on topic {topic} at {timestamp} with tolerance {tolerance}s")
    
    target_ns = int(timestamp * 1e9)  # Convert to nanoseconds
    tolerance_ns = int(tolerance * 1e9)
    
    closest_msg = None
    closest_time = None
    min_diff = float('inf')
    
    bag_files = _get_bag_files(bag_path)
    logger.debug(f"Searching {len(bag_files)} bag files")
    
    for bag_file in bag_files:
        try:
            with Reader(bag_file) as reader:
                logger.debug(f"Reading bag file: {bag_file}")
                # Check if topic exists in this bag
                if topic not in [conn.topic for conn in reader.connections]:
                    logger.debug(f"Topic {topic} not found in {bag_file}")
                    continue
                
                for conn, timestamp, rawdata in reader.messages():
                    if conn.topic != topic:
                        continue
                    
                    diff = abs(timestamp - target_ns)
                    if diff < min_diff and diff <= tolerance_ns:
                        min_diff = diff
                        closest_time = timestamp
                        # Deserialize the message with schema extraction
                        _, msg_dict = _deserialize_message(rawdata, conn.msgtype)
                        closest_msg = msg_dict
        except Exception as e:
            logger.error(f"Error reading {bag_file}: {e}")
            continue
    
    if closest_msg is None or closest_time is None:
        logger.warning(f"No message found for topic {topic} within {tolerance}s of timestamp {timestamp}")
        return {
            "error": f"No message found for topic {topic} within {tolerance}s of timestamp {timestamp}"
        }
    
    logger.info(f"Found message at {closest_time / 1e9} with time diff {min_diff / 1e9}s")
    
    return {
        "topic": topic,
        "timestamp": closest_time / 1e9,  # Convert back to seconds
        "time_diff": min_diff / 1e9,  # How far from requested time
        "message": closest_msg
    }

async def get_messages_in_range(topic: str, start_time: float, end_time: float, max_messages: int, bag_path: Optional[str] = None) -> Dict[str, Any]:
    """Get all messages within a time range."""
    logger.info(f"Getting messages for topic {topic} between {start_time} and {end_time}")
    
    start_ns = int(start_time * 1e9)
    end_ns = int(end_time * 1e9)
    
    messages = []
    
    for bag_file in _get_bag_files(bag_path):
        try:
            with Reader(bag_file) as reader:
                # Check if topic exists
                if topic not in [conn.topic for conn in reader.connections]:
                    continue
                
                for conn, timestamp, rawdata in reader.messages():
                    if conn.topic != topic:
                        continue
                    
                    if start_ns <= timestamp <= end_ns:
                        _, msg_dict = _deserialize_message(rawdata, conn.msgtype)
                        messages.append({
                            "timestamp": timestamp / 1e9,
                            "message": msg_dict
                        })
                        
                        if len(messages) >= max_messages:
                            break
            
            if len(messages) >= max_messages:
                break
        except Exception as e:
            logger.error(f"Error reading {bag_file}: {e}")
            continue
    
    logger.info(f"Found {len(messages)} messages in range")
    
    return {
        "topic": topic,
        "start_time": start_time,
        "end_time": end_time,
        "message_count": len(messages),
        "messages": messages
    }

async def list_bags(path: Optional[str] = None) -> Dict[str, Any]:
    """List all available rosbag files in the directory."""
    logger.info(f"Listing bags from {path or 'current path'}")
    
    bag_files = _get_bag_files(path)
    bags_info = []
    
    for bag_file in bag_files:
        try:
            # Get basic info for each bag
            bag_data = {
                "path": str(bag_file),
                "name": bag_file.name,
                "type": "directory" if bag_file.is_dir() else "file"
            }
            
            # Add size info
            if bag_file.is_file():
                bag_data["size_mb"] = bag_file.stat().st_size / (1024 * 1024)
            else:
                # ROS 2 bag directory
                total_size = 0
                for f in bag_file.glob("*.db3"):
                    total_size += f.stat().st_size
                for f in bag_file.glob("*.mcap"):
                    total_size += f.stat().st_size
                bag_data["size_mb"] = total_size / (1024 * 1024)
            
            # Try to get quick metadata without reading all messages
            try:
                with Reader(bag_file) as reader:
                    # Just get topic count and connection info
                    topics = [conn.topic for conn in reader.connections]
                    bag_data["topic_count"] = len(set(topics))
                    bag_data["topics"] = list(set(topics))[:10]  # First 10 topics as preview
            except Exception as e:
                logger.debug(f"Could not read metadata for {bag_file}: {e}")
                bag_data["topic_count"] = "unknown"
                bag_data["topics"] = []
            
            bags_info.append(bag_data)
            
        except Exception as e:
            logger.error(f"Error processing bag {bag_file}: {e}")
            continue
    
    logger.info(f"Found {len(bags_info)} bags")
    
    # Sort by name
    bags_info.sort(key=lambda x: x["name"])
    
    return {
        "bag_count": len(bags_info),
        "bags": bags_info,
        "current_path": str(CURRENT_BAG_PATH) if CURRENT_BAG_PATH else str(BAG_DIRECTORY)
    }

async def bag_info(bag_path: str) -> Dict[str, Any]:
    """Get detailed information about a specific rosbag including all topics."""
    logger.info(f"Getting info for bag: {bag_path}")
    
    path = Path(bag_path)
    
    if not path.exists():
        logger.error(f"Path does not exist: {bag_path}")
        return {"error": f"Path does not exist: {bag_path}"}
    
    if not (path.is_file() and path.suffix in ['.mcap', '.db3']) and not _is_ros2_bag_dir(path):
        logger.error(f"Not a valid rosbag: {bag_path}")
        return {"error": f"Not a valid rosbag: {bag_path}"}
    
    info = {
        "path": str(path),
        "size_mb": 0,
        "duration": 0,
        "start_time": None,
        "end_time": None,
        "message_count": 0,
        "topics": {},  # Will contain ALL topics with full details
        "compression": None
    }
    
    # Calculate size
    if path.is_file():
        info["size_mb"] = path.stat().st_size / (1024 * 1024)
    else:
        # ROS 2 bag directory
        total_size = 0
        for f in path.glob("*.db3"):
            total_size += f.stat().st_size
        for f in path.glob("*.mcap"):
            total_size += f.stat().st_size
        info["size_mb"] = total_size / (1024 * 1024)
    
    try:
        with Reader(path) as reader:
            # Get compression info if available
            if hasattr(reader, 'compression_mode'):
                info["compression"] = reader.compression_mode
            
            # First pass: get topics and connections
            for conn in reader.connections:
                if conn.topic not in info["topics"]:
                    info["topics"][conn.topic] = {
                        "type": conn.msgtype,
                        "count": 0,
                        "frequency": 0.0,
                        "first_time": None,
                        "last_time": None
                    }
            
            # Second pass: count messages and get time range
            for conn, timestamp, _ in reader.messages():
                time_sec = timestamp / 1e9
                
                # Update global time range
                if info["start_time"] is None or time_sec < info["start_time"]:
                    info["start_time"] = time_sec
                if info["end_time"] is None or time_sec > info["end_time"]:
                    info["end_time"] = time_sec
                
                # Update topic stats
                topic_info = info["topics"][conn.topic]
                topic_info["count"] += 1
                info["message_count"] += 1
                
                # Update topic time range
                if topic_info["first_time"] is None or time_sec < topic_info["first_time"]:
                    topic_info["first_time"] = time_sec
                if topic_info["last_time"] is None or time_sec > topic_info["last_time"]:
                    topic_info["last_time"] = time_sec
            
            # Calculate duration and frequencies
            if info["start_time"] and info["end_time"]:
                info["duration"] = info["end_time"] - info["start_time"]
                
                # Calculate frequencies for each topic
                for topic, topic_info in info["topics"].items():
                    if topic_info["first_time"] and topic_info["last_time"]:
                        topic_duration = topic_info["last_time"] - topic_info["first_time"]
                        if topic_duration > 0:
                            topic_info["frequency"] = topic_info["count"] / topic_duration
            
            # Sort topics by message count (but keep ALL topics)
            info["topics"] = dict(sorted(info["topics"].items(), 
                                       key=lambda x: x[1]["count"], 
                                       reverse=True))
            
            # Add topic count for quick reference
            info["topic_count"] = len(info["topics"])
            
    except Exception as e:
        logger.error(f"Error reading bag: {e}", exc_info=True)
        return {"error": f"Error reading bag: {str(e)}"}
    
    # Format times nicely
    if info["start_time"]:
        info["start_time_human"] = datetime.fromtimestamp(info["start_time"]).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    if info["end_time"]:
        info["end_time_human"] = datetime.fromtimestamp(info["end_time"]).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    
    logger.info(f"Bag info: {info['message_count']} messages, {info['duration']:.2f}s duration, {info['topic_count']} topics")
    
    return info

async def main():
    """Run the MCP server."""
    import argparse
    
    parser = argparse.ArgumentParser(description="MCP server for rosbag memory")
    parser.add_argument("--bag-dir", default="./rosbags", help="Directory containing rosbag files")
    parser.add_argument("--config-dir", default="./config", help="Directory containing configuration files")
    args = parser.parse_args()
    
    global BAG_DIRECTORY, CONFIG_DIR
    BAG_DIRECTORY = Path(args.bag_dir)
    CONFIG_DIR = Path(args.config_dir)
    
    logger.info(f"Starting MCP rosbag server")
    logger.info(f"Bag directory: {BAG_DIRECTORY}")
    logger.info(f"Config directory: {CONFIG_DIR}")
    logger.info(f"Log file: {log_file}")
    
    # Initialize components
    initialize_components()
    
    try:
        async with mcp.server.stdio.stdio_server() as (read_stream, write_stream):
            await server.run(
                read_stream,
                write_stream,
                server.create_initialization_options()
            )
    except Exception as e:
        logger.error(f"Server error: {e}", exc_info=True)

if __name__ == "__main__":
    asyncio.run(main())