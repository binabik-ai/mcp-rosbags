#!/usr/bin/env python3
"""
Image extraction tool for retrieving images from ROS bags in LLM-compatible format.
"""

import base64
import numpy as np
from typing import Dict, Any, Optional, Callable
import logging
from pathlib import Path
from io import BytesIO

from rosbags.rosbag2 import Reader

# Use PIL for image processing (lightweight, no OpenCV needed)
from PIL import Image

logger = logging.getLogger(__name__)


def decode_image_message(msg_dict: Dict[str, Any]) -> Optional[np.ndarray]:
    """
    Decode various ROS image message types to numpy array.
    
    Supports:
    - sensor_msgs/Image
    - sensor_msgs/CompressedImage
    """
    # Check if it's a compressed image
    if "format" in msg_dict and "data" in msg_dict:
        # sensor_msgs/CompressedImage
        format_str = msg_dict["format"]
        data = msg_dict["data"]
        
        try:
            # Convert bytes to PIL Image
            if isinstance(data, list):
                data = bytes(data)
            
            img = Image.open(BytesIO(data))
            return np.array(img)
        except Exception as e:
            logger.error(f"Failed to decode compressed image: {e}")
            return None
    
    # Check if it's a raw image
    elif "encoding" in msg_dict and "data" in msg_dict:
        # sensor_msgs/Image
        height = msg_dict.get("height", 0)
        width = msg_dict.get("width", 0)
        encoding = msg_dict.get("encoding", "")
        data = msg_dict.get("data", [])
        
        if not height or not width:
            return None
        
        try:
            # Convert data to bytes if it's a list
            if isinstance(data, list):
                data = bytes(data)
            
            # Handle different encodings
            if encoding == "rgb8":
                img_array = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
            elif encoding == "bgr8":
                img_array = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
                # Convert BGR to RGB
                img_array = img_array[:, :, ::-1]
            elif encoding == "mono8":
                img_array = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
                # Convert grayscale to RGB
                img_array = np.stack([img_array] * 3, axis=-1)
            elif encoding == "16UC1" or encoding == "mono16":
                img_array = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
                # Normalize to 8-bit
                img_array = (img_array / 256).astype(np.uint8)
                img_array = np.stack([img_array] * 3, axis=-1)
            elif encoding == "32FC1":
                img_array = np.frombuffer(data, dtype=np.float32).reshape(height, width)
                # Normalize float to 8-bit
                img_array = ((img_array - img_array.min()) / (img_array.max() - img_array.min()) * 255).astype(np.uint8)
                img_array = np.stack([img_array] * 3, axis=-1)
            else:
                logger.warning(f"Unsupported image encoding: {encoding}")
                return None
            
            return img_array
            
        except Exception as e:
            logger.error(f"Failed to decode raw image: {e}")
            return None
    
    return None


def compress_image_for_llm(img_array: np.ndarray, max_size: int = 512, quality: int = 85) -> str:
    """
    Compress and encode image for LLM consumption.
    
    Args:
        img_array: Numpy array of image
        max_size: Maximum dimension (width or height)
        quality: JPEG quality (1-100)
        
    Returns:
        Base64-encoded JPEG string suitable for LLM
    """
    try:
        # Convert to PIL Image
        img = Image.fromarray(img_array.astype(np.uint8))
        
        # Resize if needed
        if img.width > max_size or img.height > max_size:
            # Calculate new size maintaining aspect ratio
            ratio = min(max_size / img.width, max_size / img.height)
            new_width = int(img.width * ratio)
            new_height = int(img.height * ratio)
            img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
        
        # Save to JPEG in memory
        buffer = BytesIO()
        img.save(buffer, format="JPEG", quality=quality, optimize=True)
        buffer.seek(0)
        
        # Encode to base64
        img_base64 = base64.b64encode(buffer.read()).decode('utf-8')
        
        # Return in data URI format that LLMs can understand
        return f"data:image/jpeg;base64,{img_base64}"
        
    except Exception as e:
        logger.error(f"Failed to compress image: {e}")
        return None


async def get_image_at_time(
    topic: str,
    timestamp: float,
    tolerance: float = 0.1,
    max_size: int = 512,
    quality: int = 85,
    bag_path: Optional[str] = None,
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Get an image from a camera topic at a specific time, encoded for LLM consumption.
    
    Args:
        topic: Image topic name (e.g., /camera/image_raw, /camera/image/compressed)
        timestamp: Unix timestamp in seconds
        tolerance: Time tolerance in seconds
        max_size: Maximum image dimension for LLM (default: 512px)
        quality: JPEG compression quality (default: 85)
        bag_path: Optional specific bag file or directory
        
    Returns:
        Dictionary with base64-encoded image and metadata
    """
    logger.info(f"Getting image from {topic} at {timestamp}")
    
    if not _get_bag_files_fn:
        return {"error": "Bag files function not provided"}
    
    bags = _get_bag_files_fn(bag_path)
    if not bags:
        return {"error": "No bag files found"}
    
    target_ns = int(timestamp * 1e9)
    tolerance_ns = int(tolerance * 1e9)
    
    closest_img = None
    closest_time = None
    min_diff = float('inf')
    msg_type = None
    
    for bag_path in bags:
        try:
            with Reader(bag_path) as reader:
                # Check if topic exists
                topic_found = False
                for conn in reader.connections:
                    if conn.topic == topic:
                        topic_found = True
                        msg_type = conn.msgtype
                        break
                
                if not topic_found:
                    continue
                
                # Find closest image
                for conn, msg_timestamp, rawdata in reader.messages():
                    if conn.topic != topic:
                        continue
                    
                    diff = abs(msg_timestamp - target_ns)
                    if diff < min_diff and diff <= tolerance_ns:
                        min_diff = diff
                        closest_time = msg_timestamp
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        closest_img = msg_dict
                        
        except Exception as e:
            logger.error(f"Error reading {bag_path}: {e}")
            continue
    
    if closest_img is None:
        return {"error": f"No image found on topic {topic} within {tolerance}s of timestamp {timestamp}"}
    
    # Decode the image
    img_array = decode_image_message(closest_img)
    
    if img_array is None:
        return {"error": f"Failed to decode image from topic {topic}"}
    
    # Compress and encode for LLM
    img_base64 = compress_image_for_llm(img_array, max_size, quality)
    
    if img_base64 is None:
        return {"error": "Failed to compress image for LLM"}
    
    # Get image metadata
    height, width = img_array.shape[:2]
    
    result = {
        "topic": topic,
        "timestamp": closest_time / 1e9,
        "time_diff": min_diff / 1e9,
        "message_type": msg_type,
        "image": img_base64,  # This is the data URI that LLMs can directly interpret
        "metadata": {
            "original_width": width,
            "original_height": height,
            "encoding": closest_img.get("encoding", closest_img.get("format", "unknown")),
            "compressed_size_kb": len(img_base64) / 1024,
            "max_dimension": max_size,
            "jpeg_quality": quality
        }
    }
    
    # Add camera info if available (often published on a separate topic)
    if "header" in closest_img:
        result["header"] = closest_img["header"]
    
    logger.info(f"Successfully extracted image: {width}x{height} -> {max_size}px max, {len(img_base64)/1024:.1f}KB")
    
    return result


def register_image_tools(server, get_bag_files_fn, deserialize_message_fn, config):
    """Register image extraction tools with the MCP server."""
    from mcp.types import Tool
    
    tools = [
        Tool(
            name="get_image_at_time",
            description="Get a camera image at a specific time, compressed and encoded for LLM analysis. Returns base64 JPEG that LLMs can directly interpret.",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "Image topic (e.g., /camera/image_raw, /camera/compressed)"},
                    "timestamp": {"type": "number", "description": "Unix timestamp in seconds"},
                    "tolerance": {"type": "number", "description": "Time tolerance in seconds (default: 0.1)"},
                    "max_size": {"type": "integer", "description": "Max image dimension in pixels (default: 512)"},
                    "quality": {"type": "integer", "description": "JPEG quality 1-100 (default: 85)"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory"}
                },
                "required": ["topic", "timestamp"]
            }
        )
    ]
    
    # Create handler
    async def handle_get_image_at_time(args):
        return await get_image_at_time(
            args["topic"],
            args["timestamp"],
            args.get("tolerance", 0.1),
            args.get("max_size", 512),
            args.get("quality", 85),
            args.get("bag_path"),
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    return tools, {
        "get_image_at_time": handle_get_image_at_time
    }