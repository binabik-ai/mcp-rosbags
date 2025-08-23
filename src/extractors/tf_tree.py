#!/usr/bin/env python3
"""
TF tree extraction tool for understanding coordinate frame relationships.
"""

from typing import Dict, Any, List, Optional, Callable, Set
import logging
from pathlib import Path
from collections import defaultdict

from rosbags.rosbag2 import Reader

logger = logging.getLogger(__name__)


def build_tf_tree(transforms: Dict[tuple, Dict]) -> Dict[str, Any]:
    """
    Build a hierarchical tree structure from transform relationships.
    
    Args:
        transforms: Dict mapping (parent, child) tuples to transform data
        
    Returns:
        Tree structure with root frames and hierarchy
    """
    # Build parent-child relationships
    children_map = defaultdict(list)
    parents_map = defaultdict(list)
    all_frames = set()
    
    for (parent, child), data in transforms.items():
        children_map[parent].append(child)
        parents_map[child].append(parent)
        all_frames.add(parent)
        all_frames.add(child)
    
    # Find root frames (frames with no parents)
    root_frames = [f for f in all_frames if f not in parents_map or not parents_map[f]]
    
    # Build tree recursively
    def build_subtree(frame: str, visited: Set[str] = None) -> Dict[str, Any]:
        if visited is None:
            visited = set()
        
        if frame in visited:
            return {"frame": frame, "children": [], "circular_reference": True}
        
        visited.add(frame)
        
        subtree = {
            "frame": frame,
            "children": []
        }
        
        for child in children_map.get(frame, []):
            child_tree = build_subtree(child, visited.copy())
            subtree["children"].append(child_tree)
        
        return subtree
    
    # Build trees from each root
    trees = []
    for root in root_frames:
        trees.append(build_subtree(root))
    
    # Handle disconnected components (frames that form cycles)
    processed_frames = set()
    for tree in trees:
        def collect_frames(node):
            processed_frames.add(node["frame"])
            for child in node.get("children", []):
                collect_frames(child)
        collect_frames(tree)
    
    disconnected = all_frames - processed_frames
    for frame in disconnected:
        if frame not in processed_frames:
            trees.append(build_subtree(frame))
            processed_frames.add(frame)
    
    return {
        "trees": trees,
        "root_frames": root_frames,
        "all_frames": sorted(list(all_frames)),
        "frame_count": len(all_frames)
    }


async def get_tf_tree(
    timestamp: float,
    tf_topic: str = "/tf",
    static_tf_topic: str = "/tf_static",
    bag_path: Optional[str] = None,
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Get the TF tree structure at a specific timestamp.
    
    Args:
        timestamp: Unix timestamp to query TF tree
        tf_topic: Topic for dynamic transforms (default: /tf)
        static_tf_topic: Topic for static transforms (default: /tf_static)
        bag_path: Optional specific bag file or directory
        
    Returns:
        Hierarchical tree structure of coordinate frames
    """
    logger.info(f"Getting TF tree at timestamp {timestamp}")
    
    if not _get_bag_files_fn:
        return {"error": "Bag files function not provided"}
    
    bags = _get_bag_files_fn(bag_path)
    if not bags:
        return {"error": "No bag files found"}
    
    target_ns = int(timestamp * 1e9)
    tolerance_ns = int(1e9)  # 1 second window for dynamic transforms
    
    # Collect all transforms
    transforms = {}  # (parent, child) -> transform_data
    static_transforms = {}
    dynamic_transforms = {}
    
    for bag_path in bags:
        try:
            with Reader(bag_path) as reader:
                # First collect static transforms (they don't change with time)
                for conn, msg_timestamp, rawdata in reader.messages():
                    if conn.topic == static_tf_topic:
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        
                        for tf in msg_dict.get('transforms', []):
                            parent = tf.get('header', {}).get('frame_id', '')
                            child = tf.get('child_frame_id', '')
                            
                            if parent and child:
                                key = (parent, child)
                                static_transforms[key] = {
                                    "type": "static",
                                    "transform": tf.get('transform', {}),
                                    "timestamp": msg_timestamp / 1e9
                                }
                
                # Then collect dynamic transforms near the timestamp
                for conn, msg_timestamp, rawdata in reader.messages():
                    if conn.topic == tf_topic:
                        # Only consider transforms near our target time
                        if abs(msg_timestamp - target_ns) > tolerance_ns:
                            continue
                        
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        
                        for tf in msg_dict.get('transforms', []):
                            parent = tf.get('header', {}).get('frame_id', '')
                            child = tf.get('child_frame_id', '')
                            
                            if parent and child:
                                key = (parent, child)
                                # Keep the transform closest to our target time
                                if key not in dynamic_transforms or \
                                   abs(msg_timestamp - target_ns) < abs(dynamic_transforms[key]["timestamp"] * 1e9 - target_ns):
                                    dynamic_transforms[key] = {
                                        "type": "dynamic",
                                        "transform": tf.get('transform', {}),
                                        "timestamp": msg_timestamp / 1e9
                                    }
                
        except Exception as e:
            logger.error(f"Error reading {bag_path}: {e}")
            continue
    
    # Combine transforms (dynamic overrides static for same parent-child pair)
    transforms.update(static_transforms)
    transforms.update(dynamic_transforms)
    
    if not transforms:
        return {"error": "No transforms found near the specified timestamp"}
    
    # Build the tree structure
    tree_data = build_tf_tree(transforms)
    
    # Add transform details
    transform_list = []
    for (parent, child), data in transforms.items():
        transform_info = {
            "parent": parent,
            "child": child,
            "type": data["type"],
            "timestamp": data["timestamp"]
        }
        
        # Add transform values if present
        if "transform" in data and data["transform"]:
            trans = data["transform"].get("translation", {})
            rot = data["transform"].get("rotation", {})
            transform_info["translation"] = {
                "x": trans.get("x", 0),
                "y": trans.get("y", 0),
                "z": trans.get("z", 0)
            }
            transform_info["rotation"] = {
                "x": rot.get("x", 0),
                "y": rot.get("y", 0),
                "z": rot.get("z", 0),
                "w": rot.get("w", 1)
            }
        
        transform_list.append(transform_info)
    
    # Sort transforms by parent then child
    transform_list.sort(key=lambda x: (x["parent"], x["child"]))
    
    return {
        "timestamp": timestamp,
        "tree": tree_data["trees"],
        "root_frames": tree_data["root_frames"],
        "all_frames": tree_data["all_frames"],
        "frame_count": tree_data["frame_count"],
        "transform_count": len(transforms),
        "static_count": len(static_transforms),
        "dynamic_count": len(dynamic_transforms),
        "transforms": transform_list
    }


def register_tf_tools(server, get_bag_files_fn, deserialize_message_fn, config):
    """Register TF tree tools with the MCP server."""
    from mcp.types import Tool
    
    tools = [
        Tool(
            name="get_tf_tree",
            description="Get the TF (transform) tree showing coordinate frame relationships at a specific time",
            inputSchema={
                "type": "object",
                "properties": {
                    "timestamp": {"type": "number", "description": "Unix timestamp to query TF tree"},
                    "tf_topic": {"type": "string", "description": "Dynamic transforms topic (default: /tf)"},
                    "static_tf_topic": {"type": "string", "description": "Static transforms topic (default: /tf_static)"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory"}
                },
                "required": ["timestamp"]
            }
        )
    ]
    
    # Create handler
    async def handle_get_tf_tree(args):
        return await get_tf_tree(
            args["timestamp"],
            args.get("tf_topic", "/tf"),
            args.get("static_tf_topic", "/tf_static"),
            args.get("bag_path"),
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    return tools, {
        "get_tf_tree": handle_get_tf_tree
    }