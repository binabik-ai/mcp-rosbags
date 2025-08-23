#!/usr/bin/env python3
"""
LiDAR data analysis and visualization tools.
"""

import numpy as np
from typing import Dict, Any, List, Optional, Callable
import logging
from pathlib import Path
from datetime import datetime

from rosbags.rosbag2 import Reader

logger = logging.getLogger(__name__)


def analyze_scan(ranges: List[float], angle_min: float, angle_max: float, 
                 angle_increment: float, range_min: float, range_max: float) -> Dict[str, Any]:
    """Analyze a single laser scan."""
    
    # Filter out invalid ranges
    valid_ranges = [r for r in ranges if range_min <= r <= range_max and not np.isnan(r) and not np.isinf(r)]
    
    if not valid_ranges:
        return {
            "valid_points": 0,
            "min_range": None,
            "max_range": None,
            "avg_range": None
        }
    
    # Basic statistics
    stats = {
        "valid_points": len(valid_ranges),
        "invalid_points": len(ranges) - len(valid_ranges),
        "min_range": float(np.min(valid_ranges)),
        "max_range": float(np.max(valid_ranges)),
        "avg_range": float(np.mean(valid_ranges)),
        "median_range": float(np.median(valid_ranges)),
        "std_range": float(np.std(valid_ranges))
    }
    
    # Sector analysis (divide into 12 sectors of 30 degrees each)
    num_sectors = 12
    sector_size = (angle_max - angle_min) / num_sectors
    sectors = []
    
    for i in range(num_sectors):
        sector_start = angle_min + i * sector_size
        sector_end = sector_start + sector_size
        sector_center = (sector_start + sector_end) / 2
        
        # Find ranges in this sector
        start_idx = int((sector_start - angle_min) / angle_increment)
        end_idx = int((sector_end - angle_min) / angle_increment)
        
        start_idx = max(0, min(start_idx, len(ranges) - 1))
        end_idx = max(0, min(end_idx, len(ranges)))
        
        sector_ranges = ranges[start_idx:end_idx]
        sector_valid = [r for r in sector_ranges if range_min <= r <= range_max and not np.isnan(r) and not np.isinf(r)]
        
        if sector_valid:
            closest = float(np.min(sector_valid))
            has_obstacle = closest < (range_max * 0.9)  # Consider obstacle if < 90% of max range
        else:
            closest = None
            has_obstacle = False
        
        sectors.append({
            "sector_id": i,
            "angle_center_deg": float(np.degrees(sector_center)),
            "angle_center_rad": float(sector_center),
            "closest_range": closest,
            "has_obstacle": has_obstacle,
            "point_count": len(sector_valid)
        })
    
    stats["sectors"] = sectors
    
    # Gap detection (find openings)
    gaps = []
    gap_threshold = range_max * 0.8  # Consider gap if range > 80% of max
    min_gap_width = 5  # Minimum consecutive points for a gap
    
    in_gap = False
    gap_start_idx = 0
    
    for i, r in enumerate(ranges):
        is_far = r > gap_threshold or np.isnan(r) or np.isinf(r) or r >= range_max
        
        if is_far and not in_gap:
            # Start of gap
            in_gap = True
            gap_start_idx = i
        elif not is_far and in_gap:
            # End of gap
            gap_width = i - gap_start_idx
            if gap_width >= min_gap_width:
                start_angle = angle_min + gap_start_idx * angle_increment
                end_angle = angle_min + i * angle_increment
                gaps.append({
                    "start_angle_deg": float(np.degrees(start_angle)),
                    "end_angle_deg": float(np.degrees(end_angle)),
                    "width_deg": float(np.degrees(end_angle - start_angle)),
                    "point_count": gap_width
                })
            in_gap = False
    
    # Check if gap extends to end
    if in_gap:
        gap_width = len(ranges) - gap_start_idx
        if gap_width >= min_gap_width:
            start_angle = angle_min + gap_start_idx * angle_increment
            end_angle = angle_max
            gaps.append({
                "start_angle_deg": float(np.degrees(start_angle)),
                "end_angle_deg": float(np.degrees(end_angle)),
                "width_deg": float(np.degrees(end_angle - start_angle)),
                "point_count": gap_width
            })
    
    stats["gaps"] = gaps
    stats["gap_count"] = len(gaps)
    
    return stats


async def analyze_lidar_scan(
    topic: str,
    timestamp: Optional[float] = None,
    start_time: Optional[float] = None,
    end_time: Optional[float] = None,
    aggregate: bool = False,
    bag_path: Optional[str] = None,
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Analyze LiDAR scan data for obstacles and gaps.
    
    Args:
        topic: LaserScan topic name
        timestamp: Specific timestamp to analyze (single scan)
        start_time: Start time for range analysis
        end_time: End time for range analysis
        aggregate: If True, aggregate statistics over time range
        bag_path: Optional specific bag file or directory
    """
    logger.info(f"Analyzing LiDAR scan from {topic}")
    
    if not _get_bag_files_fn:
        return {"error": "Bag files function not provided"}
    
    bags = _get_bag_files_fn(bag_path)
    if not bags:
        return {"error": "No bag files found"}
    
    # Single scan analysis
    if timestamp is not None:
        target_ns = int(timestamp * 1e9)
        tolerance_ns = int(0.1 * 1e9)  # 0.1 second tolerance
        
        closest_scan = None
        closest_time = None
        min_diff = float('inf')
        
        for bag_path in bags:
            try:
                with Reader(bag_path) as reader:
                    if topic not in [conn.topic for conn in reader.connections]:
                        continue
                    
                    for conn, msg_timestamp, rawdata in reader.messages():
                        if conn.topic != topic:
                            continue
                        
                        diff = abs(msg_timestamp - target_ns)
                        if diff < min_diff and diff <= tolerance_ns:
                            min_diff = diff
                            closest_time = msg_timestamp
                            _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                            closest_scan = msg_dict
                            
            except Exception as e:
                logger.error(f"Error reading {bag_path}: {e}")
                continue
        
        if closest_scan is None:
            return {"error": f"No scan found on {topic} near timestamp {timestamp}"}
        
        # Analyze the single scan
        analysis = analyze_scan(
            closest_scan.get("ranges", []),
            closest_scan.get("angle_min", 0),
            closest_scan.get("angle_max", 0),
            closest_scan.get("angle_increment", 0),
            closest_scan.get("range_min", 0),
            closest_scan.get("range_max", 30)
        )
        
        return {
            "topic": topic,
            "timestamp": closest_time / 1e9,
            "time_diff": min_diff / 1e9,
            "scan_analysis": analysis,
            "scan_info": {
                "angle_min_deg": float(np.degrees(closest_scan.get("angle_min", 0))),
                "angle_max_deg": float(np.degrees(closest_scan.get("angle_max", 0))),
                "angle_increment_deg": float(np.degrees(closest_scan.get("angle_increment", 0))),
                "range_min": closest_scan.get("range_min", 0),
                "range_max": closest_scan.get("range_max", 0),
                "scan_time": closest_scan.get("scan_time", 0),
                "time_increment": closest_scan.get("time_increment", 0)
            }
        }
    
    # Time range analysis
    else:
        if start_time is None or end_time is None:
            return {"error": "For range analysis, both start_time and end_time are required"}
        
        start_ns = int(start_time * 1e9)
        end_ns = int(end_time * 1e9)
        
        scans = []
        all_min_ranges = []
        all_avg_ranges = []
        all_gaps = []
        
        for bag_path in bags:
            try:
                with Reader(bag_path) as reader:
                    if topic not in [conn.topic for conn in reader.connections]:
                        continue
                    
                    for conn, msg_timestamp, rawdata in reader.messages():
                        if conn.topic != topic:
                            continue
                        
                        if not (start_ns <= msg_timestamp <= end_ns):
                            continue
                        
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        
                        analysis = analyze_scan(
                            msg_dict.get("ranges", []),
                            msg_dict.get("angle_min", 0),
                            msg_dict.get("angle_max", 0),
                            msg_dict.get("angle_increment", 0),
                            msg_dict.get("range_min", 0),
                            msg_dict.get("range_max", 30)
                        )
                        
                        if analysis["valid_points"] > 0:
                            all_min_ranges.append(analysis["min_range"])
                            all_avg_ranges.append(analysis["avg_range"])
                            all_gaps.extend(analysis["gaps"])
                            
                            if not aggregate:
                                scans.append({
                                    "timestamp": msg_timestamp / 1e9,
                                    "analysis": analysis
                                })
                        
            except Exception as e:
                logger.error(f"Error reading {bag_path}: {e}")
                continue
        
        if not all_min_ranges:
            return {"error": f"No valid scans found on {topic} in time range"}
        
        # Calculate aggregate statistics
        result = {
            "topic": topic,
            "start_time": start_time,
            "end_time": end_time,
            "scan_count": len(all_min_ranges),
            "aggregate_stats": {
                "min_range_overall": float(np.min(all_min_ranges)),
                "max_range_overall": float(np.max(all_min_ranges)),
                "avg_min_range": float(np.mean(all_min_ranges)),
                "avg_avg_range": float(np.mean(all_avg_ranges)),
                "std_min_range": float(np.std(all_min_ranges)),
                "total_gaps_detected": len(all_gaps)
            }
        }
        
        if not aggregate:
            # Include individual scans (limited)
            max_scans = 100
            if len(scans) > max_scans:
                # Downsample
                step = len(scans) // max_scans
                scans = scans[::step]
            result["scans"] = scans
            result["truncated"] = len(scans) < len(all_min_ranges)
        
        # Find most common gap locations
        if all_gaps:
            gap_centers = [g["start_angle_deg"] + g["width_deg"]/2 for g in all_gaps]
            result["aggregate_stats"]["common_gap_angles_deg"] = gap_centers[:5]
        
        return result


async def plot_lidar_scan(
    topic: str,
    timestamp: float,
    title: str = "LiDAR Scan",
    show_sectors: bool = True,
    bag_path: Optional[str] = None,
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Create a polar plot of a LiDAR scan.
    
    Args:
        topic: LaserScan topic name
        timestamp: Timestamp of scan to plot
        title: Plot title
        show_sectors: Whether to show sector divisions
        bag_path: Optional specific bag file or directory
    """
    logger.info(f"Creating LiDAR plot for {topic} at {timestamp}")
    
    if not _get_bag_files_fn:
        return {"error": "Bag files function not provided"}
    
    bags = _get_bag_files_fn(bag_path)
    if not bags:
        return {"error": "No bag files found"}
    
    # Find the scan
    target_ns = int(timestamp * 1e9)
    tolerance_ns = int(0.1 * 1e9)
    
    scan_data = None
    actual_time = None
    
    for bag_path in bags:
        try:
            with Reader(bag_path) as reader:
                if topic not in [conn.topic for conn in reader.connections]:
                    continue
                
                for conn, msg_timestamp, rawdata in reader.messages():
                    if conn.topic != topic:
                        continue
                    
                    diff = abs(msg_timestamp - target_ns)
                    if diff <= tolerance_ns:
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        scan_data = msg_dict
                        actual_time = msg_timestamp / 1e9
                        break
                
                if scan_data:
                    break
                    
        except Exception as e:
            logger.error(f"Error reading {bag_path}: {e}")
            continue
    
    if scan_data is None:
        return {"error": f"No scan found on {topic} near timestamp {timestamp}"}
    
    # Extract scan parameters
    ranges = scan_data.get("ranges", [])
    angle_min = scan_data.get("angle_min", 0)
    angle_max = scan_data.get("angle_max", 0)
    angle_increment = scan_data.get("angle_increment", 0)
    range_min = scan_data.get("range_min", 0)
    range_max = scan_data.get("range_max", 30)
    
    # Generate angles for each range measurement
    angles = np.arange(angle_min, angle_max, angle_increment)
    if len(angles) > len(ranges):
        angles = angles[:len(ranges)]
    elif len(angles) < len(ranges):
        ranges = ranges[:len(angles)]
    
    # Filter invalid ranges for plotting
    valid_indices = []
    valid_ranges = []
    valid_angles = []
    
    for i, r in enumerate(ranges):
        if range_min <= r <= range_max and not np.isnan(r) and not np.isinf(r):
            valid_indices.append(i)
            valid_ranges.append(r)
            valid_angles.append(angles[i])
    
    # Create Plotly polar plot
    import plotly.graph_objects as go
    
    fig = go.Figure()
    
    # Add the scan data
    fig.add_trace(go.Scatterpolar(
        r=valid_ranges,
        theta=np.degrees(valid_angles),
        mode='markers',
        marker=dict(
            color=valid_ranges,
            colorscale='Viridis',
            size=3,
            colorbar=dict(title="Range (m)")
        ),
        name='Scan Points'
    ))
    
    # Add sector divisions if requested
    if show_sectors:
        num_sectors = 12
        sector_size = 360 / num_sectors
        for i in range(num_sectors):
            angle = i * sector_size
            fig.add_trace(go.Scatterpolar(
                r=[0, range_max],
                theta=[angle, angle],
                mode='lines',
                line=dict(color='gray', width=0.5, dash='dot'),
                showlegend=False
            ))
    
    # Add a circle at max range
    theta_circle = np.linspace(0, 360, 100)
    fig.add_trace(go.Scatterpolar(
        r=[range_max] * 100,
        theta=theta_circle,
        mode='lines',
        line=dict(color='red', width=1, dash='dash'),
        name=f'Max Range ({range_max}m)'
    ))
    
    fig.update_layout(
        title=f"{title} (t={actual_time:.2f}s)",
        polar=dict(
            radialaxis=dict(
                visible=True,
                range=[0, range_max],
                showticklabels=True,
                ticks='outside'
            ),
            angularaxis=dict(
                visible=True,
                direction='counterclockwise',
                rotation=90
            )
        ),
        showlegend=True,
        height=600,
        template='plotly_white'
    )
    
    # Save plot
    from datetime import datetime
    plots_dir = Path(bag_path).parent / ".plots" if bag_path else Path.cwd() / ".plots"
    plots_dir.mkdir(exist_ok=True)
    
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_name = f"lidar_scan_{timestamp_str}"
    
    html_path = plots_dir / f"{base_name}.html"
    html_content = fig.to_html(include_plotlyjs='cdn')
    html_path.write_text(html_content)
    
    # Also analyze the scan
    analysis = analyze_scan(ranges, angle_min, angle_max, angle_increment, range_min, range_max)
    
    return {
        "type": "lidar_plot",
        "title": title,
        "topic": topic,
        "timestamp": actual_time,
        "html_file": str(html_path),
        "interactive_link": f"file://{html_path.absolute()}",
        "scan_analysis": analysis,
        "data_points": len(valid_ranges)
    }


def register_lidar_tools(server, get_bag_files_fn, deserialize_message_fn, config):
    """Register LiDAR analysis tools with the MCP server."""
    from mcp.types import Tool
    
    tools = [
        Tool(
            name="analyze_lidar_scan",
            description="Analyze LiDAR/LaserScan data for obstacles, gaps, and statistics",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "LaserScan topic name (e.g., /scan, /laser)"},
                    "timestamp": {"type": "number", "description": "Specific timestamp for single scan analysis"},
                    "start_time": {"type": "number", "description": "Start time for range analysis"},
                    "end_time": {"type": "number", "description": "End time for range analysis"},
                    "aggregate": {"type": "boolean", "description": "Return only aggregate stats, not individual scans"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory"}
                },
                "required": ["topic"]
            }
        ),
        Tool(
            name="plot_lidar_scan",
            description="Create a polar plot visualization of a LiDAR scan",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "LaserScan topic name"},
                    "timestamp": {"type": "number", "description": "Timestamp of scan to plot"},
                    "title": {"type": "string", "description": "Plot title (default: 'LiDAR Scan')"},
                    "show_sectors": {"type": "boolean", "description": "Show sector divisions (default: true)"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory"}
                },
                "required": ["topic", "timestamp"]
            }
        )
    ]
    
    # Create handlers
    async def handle_analyze_lidar_scan(args):
        return await analyze_lidar_scan(
            args["topic"],
            args.get("timestamp"),
            args.get("start_time"),
            args.get("end_time"),
            args.get("aggregate", False),
            args.get("bag_path"),
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    async def handle_plot_lidar_scan(args):
        return await plot_lidar_scan(
            args["topic"],
            args["timestamp"],
            args.get("title", "LiDAR Scan"),
            args.get("show_sectors", True),
            args.get("bag_path"),
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    return tools, {
        "analyze_lidar_scan": handle_analyze_lidar_scan,
        "plot_lidar_scan": handle_plot_lidar_scan
    }