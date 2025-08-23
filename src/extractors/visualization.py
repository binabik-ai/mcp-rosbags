#!/usr/bin/env python3
"""
Simplified visualization tools for creating plots from ROS bag data.
"""

import json
import numpy as np
from typing import Dict, Any, List, Optional, Callable, Tuple
import logging
from pathlib import Path
from datetime import datetime

from rosbags.rosbag2 import Reader

logger = logging.getLogger(__name__)


def save_plot_files(fig, plot_type: str, bag_path: Optional[str] = None) -> Dict[str, str]:
    """Save plot as HTML file and return the path."""
    # Determine save location
    if bag_path:
        base_path = Path(bag_path)
        if base_path.is_file():
            plots_dir = base_path.parent / ".plots"
        else:
            plots_dir = base_path / ".plots"
    else:
        plots_dir = Path.cwd() / ".plots"
    
    plots_dir.mkdir(exist_ok=True)
    
    # Generate filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    if bag_path:
        bag_name = Path(bag_path).stem
    else:
        bag_name = "plot"
    
    base_name = f"{bag_name}_{plot_type}_{timestamp}"
    
    # Save HTML file
    html_path = plots_dir / f"{base_name}.html"
    html_content = fig.to_html(include_plotlyjs='cdn')
    html_path.write_text(html_content)
    
    return {
        "html_file": str(html_path),
        "interactive_link": f"file://{html_path.absolute()}"
    }


def parse_field_spec(field_spec: str) -> Tuple[str, str]:
    """Parse 'topic.field.path' format."""
    if field_spec.startswith('/'):
        # Find where topic ends (at the first dot after the topic name)
        parts = field_spec.split('.')
        if len(parts) == 1:
            return field_spec, ''
        
        # Find the topic part (everything before first dot)
        for i, char in enumerate(field_spec):
            if char == '.' and i > 0:
                return field_spec[:i], field_spec[i+1:]
        return field_spec, ''
    else:
        # Doesn't start with /, so first part before . is topic
        dot_idx = field_spec.find('.')
        if dot_idx == -1:
            return '/' + field_spec, ''
        
        topic = '/' + field_spec[:dot_idx]
        field = field_spec[dot_idx + 1:]
        return topic, field


def extract_field_value(msg_data: Dict[str, Any], field_path: str) -> Any:
    """Extract a field value using dot notation."""
    if not field_path:
        return msg_data
    
    try:
        parts = field_path.split('.')
        result = msg_data
        
        for part in parts:
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
        logger.debug(f"Failed to extract {field_path}: {e}")
        return None


async def plot_timeseries(
    fields: List[str],
    start_time: float,
    end_time: float,
    plot_style: str = "line",
    title: str = "Time Series Plot",
    x_label: str = "Time (s)",
    y_label: str = "Value",
    bag_path: Optional[str] = None,
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Create a time series plot with various styles.
    
    Args:
        fields: List of fields to plot (format: 'topic.field.path')
        start_time: Start unix timestamp in seconds
        end_time: End unix timestamp in seconds  
        plot_style: Style of plot - "line", "scatter", "step", "area", "box", "histogram"
        title: Plot title
        x_label: X axis label
        y_label: Y axis label
        bag_path: Optional specific bag file or directory
    """
    logger.info(f"Creating {plot_style} time series plot for {len(fields)} fields")
    
    if not _get_bag_files_fn:
        return {"error": "Bag files function not provided"}
    
    bags = _get_bag_files_fn(bag_path)
    if not bags:
        return {"error": "No bag files found"}
    
    # Parse fields to get topics
    field_specs = []
    for field in fields:
        topic, field_path = parse_field_spec(field)
        field_specs.append({
            "original": field,
            "topic": topic,
            "field": field_path,
            "data": []
        })
    
    start_ns = int(start_time * 1e9)
    end_ns = int(end_time * 1e9)
    
    # Collect data for each field
    for bag_path in bags:
        try:
            with Reader(bag_path) as reader:
                needed_topics = list(set(spec["topic"] for spec in field_specs))
                
                for conn, timestamp, rawdata in reader.messages():
                    if conn.topic not in needed_topics:
                        continue
                    
                    if not (start_ns <= timestamp <= end_ns):
                        continue
                    
                    time_sec = timestamp / 1e9
                    _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                    
                    for spec in field_specs:
                        if spec["topic"] == conn.topic:
                            value = extract_field_value(msg_dict, spec["field"])
                            if value is not None and isinstance(value, (int, float)):
                                spec["data"].append({
                                    "time": time_sec,
                                    "value": float(value)
                                })
                        
        except Exception as e:
            logger.error(f"Error reading {bag_path}: {e}")
            continue
    
    # Create Plotly figure
    import plotly.graph_objects as go
    
    fig = go.Figure()
    
    # Add traces based on plot style
    for spec in field_specs:
        if not spec["data"]:
            continue
            
        # Sort by time
        spec["data"].sort(key=lambda x: x["time"])
        
        times = [d["time"] - start_time for d in spec["data"]]  # Relative time
        values = [d["value"] for d in spec["data"]]
        
        if plot_style == "line":
            fig.add_trace(go.Scatter(
                x=times, y=values,
                mode='lines',
                name=spec["original"],
                line=dict(width=2)
            ))
        elif plot_style == "scatter":
            fig.add_trace(go.Scatter(
                x=times, y=values,
                mode='markers',
                name=spec["original"],
                marker=dict(size=4)
            ))
        elif plot_style == "step":
            fig.add_trace(go.Scatter(
                x=times, y=values,
                mode='lines',
                name=spec["original"],
                line=dict(shape='hv', width=2)
            ))
        elif plot_style == "area":
            fig.add_trace(go.Scatter(
                x=times, y=values,
                mode='lines',
                name=spec["original"],
                fill='tozeroy',
                line=dict(width=2)
            ))
        elif plot_style == "box":
            # For box plot, we show distribution over time windows
            fig.add_trace(go.Box(
                y=values,
                name=spec["original"],
                boxmean=True
            ))
        elif plot_style == "histogram":
            # For histogram, show distribution of values
            fig.add_trace(go.Histogram(
                x=values,
                name=spec["original"],
                opacity=0.7,
                nbinsx=30
            ))
    
    # Update layout based on plot style
    if plot_style == "histogram":
        fig.update_layout(
            title=title,
            xaxis_title=y_label,  # Swap axes for histogram
            yaxis_title="Count",
            hovermode='x unified',
            showlegend=True,
            height=500,
            template='plotly_white',
            barmode='overlay'
        )
    elif plot_style == "box":
        fig.update_layout(
            title=title,
            yaxis_title=y_label,
            showlegend=True,
            height=500,
            template='plotly_white'
        )
    else:
        fig.update_layout(
            title=title,
            xaxis_title=x_label,
            yaxis_title=y_label,
            hovermode='x unified',
            showlegend=True,
            height=500,
            template='plotly_white'
        )
    
    # Save files and get display data
    files = save_plot_files(fig, f"timeseries_{plot_style}", bag_path)
    
    # Count total data points
    total_points = sum(len(spec["data"]) for spec in field_specs)
    
    return {
        "type": "timeseries_plot",
        "plot_style": plot_style,
        "title": title,
        "fields": fields,
        "time_range": {
            "start": start_time,
            "end": end_time,
            "duration": end_time - start_time
        },
        "data_points": total_points,
        **files
    }


async def plot_2d(
    x_field: str,
    y_field: str,
    start_time: float,
    end_time: float,
    color_by_time: bool = False,
    title: str = "2D Plot",
    x_label: str = "X",
    y_label: str = "Y",
    equal_aspect: bool = True,
    bag_path: Optional[str] = None,
    _get_bag_files_fn: Callable = None,
    _deserialize_message_fn: Callable = None,
    config: Dict[str, Any] = None
) -> Dict[str, Any]:
    """Create a 2D plot (e.g., trajectory, X-Y relationship)."""
    logger.info(f"Creating 2D plot: {x_field} vs {y_field}")
    
    if not _get_bag_files_fn:
        return {"error": "Bag files function not provided"}
    
    bags = _get_bag_files_fn(bag_path)
    if not bags:
        return {"error": "No bag files found"}
    
    # Parse fields
    x_topic, x_field_path = parse_field_spec(x_field)
    y_topic, y_field_path = parse_field_spec(y_field)
    
    start_ns = int(start_time * 1e9)
    end_ns = int(end_time * 1e9)
    
    # Collect data
    data_points = []
    
    if x_topic == y_topic:
        # Same topic - efficient collection
        for bag_path in bags:
            try:
                with Reader(bag_path) as reader:
                    if x_topic not in [conn.topic for conn in reader.connections]:
                        continue
                    
                    for conn, timestamp, rawdata in reader.messages():
                        if conn.topic != x_topic:
                            continue
                        
                        if not (start_ns <= timestamp <= end_ns):
                            continue
                        
                        time_sec = timestamp / 1e9
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        
                        x_val = extract_field_value(msg_dict, x_field_path)
                        y_val = extract_field_value(msg_dict, y_field_path)
                        
                        if x_val is not None and y_val is not None:
                            data_points.append({
                                "x": float(x_val),
                                "y": float(y_val),
                                "time": time_sec
                            })
                            
            except Exception as e:
                logger.error(f"Error reading {bag_path}: {e}")
                continue
    else:
        # Different topics - need correlation
        x_messages = {}
        y_messages = {}
        
        for bag_path in bags:
            try:
                with Reader(bag_path) as reader:
                    for conn, timestamp, rawdata in reader.messages():
                        if conn.topic not in [x_topic, y_topic]:
                            continue
                        
                        if not (start_ns <= timestamp <= end_ns):
                            continue
                        
                        time_sec = timestamp / 1e9
                        _, msg_dict = _deserialize_message_fn(rawdata, conn.msgtype)
                        
                        if conn.topic == x_topic:
                            val = extract_field_value(msg_dict, x_field_path)
                            if val is not None:
                                x_messages[time_sec] = float(val)
                        else:
                            val = extract_field_value(msg_dict, y_field_path)
                            if val is not None:
                                y_messages[time_sec] = float(val)
                                
            except Exception as e:
                logger.error(f"Error reading {bag_path}: {e}")
                continue
        
        # Correlate by nearest time
        for x_time, x_val in x_messages.items():
            if y_messages:
                closest_y_time = min(y_messages.keys(), 
                                    key=lambda t: abs(t - x_time))
                
                if abs(closest_y_time - x_time) < 0.1:  # 100ms tolerance
                    data_points.append({
                        "x": x_val,
                        "y": y_messages[closest_y_time],
                        "time": x_time
                    })
    
    if not data_points:
        return {"error": "No data points found for 2D plot"}
    
    # Sort by time for trajectory
    data_points.sort(key=lambda p: p["time"])
    
    # Create Plotly figure
    import plotly.graph_objects as go
    
    fig = go.Figure()
    
    if color_by_time:
        # Color gradient based on time
        times = [p["time"] for p in data_points]
        fig.add_trace(go.Scatter(
            x=[p["x"] for p in data_points],
            y=[p["y"] for p in data_points],
            mode='lines+markers',
            marker=dict(
                size=4,
                color=times,
                colorscale='Viridis',
                showscale=True,
                colorbar=dict(title="Time (s)")
            ),
            line=dict(width=2),
            name='Trajectory'
        ))
    else:
        # Simple trajectory
        fig.add_trace(go.Scatter(
            x=[p["x"] for p in data_points],
            y=[p["y"] for p in data_points],
            mode='lines+markers',
            name='Path',
            line=dict(color='blue', width=2),
            marker=dict(size=3)
        ))
    
    # Add start and end markers
    fig.add_trace(go.Scatter(
        x=[data_points[0]["x"]],
        y=[data_points[0]["y"]],
        mode='markers',
        name='Start',
        marker=dict(color='green', size=12, symbol='circle')
    ))
    
    fig.add_trace(go.Scatter(
        x=[data_points[-1]["x"]],
        y=[data_points[-1]["y"]],
        mode='markers',
        name='End',
        marker=dict(color='red', size=12, symbol='square')
    ))
    
    # Update layout
    layout_dict = {
        'title': title,
        'xaxis_title': x_label,
        'yaxis_title': y_label,
        'hovermode': 'closest',
        'showlegend': True,
        'height': 600,
        'template': 'plotly_white'
    }
    
    if equal_aspect:
        layout_dict['yaxis'] = dict(scaleanchor="x", scaleratio=1)
    
    fig.update_layout(**layout_dict)
    
    # Calculate path statistics
    total_distance = 0
    for i in range(1, len(data_points)):
        dx = data_points[i]["x"] - data_points[i-1]["x"]
        dy = data_points[i]["y"] - data_points[i-1]["y"]
        total_distance += (dx**2 + dy**2) ** 0.5
    
    # Save files
    files = save_plot_files(fig, "2d_plot", bag_path)
    
    return {
        "type": "2d_plot",
        "title": title,
        "x_field": x_field,
        "y_field": y_field,
        "data_points": len(data_points),
        "statistics": {
            "total_distance": round(total_distance, 3),
            "start_point": {"x": round(data_points[0]["x"], 3), "y": round(data_points[0]["y"], 3)},
            "end_point": {"x": round(data_points[-1]["x"], 3), "y": round(data_points[-1]["y"], 3)}
        },
        **files
    }


def register_visualization_tools(server, get_bag_files_fn, deserialize_message_fn, config):
    """Register visualization tools with the MCP server."""
    from mcp.types import Tool
    
    tools = [
        Tool(
            name="plot_timeseries",
            description="Plot time series data with various styles. Fields format: 'topic.field.path' or '/topic/name.field.path'",
            inputSchema={
                "type": "object",
                "properties": {
                    "fields": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "List of fields to plot (e.g., ['cmd_vel.linear.x', 'odom.twist.twist.linear.x'])"
                    },
                    "start_time": {"type": "number", "description": "Start unix timestamp in seconds"},
                    "end_time": {"type": "number", "description": "End unix timestamp in seconds"},
                    "plot_style": {"type": "string", 
                                 "enum": ["line", "scatter", "step", "area", "box", "histogram"],
                                 "description": "Style of plot (default: line)"},
                    "title": {"type": "string", "description": "Plot title"},
                    "x_label": {"type": "string", "description": "X axis label"},
                    "y_label": {"type": "string", "description": "Y axis label"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory"}
                },
                "required": ["fields", "start_time", "end_time"]
            }
        ),
        Tool(
            name="plot_2d",
            description="Create 2D plots (trajectories, X-Y relationships). Can color by time.",
            inputSchema={
                "type": "object",
                "properties": {
                    "x_field": {"type": "string", "description": "X axis field (e.g., 'odom.pose.pose.position.x')"},
                    "y_field": {"type": "string", "description": "Y axis field (e.g., 'odom.pose.pose.position.y')"},
                    "start_time": {"type": "number", "description": "Start unix timestamp in seconds"},
                    "end_time": {"type": "number", "description": "End unix timestamp in seconds"},
                    "color_by_time": {"type": "boolean", "description": "Color gradient based on time (default: false)"},
                    "title": {"type": "string", "description": "Plot title"},
                    "x_label": {"type": "string", "description": "X axis label"},
                    "y_label": {"type": "string", "description": "Y axis label"},
                    "equal_aspect": {"type": "boolean", "description": "Keep aspect ratio equal (default: true)"},
                    "bag_path": {"type": "string", "description": "Optional: specific bag file or directory"}
                },
                "required": ["x_field", "y_field", "start_time", "end_time"]
            }
        )
    ]
    
    # Create handlers
    async def handle_plot_timeseries(args):
        return await plot_timeseries(
            args["fields"],
            args["start_time"],
            args["end_time"],
            args.get("plot_style", "line"),
            args.get("title", "Time Series Plot"),
            args.get("x_label", "Time (s)"),
            args.get("y_label", "Value"),
            args.get("bag_path"),
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    async def handle_plot_2d(args):
        return await plot_2d(
            args["x_field"],
            args["y_field"],
            args["start_time"],
            args["end_time"],
            args.get("color_by_time", False),
            args.get("title", "2D Plot"),
            args.get("x_label", "X"),
            args.get("y_label", "Y"),
            args.get("equal_aspect", True),
            args.get("bag_path"),
            get_bag_files_fn,
            deserialize_message_fn,
            config
        )
    
    return tools, {
        "plot_timeseries": handle_plot_timeseries,
        "plot_2d": handle_plot_2d
    }