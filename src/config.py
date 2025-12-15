"""
VTOL Drone Mission Control System - Configuration
Centralized configuration management for the GCS application
"""

import os
from pathlib import Path
from typing import Dict, Any
import yaml

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
CONFIG_DIR = PROJECT_ROOT / "config"
RESOURCES_DIR = PROJECT_ROOT / "resources"
DATA_DIR = PROJECT_ROOT / "data"

# Create directories if they don't exist
DATA_DIR.mkdir(exist_ok=True)

# ============================================
# UAV PARAMETERS (Ported from MATLAB)
# ============================================

class UAVConfig:
    """VTOL UAV physical parameters and constraints"""
    
    # Battery configuration
    BATTERY_CAPACITY = 1500  # mAh
    CRUISE_CURRENT = 120     # mAh/min (aircraft mode)
    VTOL_CURRENT = 220       # mAh/min (vertical takeoff/landing)
    BATTERY_CRITICAL = 250   # mAh (emergency threshold)
    
    # Speed parameters
    CRUISE_SPEED = 28.0      # m/s (~100 km/h)
    VTOL_SPEED = 3.0         # m/s
    
    # Flight constraints
    TAKEOFF_ALTITUDE = 120   # m AGL
    MIN_TURN_RADIUS = 300    # m (aircraft-style constraint)
    MAX_CLIMB_RATE = 5.0     # m/s
    MAX_DESCENT_RATE = 3.0   # m/s
    
    # Guidance parameters
    LOOKAHEAD_DISTANCE = 1200   # m (predictive avoidance)
    FRONT_CONE_ANGLE = 60       # degrees (detection cone)
    TURN_BIAS_GAIN = 0.9
    SMOOTH_FACTOR = 0.15
    
    # Sample time
    SAMPLE_TIME = 0.1        # seconds (10 Hz control loop)
    
    # Safety margins
    OBSTACLE_BUFFER = 50     # m (minimum clearance)
    GEOFENCE_BUFFER = 100    # m


class MissionConfig:
    """Mission planning and execution parameters"""
    
    # Default waypoint
    HOME_LOCATION = [0.0, 0.0, 0.0]  # [x, y, z] in meters (local frame)
    
    # Mission types
    MISSION_TYPES = [
        "delivery",
        "survey",
        "mapping",
        "inspection",
        "emergency"
    ]
    
    # Path planning
    WAYPOINT_TOLERANCE = 10.0    # m (waypoint reached threshold)
    ALTITUDE_TOLERANCE = 5.0     # m
    MAX_WAYPOINTS = 100
    
    # Energy optimization
    ENERGY_WEIGHT = 0.6          # Balance between time and energy
    TIME_WEIGHT = 0.4


class SimulationConfig:
    """PX4/Gazebo simulation parameters"""
    
    # Connection settings
    PX4_SIM_HOST = "127.0.0.1"
    PX4_SIM_PORT = 14540
    MAVLINK_PORT = 14550
    
    # Simulation environment
    GAZEBO_WORLD = "default"
    PHYSICS_ENGINE = "ode"
    REAL_TIME_FACTOR = 1.0
    
    # Visualization
    ENABLE_3D_VIZ = True
    UPDATE_RATE = 30  # Hz


class ROS2Config:
    """ROS 2 communication settings"""
    
    # Node configuration
    NODE_NAME = "vtol_mission_control"
    NAMESPACE = "/uav"
    
    # Topics
    TOPIC_TELEMETRY = "/uav/telemetry"
    TOPIC_WAYPOINT = "/uav/waypoint"
    TOPIC_STATUS = "/uav/status"
    TOPIC_BATTERY = "/uav/battery"
    
    # QoS settings
    QOS_DEPTH = 10
    QOS_RELIABLE = True


class MappingConfig:
    """3D mapping and photogrammetry settings"""
    
    # Point cloud processing
    VOXEL_SIZE = 0.05           # m (downsampling resolution)
    MAX_POINTS = 10_000_000     # Maximum points in cloud
    
    # Reconstruction
    MIN_IMAGES = 3              # Minimum for triangulation
    MAX_REPROJECTION_ERROR = 4  # pixels
    
    # LiDAR (if applicable)
    LIDAR_RANGE = 100           # meters
    LIDAR_RESOLUTION = 0.01     # angular resolution (degrees)


class UIConfig:
    """User interface settings"""
    
    # Window configuration
    WINDOW_TITLE = "VTOL Mission Control - GCS"
    WINDOW_WIDTH = 1920
    WINDOW_HEIGHT = 1080
    MIN_WIDTH = 1280
    MIN_HEIGHT = 720
    
    # Theme
    THEME = "dark"
    ACCENT_COLOR = "#00AAFF"
    
    # 3D View settings
    FOV = 60                    # degrees
    NEAR_CLIP = 0.1
    FAR_CLIP = 100000           # meters
    
    # Update rates
    TELEMETRY_UPDATE_HZ = 10
    MAP_UPDATE_HZ = 5
    UI_REFRESH_HZ = 30


class SecurityConfig:
    """Security and safety parameters"""
    
    # Command validation
    REQUIRE_SIGNATURE = True
    SESSION_TIMEOUT = 3600      # seconds
    
    # Geofencing
    ENFORCE_GEOFENCE = True
    MAX_ALTITUDE = 500          # m AGL (regulatory limit)
    MAX_DISTANCE = 10000        # m from home
    
    # Emergency protocols
    AUTO_RTL_ON_SIGNAL_LOSS = True
    SIGNAL_LOSS_TIMEOUT = 5     # seconds
    AUTO_LAND_ON_CRITICAL_BATTERY = True


# ============================================
# Configuration Loader
# ============================================

def load_config(config_file: str = "mission_config.yaml") -> Dict[str, Any]:
    """Load configuration from YAML file"""
    config_path = CONFIG_DIR / config_file
    
    if config_path.exists():
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    else:
        # Return default configuration
        return {
            "uav": vars(UAVConfig),
            "mission": vars(MissionConfig),
            "simulation": vars(SimulationConfig),
            "ros2": vars(ROS2Config),
            "mapping": vars(MappingConfig),
            "ui": vars(UIConfig),
            "security": vars(SecurityConfig)
        }


def save_config(config: Dict[str, Any], config_file: str = "mission_config.yaml"):
    """Save configuration to YAML file"""
    CONFIG_DIR.mkdir(exist_ok=True)
    config_path = CONFIG_DIR / config_file
    
    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, indent=2)


# Export all configurations
__all__ = [
    'UAVConfig',
    'MissionConfig',
    'SimulationConfig',
    'ROS2Config',
    'MappingConfig',
    'UIConfig',
    'SecurityConfig',
    'load_config',
    'save_config'
]
