"""
UAV State Model - Ported from MATLAB
Represents the complete state of a VTOL drone
"""

import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from datetime import datetime


class FlightMode(Enum):
    """UAV flight modes"""
    IDLE = "IDLE"
    VTOL_TAKEOFF = "VTOL_TAKEOFF"
    CRUISE = "CRUISE"
    VTOL_LAND = "VTOL_LAND"
    EMERGENCY_RTL = "EMERGENCY_RTL"
    EMERGENCY_LAND = "EMERGENCY_LAND"
    HOVER = "HOVER"


class MissionPhase(Enum):
    """Mission execution phases"""
    PREFLIGHT = "PREFLIGHT"
    TAKEOFF = "TAKEOFF"
    TRANSIT_TO_TARGET = "TRANSIT_TO_TARGET"
    AT_TARGET = "AT_TARGET"
    RETURN_HOME = "RETURN_HOME"
    LANDING = "LANDING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


@dataclass
class Position:
    """3D position in local frame (NED or ENU)"""
    x: float = 0.0  # North/East (m)
    y: float = 0.0  # East/North (m)
    z: float = 0.0  # Down/Up (m)
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    @staticmethod
    def from_array(arr: np.ndarray) -> 'Position':
        return Position(float(arr[0]), float(arr[1]), float(arr[2]))
    
    def distance_to(self, other: 'Position') -> float:
        """Euclidean distance to another position"""
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return np.sqrt(dx*dx + dy*dy + dz*dz)
    
    def horizontal_distance_to(self, other: 'Position') -> float:
        """Horizontal distance (ignore altitude)"""
        dx = self.x - other.x
        dy = self.y - other.y
        return np.sqrt(dx*dx + dy*dy)


@dataclass
class Velocity:
    """3D velocity vector"""
    vx: float = 0.0  # m/s
    vy: float = 0.0  # m/s
    vz: float = 0.0  # m/s
    
    def to_array(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.vz])
    
    @staticmethod
    def from_array(arr: np.ndarray) -> 'Velocity':
        return Velocity(float(arr[0]), float(arr[1]), float(arr[2]))
    
    def magnitude(self) -> float:
        """Speed (magnitude of velocity)"""
        return np.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
    
    def normalize(self) -> np.ndarray:
        """Return unit vector in velocity direction"""
        mag = self.magnitude()
        if mag > 0:
            return self.to_array() / mag
        return np.array([1.0, 0.0, 0.0])


@dataclass
class Attitude:
    """Orientation in Euler angles"""
    roll: float = 0.0   # radians
    pitch: float = 0.0  # radians
    yaw: float = 0.0    # radians (heading)
    
    def to_array(self) -> np.ndarray:
        return np.array([self.roll, self.pitch, self.yaw])


@dataclass
class Battery:
    """Battery state"""
    capacity_mah: float = 1500.0      # Total capacity
    remaining_mah: float = 1500.0     # Current charge
    voltage: float = 12.6             # Volts
    current: float = 0.0              # Amps
    temperature: float = 25.0         # Celsius
    
    def percentage(self) -> float:
        """Battery percentage (0-100)"""
        return (self.remaining_mah / self.capacity_mah) * 100.0
    
    def is_critical(self, threshold_mah: float = 250.0) -> bool:
        """Check if battery is below critical threshold"""
        return self.remaining_mah < threshold_mah
    
    def time_remaining(self, current_draw: float) -> float:
        """Estimated time remaining in minutes"""
        if current_draw <= 0:
            return float('inf')
        return self.remaining_mah / current_draw


@dataclass
class Obstacle:
    """Obstacle representation (sphere)"""
    position: Position
    radius: float  # meters
    
    def distance_to_point(self, point: Position) -> float:
        """Distance from surface to point"""
        return self.position.distance_to(point) - self.radius
    
    def is_point_inside(self, point: Position) -> bool:
        """Check if point is inside obstacle"""
        return self.position.distance_to(point) < self.radius


@dataclass
class NoFlyZone:
    """No-fly zone (hard constraint)"""
    position: Position
    radius: float  # meters
    altitude_min: float = 0.0
    altitude_max: float = 1000.0
    
    def is_violation(self, point: Position) -> bool:
        """Check if point violates no-fly zone"""
        horizontal_dist = self.position.horizontal_distance_to(point)
        in_horizontal = horizontal_dist < self.radius
        in_vertical = self.altitude_min <= point.z <= self.altitude_max
        return in_horizontal and in_vertical


@dataclass
class UAVState:
    """Complete UAV state representation"""
    
    # Identity
    uav_id: str = "VTOL_001"
    timestamp: datetime = field(default_factory=datetime.now)
    
    # Position and motion
    position: Position = field(default_factory=Position)
    velocity: Velocity = field(default_factory=Velocity)
    attitude: Attitude = field(default_factory=Attitude)
    
    # Power
    battery: Battery = field(default_factory=Battery)
    
    # Flight status
    mode: FlightMode = FlightMode.IDLE
    mission_phase: MissionPhase = MissionPhase.PREFLIGHT
    is_armed: bool = False
    is_airborne: bool = False
    current_waypoint_index: int = 0  # Track waypoint navigation progress
    
    # Mission data
    home_position: Position = field(default_factory=Position)
    target_position: Optional[Position] = None
    delivered: bool = False
    
    # Telemetry
    gps_fix: int = 0  # 0=no fix, 3=3D fix
    satellite_count: int = 0
    ground_speed: float = 0.0  # m/s
    airspeed: float = 0.0      # m/s
    altitude_agl: float = 0.0  # Above ground level
    altitude_msl: float = 0.0  # Above mean sea level
    
    # Safety
    geofence_violation: bool = False
    obstacle_detected: bool = False
    emergency_active: bool = False
    
    def update_position(self, x: float, y: float, z: float):
        """Update position from coordinates"""
        self.position = Position(x, y, z)
        self.timestamp = datetime.now()
    
    def update_velocity(self, vx: float, vy: float, vz: float):
        """Update velocity vector"""
        self.velocity = Velocity(vx, vy, vz)
        self.ground_speed = self.velocity.magnitude()
        # Airspeed is approximately ground speed (simplified, ignoring wind)
        self.airspeed = self.ground_speed
        # Update altitude
        self.altitude_agl = max(0.0, self.position.z)
        self.altitude_msl = self.position.z
    
    def distance_to_home(self) -> float:
        """Distance from current position to home"""
        return self.position.distance_to(self.home_position)
    
    def distance_to_target(self) -> float:
        """Distance from current position to target"""
        if self.target_position:
            return self.position.distance_to(self.target_position)
        return float('inf')
    
    def heading_to_target(self) -> float:
        """Heading angle to target (radians)"""
        if self.target_position:
            dx = self.target_position.x - self.position.x
            dy = self.target_position.y - self.position.y
            return np.arctan2(dy, dx)
        return 0.0
    
    def to_dict(self) -> dict:
        """Convert to dictionary for logging/transmission"""
        return {
            "uav_id": self.uav_id,
            "timestamp": self.timestamp.isoformat(),
            "position": [self.position.x, self.position.y, self.position.z],
            "velocity": [self.velocity.vx, self.velocity.vy, self.velocity.vz],
            "attitude": [self.attitude.roll, self.attitude.pitch, self.attitude.yaw],
            "battery_percent": self.battery.percentage(),
            "mode": self.mode.value,
            "mission_phase": self.mission_phase.value,
            "is_armed": self.is_armed,
            "ground_speed": self.ground_speed,
            "altitude": self.altitude_agl
        }


@dataclass
class MissionData:
    """Mission-specific data and obstacles"""
    
    home: Position = field(default_factory=Position)
    delivery_point: Position = field(default_factory=lambda: Position(5500, 700, 0))
    
    # Environment (ported from MATLAB)
    obstacles: List[Obstacle] = field(default_factory=list)
    no_fly_zones: List[NoFlyZone] = field(default_factory=list)
    
    # Mission tracking
    waypoints: List[Position] = field(default_factory=list)
    current_waypoint_index: int = 0
    
    # Path history
    path_history: List[Position] = field(default_factory=list)
    
    def add_default_obstacles(self):
        """Add default obstacles from MATLAB code"""
        # Obstacles from MATLAB: [x y z radius]
        obstacle_data = [
            [1200, 200, 120, 140],
            [2000, 300, 120, 160],
            [2800, 400, 120, 180],
            [3500, 300, 120, 160],
            [4300, 500, 120, 200],
            [4800, 600, 120, 170]
        ]
        
        for obs in obstacle_data:
            self.obstacles.append(
                Obstacle(
                    position=Position(obs[0], obs[1], obs[2]),
                    radius=obs[3]
                )
            )
    
    def add_default_no_fly_zones(self):
        """Add default no-fly zones from MATLAB code"""
        # No-fly zones: [x y z radius]
        nfz_data = [
            [2600, 250, 120, 300],
            [4100, 400, 120, 350]
        ]
        
        for nfz in nfz_data:
            self.no_fly_zones.append(
                NoFlyZone(
                    position=Position(nfz[0], nfz[1], nfz[2]),
                    radius=nfz[3]
                )
            )
    
    def check_obstacle_collision(self, point: Position, buffer: float = 50.0) -> Optional[Obstacle]:
        """Check if point collides with any obstacle (with buffer)"""
        for obs in self.obstacles:
            if obs.distance_to_point(point) < buffer:
                return obs
        return None
    
    def check_nfz_violation(self, point: Position) -> Optional[NoFlyZone]:
        """Check if point violates any no-fly zone"""
        for nfz in self.no_fly_zones:
            if nfz.is_violation(point):
                return nfz
        return None
