"""
Mission Planner - Core flight logic ported from MATLAB
Handles autonomous navigation, obstacle avoidance, and energy management
"""

import numpy as np
import logging
from typing import Optional, Tuple, List
from dataclasses import dataclass

from src.core.uav_state import (
    UAVState, Position, Velocity, FlightMode, 
    MissionPhase, MissionData, Obstacle
)
from src.config import UAVConfig

logger = logging.getLogger(__name__)


class MissionPlanner:
    """
    Autonomous mission planning and execution
    Ported from VTOL_UAV_Delivery.m and VTOL_UAV_Delivery_new.m
    """
    
    def __init__(self, mission_data: MissionData, config: UAVConfig = UAVConfig(), custom_params: dict = None):
        self.mission = mission_data
        self.config = config
        
        # Initialize default custom parameters
        self.wind_speed = 0.0
        self.wind_direction = 0.0
        self.drone_weight = 2.5
        self.payload_weight = 0.5
        
        # Apply custom parameters if provided
        self.custom_params = custom_params or {}
        if custom_params:
            self._apply_custom_parameters(custom_params)
        
        # State tracking
        self.current_time = 0.0
        self.mission_complete = False
    
    def _apply_custom_parameters(self, params: dict):
        """Apply custom UAV parameters to override config defaults"""
        # Update UAV config with custom parameters
        if 'cruise_speed' in params:
            self.config.CRUISE_SPEED = params['cruise_speed']
        if 'vtol_speed' in params:
            self.config.VTOL_SPEED = params['vtol_speed']
        if 'max_altitude' in params:
            self.config.MAX_ALTITUDE = params['max_altitude']
        if 'cruise_altitude' in params:
            self.config.TAKEOFF_ALTITUDE = params['cruise_altitude']
        if 'battery_capacity_mah' in params:
            self.config.BATTERY_CAPACITY = params['battery_capacity_mah']
        if 'battery_voltage' in params:
            self.config.BATTERY_VOLTAGE = params['battery_voltage']
        
        # Store additional params for calculations
        self.wind_speed = params.get('wind_speed', 0.0)
        self.wind_direction = params.get('wind_direction', 0.0)
        self.drone_weight = params.get('drone_weight', 2.5)
        self.payload_weight = params.get('payload_weight', 0.5)
        
    def update(self, state: UAVState, dt: float) -> Tuple[Position, Velocity, FlightMode]:
        """
        Main control loop - execute one planning step
        Returns: (target_position, desired_velocity, flight_mode)
        """
        self.current_time += dt
        
        # Determine target based on mission phase and waypoints
        if not state.delivered:
            # OUTBOUND JOURNEY - follow waypoints to delivery point
            if len(self.mission.waypoints) > 0:
                # Follow waypoints in order
                current_waypoint_idx = getattr(state, 'current_waypoint_index', 0)
                
                # Check if reached current waypoint
                if current_waypoint_idx < len(self.mission.waypoints):
                    current_waypoint = self.mission.waypoints[current_waypoint_idx]
                    dist_to_waypoint = state.position.horizontal_distance_to(current_waypoint)
                    
                    if dist_to_waypoint < 50.0:  # Within 50m of waypoint
                        # Move to next waypoint
                        current_waypoint_idx += 1
                        state.current_waypoint_index = current_waypoint_idx
                        logger.info(f"Reached waypoint {current_waypoint_idx}, advancing")
                    
                    # Set target to current waypoint or delivery point if all waypoints passed
                    if current_waypoint_idx < len(self.mission.waypoints):
                        target = Position(
                            self.mission.waypoints[current_waypoint_idx].x,
                            self.mission.waypoints[current_waypoint_idx].y,
                            self.config.TAKEOFF_ALTITUDE
                        )
                    else:
                        # All waypoints reached, go to delivery point
                        target = Position(
                            self.mission.delivery_point.x,
                            self.mission.delivery_point.y,
                            self.config.TAKEOFF_ALTITUDE
                        )
                else:
                    # All waypoints passed, go to delivery
                    target = Position(
                        self.mission.delivery_point.x,
                        self.mission.delivery_point.y,
                        self.config.TAKEOFF_ALTITUDE
                    )
            else:
                # No waypoints, go directly to delivery point
                target = Position(
                    self.mission.delivery_point.x,
                    self.mission.delivery_point.y,
                    self.config.TAKEOFF_ALTITUDE
                )
        else:
            # RETURN JOURNEY - go directly home
            target = Position(
                self.mission.home.x,
                self.mission.home.y,
                self.config.TAKEOFF_ALTITUDE
            )
        
        state.target_position = target
        
        # Flight mode state machine
        new_mode = self._update_flight_mode(state)
        
        # Calculate desired velocity based on mode
        desired_vel = self._calculate_desired_velocity(state, target, new_mode)
        
        # Apply obstacle avoidance during flying modes (but NOT emergency landing)
        if new_mode in [FlightMode.CRUISE, FlightMode.VTOL_TAKEOFF, FlightMode.EMERGENCY_RTL]:
            desired_vel = self._apply_obstacle_avoidance(state, desired_vel)
        
        # Update battery consumption
        self._update_battery(state, new_mode, dt)
        
        # Check mission completion
        self._check_mission_status(state)
        
        return target, Velocity.from_array(desired_vel), new_mode
    
    def _update_flight_mode(self, state: UAVState) -> FlightMode:
        """
        Flight mode state machine logic (ported from MATLAB)
        """
        current_mode = state.mode
        
        # Emergency checks (highest priority)
        if state.battery.is_critical(self.config.BATTERY_CRITICAL):
            return FlightMode.EMERGENCY_LAND
        
        if state.geofence_violation:
            return FlightMode.EMERGENCY_RTL
        
        # Normal mode transitions
        if current_mode == FlightMode.IDLE:
            if state.is_armed:
                return FlightMode.VTOL_TAKEOFF
            return FlightMode.IDLE
        
        elif current_mode == FlightMode.VTOL_TAKEOFF:
            # Transition to cruise when at altitude
            if abs(state.position.z - self.config.TAKEOFF_ALTITUDE) < 5.0:
                state.mission_phase = MissionPhase.TRANSIT_TO_TARGET
                return FlightMode.CRUISE
            return FlightMode.VTOL_TAKEOFF
        
        elif current_mode == FlightMode.CRUISE:
            # Check if reached target
            if state.target_position:
                dist = state.position.horizontal_distance_to(state.target_position)
                
                if dist < 20.0:  # Within 20m of target
                    if not state.delivered:
                        # Reached delivery point, prepare to land
                        state.mission_phase = MissionPhase.AT_TARGET
                        return FlightMode.VTOL_LAND
                    else:
                        # Reached home
                        state.mission_phase = MissionPhase.LANDING
                        return FlightMode.VTOL_LAND
            
            return FlightMode.CRUISE
        
        elif current_mode == FlightMode.VTOL_LAND:
            # Check if landed
            if state.position.z < 2.0:  # On ground
                if state.delivered and state.position.horizontal_distance_to(state.home_position) < 20.0:
                    # Landed at home after delivery - MISSION COMPLETE
                    state.mission_phase = MissionPhase.COMPLETE
                    self.mission_complete = True
                    state.is_armed = False  # DISARM to prevent re-takeoff
                    return FlightMode.IDLE
                elif not state.delivered and state.position.horizontal_distance_to(self.mission.delivery_point) < 20.0:
                    # Just landed at delivery point - mark as delivered and takeoff to return home
                    state.delivered = True
                    state.mission_phase = MissionPhase.RETURN_HOME
                    return FlightMode.VTOL_TAKEOFF
                else:
                    # Landed somewhere unexpected, stay grounded
                    return FlightMode.VTOL_LAND
            
            return FlightMode.VTOL_LAND
        
        elif current_mode == FlightMode.EMERGENCY_RTL:
            # Return to home
            if state.position.horizontal_distance_to(state.home_position) < 10.0:
                return FlightMode.EMERGENCY_LAND
            return FlightMode.EMERGENCY_RTL
        
        elif current_mode == FlightMode.EMERGENCY_LAND:
            if state.position.z < 2.0:
                return FlightMode.IDLE
            return FlightMode.EMERGENCY_LAND
        
        return current_mode
    
    def _calculate_desired_velocity(
        self, 
        state: UAVState, 
        target: Position, 
        mode: FlightMode
    ) -> np.ndarray:
        """
        Calculate desired velocity vector based on flight mode
        """
        if mode == FlightMode.IDLE:
            return np.array([0.0, 0.0, 0.0])
        
        elif mode == FlightMode.VTOL_TAKEOFF:
            # Vertical ascent
            return np.array([0.0, 0.0, self.config.VTOL_SPEED])
        
        elif mode == FlightMode.CRUISE:
            # Horizontal cruise toward target at cruise altitude
            to_target = target.to_array() - state.position.to_array()
            
            # Calculate horizontal movement
            horizontal_dist = np.linalg.norm(to_target[:2])
            if horizontal_dist > 0.1:
                direction = to_target / np.linalg.norm(to_target)
                direction[2] = 0  # Zero out vertical component
                
                # Add altitude correction to maintain cruise altitude
                altitude_error = self.config.TAKEOFF_ALTITUDE - state.position.z
                vertical_correction = np.clip(altitude_error * 0.5, -2.0, 2.0)  # Gentle altitude hold
                
                velocity = direction * self.config.CRUISE_SPEED
                velocity[2] = vertical_correction  # Maintain altitude
                return velocity
            else:
                # At waypoint - just maintain altitude
                altitude_error = self.config.TAKEOFF_ALTITUDE - state.position.z
                vertical_correction = np.clip(altitude_error * 0.5, -2.0, 2.0)
                return np.array([0.0, 0.0, vertical_correction])
        
        elif mode == FlightMode.VTOL_LAND:
            # Vertical descent
            return np.array([0.0, 0.0, -self.config.VTOL_SPEED])
        
        elif mode == FlightMode.EMERGENCY_RTL:
            # Fast return to home at cruise altitude
            to_home = state.home_position.to_array() - state.position.to_array()
            
            horizontal_dist = np.linalg.norm(to_home[:2])
            if horizontal_dist > 0.1:
                direction = to_home / np.linalg.norm(to_home)
                direction[2] = 0  # Zero out vertical
                
                # Add altitude correction
                altitude_error = self.config.TAKEOFF_ALTITUDE - state.position.z
                vertical_correction = np.clip(altitude_error * 0.5, -2.0, 2.0)
                
                velocity = direction * self.config.CRUISE_SPEED * 1.2  # 20% faster
                velocity[2] = vertical_correction  # Maintain altitude
                return velocity
            else:
                # At home - maintain altitude until switched to EMERGENCY_LAND
                altitude_error = self.config.TAKEOFF_ALTITUDE - state.position.z
                vertical_correction = np.clip(altitude_error * 0.5, -2.0, 2.0)
                return np.array([0.0, 0.0, vertical_correction])
        
        elif mode == FlightMode.EMERGENCY_LAND:
            # Emergency landing - descend straight down at current position
            # No horizontal movement, just vertical descent at faster rate
            return np.array([0.0, 0.0, -self.config.VTOL_SPEED * 1.5])  # 50% faster descent
        
        return np.array([0.0, 0.0, 0.0])
    
    def _apply_obstacle_avoidance(
        self, 
        state: UAVState, 
        desired_vel: np.ndarray
    ) -> np.ndarray:
        """
        Balanced obstacle avoidance - avoid obstacles while staying on course
        Uses moderate repulsion from obstacles with strong waypoint attraction
        """
        pos = state.position.to_array()
        vel_mag = np.linalg.norm(desired_vel)
        
        if vel_mag < 0.1:
            return desired_vel
        
        vel_dir = desired_vel / vel_mag
        
        # Total force accumulator (repulsion + attraction)
        total_force = np.array([0.0, 0.0, 0.0])
        
        # REPULSIVE FORCES from obstacles (moderate strength)
        for obs in self.mission.obstacles:
            obs_pos = obs.position.to_array()
            to_obstacle = obs_pos - pos
            
            # Full 3D distance to obstacle surface
            dist_3d = np.linalg.norm(to_obstacle)
            dist_to_surface = dist_3d - obs.radius
            
            # Check altitude overlap
            altitude_diff = abs(pos[2] - obs.position.z)
            if altitude_diff > obs.radius + 100:  # If well above/below, skip
                continue
            
            # Only apply repulsion if within influence zone (reduced from 500m to 250m)
            influence_distance = 250  # Meters - shorter influence for tighter paths
            
            if dist_to_surface < influence_distance:
                state.obstacle_detected = True
                
                # Calculate repulsion direction (away from obstacle)
                if dist_3d > 0.1:
                    repulsion_dir = -to_obstacle / dist_3d
                else:
                    # If exactly at obstacle center, push in opposite of velocity
                    repulsion_dir = -vel_dir
                
                # Moderate repulsion strength - just enough to avoid collision
                if dist_to_surface < obs.radius:  # Inside obstacle!
                    strength = 8.0  # Strong but not excessive
                elif dist_to_surface < 30:  # Very close
                    strength = 5.0
                elif dist_to_surface < 60:  # Close
                    strength = 3.0
                elif dist_to_surface < 100:  # Near
                    strength = 1.5
                else:  # Distant but still in influence
                    strength = 100.0 / max(10.0, dist_to_surface)
                
                # Apply repulsive force (keep it horizontal for smooth flight)
                repulsion_force = repulsion_dir * strength
                repulsion_force[2] = 0  # Zero out vertical - maintain altitude in cruise
                
                total_force += repulsion_force
        
        # REPULSIVE FORCES from No-Fly Zones (stronger than obstacles)
        for nfz in self.mission.no_fly_zones:
            nfz_pos = nfz.position.to_array()
            to_nfz = nfz_pos - pos
            
            dist_3d = np.linalg.norm(to_nfz)
            dist_to_surface = dist_3d - nfz.radius
            
            altitude_diff = abs(pos[2] - nfz.position.z)
            if altitude_diff > nfz.radius + 200:
                continue
            
            influence_distance = 300  # NFZ has larger influence (reduced from 600m)
            
            if dist_to_surface < influence_distance:
                state.geofence_violation = True
                
                if dist_3d > 0.1:
                    repulsion_dir = -to_nfz / dist_3d
                else:
                    repulsion_dir = -vel_dir
                
                # NFZ repulsion is stronger but not excessive
                if dist_to_surface < nfz.radius:
                    strength = 12.0  # Strong if inside NFZ
                elif dist_to_surface < 50:
                    strength = 8.0
                elif dist_to_surface < 100:
                    strength = 5.0
                else:
                    strength = 200.0 / max(10.0, dist_to_surface)
                
                repulsion_force = repulsion_dir * strength
                repulsion_force[2] = 0  # Zero out vertical - maintain altitude
                
                total_force += repulsion_force
        
        # STRONG ATTRACTIVE FORCE toward target waypoint - keeps drone on track
        if state.target_position:
            to_target = state.target_position.to_array() - pos
            to_target[2] = 0  # Keep attraction horizontal
            dist_to_target = np.linalg.norm(to_target)
            
            if dist_to_target > 1.0:
                # Strong attraction to waypoint - balances repulsion
                attraction_dir = to_target / dist_to_target
                
                # Scale attraction based on distance - stronger when far from waypoint
                if dist_to_target > 200:
                    attraction_strength = 8.0  # Strong pull when far off course
                elif dist_to_target > 100:
                    attraction_strength = 6.0
                elif dist_to_target > 50:
                    attraction_strength = 4.0
                else:
                    attraction_strength = 2.0
                
                total_force += attraction_dir * attraction_strength
        
        # BOUNDARY CONSTRAINT - prevent going too far off the map
        # Map bounds: X(0-6000), Y(-200-1000)
        boundary_force = np.array([0.0, 0.0, 0.0])
        
        # X boundaries (0 to 6000m)
        if pos[0] < 200:  # Too close to left edge
            boundary_force[0] = 5.0 * (200 - pos[0]) / 200.0
        elif pos[0] > 5800:  # Too close to right edge
            boundary_force[0] = -5.0 * (pos[0] - 5800) / 200.0
        
        # Y boundaries (-200 to 1000m)
        if pos[1] < 0:  # Too close to bottom edge
            boundary_force[1] = 5.0 * (0 - pos[1]) / 200.0
        elif pos[1] > 800:  # Too close to top edge
            boundary_force[1] = -5.0 * (pos[1] - 800) / 200.0
        
        total_force += boundary_force
        
        # Apply total force to modify velocity
        if np.linalg.norm(total_force) > 0.01:
            # Combine desired velocity with force field - reduced influence
            # Force field modifies direction while trying to maintain speed
            modified_vel = desired_vel + total_force * vel_mag * 0.3  # Reduced from 0.5
            
            # Limit maximum deviation to maintain reasonable paths
            modified_mag = np.linalg.norm(modified_vel)
            if modified_mag > 0.1:
                # Scale to maintain original speed
                modified_vel = (modified_vel / modified_mag) * vel_mag
                
                # Ensure the path stays reasonably close to original direction
                original_dir = desired_vel / vel_mag
                new_dir = modified_vel / vel_mag
                
                # Check if we're deviating too much from target direction
                dot_product = np.dot(original_dir, new_dir)
                if dot_product < 0.3:  # Too much deviation (changed from 0)
                    # Blend more conservatively to stay on course
                    modified_vel = desired_vel * 0.8 + modified_vel * 0.2
                    modified_mag = np.linalg.norm(modified_vel)
                    if modified_mag > 0.1:
                        modified_vel = (modified_vel / modified_mag) * vel_mag
            
            return modified_vel
        else:
            state.obstacle_detected = False
            state.geofence_violation = False
            return desired_vel
    
    def _update_battery(self, state: UAVState, mode: FlightMode, dt: float):
        """
        Update battery consumption based on flight mode
        Uses custom battery capacity and weight parameters if available
        """
        # Get total weight (affects power consumption)
        total_weight = self.drone_weight + (self.payload_weight if not state.delivered else 0.0)
        weight_factor = total_weight / 3.0  # Normalized to default 3kg
        
        # Current draw depends on mode (from MATLAB parameters)
        if mode in [FlightMode.VTOL_TAKEOFF, FlightMode.VTOL_LAND, FlightMode.HOVER]:
            current_draw = self.config.VTOL_CURRENT * weight_factor  # mAh/min
        elif mode == FlightMode.CRUISE:
            # Add wind resistance effect
            wind_effect = 1.0 + (self.wind_speed / 20.0)  # Up to 50% more at 20m/s wind
            current_draw = self.config.CRUISE_CURRENT * weight_factor * wind_effect  # mAh/min
        elif mode == FlightMode.EMERGENCY_RTL:
            current_draw = self.config.CRUISE_CURRENT * 1.3 * weight_factor  # Higher power
        else:
            current_draw = 0  # Idle
        
        # Convert to mAh consumed in this timestep
        dt_minutes = dt / 60.0
        mah_consumed = current_draw * dt_minutes
        
        # Update battery
        state.battery.remaining_mah -= mah_consumed
        state.battery.remaining_mah = max(0, state.battery.remaining_mah)
    
    def _check_mission_status(self, state: UAVState):
        """
        Update mission phase and completion status
        """
        if state.mode == FlightMode.IDLE and state.delivered:
            if state.position.horizontal_distance_to(state.home_position) < 10.0:
                self.mission_complete = True
                state.mission_phase = MissionPhase.COMPLETE


def calculate_energy_optimal_path(
    start: Position,
    end: Position,
    obstacles: List[Obstacle],
    altitude: float = 120.0
) -> List[Position]:
    """
    Calculate energy-optimal path avoiding obstacles
    Simple A* implementation for waypoint generation
    """
    # TODO: Implement A* or RRT* path planning
    # For now, return direct path
    waypoints = [
        start,
        Position(start.x, start.y, altitude),  # Climb
        Position(end.x, end.y, altitude),       # Cruise
        Position(end.x, end.y, end.z),          # Descend
        end
    ]
    
    return waypoints


def estimate_mission_energy(
    waypoints: List[Position],
    cruise_speed: float = 28.0,
    cruise_current: float = 120.0,
    vtol_current: float = 220.0
) -> Tuple[float, float]:
    """
    Estimate total energy and time for mission
    Returns: (total_mah, total_seconds)
    """
    total_mah = 0.0
    total_time = 0.0
    
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        
        dist = p1.distance_to(p2)
        dz = abs(p2.z - p1.z)
        
        # Vertical segment (VTOL mode)
        if dz > dist * 0.8:  # Mostly vertical
            time = dist / 3.0  # VTOL speed
            total_mah += vtol_current * (time / 60.0)
            total_time += time
        else:
            # Horizontal segment (cruise mode)
            time = dist / cruise_speed
            total_mah += cruise_current * (time / 60.0)
            total_time += time
    
    return total_mah, total_time
