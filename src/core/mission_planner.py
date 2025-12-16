"""
Mission Planner - Core flight logic with trajectory scoring and multi-objective optimization
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


@dataclass
class TrajectoryScore:
    """Scoring metrics for trajectory evaluation"""
    safety_score: float      # 0-1: Higher = safer (distance from obstacles)
    energy_score: float      # 0-1: Higher = more efficient
    progress_score: float    # 0-1: Higher = closer to goal
    total_score: float       # Weighted combination
    recommended_speed: float # Optimal speed for this trajectory


class MissionPlanner:
    """
    Autonomous mission planning with multi-objective optimization
    Features:
    - Trajectory scoring (safety, energy, progress)
    - Speed optimization based on battery and distance
    - Lookahead prediction with tangential avoidance
    """
    
    def __init__(self, mission_data: MissionData, config: UAVConfig = UAVConfig(), custom_params: dict = None):
        self.mission = mission_data
        self.config = config
        
        # Initialize default custom parameters
        self.wind_speed = 0.0
        self.wind_direction = 0.0
        self.drone_weight = 2.5
        self.payload_weight = 0.5
        
        # Multi-objective optimization weights (sum to 1.0)
        self.WEIGHT_SAFETY = 0.5    # Safety is most important
        self.WEIGHT_ENERGY = 0.3    # Energy efficiency second
        self.WEIGHT_PROGRESS = 0.2  # Progress toward goal third
        
        # Apply custom parameters if provided
        self.custom_params = custom_params or {}
        if custom_params:
            self._apply_custom_parameters(custom_params)
        
        # State tracking
        self.current_time = 0.0
        self.mission_complete = False
        self.last_trajectory_score = None
    
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
            # Stay in IDLE even when armed - require explicit takeoff command
            # Takeoff is initiated by START MISSION or manual TAKEOFF button
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
                
                # SPEED OPTIMIZATION - Calculate energy-efficient cruise speed
                optimal_speed = self._calculate_optimal_speed(state, horizontal_dist)
                
                # Add altitude correction to maintain cruise altitude
                altitude_error = self.config.TAKEOFF_ALTITUDE - state.position.z
                vertical_correction = np.clip(altitude_error * 0.5, -2.0, 2.0)  # Gentle altitude hold
                
                velocity = direction * optimal_speed
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
    
    def _calculate_optimal_speed(self, state: UAVState, distance_to_target: float) -> float:
        """
        Calculate energy-efficient cruise speed based on:
        - Battery level (conserve when low)
        - Distance to target (faster when far, slower when close)
        - Wind conditions (adjust for headwind/tailwind)
        
        Returns: Optimal speed in m/s
        """
        base_speed = self.config.CRUISE_SPEED
        
        # 1. BATTERY-BASED ADJUSTMENT
        battery_percent = (state.battery.remaining_mah / state.battery.capacity_mah) * 100
        
        if battery_percent < 20:
            # Critical battery - reduce to minimum safe speed
            battery_factor = 0.7  # 70% of normal speed
        elif battery_percent < 40:
            # Low battery - moderate reduction
            battery_factor = 0.85  # 85% of normal speed
        else:
            # Good battery - full speed
            battery_factor = 1.0
        
        # 2. DISTANCE-BASED ADJUSTMENT
        # Slow down when approaching target for smooth landing transition
        if distance_to_target < 100:
            # Within 100m - start slowing down
            distance_factor = 0.6 + (distance_to_target / 100) * 0.4  # 60-100%
        elif distance_to_target > 1000:
            # Far away - maintain high speed for efficiency
            distance_factor = 1.0
        else:
            # Medium distance - normal speed
            distance_factor = 0.9
        
        # 3. WIND COMPENSATION (if wind data available)
        wind_factor = 1.0
        if self.wind_speed > 0:
            # Simplified wind effect (increase speed in headwind, decrease in tailwind)
            wind_factor = 1.0 + (self.wind_speed / 50.0)  # Up to 20% adjustment
            wind_factor = np.clip(wind_factor, 0.8, 1.2)
        
        # COMBINE ALL FACTORS
        optimal_speed = base_speed * battery_factor * distance_factor * wind_factor
        
        # Ensure speed stays within safe limits
        min_safe_speed = base_speed * 0.5  # Never below 50% of cruise speed
        max_safe_speed = base_speed * 1.0  # Never above nominal cruise
        
        return np.clip(optimal_speed, min_safe_speed, max_safe_speed)
    
    def _score_trajectory(
        self, 
        state: UAVState, 
        velocity: np.ndarray,
        avoidance_active: bool,
        clearance_distance: float
    ) -> TrajectoryScore:
        """
        Score current trajectory based on multiple objectives:
        - Safety: Distance from obstacles (higher = safer)
        - Energy: Speed efficiency and battery consumption
        - Progress: Movement toward goal
        
        Returns: TrajectoryScore with individual and total scores
        """
        # 1. SAFETY SCORE (0-1, higher is safer)
        # Based on clearance distance from nearest obstacle
        if clearance_distance < 10:
            safety_score = 0.0  # Dangerously close
        elif clearance_distance < 50:
            safety_score = 0.3 + (clearance_distance - 10) / 40 * 0.4  # 0.3-0.7
        elif clearance_distance < 150:
            safety_score = 0.7 + (clearance_distance - 50) / 100 * 0.2  # 0.7-0.9
        else:
            safety_score = 1.0  # Safe distance
        
        # Penalty for active avoidance maneuvers
        if avoidance_active:
            safety_score *= 0.8  # 20% reduction when actively avoiding
        
        # 2. ENERGY SCORE (0-1, higher is more efficient)
        speed = np.linalg.norm(velocity)
        
        # Energy consumption roughly proportional to speed^2 for air resistance
        # But too slow is also inefficient (more time = more hover energy)
        optimal_cruise = self.config.CRUISE_SPEED
        
        if speed < optimal_cruise * 0.5:
            # Too slow - inefficient hovering
            energy_score = speed / (optimal_cruise * 0.5)  # 0-1
        elif speed > optimal_cruise:
            # Too fast - wasting energy on air resistance
            excess = speed - optimal_cruise
            energy_score = max(0.5, 1.0 - (excess / optimal_cruise))  # Penalty
        else:
            # In optimal range
            energy_score = 0.8 + (1.0 - abs(speed - optimal_cruise) / optimal_cruise) * 0.2
        
        # Battery consideration
        battery_percent = (state.battery.remaining_mah / state.battery.capacity_mah) * 100
        if battery_percent < 30:
            energy_score *= (battery_percent / 30)  # Penalty for low battery
        
        # 3. PROGRESS SCORE (0-1, higher is better progress)
        # Measure alignment with target direction
        if state.target_position:
            to_target = state.target_position.to_array() - state.position.to_array()
            to_target_norm = np.linalg.norm(to_target)
            
            if to_target_norm > 0.1 and speed > 0.1:
                # Dot product of velocity and target direction
                vel_norm = velocity / speed
                target_norm = to_target / to_target_norm
                alignment = np.dot(vel_norm, target_norm)
                
                # Convert to 0-1 score (alignment ranges from -1 to 1)
                progress_score = (alignment + 1.0) / 2.0  # 0 to 1
            else:
                progress_score = 0.5  # Neutral when at target or stopped
        else:
            progress_score = 0.5  # Neutral when no target
        
        # 4. CALCULATE TOTAL SCORE
        total_score = (
            self.WEIGHT_SAFETY * safety_score +
            self.WEIGHT_ENERGY * energy_score +
            self.WEIGHT_PROGRESS * progress_score
        )
        
        # Recommend speed adjustment based on scores
        if safety_score < 0.5:
            # Low safety - recommend slower speed
            recommended_speed = speed * 0.6
        elif energy_score < 0.6:
            # Poor energy efficiency - adjust toward optimal
            recommended_speed = optimal_cruise * 0.85
        else:
            # Good balance - maintain current speed
            recommended_speed = speed
        
        return TrajectoryScore(
            safety_score=safety_score,
            energy_score=energy_score,
            progress_score=progress_score,
            total_score=total_score,
            recommended_speed=recommended_speed
        )
    
    def _apply_obstacle_avoidance(
        self, 
        state: UAVState, 
        desired_vel: np.ndarray
    ) -> np.ndarray:
        """
        Advanced obstacle avoidance with lookahead prediction, tangential navigation,
        and trajectory scoring for multi-objective optimization.
        
        Algorithm:
        1. Predict future position based on current velocity (5 second lookahead)
        2. Check if future position will intersect with obstacles/NFZ
        3. If collision predicted, calculate tangential avoidance vector
        4. Score trajectory (safety, energy, progress)
        5. Blend avoidance with path-following to stay on course
        6. Modulate speed based on proximity and trajectory score
        """
        pos = state.position.to_array()
        vel_mag = np.linalg.norm(desired_vel)
        
        if vel_mag < 0.1:
            return desired_vel
        
        vel_dir = desired_vel / vel_mag
        
        # LOOKAHEAD PREDICTION - predict position 5 seconds ahead
        lookahead_time = 5.0  # seconds
        future_pos = pos + desired_vel * lookahead_time
        
        # Track if avoidance is needed and minimum clearance distance
        avoidance_needed = False
        avoidance_vector = np.array([0.0, 0.0, 0.0])
        speed_reduction_factor = 1.0  # 1.0 = full speed, 0.5 = half speed
        min_clearance = float('inf')  # Track minimum distance to obstacles
        
        # OBSTACLE AVOIDANCE - Check all obstacles with lookahead
        for obs in self.mission.obstacles:
            obs_pos = obs.position.to_array()
            
            # Safety margin and turn radius for smooth avoidance
            safety_margin = 50.0  # meters
            turn_radius = 100.0   # meters for smooth arcs
            detection_radius = obs.radius + safety_margin + turn_radius
            
            # Check altitude overlap first (skip if well above/below)
            altitude_diff = abs(pos[2] - obs.position.z)
            if altitude_diff > obs.radius + 100:
                continue
            
            # Distance from FUTURE position to obstacle center
            to_obs_future = obs_pos - future_pos
            dist_future = np.linalg.norm(to_obs_future)
            
            # Distance from CURRENT position to obstacle
            to_obs_current = obs_pos - pos
            dist_current = np.linalg.norm(to_obs_current)
            
            # Check if we're approaching the obstacle (future closer than current)
            if dist_future < detection_radius:
                avoidance_needed = True
                state.obstacle_detected = True
                
                # Calculate tangential avoidance direction
                # Direction away from obstacle center
                if dist_current > 0.1:
                    repulsion_dir = -to_obs_current / dist_current
                else:
                    repulsion_dir = -vel_dir
                
                # Calculate influence based on proximity (0=far, 1=very close)
                influence = max(0.0, (detection_radius - dist_future) / detection_radius)
                
                # Tangential avoidance: rotate repulsion by 90 degrees for smooth arc
                # This makes the drone go AROUND the obstacle, not just away from it
                repulsion_gain = 0.4  # Tuning parameter (0.2-0.5 range)
                angle_offset = repulsion_gain * influence * (np.pi / 2)  # Up to 90 degrees
                
                # Rotate the repulsion direction (2D rotation in XY plane)
                cos_angle = np.cos(angle_offset)
                sin_angle = np.sin(angle_offset)
                rotated_x = repulsion_dir[0] * cos_angle - repulsion_dir[1] * sin_angle
                rotated_y = repulsion_dir[0] * sin_angle + repulsion_dir[1] * cos_angle
                
                tangential_dir = np.array([rotated_x, rotated_y, 0.0])
                
                # Strength based on proximity
                if dist_future < obs.radius + safety_margin:
                    strength = 3.0 * influence  # Strong avoidance when very close
                else:
                    strength = 1.5 * influence  # Moderate avoidance when approaching
                
                avoidance_vector += tangential_dir * strength
                
                # Reduce speed when avoiding obstacles
                speed_reduction_factor = min(speed_reduction_factor, 0.6)  # Max 40% reduction
                
                # Track minimum clearance for trajectory scoring
                min_clearance = min(min_clearance, dist_current - obs.radius)

        
        # NO-FLY ZONE AVOIDANCE - Stronger avoidance than regular obstacles
        for nfz in self.mission.no_fly_zones:
            nfz_pos = nfz.position.to_array()
            
            # Larger safety margins for NFZ
            safety_margin = 50.0
            turn_radius = 100.0
            detection_radius = nfz.radius + safety_margin + turn_radius
            
            # Check altitude overlap
            altitude_diff = abs(pos[2] - nfz.position.z)
            if altitude_diff > nfz.radius + 200:
                continue
            
            # Distance from FUTURE position to NFZ center
            to_nfz_future = nfz_pos - future_pos
            dist_future = np.linalg.norm(to_nfz_future)
            
            # Distance from CURRENT position to NFZ
            to_nfz_current = nfz_pos - pos
            dist_current = np.linalg.norm(to_nfz_current)
            
            if dist_future < detection_radius:
                avoidance_needed = True
                state.geofence_violation = True
                
                # Calculate tangential avoidance (stronger for NFZ)
                if dist_current > 0.1:
                    repulsion_dir = -to_nfz_current / dist_current
                else:
                    repulsion_dir = -vel_dir
                
                # Higher influence for no-fly zones
                influence = max(0.0, (detection_radius - dist_future) / detection_radius)
                
                # Stronger tangential rotation for NFZ
                repulsion_gain = 0.5  # Higher than obstacles (0.5 vs 0.4)
                angle_offset = repulsion_gain * influence * (np.pi / 2)
                
                # 2D rotation
                cos_angle = np.cos(angle_offset)
                sin_angle = np.sin(angle_offset)
                rotated_x = repulsion_dir[0] * cos_angle - repulsion_dir[1] * sin_angle
                rotated_y = repulsion_dir[0] * sin_angle + repulsion_dir[1] * cos_angle
                
                tangential_dir = np.array([rotated_x, rotated_y, 0.0])
                
                # Stronger avoidance for NFZ
                if dist_future < nfz.radius + safety_margin:
                    strength = 5.0 * influence  # Very strong when inside
                else:
                    strength = 2.5 * influence  # Strong when approaching
                
                avoidance_vector += tangential_dir * strength
                
                # Stronger speed reduction for NFZ
                speed_reduction_factor = min(speed_reduction_factor, 0.5)  # Max 50% reduction
                
                # Track minimum clearance for trajectory scoring
                min_clearance = min(min_clearance, dist_current - nfz.radius)

                avoidance_vector += tangential_dir * strength
                
                # Stronger speed reduction for NFZ
                speed_reduction_factor = min(speed_reduction_factor, 0.5)  # Max 50% reduction

        
        # PATH ATTRACTION - Strong pull toward target to stay on course
        path_attraction = np.array([0.0, 0.0, 0.0])
        if state.target_position:
            to_target = state.target_position.to_array() - pos
            to_target[2] = 0  # Keep horizontal
            dist_to_target = np.linalg.norm(to_target)
            
            if dist_to_target > 1.0:
                # Normalize direction to target
                target_dir = to_target / dist_to_target
                
                # Strong attraction that increases with distance from path
                # This keeps the drone on track even while avoiding
                if avoidance_needed:
                    # When avoiding, increase path attraction to counterbalance
                    attraction_strength = 4.0
                else:
                    # Normal path following
                    attraction_strength = 2.0
                
                path_attraction = target_dir * attraction_strength
        
        # BOUNDARY CONSTRAINTS - Prevent going off map edges
        boundary_force = np.array([0.0, 0.0, 0.0])
        
        # X boundaries (200m to 5800m)
        if pos[0] < 200:
            boundary_force[0] = 3.0 * (200 - pos[0]) / 200.0
        elif pos[0] > 5800:
            boundary_force[0] = -3.0 * (pos[0] - 5800) / 200.0
        
        # Y boundaries (0m to 800m)
        if pos[1] < 0:
            boundary_force[1] = 3.0 * abs(pos[1]) / 200.0
        elif pos[1] > 800:
            boundary_force[1] = -3.0 * (pos[1] - 800) / 200.0
        
        # COMBINE ALL FORCES
        if avoidance_needed:
            # When avoiding: blend avoidance vector with path attraction
            # Avoidance gets 60%, path attraction gets 40%
            combined_direction = avoidance_vector * 0.6 + path_attraction * 0.4 + boundary_force
            
            # Normalize the combined direction
            combined_mag = np.linalg.norm(combined_direction)
            if combined_mag > 0.1:
                combined_direction = combined_direction / combined_mag
            else:
                combined_direction = vel_dir  # Fallback to original direction
            
            # Apply speed reduction during avoidance
            adjusted_speed = vel_mag * speed_reduction_factor
            
            # Create final velocity with reduced speed
            final_velocity = combined_direction * adjusted_speed
            
            # Maintain vertical component from desired velocity (altitude hold)
            final_velocity[2] = desired_vel[2]
            
            # TRAJECTORY SCORING - Evaluate the avoidance maneuver
            clearance = max(10.0, min_clearance) if min_clearance != float('inf') else 200.0
            self.last_trajectory_score = self._score_trajectory(
                state, final_velocity, True, clearance
            )
            
            # Optional: Apply speed recommendation from trajectory score
            if self.last_trajectory_score.total_score < 0.6:
                # Low score - use recommended speed adjustment
                speed_mag = np.linalg.norm(final_velocity[:2])
                if speed_mag > 0.1:
                    speed_adjust = self.last_trajectory_score.recommended_speed / speed_mag
                    final_velocity[:2] *= np.clip(speed_adjust, 0.5, 1.0)
            
            return final_velocity
        else:
            # No avoidance needed - score the straight path
            clearance = max(10.0, min_clearance) if min_clearance != float('inf') else 200.0
            self.last_trajectory_score = self._score_trajectory(
                state, desired_vel, False, clearance
            )
            
            # No avoidance needed - just apply small boundary correction if needed
            boundary_mag = np.linalg.norm(boundary_force)
            if boundary_mag > 0.1:
                # Small nudge toward safe area
                corrected_vel = desired_vel.copy()
                corrected_vel[:2] += boundary_force[:2] * 0.3  # Gentle correction
                return corrected_vel
            else:
                # Free and clear - follow desired path exactly
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
