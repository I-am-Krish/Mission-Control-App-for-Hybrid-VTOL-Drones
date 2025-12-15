"""
Unit Tests for Mission Planner
"""

import pytest
import numpy as np
from src.core.mission_planner import (
    MissionPlanner, calculate_energy_optimal_path,
    estimate_mission_energy
)
from src.core.uav_state import (
    UAVState, MissionData, Position, FlightMode
)


class TestMissionPlanner:
    """Test MissionPlanner class"""
    
    def test_initialization(self):
        mission_data = MissionData()
        planner = MissionPlanner(mission_data)
        assert planner.mission_complete == False
    
    def test_flight_mode_transition_takeoff(self):
        mission_data = MissionData()
        planner = MissionPlanner(mission_data)
        
        state = UAVState()
        state.mode = FlightMode.IDLE
        state.is_armed = True
        
        new_mode = planner._update_flight_mode(state)
        assert new_mode == FlightMode.VTOL_TAKEOFF
    
    def test_flight_mode_transition_cruise(self):
        mission_data = MissionData()
        planner = MissionPlanner(mission_data)
        
        state = UAVState()
        state.mode = FlightMode.VTOL_TAKEOFF
        state.position = Position(0, 0, 120)  # At altitude
        
        new_mode = planner._update_flight_mode(state)
        assert new_mode == FlightMode.CRUISE
    
    def test_battery_update(self):
        mission_data = MissionData()
        planner = MissionPlanner(mission_data)
        
        state = UAVState()
        initial_battery = state.battery.remaining_mah
        
        # Simulate 1 second of cruise
        planner._update_battery(state, FlightMode.CRUISE, 1.0)
        
        # Should consume cruise_current/60 mAh (120/60 = 2 mAh)
        assert state.battery.remaining_mah < initial_battery
    
    def test_emergency_on_low_battery(self):
        mission_data = MissionData()
        planner = MissionPlanner(mission_data)
        
        state = UAVState()
        state.mode = FlightMode.CRUISE
        state.battery.remaining_mah = 200  # Below critical (250)
        
        new_mode = planner._update_flight_mode(state)
        assert new_mode == FlightMode.EMERGENCY_LAND


class TestEnergyEstimation:
    """Test energy estimation functions"""
    
    def test_estimate_mission_energy(self):
        waypoints = [
            Position(0, 0, 0),
            Position(0, 0, 120),      # Climb (VTOL)
            Position(1000, 0, 120),   # Cruise
            Position(1000, 0, 0)      # Descend (VTOL)
        ]
        
        energy, time = estimate_mission_energy(waypoints)
        
        assert energy > 0
        assert time > 0
    
    def test_calculate_energy_optimal_path(self):
        start = Position(0, 0, 0)
        end = Position(1000, 500, 0)
        obstacles = []
        
        path = calculate_energy_optimal_path(start, end, obstacles)
        
        assert len(path) >= 2
        assert path[0].x == start.x
        assert path[-1].x == end.x


class TestObstacleAvoidance:
    """Test obstacle avoidance logic"""
    
    def test_obstacle_detection(self):
        mission_data = MissionData()
        mission_data.add_default_obstacles()
        
        planner = MissionPlanner(mission_data)
        
        state = UAVState()
        state.position = Position(1000, 200, 120)
        state.mode = FlightMode.CRUISE
        
        # Heading toward first obstacle
        desired_vel = np.array([1.0, 0.0, 0.0]) * 28  # Cruise speed
        
        # Should detect obstacle and adjust
        new_vel = planner._apply_obstacle_avoidance(state, desired_vel)
        
        # Velocity should be modified
        assert not np.array_equal(new_vel, desired_vel)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
