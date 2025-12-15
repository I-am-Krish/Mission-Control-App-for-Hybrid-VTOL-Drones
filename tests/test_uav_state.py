"""
Unit Tests for UAV State Model
"""

import pytest
import numpy as np
from src.core.uav_state import (
    Position, Velocity, UAVState, Obstacle, 
    NoFlyZone, FlightMode, MissionData
)


class TestPosition:
    """Test Position class"""
    
    def test_creation(self):
        pos = Position(100, 200, 50)
        assert pos.x == 100
        assert pos.y == 200
        assert pos.z == 50
    
    def test_to_array(self):
        pos = Position(10, 20, 30)
        arr = pos.to_array()
        assert np.array_equal(arr, np.array([10, 20, 30]))
    
    def test_from_array(self):
        arr = np.array([15, 25, 35])
        pos = Position.from_array(arr)
        assert pos.x == 15
        assert pos.y == 25
        assert pos.z == 35
    
    def test_distance_to(self):
        pos1 = Position(0, 0, 0)
        pos2 = Position(3, 4, 0)
        assert pos1.distance_to(pos2) == 5.0
    
    def test_horizontal_distance(self):
        pos1 = Position(0, 0, 100)
        pos2 = Position(3, 4, 200)
        assert pos1.horizontal_distance_to(pos2) == 5.0


class TestVelocity:
    """Test Velocity class"""
    
    def test_magnitude(self):
        vel = Velocity(3, 4, 0)
        assert vel.magnitude() == 5.0
    
    def test_normalize(self):
        vel = Velocity(10, 0, 0)
        norm = vel.normalize()
        assert np.allclose(norm, [1, 0, 0])


class TestObstacle:
    """Test Obstacle class"""
    
    def test_distance_to_point(self):
        obs = Obstacle(Position(100, 100, 100), radius=50)
        point = Position(100, 100, 200)
        # Distance: 100 - radius: 50 = 50
        assert obs.distance_to_point(point) == 50
    
    def test_is_point_inside(self):
        obs = Obstacle(Position(0, 0, 0), radius=10)
        assert obs.is_point_inside(Position(5, 0, 0)) == True
        assert obs.is_point_inside(Position(15, 0, 0)) == False


class TestNoFlyZone:
    """Test NoFlyZone class"""
    
    def test_violation(self):
        nfz = NoFlyZone(
            Position(100, 100, 50),
            radius=50,
            altitude_min=0,
            altitude_max=200
        )
        
        # Inside zone
        assert nfz.is_violation(Position(100, 120, 50)) == True
        
        # Outside horizontal range
        assert nfz.is_violation(Position(200, 100, 50)) == False
        
        # Outside altitude range
        assert nfz.is_violation(Position(100, 100, 250)) == False


class TestUAVState:
    """Test UAVState class"""
    
    def test_initial_state(self):
        state = UAVState()
        assert state.mode == FlightMode.IDLE
        assert state.is_armed == False
        assert state.delivered == False
    
    def test_update_position(self):
        state = UAVState()
        state.update_position(100, 200, 50)
        assert state.position.x == 100
        assert state.position.y == 200
        assert state.position.z == 50
    
    def test_distance_to_home(self):
        state = UAVState()
        state.home_position = Position(0, 0, 0)
        state.update_position(3, 4, 0)
        assert state.distance_to_home() == 5.0
    
    def test_to_dict(self):
        state = UAVState()
        data = state.to_dict()
        assert "position" in data
        assert "velocity" in data
        assert "battery_percent" in data


class TestMissionData:
    """Test MissionData class"""
    
    def test_add_default_obstacles(self):
        mission = MissionData()
        mission.add_default_obstacles()
        assert len(mission.obstacles) == 6
    
    def test_add_default_nfz(self):
        mission = MissionData()
        mission.add_default_no_fly_zones()
        assert len(mission.no_fly_zones) == 2
    
    def test_check_collision(self):
        mission = MissionData()
        mission.add_default_obstacles()
        
        # Check first obstacle position (1200, 200, 120, r=140)
        collision = mission.check_obstacle_collision(Position(1200, 200, 120))
        assert collision is not None
    
    def test_check_nfz_violation(self):
        mission = MissionData()
        mission.add_default_no_fly_zones()
        
        # Check first NFZ (2600, 250, 120, r=300)
        violation = mission.check_nfz_violation(Position(2600, 250, 120))
        assert violation is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
