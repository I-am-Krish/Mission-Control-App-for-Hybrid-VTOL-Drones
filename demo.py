"""
Demo Script - Automated Mission Demonstration
Showcases the VTOL Mission Control System capabilities
"""

import sys
import time
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.core.uav_state import UAVState, MissionData, Position, FlightMode, MissionPhase
from src.core.mission_planner import MissionPlanner, estimate_mission_energy
from src.config import UAVConfig

import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger("Demo")


def print_header(title):
    """Print formatted header"""
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60 + "\n")


def print_state(state: UAVState, step: int):
    """Print current UAV state"""
    print(f"Step {step:4d} | Mode: {state.mode.value:15s} | "
          f"Pos: ({state.position.x:7.1f}, {state.position.y:7.1f}, {state.position.z:6.1f}) | "
          f"Speed: {state.ground_speed:5.1f} m/s | "
          f"Battery: {state.battery.percentage():5.1f}%")


def main():
    """Run demo simulation"""
    
    print_header("VTOL MISSION CONTROL - DEMO SIMULATION")
    
    logger.info("Initializing mission data...")
    
    # Create mission data
    mission = MissionData()
    mission.add_default_obstacles()
    mission.add_default_no_fly_zones()
    
    # Create mission planner
    planner = MissionPlanner(mission)
    
    # Create UAV state
    state = UAVState()
    state.home_position = mission.home
    
    print_header("MISSION PARAMETERS")
    print(f"Home Position:     ({mission.home.x:.1f}, {mission.home.y:.1f}, {mission.home.z:.1f})")
    print(f"Delivery Point:    ({mission.delivery_point.x:.1f}, {mission.delivery_point.y:.1f}, {mission.delivery_point.z:.1f})")
    print(f"Direct Distance:   {mission.home.distance_to(mission.delivery_point):.1f} m")
    print(f"Obstacles:         {len(mission.obstacles)}")
    print(f"No-Fly Zones:      {len(mission.no_fly_zones)}")
    print(f"\nBattery Capacity:  {UAVConfig.BATTERY_CAPACITY} mAh")
    print(f"Cruise Speed:      {UAVConfig.CRUISE_SPEED} m/s ({UAVConfig.CRUISE_SPEED * 3.6:.0f} km/h)")
    print(f"VTOL Speed:        {UAVConfig.VTOL_SPEED} m/s")
    print(f"Takeoff Altitude:  {UAVConfig.TAKEOFF_ALTITUDE} m")
    
    input("\nPress ENTER to ARM and start mission...")
    
    # ARM the vehicle
    state.is_armed = True
    logger.info("✅ UAV ARMED")
    
    print_header("MISSION EXECUTION")
    
    # Simulation parameters
    dt = 0.1  # 10 Hz
    max_steps = 30000  # Max 50 minutes
    print_interval = 100  # Print every 10 seconds
    
    step = 0
    
    # Simulation loop
    while step < max_steps and not planner.mission_complete:
        
        # Run mission planner
        target, desired_vel, new_mode = planner.update(state, dt)
        
        # Update UAV state
        state.mode = new_mode
        
        # Simple physics integration
        vel_array = desired_vel.to_array()
        pos_array = state.position.to_array()
        
        # Update position
        new_pos = pos_array + vel_array * dt
        state.update_position(new_pos[0], new_pos[1], new_pos[2])
        
        # Update velocity
        state.update_velocity(vel_array[0], vel_array[1], vel_array[2])
        
        # Update GPS (simulated)
        state.gps_fix = 3
        state.satellite_count = 12
        
        # Print status periodically
        if step % print_interval == 0:
            print_state(state, step)
        
        # Check for key events
        if state.mode == FlightMode.CRUISE and step % print_interval == 0:
            if state.obstacle_detected:
                logger.info("  ⚠️  Obstacle detected - avoiding...")
        
        # Mode change notifications
        if step > 0 and state.mode != new_mode:
            logger.info(f"  🔄 Mode changed to: {new_mode.value}")
        
        # Delivery notification
        if state.delivered and state.mission_phase == MissionPhase.AT_TARGET:
            logger.info("  📦 Package delivered!")
        
        step += 1
    
    # Mission complete
    print_header("MISSION COMPLETE")
    
    total_time = step * dt
    total_distance = len(mission.path_history) * state.ground_speed * dt
    
    print(f"Total Time:        {total_time/60:.1f} minutes ({total_time:.1f} seconds)")
    print(f"Battery Used:      {UAVConfig.BATTERY_CAPACITY - state.battery.remaining_mah:.1f} mAh")
    print(f"Battery Remaining: {state.battery.remaining_mah:.1f} mAh ({state.battery.percentage():.1f}%)")
    print(f"Final Position:    ({state.position.x:.1f}, {state.position.y:.1f}, {state.position.z:.1f})")
    print(f"Distance to Home:  {state.distance_to_home():.1f} m")
    print(f"Package Delivered: {'✅ YES' if state.delivered else '❌ NO'}")
    print(f"Mission Status:    {'✅ SUCCESS' if planner.mission_complete else '❌ INCOMPLETE'}")
    
    print_header("PERFORMANCE SUMMARY")
    
    efficiency = (UAVConfig.BATTERY_CAPACITY - state.battery.remaining_mah) / total_time * 60
    print(f"Average Power:     {efficiency:.1f} mAh/min")
    print(f"Energy Efficiency: {total_distance / (UAVConfig.BATTERY_CAPACITY - state.battery.remaining_mah):.2f} m/mAh")
    
    print("\n✅ Demo simulation completed successfully!")
    print("\nTo see this in 3D with the full GUI, run:")
    print("  python src/main.py --sim")
    print("\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Mission aborted by user")
    except Exception as e:
        logger.error(f"\n❌ Error: {e}", exc_info=True)
