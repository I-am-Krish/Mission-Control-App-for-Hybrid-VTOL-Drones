# VTOL Mission Control - Ground Control Station

A professional Ground Control Station (GCS) application for hybrid VTOL (Vertical Take-Off and Landing) UAV delivery missions, built with PyQt6 and real-time 3D visualization.

## 🚁 Overview

This project is a fully autonomous mission control system designed for hybrid VTOL drones to perform delivery missions. The drone takes off vertically, cruises to a delivery point following custom waypoints, lands to deliver the package, and returns home autonomously - all while avoiding obstacles and maintaining safe flight parameters.

## ✨ Key Features

### Autonomous Flight Control
- **Complete State Machine**: IDLE → VTOL_TAKEOFF → CRUISE → VTOL_LAND → (delivery) → VTOL_TAKEOFF → CRUISE → VTOL_LAND → IDLE
- **Sequential Waypoint Navigation**: Follow custom waypoints in order with 50m proximity detection
- **Intelligent Obstacle Avoidance**: Potential field-based system with balanced repulsion and attraction forces
- **Active Altitude Hold**: P-controller maintains cruise altitude during horizontal flight
- **Ground Constraint**: Prevents impossible negative altitude (underground flight)
- **Boundary Protection**: Keeps drone within safe operational area

### Real-time Visualization
- **3D Interactive Map**: Live flight path visualization with obstacles, waypoints, and drone position
- **Resizable Panels**: Adjustable 3-panel layout (Telemetry | Map | Settings)
- **Flight Path Tracking**: Historical path with color-coded segments
- **Click-to-Add Waypoints**: Interactive waypoint placement on the map

### Comprehensive Telemetry
- **Position Data**: Real-time X, Y, Z coordinates and distance from home
- **Velocity Monitoring**: Ground speed, air speed, and vertical speed
- **Battery Management**: Percentage, capacity, voltage, current, and estimated time remaining
- **Status Indicators**: Flight mode, mission phase, armed state, GPS fix, delivery status

### Mission Configuration
- **Flight Parameters**: Cruise speed (50 m/s), VTOL speed (10 m/s), max/cruise altitude
- **Battery Settings**: Configurable capacity (1500 mAh) and voltage (12.6V)
- **Physical Properties**: Drone weight (2.5 kg) and payload weight (0.5 kg)
- **Mission Points**: Home position and delivery point coordinates
- **Obstacles & NFZ**: Scrollable panel for adding obstacles and no-fly zones

### Emergency Controls
- **RETURN TO HOME**: Immediate return at cruise altitude with 20% speed boost
- **EMERGENCY LAND**: Straight-down descent at 1.5x VTOL speed (bypasses obstacle avoidance)

## 🏗️ System Architecture

### Core Components

#### Mission Planner (`src/core/mission_planner.py`)
The brain of the autonomous system - handles all flight logic:
- **Flight State Machine** (lines 160-221): Manages transitions between flight modes
  - VTOL Takeoff: Vertical ascent at 10 m/s until reaching cruise altitude
  - Cruise: Horizontal flight with active altitude hold
  - VTOL Land: Vertical descent; detects landing and manages delivery state
- **Waypoint Navigation** (lines 64-123): Sequential progression through custom waypoints
- **Obstacle Avoidance** (lines 267-377): Potential field algorithm with:
  - Obstacle repulsion: 8x max force, 250m influence radius
  - NFZ repulsion: 12x max force, 300m influence radius
  - Waypoint attraction: 2-8x based on distance
  - Boundary forces: 5x at map edges
- **Velocity Calculation** (lines 223-282): Mode-specific velocity commands
- **Altitude Hold** (lines 250-252): P-controller with 0.5 gain, ±2 m/s limits

#### UAV State Management (`src/core/uav_state.py`)
Data structures and state tracking:
- Position, velocity, and orientation
- Battery state and energy consumption
- Flight mode and mission phase enums
- Current waypoint index tracking
- Delivered flag for mission logic

#### Main Window (`src/ui/main_window.py`)
Application orchestration:
- **3-Panel Layout**: Resizable splitters (270px | 2200px | 250px)
- **Simulation Loop** (lines 350-380): 100 Hz Euler integration
  - Position update: `new_pos = pos + vel * dt`
  - Ground constraint (lines 357-361): Clamps altitude ≥ 0
- **Signal/Slot Connections**: Real-time UI updates
- **Timer Management**: 100 Hz simulation, 10 Hz telemetry, 5 Hz map refresh

#### UI Panels
- **Control Panel** (`control_panel.py`): TAKEOFF and LAND buttons
- **Telemetry Panel** (`telemetry_panel.py`): Auto-refreshing status display
- **Mission Panel** (`mission_panel.py`): Scrollable obstacle/waypoint manager with click-to-add
- **Settings Panel** (`uav_settings_panel.py`): Flight parameters and battery config
- **Map Widget** (`map_widget.py`): 3D Matplotlib visualization (16x12 inches)

## 🛠️ Installation

### Prerequisites
- Python 3.10 or higher
- Git

### Setup Steps

1. **Clone the repository**:
```bash
git clone https://github.com/I-am-Krish/Mission-Control-App-for-Hybrid-VTOL-Drones.git
cd Mission-Control-App-for-Hybrid-VTOL-Drones
```

2. **Create virtual environment**:
```bash
python -m venv venv
```

3. **Activate virtual environment**:
   - **Windows**: `venv\Scripts\activate`
   - **Linux/macOS**: `source venv/bin/activate`

4. **Install dependencies**:
```bash
pip install -r requirements.txt
```

### Required Packages
```
PyQt6>=6.7.0         # GUI framework
matplotlib>=3.8.0    # 3D visualization
numpy                # Numerical computations
PyYAML               # Configuration files
```

## 🚀 Quick Start

### Launch the Application

**Windows**:
```bash
.\launch.bat
```

**Linux/macOS**:
```bash
chmod +x launch.sh
./launch.sh
```

**Or manually**:
```bash
python demo.py
```

### Running Your First Mission

1. **Configure the Mission**:
   - Home: 0, 0, 0 (default)
   - Delivery: 5000, 800, 120
   - Click the map to add intermediate waypoints (optional)

2. **Set Flight Parameters**:
   - Cruise Speed: 50 m/s
   - Cruise Altitude: 50 m
   - Battery: 1500 mAh, 12.6V

3. **Add Obstacles** (optional):
   - Scroll down in Mission Panel to "Obstacles" section
   - Add cylindrical obstacles with position and radius
   - Add No-Fly Zones for restricted airspace

4. **Execute Mission**:
   - Click **DISARM** → Changes to "ARM" (drone is now armed)
   - Click **START MISSION**
   - Watch the autonomous flight:
     - VTOL Takeoff → Climb to 50m
     - Cruise → Follow waypoints to delivery point
     - VTOL Land → Land and deliver package
     - VTOL Takeoff → Return to air
     - Cruise → Navigate back to home
     - VTOL Land → Land at home base
     - **IDLE** → Mission complete, disarmed

5. **Monitor Progress**:
   - Left panel: Real-time telemetry (position, velocity, battery)
   - Center panel: 3D flight path visualization
   - Status bar: Mode, phase, and delivery status

## 🎯 Flight Logic Details

### VTOL Takeoff → Cruise → Land Sequence

**Location**: [mission_planner.py](src/core/mission_planner.py)

#### VTOL Takeoff (lines 168-172)
```python
# Vertical ascent at VTOL_SPEED (10 m/s)
# Transitions when within 5m of TAKEOFF_ALTITUDE (50m default)
```

#### Cruise Mode (lines 174-189, 240-262)
```python
# Horizontal navigation toward target waypoint
# Active altitude hold: vertical_correction = (target_alt - current_alt) * 0.5
# Clipped to ±2 m/s to prevent oscillation
# Transitions to VTOL_LAND when within 20m of target
```

#### VTOL Land (lines 191-209)
```python
# Vertical descent at -VTOL_SPEED (-10 m/s)
# On landing (altitude < 2m):
#   - At delivery: Mark delivered, takeoff to return home
#   - At home (delivered=True): Disarm, stay in IDLE
```

### Waypoint Navigation (lines 64-123)
- **Outbound**: Follow waypoints sequentially → delivery point
- **Return**: Fly directly home (no waypoints)
- **Advancement**: 50m proximity threshold triggers next waypoint
- **Tracking**: `current_waypoint_index` in UAVState

### Obstacle Avoidance (lines 267-377)
Balanced potential field algorithm:
- **Repulsion from obstacles**: 8x max (was 20x), 250m influence
- **Repulsion from NFZ**: 12x max (was 50x), 300m influence
- **Attraction to waypoint**: 2-8x based on distance (stronger when far)
- **Vertical forces**: Zeroed to maintain altitude stability
- **Boundary forces**: 5x push at map edges (X: 200-5800m, Y: 0-800m)

### Ground Constraint (main_window.py lines 357-361)
```python
# Prevents negative altitude (underground flight)
if new_pos[2] < 0.0:
    new_pos[2] = 0.0  # Clamp to ground
    vel_array[2] = 0.0  # Stop descent
```

## 🔧 Recent Development Updates

### December 2025 - Major Fixes & Improvements

#### ✅ Fixed Landing Loop Issue
**Problem**: Drone repeatedly took off and landed at home base after delivery  
**Solution**: Added `state.is_armed = False` when landing at home with `delivered=True`  
**Location**: [mission_planner.py](src/core/mission_planner.py#L198)

#### ✅ Fixed Underground Flight Bug
**Problem**: Drone altitude went negative (-112m shown in telemetry)  
**Solution**: Added ground constraint in physics integration  
**Location**: [main_window.py](src/ui/main_window.py#L357-L361)

#### ✅ Implemented Sequential Waypoint Navigation
**Problem**: Drone ignored waypoints and flew directly to delivery  
**Solution**: Added `current_waypoint_index` tracking with 50m proximity threshold  
**Location**: [mission_planner.py](src/core/mission_planner.py#L64-L123)

#### ✅ Balanced Obstacle Avoidance
**Problem**: Drone avoided obstacles too aggressively and went off map  
**Solution**: 
- Reduced repulsion forces (8x obstacles, 12x NFZ)
- Increased waypoint attraction (2-8x based on distance)
- Zeroed vertical forces for stable altitude
- Added boundary constraints

#### ✅ Active Altitude Hold
**Problem**: Altitude gradually decreased during cruise  
**Solution**: Added P-controller in CRUISE and EMERGENCY_RTL modes  
**Location**: [mission_planner.py](src/core/mission_planner.py#L250-L252)

#### ✅ Fixed Emergency Landing
**Problem**: Emergency landing tried to avoid obstacles  
**Solution**: Removed EMERGENCY_LAND from obstacle avoidance modes  
**Location**: [mission_planner.py](src/core/mission_planner.py#L137)

#### ✅ UI Cleanup
**Removed**:
- Target Altitude slider (redundant with cruise altitude)
- Flight Mode dropdown (automatic state machine)

**Enhanced**:
- Made obstacles panel scrollable
- Increased map size to 16x12 inches
- Added waypoint visualization (yellow stars)
- Improved panel layout with splitters

## 📊 Performance Specifications

- **Simulation Rate**: 100 Hz (dt = 0.01s)
- **Telemetry Update**: 10 Hz
- **Map Refresh**: 5 Hz + signal-triggered updates
- **Integration Method**: Euler (position += velocity * dt)
- **Cruise Speed**: 50 m/s (configurable)
- **VTOL Speed**: 10 m/s (configurable)
- **Cruise Altitude**: 50 m (configurable)
- **Waypoint Threshold**: 50 m
- **Landing Detection**: Altitude < 2 m
- **Obstacle Influence**: 250 m
- **NFZ Influence**: 300 m

## 🧪 Testing

Run the test suite:
```bash
python -m pytest tests/
```

**Test Coverage**:
- `test_uav_state.py`: State management, battery calculations
- `test_mission_planner.py`: Flight mode transitions, obstacle avoidance

## 📁 Project Structure

```
Mission-Control-App-for-Hybrid-VTOL-Drones/
├── src/
│   ├── core/
│   │   ├── mission_planner.py    # Main autonomous flight controller
│   │   └── uav_state.py          # State data structures
│   ├── ui/
│   │   ├── main_window.py        # Application window & simulation loop
│   │   ├── control_panel.py      # Flight control buttons
│   │   ├── telemetry_panel.py    # Status display
│   │   ├── mission_panel.py      # Mission config & waypoints
│   │   ├── uav_settings_panel.py # Flight parameters
│   │   └── map_widget.py         # 3D visualization
│   ├── config.py                 # Configuration constants
│   └── main.py                   # Application entry point
├── tests/                        # Unit tests
├── config/                       # Configuration files
├── demo.py                       # Quick launch script
├── launch.bat                    # Windows launcher
├── launch.sh                     # Linux/macOS launcher
├── requirements.txt              # Python dependencies
├── .gitignore                    # Git ignore patterns
└── README.md                     # This file
```

## 🤝 Contributing

Contributions are welcome! To contribute:

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make your changes with clear commit messages
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **PyQt6**: Cross-platform GUI framework
- **Matplotlib**: 3D visualization engine
- Inspired by professional GCS systems like QGroundControl and Mission Planner

## 📧 Support

For questions, issues, or feature requests, please open an issue on GitHub.

---

**Project Status**: ✅ Active Development  
**Version**: 1.0.0  
**Last Updated**: December 2025  
**Maintainer**: [I-am-Krish](https://github.com/I-am-Krish)