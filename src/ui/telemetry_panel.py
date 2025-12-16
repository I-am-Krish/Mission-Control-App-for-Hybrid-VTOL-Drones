"""
Telemetry Panel - Real-time UAV status display
Shows position, velocity, battery, and flight mode
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QProgressBar
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6 import uic
import os

from src.core.uav_state import UAVState


class TelemetryPanel(QWidget):
    """Real-time telemetry display panel"""
    
    def __init__(self, mission_data=None, parent=None):
        super().__init__(parent)
        self.mission_data = mission_data
        
        # Initialize trajectory score labels (used by both UI loading paths)
        self.score_safety_label = None
        self.score_energy_label = None
        self.score_progress_label = None
        self.score_total_bar = None
        
        # Try to load UI from .ui file first
        ui_path = os.path.join(os.path.dirname(__file__), '..', '..', 'ui', 'telemetry_panel.ui')
        if os.path.exists(ui_path):
            uic.loadUi(ui_path, self)
            print(f"✓ Loaded UI from {ui_path}")
            # Add trajectory score group to existing UI
            self._add_trajectory_score_group()
        else:
            # Fallback to manual UI creation
            print(f"⚠ .ui file not found, using manual UI")
            self.init_ui()
    
    def _add_trajectory_score_group(self):
        """Add trajectory score group to existing UI layout"""
        # Find the main layout
        main_layout = self.layout()
        if main_layout is None:
            return
        
        # Create and add the trajectory score group
        score_group = self.create_trajectory_score_group()
        
        # Insert before the stretch at the end
        # Find the stretch item and insert before it
        count = main_layout.count()
        if count > 0:
            # Remove stretch if it exists
            last_item = main_layout.itemAt(count - 1)
            if last_item and last_item.spacerItem():
                main_layout.removeItem(last_item)
            
            # Add trajectory group
            main_layout.addWidget(score_group)
            
            # Re-add stretch
            main_layout.addStretch()
    
    def init_ui(self):
        """Initialize UI components (fallback if .ui file not found)"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(5)  # Reduced base spacing since we'll add custom spacing
        
        # Title - BIGGER
        title = QLabel("TELEMETRY")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(16)  # Increased from 12
        title.setFont(title_font)
        layout.addWidget(title)
        layout.addSpacing(10)  # Space after title
        
        # Position group
        pos_group = self.create_position_group()
        layout.addWidget(pos_group)
        layout.addSpacing(15)  # Extra space between Position and Velocity
        
        # Velocity group
        vel_group = self.create_velocity_group()
        layout.addWidget(vel_group)
        layout.addSpacing(15)  # Consistent spacing
        
        # Battery group
        battery_group = self.create_battery_group()
        layout.addWidget(battery_group)
        layout.addSpacing(15)  # Extra space between Battery and Flight Status
        
        # Flight status group
        status_group = self.create_status_group()
        layout.addWidget(status_group)
        layout.addSpacing(15)  # Extra space between Flight Status and Trajectory
        
        # Trajectory score group
        score_group = self.create_trajectory_score_group()
        layout.addWidget(score_group)
        
        # Note: Mini-map is added in __init__ after UI setup
        
        layout.addStretch()
    
    def create_position_group(self) -> QGroupBox:
        """Create position display group"""
        group = QGroupBox("Position (m)")
        group_font = QFont()
        group_font.setPointSize(12)
        group_font.setBold(True)
        group.setFont(group_font)
        
        grid = QGridLayout()
        grid.setSpacing(12)
        grid.setContentsMargins(10, 15, 10, 15)
        grid.setVerticalSpacing(12)
        grid.setHorizontalSpacing(15)
        
        # Create bigger fonts for labels and values
        label_font = QFont()
        label_font.setPointSize(11)
        value_font = QFont()
        value_font.setPointSize(13)
        value_font.setBold(True)
        
        # X position
        x_label = QLabel("X:")
        x_label.setFont(label_font)
        grid.addWidget(x_label, 0, 0)
        self.pos_x_label = QLabel("0.0")
        self.pos_x_label.setFont(value_font)
        self.pos_x_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.pos_x_label, 0, 1)
        
        # Y position
        y_label = QLabel("Y:")
        y_label.setFont(label_font)
        grid.addWidget(y_label, 1, 0)
        self.pos_y_label = QLabel("0.0")
        self.pos_y_label.setFont(value_font)
        self.pos_y_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.pos_y_label, 1, 1)
        
        # Z position (altitude)
        z_label = QLabel("Alt:")
        z_label.setFont(label_font)
        grid.addWidget(z_label, 2, 0)
        self.pos_z_label = QLabel("0.0")
        self.pos_z_label.setFont(value_font)
        self.pos_z_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.pos_z_label, 2, 1)
        
        # Distance to home
        dist_label = QLabel("Dist Home:")
        dist_label.setFont(label_font)
        grid.addWidget(dist_label, 3, 0)
        self.dist_home_label = QLabel("0.0")
        self.dist_home_label.setFont(value_font)
        self.dist_home_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.dist_home_label, 3, 1)
        
        group.setLayout(grid)
        return group
    
    def create_velocity_group(self) -> QGroupBox:
        """Create velocity display group"""
        group = QGroupBox("Velocity (m/s)")
        group_font = QFont()
        group_font.setPointSize(12)
        group_font.setBold(True)
        group.setFont(group_font)
        
        grid = QGridLayout()
        grid.setSpacing(12)
        grid.setContentsMargins(10, 15, 10, 15)
        grid.setVerticalSpacing(12)
        grid.setHorizontalSpacing(15)
        
        label_font = QFont()
        label_font.setPointSize(11)
        value_font = QFont()
        value_font.setPointSize(13)
        value_font.setBold(True)
        
        # Ground speed
        gnd_label = QLabel("Ground:")
        gnd_label.setFont(label_font)
        grid.addWidget(gnd_label, 0, 0)
        self.vel_ground_label = QLabel("0.0")
        self.vel_ground_label.setFont(value_font)
        self.vel_ground_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.vel_ground_label, 0, 1)
        
        # Airspeed
        air_label = QLabel("Air:")
        air_label.setFont(label_font)
        grid.addWidget(air_label, 1, 0)
        self.vel_air_label = QLabel("0.0")
        self.vel_air_label.setFont(value_font)
        self.vel_air_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.vel_air_label, 1, 1)
        
        # Vertical speed
        vert_label = QLabel("Vertical:")
        vert_label.setFont(label_font)
        grid.addWidget(vert_label, 2, 0)
        self.vel_vert_label = QLabel("0.0")
        self.vel_vert_label.setFont(value_font)
        self.vel_vert_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.vel_vert_label, 2, 1)
        
        group.setLayout(grid)
        return group
    
    def create_battery_group(self) -> QGroupBox:
        """Create battery status group"""
        group = QGroupBox("Battery")
        group_font = QFont()
        group_font.setPointSize(12)
        group_font.setBold(True)
        group.setFont(group_font)
        
        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(10, 15, 10, 15)
        
        # Battery percentage bar - BIGGER
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(100)
        self.battery_bar.setTextVisible(True)
        self.battery_bar.setFormat("%p%")
        self.battery_bar.setMinimumHeight(35)  # Make taller
        bar_font = QFont()
        bar_font.setPointSize(13)
        bar_font.setBold(True)
        self.battery_bar.setFont(bar_font)
        layout.addWidget(self.battery_bar)
        
        # Battery details grid
        grid = QGridLayout()
        grid.setSpacing(12)
        grid.setVerticalSpacing(12)
        grid.setHorizontalSpacing(15)
        
        label_font = QFont()
        label_font.setPointSize(11)
        value_font = QFont()
        value_font.setPointSize(13)
        value_font.setBold(True)
        
        rem_label = QLabel("Remaining:")
        rem_label.setFont(label_font)
        grid.addWidget(rem_label, 0, 0)
        self.battery_mah_label = QLabel("1500 mAh")
        self.battery_mah_label.setFont(value_font)
        self.battery_mah_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.battery_mah_label, 0, 1)
        
        volt_label = QLabel("Voltage:")
        volt_label.setFont(label_font)
        grid.addWidget(volt_label, 1, 0)
        self.battery_volt_label = QLabel("12.6 V")
        self.battery_volt_label.setFont(value_font)
        self.battery_volt_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.battery_volt_label, 1, 1)
        
        curr_label = QLabel("Current:")
        curr_label.setFont(label_font)
        grid.addWidget(curr_label, 2, 0)
        self.battery_current_label = QLabel("0.0 A")
        self.battery_current_label.setFont(value_font)
        self.battery_current_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.battery_current_label, 2, 1)
        
        time_label = QLabel("Time Left:")
        time_label.setFont(label_font)
        grid.addWidget(time_label, 3, 0)
        self.battery_time_label = QLabel("∞")
        self.battery_time_label.setFont(value_font)
        self.battery_time_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.battery_time_label, 3, 1)
        
        layout.addLayout(grid)
        group.setLayout(layout)
        return group
    
    def create_status_group(self) -> QGroupBox:
        """Create flight status group"""
        group = QGroupBox("Flight Status")
        group_font = QFont()
        group_font.setPointSize(12)
        group_font.setBold(True)
        group.setFont(group_font)
        
        grid = QGridLayout()
        grid.setSpacing(12)
        grid.setContentsMargins(10, 15, 10, 15)
        grid.setVerticalSpacing(12)
        grid.setHorizontalSpacing(15)
        
        label_font = QFont()
        label_font.setPointSize(11)
        value_font = QFont()
        value_font.setPointSize(13)
        value_font.setBold(True)
        
        mode_lbl = QLabel("Mode:")
        mode_lbl.setFont(label_font)
        grid.addWidget(mode_lbl, 0, 0)
        self.mode_label = QLabel("IDLE")
        self.mode_label.setFont(value_font)
        self.mode_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.mode_label, 0, 1)
        
        phase_lbl = QLabel("Phase:")
        phase_lbl.setFont(label_font)
        grid.addWidget(phase_lbl, 1, 0)
        self.phase_label = QLabel("PREFLIGHT")
        self.phase_label.setFont(value_font)
        self.phase_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.phase_label, 1, 1)
        
        armed_lbl = QLabel("Armed:")
        armed_lbl.setFont(label_font)
        grid.addWidget(armed_lbl, 2, 0)
        self.armed_label = QLabel("❌")
        self.armed_label.setFont(value_font)
        self.armed_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.armed_label, 2, 1)
        
        gps_lbl = QLabel("GPS:")
        gps_lbl.setFont(label_font)
        grid.addWidget(gps_lbl, 3, 0)
        self.gps_label = QLabel("No Fix")
        self.gps_label.setFont(value_font)
        self.gps_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.gps_label, 3, 1)
        
        deliv_lbl = QLabel("Delivered:")
        deliv_lbl.setFont(label_font)
        grid.addWidget(deliv_lbl, 4, 0)
        self.delivered_label = QLabel("❌")
        self.delivered_label.setFont(value_font)
        self.delivered_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.delivered_label, 4, 1)
        
        group.setLayout(grid)
        return group
    
    def create_trajectory_score_group(self) -> QGroupBox:
        """Create trajectory scoring display group"""
        group = QGroupBox("Trajectory Score")
        group_font = QFont()
        group_font.setPointSize(12)
        group_font.setBold(True)
        group.setFont(group_font)
        
        grid = QGridLayout()
        grid.setSpacing(12)  # Increased from 8 for more vertical spacing
        grid.setContentsMargins(10, 15, 10, 15)  # Add margins inside the group
        grid.setVerticalSpacing(15)  # Extra vertical spacing between rows
        grid.setHorizontalSpacing(20)  # More horizontal spacing between columns
        
        label_font = QFont()
        label_font.setPointSize(11)  # Slightly larger
        value_font = QFont()
        value_font.setPointSize(11)  # Slightly larger
        value_font.setBold(True)
        
        # Safety Score
        safety_lbl = QLabel("Safety:")
        safety_lbl.setFont(label_font)
        grid.addWidget(safety_lbl, 0, 0)
        self.score_safety_label = QLabel("--")
        self.score_safety_label.setFont(value_font)
        self.score_safety_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.score_safety_label.setMinimumWidth(60)  # Ensure enough width
        grid.addWidget(self.score_safety_label, 0, 1)
        
        # Energy Score
        energy_lbl = QLabel("Energy:")
        energy_lbl.setFont(label_font)
        grid.addWidget(energy_lbl, 1, 0)
        self.score_energy_label = QLabel("--")
        self.score_energy_label.setFont(value_font)
        self.score_energy_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.score_energy_label.setMinimumWidth(60)
        grid.addWidget(self.score_energy_label, 1, 1)
        
        # Progress Score
        progress_lbl = QLabel("Progress:")
        progress_lbl.setFont(label_font)
        grid.addWidget(progress_lbl, 2, 0)
        self.score_progress_label = QLabel("--")
        self.score_progress_label.setFont(value_font)
        self.score_progress_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.score_progress_label.setMinimumWidth(60)
        grid.addWidget(self.score_progress_label, 2, 1)
        
        # Total Score with progress bar
        total_lbl = QLabel("Total:")
        total_lbl.setFont(label_font)
        grid.addWidget(total_lbl, 3, 0)
        
        # Total score bar
        self.score_total_bar = QProgressBar()
        self.score_total_bar.setRange(0, 100)
        self.score_total_bar.setValue(0)
        self.score_total_bar.setTextVisible(True)
        self.score_total_bar.setFormat("%p%")
        self.score_total_bar.setMinimumHeight(30)  # Increased from 25
        self.score_total_bar.setMinimumWidth(100)  # Ensure enough width
        score_bar_font = QFont()
        score_bar_font.setPointSize(10)
        score_bar_font.setBold(True)
        self.score_total_bar.setFont(score_bar_font)
        grid.addWidget(self.score_total_bar, 3, 1)
        
        # Set minimum height for the group box
        group.setMinimumHeight(180)
        
        group.setLayout(grid)
        return group
    
    def update_state(self, state: UAVState, trajectory_score=None):
        """Update all telemetry displays with new state"""
        
        # Position
        self.pos_x_label.setText(f"{state.position.x:.1f}")
        self.pos_y_label.setText(f"{state.position.y:.1f}")
        self.pos_z_label.setText(f"{state.position.z:.1f}")
        self.dist_home_label.setText(f"{state.distance_to_home():.1f}")
        
        # Velocity
        self.vel_ground_label.setText(f"{state.ground_speed:.1f}")
        self.vel_air_label.setText(f"{state.airspeed:.1f}")
        self.vel_vert_label.setText(f"{state.velocity.vz:.1f}")
        
        # Battery
        battery_pct = state.battery.percentage()
        self.battery_bar.setValue(int(battery_pct))
        self.battery_mah_label.setText(f"{state.battery.remaining_mah:.0f} mAh")
        self.battery_volt_label.setText(f"{state.battery.voltage:.1f} V")
        self.battery_current_label.setText(f"{state.battery.current:.1f} A")
        
        # Battery color coding
        if battery_pct < 20:
            self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: red; }")
        elif battery_pct < 40:
            self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: orange; }")
        else:
            self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: green; }")
        
        # Time remaining
        if state.mode.value in ["CRUISE"]:
            time_left = state.battery.time_remaining(120)  # cruise current
        elif state.mode.value in ["VTOL_TAKEOFF", "VTOL_LAND"]:
            time_left = state.battery.time_remaining(220)  # VTOL current
        else:
            time_left = float('inf')
        
        if time_left == float('inf'):
            self.battery_time_label.setText("∞")
        else:
            self.battery_time_label.setText(f"{time_left:.1f} min")
        
        # Flight status
        self.mode_label.setText(state.mode.value)
        self.phase_label.setText(state.mission_phase.value)
        self.armed_label.setText("✅" if state.is_armed else "❌")
        
        # GPS status
        if state.gps_fix >= 3:
            self.gps_label.setText(f"3D Fix ({state.satellite_count} sats)")
            self.gps_label.setStyleSheet("color: green;")
        else:
            self.gps_label.setText("No Fix")
            self.gps_label.setStyleSheet("color: red;")
        
        # Delivery status
        self.delivered_label.setText("✅" if state.delivered else "❌")
        
        # Trajectory Scores (if available and labels exist)
        if trajectory_score and self.score_safety_label is not None:
            # Safety score
            safety_pct = int(trajectory_score.safety_score * 100)
            self.score_safety_label.setText(f"{safety_pct}%")
            if safety_pct >= 70:
                self.score_safety_label.setStyleSheet("color: green;")
            elif safety_pct >= 40:
                self.score_safety_label.setStyleSheet("color: orange;")
            else:
                self.score_safety_label.setStyleSheet("color: red;")
            
            # Energy score
            energy_pct = int(trajectory_score.energy_score * 100)
            self.score_energy_label.setText(f"{energy_pct}%")
            if energy_pct >= 70:
                self.score_energy_label.setStyleSheet("color: green;")
            elif energy_pct >= 50:
                self.score_energy_label.setStyleSheet("color: orange;")
            else:
                self.score_energy_label.setStyleSheet("color: red;")
            
            # Progress score
            progress_pct = int(trajectory_score.progress_score * 100)
            self.score_progress_label.setText(f"{progress_pct}%")
            if progress_pct >= 70:
                self.score_progress_label.setStyleSheet("color: green;")
            elif progress_pct >= 40:
                self.score_progress_label.setStyleSheet("color: orange;")
            else:
                self.score_progress_label.setStyleSheet("color: red;")
            
            # Total score bar
            total_pct = int(trajectory_score.total_score * 100)
            self.score_total_bar.setValue(total_pct)
            if total_pct >= 70:
                self.score_total_bar.setStyleSheet("QProgressBar::chunk { background-color: green; }")
            elif total_pct >= 50:
                self.score_total_bar.setStyleSheet("QProgressBar::chunk { background-color: orange; }")
            else:
                self.score_total_bar.setStyleSheet("QProgressBar::chunk { background-color: red; }")
        elif self.score_safety_label is not None:
            # No trajectory score available but labels exist
            self.score_safety_label.setText("--")
            self.score_safety_label.setStyleSheet("")
            self.score_energy_label.setText("--")
            self.score_energy_label.setStyleSheet("")
            self.score_progress_label.setText("--")
            self.score_progress_label.setStyleSheet("")
            self.score_total_bar.setValue(0)
            self.score_total_bar.setStyleSheet("")
