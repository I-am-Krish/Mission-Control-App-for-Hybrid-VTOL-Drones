"""
Main Window - Qt6 Ground Control Station Interface
Professional mission control interface with 3D visualization
"""

import sys
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QTabWidget, QStatusBar, QMenuBar,
    QMenu, QToolBar, QPushButton, QLabel, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QIcon, QKeySequence

import logging
from datetime import datetime

from src.config import UIConfig
from src.core.uav_state import UAVState, MissionData, FlightMode
from src.core.mission_planner import MissionPlanner

from src.ui.telemetry_panel import TelemetryPanel
from src.ui.mission_panel import MissionPanel
from src.ui.map_widget import MapWidget
from src.ui.control_panel import ControlPanel
from src.ui.uav_settings_panel import UAVSettingsPanel
from src.ui.minimap_widget import MiniMapWidget


logger = logging.getLogger(__name__)


class MissionControlWindow(QMainWindow):
    """
    Main application window for VTOL Mission Control
    """
    
    # Signals
    telemetry_updated = pyqtSignal(UAVState)
    mission_status_changed = pyqtSignal(str)
    
    def __init__(
        self,
        simulation_mode: bool = True,
        ros2_enabled: bool = False,
        mavlink_port: int = 14540
    ):
        super().__init__()
        
        self.simulation_mode = simulation_mode
        self.ros2_enabled = ros2_enabled
        self.mavlink_port = mavlink_port
        
        # Initialize UAV state
        self.uav_state = UAVState()
        self.mission_data = MissionData()
        
        # Add default obstacles and no-fly zones (from MATLAB)
        self.mission_data.add_default_obstacles()
        self.mission_data.add_default_no_fly_zones()
        
        # Initialize mission planner
        self.mission_planner = MissionPlanner(self.mission_data)
        
        # UI update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_telemetry)
        
        # Simulation timer (if in sim mode)
        self.sim_timer = QTimer()
        if self.simulation_mode:
            self.sim_timer.timeout.connect(self.simulate_step)
        
        # Initialize UI
        self.init_ui()
        
        # Start updates
        self.start_updates()
        
        logger.info("Mission Control Window initialized")
    
    def init_ui(self):
        """Initialize the user interface"""
        
        # Set window properties
        self.setWindowTitle(UIConfig.WINDOW_TITLE)
        self.setGeometry(100, 100, UIConfig.WINDOW_WIDTH, UIConfig.WINDOW_HEIGHT)
        self.setMinimumSize(UIConfig.MIN_WIDTH, UIConfig.MIN_HEIGHT)
        
        # Apply Solarized Dark color scheme
        self.setStyleSheet("""
            QMainWindow {
                background-color: #002b36;
            }
            QWidget {
                background-color: #002b36;
                color: #839496;
            }
            QSplitter::handle {
                background-color: #073642;
                width: 2px;
            }
            QSplitter::handle:hover {
                background-color: #268bd2;
            }
            QGroupBox {
                border: 2px solid #268bd2;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                color: #93a1a1;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #268bd2;
            }
            QLabel {
                color: #93a1a1;
            }
            QPushButton {
                background-color: #268bd2;
                color: #fdf6e3;
                border: none;
                padding: 8px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2aa198;
            }
            QPushButton:pressed {
                background-color: #073642;
            }
            QPushButton:disabled {
                background-color: #073642;
                color: #586e75;
            }
            QLineEdit, QSpinBox, QDoubleSpinBox {
                background-color: #073642;
                color: #93a1a1;
                border: 1px solid #268bd2;
                border-radius: 3px;
                padding: 5px;
                selection-background-color: #268bd2;
            }
            QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
                border: 2px solid #2aa198;
            }
            QTextEdit, QListWidget {
                background-color: #073642;
                color: #93a1a1;
                border: 1px solid #268bd2;
                border-radius: 3px;
            }
            QTabWidget::pane {
                border: 1px solid #268bd2;
                background-color: #002b36;
            }
            QTabBar::tab {
                background-color: #073642;
                color: #93a1a1;
                border: 1px solid #268bd2;
                padding: 8px 12px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: #268bd2;
                color: #fdf6e3;
            }
            QTabBar::tab:hover {
                background-color: #2aa198;
                color: #fdf6e3;
            }
            QProgressBar {
                border: 1px solid #268bd2;
                border-radius: 3px;
                text-align: center;
                background-color: #073642;
                color: #93a1a1;
            }
            QProgressBar::chunk {
                background-color: #859900;
            }
            QScrollBar:vertical {
                background-color: #073642;
                width: 12px;
                border: 1px solid #268bd2;
            }
            QScrollBar::handle:vertical {
                background-color: #268bd2;
                border-radius: 4px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #2aa198;
            }
            QScrollBar:horizontal {
                background-color: #073642;
                height: 12px;
                border: 1px solid #268bd2;
            }
            QScrollBar::handle:horizontal {
                background-color: #268bd2;
                border-radius: 4px;
                min-width: 20px;
            }
            QScrollBar::handle:horizontal:hover {
                background-color: #2aa198;
            }
            QMenuBar {
                background-color: #002b36;
                color: #93a1a1;
            }
            QMenuBar::item:selected {
                background-color: #268bd2;
                color: #fdf6e3;
            }
            QMenu {
                background-color: #073642;
                color: #93a1a1;
                border: 1px solid #268bd2;
            }
            QMenu::item:selected {
                background-color: #268bd2;
                color: #fdf6e3;
            }
            QStatusBar {
                background-color: #073642;
                color: #93a1a1;
            }
            QToolBar {
                background-color: #073642;
                border: 1px solid #268bd2;
                spacing: 3px;
            }
        """)
        
        # Create menu bar
        self.create_menu_bar()
        
        # Create toolbar
        self.create_toolbar()
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Create main splitter (horizontal)
        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Left panel (Telemetry + Controls)
        left_panel = self.create_left_panel()
        
        # Center panel (3D Map)
        center_panel = self.create_center_panel()
        
        # Right panel (Mission Planning)
        right_panel = self.create_right_panel()
        
        # Add panels to splitter
        main_splitter.addWidget(left_panel)
        main_splitter.addWidget(center_panel)
        main_splitter.addWidget(right_panel)
        
        # Connect map widget signals to mission panel (after both are created)
        self.map_widget.waypoint_added.connect(self.mission_panel.update_waypoint_list)
        
        # Set splitter sizes (ratios) - Expanded right panel for better accessibility
        main_splitter.setSizes([270, 1930, 400])
        
        main_layout.addWidget(main_splitter)
        
        # Create status bar
        self.create_status_bar()
        
        logger.debug("UI components created")
    
    def create_menu_bar(self):
        """Create application menu bar"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu('&File')
        
        load_mission_action = QAction('&Load Mission', self)
        load_mission_action.setShortcut(QKeySequence.StandardKey.Open)
        file_menu.addAction(load_mission_action)
        
        save_mission_action = QAction('&Save Mission', self)
        save_mission_action.setShortcut(QKeySequence.StandardKey.Save)
        file_menu.addAction(save_mission_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction('E&xit', self)
        exit_action.setShortcut(QKeySequence.StandardKey.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # View menu
        view_menu = menubar.addMenu('&View')
        
        fullscreen_action = QAction('&Fullscreen', self)
        fullscreen_action.setShortcut(QKeySequence.StandardKey.FullScreen)
        fullscreen_action.triggered.connect(self.toggle_fullscreen)
        view_menu.addAction(fullscreen_action)
        
        # Mission menu
        mission_menu = menubar.addMenu('&Mission')
        
        start_mission_action = QAction('&Start Mission', self)
        start_mission_action.triggered.connect(self.start_mission)
        mission_menu.addAction(start_mission_action)
        
        abort_mission_action = QAction('&Abort Mission', self)
        abort_mission_action.triggered.connect(self.abort_mission)
        mission_menu.addAction(abort_mission_action)
        
        # Help menu
        help_menu = menubar.addMenu('&Help')
        
        about_action = QAction('&About', self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
    
    def create_toolbar(self):
        """Create main toolbar"""
        toolbar = QToolBar("Main Toolbar")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        
        # Arm/Disarm button
        self.arm_button = QPushButton("ARM")
        self.arm_button.setCheckable(True)
        self.arm_button.setStyleSheet("""
            QPushButton {
                background-color: #268bd2;
                color: #fdf6e3;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #2aa198;
            }
            QPushButton:checked {
                background-color: #859900;
                color: #fdf6e3;
            }
        """)
        self.arm_button.clicked.connect(self.toggle_arm)
        toolbar.addWidget(self.arm_button)
        
        toolbar.addSeparator()
        
        # Start Mission button
        start_btn = QPushButton("START MISSION")
        start_btn.setStyleSheet("""
            QPushButton {
                background-color: #859900;
                color: #fdf6e3;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #b58900;
            }
        """)
        start_btn.clicked.connect(self.start_mission)
        toolbar.addWidget(start_btn)
        
        # RTL button
        rtl_btn = QPushButton("RETURN TO HOME")
        rtl_btn.setStyleSheet("""
            QPushButton {
                background-color: #cb4b16;
                color: #fdf6e3;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #dc322f;
            }
        """)
        rtl_btn.clicked.connect(self.return_to_home)
        toolbar.addWidget(rtl_btn)
        
        # Emergency Land button
        emergency_btn = QPushButton("EMERGENCY LAND")
        emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc322f;
                color: #fdf6e3;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #d33682;
            }
        """)
        emergency_btn.clicked.connect(self.emergency_land)
        toolbar.addWidget(emergency_btn)
        
        toolbar.addSeparator()
        
        # Mode indicator
        toolbar.addWidget(QLabel("Mode:"))
        self.mode_label = QLabel("IDLE")
        self.mode_label.setStyleSheet("font-weight: bold; padding: 5px;")
        toolbar.addWidget(self.mode_label)
    
    def create_left_panel(self) -> QWidget:
        """Create left panel with telemetry and controls"""
        from PyQt6.QtWidgets import QScrollArea
        
        panel = QWidget()
        panel.setMinimumWidth(280)  # Reduced to give more space to map
        panel.setMaximumWidth(320)  # Cap maximum width
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        # Telemetry display in scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.telemetry_panel = TelemetryPanel(self.mission_data)
        scroll.setWidget(self.telemetry_panel)
        layout.addWidget(scroll, stretch=3)  # Give more space to telemetry
        
        # Control panel
        self.control_panel = ControlPanel()
        # Connect control panel signals
        self.control_panel.takeoff_requested.connect(self.manual_takeoff)
        self.control_panel.land_requested.connect(self.manual_land)
        layout.addWidget(self.control_panel, stretch=1)  # Controls take less space
        
        return panel
    
    def create_center_panel(self) -> QWidget:
        """Create center panel with 3D map visualization"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 3D Map widget
        self.map_widget = MapWidget(self.mission_data)
        layout.addWidget(self.map_widget)
        
        return panel
    
    def create_right_panel(self) -> QWidget:
        """Create right panel with mission planning and settings"""
        panel = QWidget()
        panel.setMaximumWidth(250)  # Cap right panel width for bigger map
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create tab widget for mission and settings
        tabs = QTabWidget()
        
        # Mission planning panel
        self.mission_panel = MissionPanel(self.mission_data)
        tabs.addTab(self.mission_panel, "Mission")
        
        # Connect mission panel signals
        self.mission_panel.map_refresh_requested.connect(self.refresh_map)
        
        # UAV Settings panel (NEW!)
        self.settings_panel = UAVSettingsPanel()
        self.settings_panel.settings_updated.connect(self.apply_custom_settings)
        tabs.addTab(self.settings_panel, "⚙️ Settings")
        
        layout.addWidget(tabs)
        
        return panel
    
    def create_status_bar(self):
        """Create application status bar"""
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        
        # Connection status
        self.connection_label = QLabel("⚫ Disconnected")
        self.statusBar.addPermanentWidget(self.connection_label)
        
        # Battery status
        self.battery_label = QLabel("🔋 100%")
        self.statusBar.addPermanentWidget(self.battery_label)
        
        # GPS status
        self.gps_label = QLabel("📡 No GPS")
        self.statusBar.addPermanentWidget(self.gps_label)
        
        self.statusBar.showMessage("Ready")
    
    def start_updates(self):
        """Start telemetry and simulation updates"""
        # Start telemetry update timer (10 Hz)
        self.update_timer.start(int(1000 / UIConfig.TELEMETRY_UPDATE_HZ))
        
        # Start simulation timer if in sim mode (100 Hz for smooth simulation)
        if self.simulation_mode:
            self.sim_timer.start(int(1000 * 0.01))  # 10ms = 100Hz
            self.connection_label.setText("🟢 Connected (SIM)")
    
    def update_telemetry(self):
        """Update telemetry displays"""
        # Get trajectory score from mission planner if available
        trajectory_score = getattr(self.mission_planner, 'last_trajectory_score', None)
        
        # Update telemetry panel with trajectory scores
        self.telemetry_panel.update_state(self.uav_state, trajectory_score)
        
        # Update 3D map
        self.map_widget.update_uav_position(self.uav_state.position)
        
        # Update mode label
        self.mode_label.setText(self.uav_state.mode.value)
        
        # Update status bar
        battery_pct = self.uav_state.battery.percentage()
        self.battery_label.setText(f"🔋 {battery_pct:.0f}%")
        
        if battery_pct < 20:
            self.battery_label.setStyleSheet("color: red; font-weight: bold;")
        elif battery_pct < 40:
            self.battery_label.setStyleSheet("color: orange; font-weight: bold;")
        else:
            self.battery_label.setStyleSheet("color: green;")
        
        # Update GPS status
        if self.uav_state.gps_fix >= 3:
            self.gps_label.setText(f"📡 GPS {self.uav_state.satellite_count} sats")
        else:
            self.gps_label.setText("📡 No GPS")
        
        # Emit signal
        self.telemetry_updated.emit(self.uav_state)
    
    def simulate_step(self):
        """Execute one simulation step (only in simulation mode)"""
        if not self.simulation_mode:
            return
        
        dt = 0.01  # 100 Hz = 0.01s
        
        # Run mission planner
        target, desired_vel, new_mode = self.mission_planner.update(self.uav_state, dt)
        
        # Update UAV state
        self.uav_state.mode = new_mode
        
        # Simple physics integration
        vel_array = desired_vel.to_array()
        pos_array = self.uav_state.position.to_array()
        
        # Update position
        new_pos = pos_array + vel_array * dt
        
        # GROUND CONSTRAINT - prevent negative altitude (going underground)
        if new_pos[2] < 0.0:
            new_pos[2] = 0.0  # Clamp to ground level
            vel_array[2] = 0.0  # Stop vertical descent
        
        self.uav_state.update_position(new_pos[0], new_pos[1], new_pos[2])
        
        # Update velocity
        self.uav_state.update_velocity(vel_array[0], vel_array[1], vel_array[2])
        
        # Update GPS (simulated)
        self.uav_state.gps_fix = 3
        self.uav_state.satellite_count = 12
        
        # Update airborne status
        self.uav_state.is_airborne = self.uav_state.position.z > 1.0
        
        # Add to path history every 10 steps to avoid too many points
        import copy
        step_count = getattr(self, '_step_count', 0)
        if step_count % 10 == 0:
            self.mission_data.path_history.append(copy.copy(self.uav_state.position))
            self.map_widget.update_path(self.mission_data.path_history)
        self._step_count = step_count + 1
        
        # Update status bar with flight info
        if new_mode == FlightMode.VTOL_TAKEOFF:
            self.statusBar.showMessage(f"Taking off - Altitude: {self.uav_state.position.z:.1f}m")
        elif new_mode == FlightMode.CRUISE:
            dist = self.uav_state.distance_to_target()
            self.statusBar.showMessage(f"Cruising - Distance to target: {dist:.0f}m | Speed: {self.uav_state.ground_speed:.1f}m/s")
        elif new_mode == FlightMode.VTOL_LAND:
            self.statusBar.showMessage(f"Landing - Altitude: {self.uav_state.position.z:.1f}m")
        elif new_mode == FlightMode.HOVER:
            self.statusBar.showMessage(f"Hovering at {self.uav_state.position.z:.1f}m")
    
    def toggle_arm(self):
        """Toggle arm/disarm"""
        self.uav_state.is_armed = self.arm_button.isChecked()
        
        if self.uav_state.is_armed:
            self.arm_button.setText("DISARM")
            self.arm_button.setStyleSheet("background-color: #00aa00;")
            logger.info("UAV ARMED")
        else:
            self.arm_button.setText("ARM")
            self.arm_button.setStyleSheet("")
            logger.info("UAV DISARMED")
    
    def start_mission(self):
        """Start autonomous mission"""
        if not self.uav_state.is_armed:
            QMessageBox.warning(self, "Warning", "Please ARM the vehicle first")
            return
        
        logger.info("Starting mission")
        
        # Ensure UAV is in takeoff mode to start the mission
        if self.uav_state.mode == FlightMode.IDLE:
            self.uav_state.mode = FlightMode.VTOL_TAKEOFF
            logger.info("Initiating VTOL takeoff")
        
        # Update mission phase
        self.uav_state.mission_phase = MissionPhase.TRANSIT_TO_TARGET
        
        self.statusBar.showMessage("Mission started - Taking off...")
        self.mission_status_changed.emit("STARTED")
    
    def abort_mission(self):
        """Abort current mission"""
        reply = QMessageBox.question(
            self,
            'Abort Mission',
            'Are you sure you want to abort the mission?',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            logger.warning("Mission aborted by user")
            self.return_to_home()
    
    def return_to_home(self):
        """Command return to home"""
        logger.info("RTL commanded")
        self.uav_state.mode = FlightMode.EMERGENCY_RTL
        self.statusBar.showMessage("Returning to home")
    
    def emergency_land(self):
        """Emergency landing"""
        logger.critical("EMERGENCY LAND commanded")
        self.uav_state.mode = FlightMode.EMERGENCY_LAND
        self.statusBar.showMessage("EMERGENCY LANDING")
    
    def manual_takeoff(self):
        """Handle manual takeoff from control panel"""
        if not self.uav_state.is_armed:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "Warning", "Please ARM the vehicle first")
            return
        logger.info("Manual takeoff commanded")
        self.uav_state.mode = FlightMode.VTOL_TAKEOFF
        self.statusBar.showMessage("Manual takeoff initiated")
    
    def manual_land(self):
        """Handle manual landing from control panel"""
        logger.info("Manual landing commanded")
        self.uav_state.mode = FlightMode.VTOL_LAND
        self.statusBar.showMessage("Manual landing initiated")
    
    def refresh_map(self):
        """Refresh the 3D map display (called when obstacles/NFZ are added/removed)"""
        logger.info("Refreshing map with updated obstacles/NFZ")
        self.map_widget.setup_3d_scene()
        self.statusBar.showMessage("Map refreshed with updated obstacles/NFZ")
    
    def apply_custom_settings(self, settings: dict):
        """Apply custom UAV settings from the settings panel"""
        logger.info(f"Applying custom UAV settings: {settings}")
        
        # Update mission data with new home and delivery positions
        from src.core.uav_state import Position
        
        # Update home position (first waypoint if exists)
        home_pos = Position(settings['home_x'], settings['home_y'], 0.0)
        
        # Update delivery position
        delivery_pos = Position(
            settings['delivery_x'],
            settings['delivery_y'],
            settings['delivery_z']
        )
        
        # Update mission data waypoints
        if len(self.mission_data.waypoints) == 0:
            self.mission_data.waypoints.append(delivery_pos)
        else:
            self.mission_data.waypoints[0] = delivery_pos
        
        # Update UAV starting position to home
        self.uav_state.position = home_pos
        
        # Store custom parameters for mission planner
        self.custom_params = settings
        
        # Recreate mission planner with new data
        self.mission_planner = MissionPlanner(self.mission_data, custom_params=settings)
        
        # Refresh mission panel and map
        self.mission_panel.update_waypoint_list()
        self.map_widget.update_plot()
        
        logger.info("Custom settings applied successfully")
        self.statusBar.showMessage("✅ Custom UAV settings applied")
    
    def toggle_fullscreen(self):
        """Toggle fullscreen mode"""
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
    
    def show_about(self):
        """Show about dialog"""
        QMessageBox.about(
            self,
            "About VTOL Mission Control",
            "<h2>VTOL Drone Mission Control System</h2>"
            "<p>Professional Ground Control Station for Hybrid VTOL UAVs</p>"
            "<p><b>Features:</b></p>"
            "<ul>"
            "<li>Autonomous mission planning</li>"
            "<li>3D environment visualization</li>"
            "<li>Real-time telemetry monitoring</li>"
            "<li>Obstacle avoidance</li>"
            "<li>Energy-optimized routing</li>"
            "</ul>"
            "<p><b>Version:</b> 1.0.0</p>"
            "<p><b>Author:</b>Krishnashis Das</p>"
            "<p>This is a demo application which is underdevelopment.</p>"
        )
    
    def closeEvent(self, event):
        """Handle application close"""
        reply = QMessageBox.question(
            self,
            'Exit',
            'Are you sure you want to exit?',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            # Stop timers
            self.update_timer.stop()
            if self.simulation_mode:
                self.sim_timer.stop()
            
            logger.info("Application closing")
            event.accept()
        else:
            event.ignore()
