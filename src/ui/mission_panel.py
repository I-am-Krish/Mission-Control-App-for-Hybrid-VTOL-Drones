"""
Mission Planning Panel - Waypoint editor and mission configuration
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QPushButton, QListWidget,
    QLineEdit, QTextEdit, QTabWidget, QMessageBox, QScrollArea
)
from PyQt6.QtCore import Qt, pyqtSignal

from src.core.uav_state import MissionData, Position, Obstacle


class MissionPanel(QWidget):
    """Mission planning and configuration panel"""
    
    # Signals
    waypoint_added = pyqtSignal(Position)
    waypoint_removed = pyqtSignal(int)
    mission_loaded = pyqtSignal()
    map_refresh_requested = pyqtSignal()  # Signal to refresh map when obstacles/NFZ change
    
    def __init__(self, mission_data: MissionData, parent=None):
        super().__init__(parent)
        self.mission_data = mission_data
        self.init_ui()
    
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Tab widget for different mission aspects
        tabs = QTabWidget()
        
        # Waypoints tab
        waypoints_tab = self.create_waypoints_tab()
        tabs.addTab(waypoints_tab, "Waypoints")
        
        # Mission info tab
        info_tab = self.create_info_tab()
        tabs.addTab(info_tab, "Mission Info")
        
        # Obstacles tab
        obstacles_tab = self.create_obstacles_tab()
        tabs.addTab(obstacles_tab, "Obstacles")
        
        layout.addWidget(tabs)
    
    def create_waypoints_tab(self) -> QWidget:
        """Create waypoints management tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Waypoint list
        list_group = QGroupBox("Waypoints")
        list_layout = QVBoxLayout()
        
        self.waypoint_list = QListWidget()
        self.update_waypoint_list()
        list_layout.addWidget(self.waypoint_list)
        
        # Buttons
        btn_layout = QHBoxLayout()
        
        add_btn = QPushButton("Add")
        add_btn.clicked.connect(self.add_waypoint_dialog)
        btn_layout.addWidget(add_btn)
        
        remove_btn = QPushButton("Remove")
        remove_btn.clicked.connect(self.remove_selected_waypoint)
        btn_layout.addWidget(remove_btn)
        
        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self.clear_waypoints)
        btn_layout.addWidget(clear_btn)
        
        list_layout.addLayout(btn_layout)
        list_group.setLayout(list_layout)
        layout.addWidget(list_group)
        
        # Quick waypoint input
        input_group = QGroupBox("Add Waypoint")
        input_layout = QGridLayout()
        
        input_layout.addWidget(QLabel("X:"), 0, 0)
        self.wp_x_input = QLineEdit("0")
        input_layout.addWidget(self.wp_x_input, 0, 1)
        
        input_layout.addWidget(QLabel("Y:"), 1, 0)
        self.wp_y_input = QLineEdit("0")
        input_layout.addWidget(self.wp_y_input, 1, 1)
        
        input_layout.addWidget(QLabel("Z:"), 2, 0)
        self.wp_z_input = QLineEdit("120")
        input_layout.addWidget(self.wp_z_input, 2, 1)
        
        add_quick_btn = QPushButton("Add Waypoint")
        add_quick_btn.clicked.connect(self.add_waypoint_from_input)
        input_layout.addWidget(add_quick_btn, 3, 0, 1, 2)
        
        input_group.setLayout(input_layout)
        layout.addWidget(input_group)
        
        return widget
    
    def create_info_tab(self) -> QWidget:
        """Create mission information tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Mission summary
        summary_group = QGroupBox("Mission Summary")
        summary_layout = QVBoxLayout()
        
        self.mission_info_text = QTextEdit()
        self.mission_info_text.setReadOnly(True)
        self.update_mission_info()
        summary_layout.addWidget(self.mission_info_text)
        
        summary_group.setLayout(summary_layout)
        layout.addWidget(summary_group)
        
        # Estimate button
        estimate_btn = QPushButton("Calculate Energy Estimate")
        estimate_btn.clicked.connect(self.calculate_estimate)
        layout.addWidget(estimate_btn)
        
        return widget
    
    def create_obstacles_tab(self) -> QWidget:
        """Create obstacles display tab"""
        widget = QWidget()
        main_layout = QVBoxLayout(widget)
        
        # Create scroll area for all content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        
        # Content widget inside scroll area
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)
        
        # Add Custom Obstacle section
        add_obs_group = QGroupBox("Add Custom Obstacle")
        add_obs_layout = QGridLayout()
        
        add_obs_layout.addWidget(QLabel("X:"), 0, 0)
        self.obs_x_input = QLineEdit("1000")
        add_obs_layout.addWidget(self.obs_x_input, 0, 1)
        
        add_obs_layout.addWidget(QLabel("Y:"), 1, 0)
        self.obs_y_input = QLineEdit("500")
        add_obs_layout.addWidget(self.obs_y_input, 1, 1)
        
        add_obs_layout.addWidget(QLabel("Z:"), 2, 0)
        self.obs_z_input = QLineEdit("60")
        add_obs_layout.addWidget(self.obs_z_input, 2, 1)
        
        add_obs_layout.addWidget(QLabel("Radius:"), 3, 0)
        self.obs_radius_input = QLineEdit("100")
        add_obs_layout.addWidget(self.obs_radius_input, 3, 1)
        
        add_obstacle_btn = QPushButton("➕ Add Obstacle")
        add_obstacle_btn.clicked.connect(self.add_custom_obstacle)
        add_obstacle_btn.setStyleSheet("background-color: #8AFFFF; color: #6B6B6B; font-weight: bold;")
        add_obs_layout.addWidget(add_obstacle_btn, 4, 0, 1, 2)
        
        add_obs_group.setLayout(add_obs_layout)
        layout.addWidget(add_obs_group)
        
        # Obstacles list
        obs_group = QGroupBox(f"Obstacles ({len(self.mission_data.obstacles)})")
        obs_layout = QVBoxLayout()
        
        self.obstacle_list = QListWidget()
        self.update_obstacle_list()
        obs_layout.addWidget(self.obstacle_list)
        
        # Remove obstacle button
        remove_obs_btn = QPushButton("🗑️ Remove Selected")
        remove_obs_btn.clicked.connect(self.remove_selected_obstacle)
        obs_layout.addWidget(remove_obs_btn)
        
        obs_group.setLayout(obs_layout)
        layout.addWidget(obs_group)
        
        # No-Fly Zones section
        nfz_group = QGroupBox("No-Fly Zones (NFZ)")
        nfz_layout = QVBoxLayout()
        
        # Add NFZ controls
        add_nfz_section = QGroupBox("Add No-Fly Zone")
        add_nfz_layout = QGridLayout()
        
        add_nfz_layout.addWidget(QLabel("X:"), 0, 0)
        self.nfz_x_input = QLineEdit("2500")
        add_nfz_layout.addWidget(self.nfz_x_input, 0, 1)
        
        add_nfz_layout.addWidget(QLabel("Y:"), 1, 0)
        self.nfz_y_input = QLineEdit("500")
        add_nfz_layout.addWidget(self.nfz_y_input, 1, 1)
        
        add_nfz_layout.addWidget(QLabel("Z:"), 2, 0)
        self.nfz_z_input = QLineEdit("100")
        add_nfz_layout.addWidget(self.nfz_z_input, 2, 1)
        
        add_nfz_layout.addWidget(QLabel("Radius:"), 3, 0)
        self.nfz_radius_input = QLineEdit("300")
        add_nfz_layout.addWidget(self.nfz_radius_input, 3, 1)
        
        add_nfz_btn = QPushButton("➕ Add No-Fly Zone")
        add_nfz_btn.clicked.connect(self.add_custom_nfz)
        add_nfz_btn.setStyleSheet("background-color: #FF8A8A; color: white; font-weight: bold;")
        add_nfz_layout.addWidget(add_nfz_btn, 4, 0, 1, 2)
        
        add_nfz_section.setLayout(add_nfz_layout)
        nfz_layout.addWidget(add_nfz_section)
        
        # NFZ list
        self.nfz_list = QListWidget()
        self.update_nfz_list()
        nfz_layout.addWidget(self.nfz_list)
        
        # Remove NFZ button
        remove_nfz_btn = QPushButton("🗑️ Remove Selected NFZ")
        remove_nfz_btn.clicked.connect(self.remove_selected_nfz)
        nfz_layout.addWidget(remove_nfz_btn)
        
        nfz_group.setLayout(nfz_layout)
        layout.addWidget(nfz_group)
        
        # Set the content widget to the scroll area
        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)
        
        return widget
    
    def update_waypoint_list(self):
        """Update waypoint list display"""
        self.waypoint_list.clear()
        for i, wp in enumerate(self.mission_data.waypoints):
            self.waypoint_list.addItem(
                f"WP{i+1}: ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})"
            )
    
    def update_obstacle_list(self):
        """Update obstacle list display"""
        self.obstacle_list.clear()
        for i, obs in enumerate(self.mission_data.obstacles):
            self.obstacle_list.addItem(
                f"Obstacle {i+1}: Pos({obs.position.x:.0f}, {obs.position.y:.0f}, "
                f"{obs.position.z:.0f}) R={obs.radius:.0f}m"
            )
    
    def update_nfz_list(self):
        """Update no-fly zone list display"""
        self.nfz_list.clear()
        for i, nfz in enumerate(self.mission_data.no_fly_zones):
            self.nfz_list.addItem(
                f"NFZ {i+1}: Pos({nfz.position.x:.0f}, {nfz.position.y:.0f}, "
                f"{nfz.position.z:.0f}) R={nfz.radius:.0f}m"
            )
    
    def add_custom_obstacle(self):
        """Add a custom obstacle from user input"""
        try:
            x = float(self.obs_x_input.text())
            y = float(self.obs_y_input.text())
            z = float(self.obs_z_input.text())
            radius = float(self.obs_radius_input.text())
            
            # Create new obstacle
            new_obstacle = Obstacle(
                position=Position(x, y, z),
                radius=radius
            )
            
            # Add to mission data
            self.mission_data.obstacles.append(new_obstacle)
            
            # Update display
            self.update_obstacle_list()
            
            # Trigger map refresh
            self.map_refresh_requested.emit()
            
            QMessageBox.information(
                self,
                "Obstacle Added",
                f"Added obstacle at ({x}, {y}, {z}) with radius {radius}m"
            )
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                "Please enter valid numbers for obstacle parameters."
            )
    
    def remove_selected_obstacle(self):
        """Remove selected obstacle from list"""
        current_row = self.obstacle_list.currentRow()
        if current_row >= 0 and current_row < len(self.mission_data.obstacles):
            removed = self.mission_data.obstacles.pop(current_row)
            self.update_obstacle_list()
            # Trigger map refresh
            self.map_refresh_requested.emit()
            QMessageBox.information(
                self,
                "Obstacle Removed",
                f"Removed obstacle at ({removed.position.x:.0f}, {removed.position.y:.0f})"
            )
    
    def add_custom_nfz(self):
        """Add a custom no-fly zone from user input"""
        try:
            x = float(self.nfz_x_input.text())
            y = float(self.nfz_y_input.text())
            z = float(self.nfz_z_input.text())
            radius = float(self.nfz_radius_input.text())
            
            # Create new NFZ
            new_nfz = Obstacle(
                position=Position(x, y, z),
                radius=radius
            )
            
            # Add to mission data
            self.mission_data.no_fly_zones.append(new_nfz)
            
            # Update display
            self.update_nfz_list()
            
            # Trigger map refresh
            self.map_refresh_requested.emit()
            
            QMessageBox.information(
                self,
                "No-Fly Zone Added",
                f"Added NFZ at ({x}, {y}, {z}) with radius {radius}m\n⚠️ CRITICAL: UAV will never enter this zone!"
            )
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                "Please enter valid numbers for NFZ parameters."
            )
    
    def remove_selected_nfz(self):
        """Remove selected no-fly zone from list"""
        current_row = self.nfz_list.currentRow()
        if current_row >= 0 and current_row < len(self.mission_data.no_fly_zones):
            removed = self.mission_data.no_fly_zones.pop(current_row)
            self.update_nfz_list()
            # Trigger map refresh
            self.map_refresh_requested.emit()
            QMessageBox.information(
                self,
                "No-Fly Zone Removed",
                f"Removed NFZ at ({removed.position.x:.0f}, {removed.position.y:.0f})"
            )
    
    def update_mission_info(self):
        """Update mission information display"""
        info = f"""
<b>Mission Configuration</b><br>
<br>
<b>Home:</b> ({self.mission_data.home.x:.1f}, {self.mission_data.home.y:.1f}, {self.mission_data.home.z:.1f})<br>
<b>Delivery:</b> ({self.mission_data.delivery_point.x:.1f}, {self.mission_data.delivery_point.y:.1f}, {self.mission_data.delivery_point.z:.1f})<br>
<br>
<b>Total Waypoints:</b> {len(self.mission_data.waypoints)}<br>
<b>Obstacles:</b> {len(self.mission_data.obstacles)}<br>
<b>No-Fly Zones:</b> {len(self.mission_data.no_fly_zones)}<br>
<br>
<b>Direct Distance:</b> {self.mission_data.home.distance_to(self.mission_data.delivery_point):.1f} m<br>
        """
        self.mission_info_text.setHtml(info)
    
    def add_waypoint_from_input(self):
        """Add waypoint from input fields"""
        try:
            x = float(self.wp_x_input.text())
            y = float(self.wp_y_input.text())
            z = float(self.wp_z_input.text())
            
            wp = Position(x, y, z)
            self.mission_data.waypoints.append(wp)
            self.update_waypoint_list()
            self.update_mission_info()
            self.waypoint_added.emit(wp)
            self.map_refresh_requested.emit()  # Refresh map to show new waypoint
            
            # Success feedback
            QMessageBox.information(
                self,
                "Waypoint Added",
                f"✓ Added waypoint at ({x:.1f}, {y:.1f}, {z:.1f})\n\n"
                f"Total waypoints: {len(self.mission_data.waypoints)}"
            )
            
            # Clear inputs for next waypoint
            self.wp_x_input.clear()
            self.wp_y_input.clear()
            self.wp_z_input.setText("120")  # Reset to default altitude
            
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                "Please enter valid numbers for X, Y, and Z coordinates."
            )
    
    def add_waypoint_dialog(self):
        """Open dialog to add waypoint"""
        self.add_waypoint_from_input()
    
    def remove_selected_waypoint(self):
        """Remove selected waypoint"""
        current_row = self.waypoint_list.currentRow()
        if current_row >= 0:
            self.mission_data.waypoints.pop(current_row)
            self.update_waypoint_list()
            self.update_mission_info()
            self.waypoint_removed.emit(current_row)
            self.map_refresh_requested.emit()  # Refresh map to remove waypoint
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        self.mission_data.waypoints.clear()
        self.update_waypoint_list()
        self.update_mission_info()
        self.map_refresh_requested.emit()  # Refresh map to clear all waypoints
    
    def calculate_estimate(self):
        """Calculate mission energy estimate"""
        from src.core.mission_planner import estimate_mission_energy
        
        if len(self.mission_data.waypoints) < 2:
            return
        
        energy_mah, time_s = estimate_mission_energy(self.mission_data.waypoints)
        
        info = f"""
<b>Mission Estimate</b><br>
<br>
<b>Estimated Energy:</b> {energy_mah:.1f} mAh<br>
<b>Estimated Time:</b> {time_s/60:.1f} minutes<br>
<b>Battery Reserve:</b> {1500 - energy_mah:.1f} mAh remaining<br>
        """
        
        self.mission_info_text.append(info)
