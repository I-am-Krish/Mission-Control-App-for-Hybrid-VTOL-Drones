"""
3D Map Widget - Visualization of UAV, obstacles, and flight path
Using matplotlib for 3D plotting (can be upgraded to PyOpenGL later)
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QInputDialog, QMessageBox, QLabel
from PyQt6.QtCore import QTimer, pyqtSignal

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from src.core.uav_state import Position, MissionData
from typing import List


class MapWidget(QWidget):
    """3D map visualization widget"""
    
    waypoint_added = pyqtSignal(Position)  # Signal emitted when waypoint is added
    
    def __init__(self, mission_data: MissionData, parent=None):
        super().__init__(parent)
        
        self.mission_data = mission_data
        self.uav_position = Position(0, 0, 0)
        self.path_history = []
        self.add_waypoint_mode = False  # Toggle for adding waypoints
        self.waypoint_markers = []  # Store waypoint markers
        
        self.init_ui()
        self.setup_3d_scene()
        
        # Auto-refresh timer
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.update_plot)
        self.refresh_timer.start(200)  # 5 Hz refresh
    
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Add toolbar for waypoint controls
        toolbar = QHBoxLayout()
        
        self.add_waypoint_btn = QPushButton("➕ Click Map to Add Waypoint")
        self.add_waypoint_btn.setCheckable(True)
        self.add_waypoint_btn.clicked.connect(self.toggle_waypoint_mode)
        toolbar.addWidget(self.add_waypoint_btn)
        
        clear_waypoints_btn = QPushButton("�️ Clear Custom Waypoints")
        clear_waypoints_btn.clicked.connect(self.clear_custom_waypoints)
        toolbar.addWidget(clear_waypoints_btn)
        
        self.waypoint_info_label = QLabel("")
        toolbar.addWidget(self.waypoint_info_label)
        
        toolbar.addStretch()
        layout.addLayout(toolbar)
        
        # Create matplotlib figure
        self.figure = Figure(figsize=(16, 12), facecolor='#2d2d2d')  # Extra large figure size
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumSize(400, 300)  # Allow resizing but set minimum
        
        # Create 3D axes
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#1e1e1e')
        
        # Connect matplotlib click event
        self.canvas.mpl_connect('button_press_event', self.on_map_click)
        
        layout.addWidget(self.canvas)
    
    def setup_3d_scene(self):
        """Setup 3D scene with obstacles and environment"""
                # Clear existing plot
        self.ax.clear()
                # Set labels
        self.ax.set_xlabel('X (m)', color='white')
        self.ax.set_ylabel('Y (m)', color='white')
        self.ax.set_zlabel('Altitude (m)', color='white')
        
        # Set title
        self.ax.set_title('Mission Airspace View', color='white', fontsize=14, fontweight='bold')
        
        # Set axis limits (from MATLAB code)
        self.ax.set_xlim([-200, 6000])
        self.ax.set_ylim([-200, 1000])
        self.ax.set_zlim([0, 200])
        
        # Customize tick colors
        self.ax.tick_params(colors='white')
        
        # Grid
        self.ax.grid(True, alpha=0.3)
        
        # Plot home position
        home = self.mission_data.home
        self.ax.scatter(
            [home.x], [home.y], [home.z],
            c='green', marker='s', s=200,
            label='Home', edgecolors='white', linewidths=2
        )
        
        # Plot delivery point
        delivery = self.mission_data.delivery_point
        self.ax.scatter(
            [delivery.x], [delivery.y], [delivery.z],
            c='red', marker='p', s=200,
            label='Delivery Point', edgecolors='white', linewidths=2
        )
        
        # Plot waypoints
        if len(self.mission_data.waypoints) > 0:
            wp_x = [wp.x for wp in self.mission_data.waypoints]
            wp_y = [wp.y for wp in self.mission_data.waypoints]
            wp_z = [wp.z for wp in self.mission_data.waypoints]
            
            # Plot waypoint markers
            self.ax.scatter(
                wp_x, wp_y, wp_z,
                c='yellow', marker='*', s=300,
                label='Waypoints', edgecolors='red', linewidths=2
            )
            
            # Draw connecting lines between waypoints
            if len(self.mission_data.waypoints) > 1:
                self.ax.plot(wp_x, wp_y, wp_z, 'y--', linewidth=2, alpha=0.7)
        
        # Plot obstacles (spheres)
        for obs in self.mission_data.obstacles:
            self.plot_sphere(
                obs.position.x, obs.position.y, obs.position.z,
                obs.radius, color='blue', alpha=0.3
            )
        
        # Plot no-fly zones (red spheres)
        for nfz in self.mission_data.no_fly_zones:
            self.plot_sphere(
                nfz.position.x, nfz.position.y, nfz.position.z,
                nfz.radius, color='red', alpha=0.15
            )
        
        # Initialize UAV marker (will be updated)
        self.uav_marker = self.ax.scatter(
            [0], [0], [0],
            c='lime', marker='o', s=150,
            label='UAV', edgecolors='yellow', linewidths=2
        )
        
        # Initialize path line (white for visibility on dark background)
        self.path_line, = self.ax.plot(
            [], [], [],
            'w-', linewidth=2, label='Flight Path', alpha=0.8
        )
        
        # Legend
        self.ax.legend(loc='upper left', facecolor='#2d2d2d', edgecolor='white', labelcolor='white')
        
        # Set view angle
        self.ax.view_init(elev=25, azim=45)
        
        self.canvas.draw()
    
    def plot_sphere(self, x: float, y: float, z: float, radius: float, color='blue', alpha=0.3):
        """Plot a sphere representing obstacle or no-fly zone"""
        
        # Create sphere mesh
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 15)
        
        sphere_x = radius * np.outer(np.cos(u), np.sin(v)) + x
        sphere_y = radius * np.outer(np.sin(u), np.sin(v)) + y
        sphere_z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + z
        
        self.ax.plot_surface(
            sphere_x, sphere_y, sphere_z,
            color=color, alpha=alpha, linewidth=0, antialiased=False
        )
    
    def update_uav_position(self, position: Position):
        """Update UAV position marker"""
        self.uav_position = position
    
    def update_path(self, path_history: List[Position]):
        """Update flight path display"""
        self.path_history = path_history
    
    def update_plot(self):
        """Refresh the 3D plot with latest data"""
        
        # Update UAV marker position
        if self.uav_marker:
            self.uav_marker._offsets3d = (
                [self.uav_position.x],
                [self.uav_position.y],
                [self.uav_position.z]
            )
        
        # Update path line
        if len(self.path_history) > 1 and self.path_line:
            path_x = [p.x for p in self.path_history]
            path_y = [p.y for p in self.path_history]
            path_z = [p.z for p in self.path_history]
            
            self.path_line.set_data(path_x, path_y)
            self.path_line.set_3d_properties(path_z)
        
        # Redraw canvas
        self.canvas.draw_idle()
    
    def reset_view(self):
        """Reset camera view to default"""
        self.ax.view_init(elev=25, azim=45)
        self.canvas.draw()
    
    def clear_path(self):
        """Clear flight path history"""
        self.path_history.clear()
        self.path_line.set_data([], [])
        self.path_line.set_3d_properties([])
        self.canvas.draw()
    
    def toggle_waypoint_mode(self, checked: bool):
        """Toggle waypoint adding mode"""
        self.add_waypoint_mode = checked
        if checked:
            self.add_waypoint_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
            self.waypoint_info_label.setText("Click on map to add waypoint")
        else:
            self.add_waypoint_btn.setStyleSheet("")
            self.waypoint_info_label.setText("")
    
    def on_map_click(self, event):
        """Handle mouse click on 3D map"""
        if not self.add_waypoint_mode or event.inaxes != self.ax:
            return
        
        # Get approximate 3D coordinates from 2D click
        if event.xdata is None or event.ydata is None:
            return
        
        x = event.xdata
        y = event.ydata
        
        # Ask user for altitude
        altitude, ok = QInputDialog.getDouble(
            self,
            "Set Waypoint Altitude",
            f"Waypoint at X={x:.0f}, Y={y:.0f}\\nEnter altitude (m):",
            value=120.0,
            min=10.0,
            max=200.0,
            decimals=1
        )
        
        if ok:
            new_waypoint = Position(x, y, altitude)
            
            # Add to mission data
            self.mission_data.waypoints.append(new_waypoint)
            
            # Draw marker on map
            marker = self.ax.scatter(
                [x], [y], [altitude],
                c='yellow', marker='*', s=300,
                edgecolors='red', linewidths=2,
                label=f'Custom WP {len(self.waypoint_markers)+1}'
            )
            self.waypoint_markers.append(marker)
            
            # Draw connecting lines between waypoints
            if len(self.mission_data.waypoints) > 1:
                wp_x = [wp.x for wp in self.mission_data.waypoints]
                wp_y = [wp.y for wp in self.mission_data.waypoints]
                wp_z = [wp.z for wp in self.mission_data.waypoints]
                self.ax.plot(wp_x, wp_y, wp_z, 'y--', linewidth=2, alpha=0.7)
            
            self.canvas.draw()
            
            # Emit signal
            self.waypoint_added.emit(new_waypoint)
            
            # Update info label
            self.waypoint_info_label.setText(
                f"Added waypoint at ({x:.0f}, {y:.0f}, {altitude:.0f})"
            )
    
    def clear_custom_waypoints(self):
        """Clear all custom waypoints"""
        if len(self.waypoint_markers) == 0:
            QMessageBox.information(self, "No Waypoints", "No custom waypoints to clear.")
            return
        
        reply = QMessageBox.question(
            self,
            "Clear Waypoints",
            f"Clear all {len(self.waypoint_markers)} custom waypoints?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            # Remove markers from plot
            for marker in self.waypoint_markers:
                marker.remove()
            self.waypoint_markers.clear()
            
            # Clear from mission data (keep only delivery point if it was first)
            if len(self.mission_data.waypoints) > 1:
                self.mission_data.waypoints = [self.mission_data.waypoints[0]]  # Keep delivery point
            
            # Redraw
            self.setup_3d_scene()
            self.canvas.draw()
            
            self.waypoint_info_label.setText("Custom waypoints cleared")
    
    def clear_path(self):
        """Clear flight path history"""
        self.path_history.clear()
        self.path_line.set_data([], [])
        self.path_line.set_3d_properties([])
        self.canvas.draw()
    
    def toggle_waypoint_mode(self, checked: bool):
        """Toggle waypoint adding mode"""
        self.add_waypoint_mode = checked
        if checked:
            self.add_waypoint_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
            self.waypoint_info_label.setText("Click on map to add waypoint")
        else:
            self.add_waypoint_btn.setStyleSheet("")
            self.waypoint_info_label.setText("")
    
    def on_map_click(self, event):
        """Handle mouse click on 3D map"""
        if not self.add_waypoint_mode or event.inaxes != self.ax:
            return
        
        # Get approximate 3D coordinates from 2D click
        # For simplicity, use click X/Y and ask user for altitude
        if event.xdata is None or event.ydata is None:
            return
        
        x = event.xdata
        y = event.ydata
        
        # Ask user for altitude
        altitude, ok = QInputDialog.getDouble(
            self,
            "Set Waypoint Altitude",
            f"Waypoint at X={x:.0f}, Y={y:.0f}\nEnter altitude (m):",
            value=120.0,
            min=10.0,
            max=200.0,
            decimals=1
        )
        
        if ok:
            new_waypoint = Position(x, y, altitude)
            
            # Add to mission data
            self.mission_data.waypoints.append(new_waypoint)
            
            # Draw marker on map
            marker = self.ax.scatter(
                [x], [y], [altitude],
                c='yellow', marker='*', s=300,
                edgecolors='red', linewidths=2,
                label=f'Custom WP {len(self.waypoint_markers)+1}'
            )
            self.waypoint_markers.append(marker)
            
            # Draw connecting lines between waypoints
            if len(self.mission_data.waypoints) > 1:
                wp_x = [wp.x for wp in self.mission_data.waypoints]
                wp_y = [wp.y for wp in self.mission_data.waypoints]
                wp_z = [wp.z for wp in self.mission_data.waypoints]
                self.ax.plot(wp_x, wp_y, wp_z, 'y--', linewidth=2, alpha=0.7)
            
            self.canvas.draw()
            
            # Emit signal
            self.waypoint_added.emit(new_waypoint)
            
            # Update info label
            self.waypoint_info_label.setText(
                f"Added waypoint at ({x:.0f}, {y:.0f}, {altitude:.0f})"
            )
    
    def clear_custom_waypoints(self):
        """Clear all custom waypoints"""
        if len(self.waypoint_markers) == 0:
            QMessageBox.information(self, "No Waypoints", "No custom waypoints to clear.")
            return
        
        reply = QMessageBox.question(
            self,
            "Clear Waypoints",
            f"Clear all {len(self.waypoint_markers)} custom waypoints?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            # Remove markers from plot
            for marker in self.waypoint_markers:
                marker.remove()
            self.waypoint_markers.clear()
            
            # Clear from mission data (keep only delivery point if it was first)
            if len(self.mission_data.waypoints) > 1:
                self.mission_data.waypoints = [self.mission_data.waypoints[0]]  # Keep delivery point
            
            # Redraw
            self.setup_3d_scene()
            self.canvas.draw()
            
            self.waypoint_info_label.setText("Custom waypoints cleared")
