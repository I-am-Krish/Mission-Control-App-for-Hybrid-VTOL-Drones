"""
Mini-Map Widget - Top-down 2D view of mission area
Shows real-time drone position, obstacles, NFZ, and waypoints
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore import Qt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.patches as mpatches
from typing import List

from src.core.uav_state import Position, MissionData


class MiniMapWidget(QWidget):
    """Small top-down 2D view widget"""
    
    def __init__(self, mission_data: MissionData, parent=None):
        super().__init__(parent)
        self.mission_data = mission_data
        self.uav_position = Position(0, 0, 0)
        self.path_history = []
        
        self.init_ui()
        self.setup_plot()
    
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Create small matplotlib figure with dark background
        self.figure = Figure(figsize=(2.8, 2.8), facecolor='#0a0a2e', dpi=80)
        self.canvas = FigureCanvas(self.figure)
        
        # Create 2D axes
        self.ax = self.figure.add_subplot(111)
        self.ax.set_facecolor('#0a0a2e')
        
        layout.addWidget(self.canvas)
        
        # Set minimum height for sidebar display (width will be controlled by parent)
        self.setMinimumHeight(240)
        self.setMaximumHeight(280)
        
        # Add semi-transparent background with border
        self.setStyleSheet("""
            QWidget {
                background-color: rgba(10, 10, 46, 220);
                border: 2px solid #00ffff;
                border-radius: 5px;
            }
        """)
    
    def setup_plot(self):
        """Setup 2D overhead view"""
        self.ax.clear()
        self.ax.set_aspect('equal')
        
        # Remove axes for cleaner look
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        
        # Set limits based on mission area
        self.ax.set_xlim([-500, 6000])
        self.ax.set_ylim([-500, 1500])
        
        # Add border
        for spine in self.ax.spines.values():
            spine.set_edgecolor('#00ffff')
            spine.set_linewidth(2)
        
        # Adjust subplot to maximize space
        self.figure.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)
        
        # Plot home position (green)
        home = self.mission_data.home
        self.home_marker, = self.ax.plot(
            [home.x], [home.y],
            'o', color='#00ff00', markersize=10,
            markeredgecolor='white', markeredgewidth=1.5
        )
        self.ax.text(
            home.x - 400, home.y, 'Home',
            color='#00ff00', fontsize=8, fontweight='bold',
            ha='right', va='center'
        )
        
        # Plot delivery point (cyan)
        delivery = self.mission_data.delivery_point
        self.delivery_marker, = self.ax.plot(
            [delivery.x], [delivery.y],
            's', color='#00ffff', markersize=10,
            markeredgecolor='white', markeredgewidth=1.5
        )
        self.ax.text(
            delivery.x + 200, delivery.y + 200, 'Delivery\nPoint',
            color='#00ffff', fontsize=7, fontweight='bold',
            ha='left', va='bottom'
        )
        
        # Plot no-fly zones (red circles with dashed outline)
        for i, nfz in enumerate(self.mission_data.no_fly_zones):
            circle = mpatches.Circle(
                (nfz.position.x, nfz.position.y),
                nfz.radius,
                fill=False,
                edgecolor='#ff0000',
                linewidth=1.5,
                linestyle='--'
            )
            self.ax.add_patch(circle)
            # Label first NFZ only to avoid clutter
            if i == 0:
                self.ax.text(
                    nfz.position.x, nfz.position.y + nfz.radius + 100,
                    'No Fly Zone',
                    color='#ff0000', fontsize=7, fontweight='bold',
                    ha='center', va='bottom'
                )
        
        # Plot obstacles (blue circles - smaller for clarity)
        for obs in self.mission_data.obstacles:
            circle = mpatches.Circle(
                (obs.position.x, obs.position.y),
                obs.radius,
                fill=True,
                facecolor='#0066ff',
                edgecolor='#0099ff',
                linewidth=1,
                alpha=0.3
            )
            self.ax.add_patch(circle)
        
        # Initialize drone marker (yellow)
        self.drone_marker, = self.ax.plot(
            [0], [0],
            'o', color='#ffff00', markersize=8,
            markeredgecolor='white', markeredgewidth=1.5,
            zorder=10
        )
        self.drone_label = self.ax.text(
            0, -200, 'Drone',
            color='#ffff00', fontsize=7, fontweight='bold',
            ha='center', va='top', zorder=10
        )
        
        # Initialize path line (cyan/white)
        self.path_line, = self.ax.plot(
            [], [],
            '-', color='#00ffff', linewidth=1.5, alpha=0.6
        )
        
        self.canvas.draw()
    
    def update_position(self, position: Position):
        """Update drone position on mini-map"""
        self.uav_position = position
        
        # Update drone marker
        self.drone_marker.set_data([position.x], [position.y])
        
        # Update label position - place it to the left to avoid overlap
        self.drone_label.set_position((position.x - 300, position.y))
    
    def update_path(self, path_history: List[Position]):
        """Update flight path on mini-map"""
        self.path_history = path_history
        
        if len(path_history) > 1:
            path_x = [p.x for p in path_history]
            path_y = [p.y for p in path_history]
            self.path_line.set_data(path_x, path_y)
    
    def refresh(self):
        """Refresh the display"""
        self.canvas.draw_idle()
