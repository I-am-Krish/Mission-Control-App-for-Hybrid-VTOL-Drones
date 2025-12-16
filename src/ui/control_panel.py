"""
Control Panel - Manual flight controls and mode selection
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QPushButton
)
from PyQt6.QtCore import Qt, pyqtSignal


class ControlPanel(QWidget):
    """Manual control panel for UAV"""
    
    # Signals
    takeoff_requested = pyqtSignal()
    land_requested = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Flight controls group
        controls_group = QGroupBox("Flight Controls")
        controls_layout = QVBoxLayout()
        
        # Takeoff button
        self.takeoff_btn = QPushButton("TAKEOFF")
        self.takeoff_btn.setStyleSheet("""
            QPushButton {
                background-color: #197278;
                color: #EDDDD4;
                font-weight: bold;
                padding: 15px;
                font-size: 12pt;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #1A8289;
            }
        """)
        self.takeoff_btn.clicked.connect(self.takeoff_requested.emit)
        controls_layout.addWidget(self.takeoff_btn)
        
        # Land button
        self.land_btn = QPushButton("LAND")
        self.land_btn.setStyleSheet("""
            QPushButton {
                background-color: #772E25;
                color: #EDDDD4;
                font-weight: bold;
                padding: 15px;
                font-size: 12pt;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #8A3529;
            }
        """)
        self.land_btn.clicked.connect(self.land_requested.emit)
        controls_layout.addWidget(self.land_btn)
        
        controls_group.setLayout(controls_layout)
        layout.addWidget(controls_group)
        
        layout.addStretch()
