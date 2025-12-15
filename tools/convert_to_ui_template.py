"""
Helper script to create Qt Designer .ui file templates
Run this to generate .ui files you can then edit in Qt Designer
"""

from PyQt6.QtWidgets import QApplication, QWidget, QGroupBox, QVBoxLayout, QGridLayout, QLabel, QProgressBar
from PyQt6 import uic
import sys

def create_telemetry_template():
    """Create a template .ui file for telemetry panel"""
    app = QApplication(sys.argv)
    
    widget = QWidget()
    widget.setObjectName("TelemetryPanel")
    widget.resize(300, 600)
    
    layout = QVBoxLayout(widget)
    
    # Title
    title = QLabel("TELEMETRY")
    title.setObjectName("titleLabel")
    layout.addWidget(title)
    
    # Position Group
    pos_group = QGroupBox("Position (m)")
    pos_group.setObjectName("positionGroup")
    pos_layout = QGridLayout()
    pos_layout.addWidget(QLabel("X:"), 0, 0)
    pos_x = QLabel("0.0")
    pos_x.setObjectName("pos_x_label")
    pos_layout.addWidget(pos_x, 0, 1)
    pos_group.setLayout(pos_layout)
    layout.addWidget(pos_group)
    
    # Velocity Group
    vel_group = QGroupBox("Velocity (m/s)")
    vel_group.setObjectName("velocityGroup")
    layout.addWidget(vel_group)
    
    # Battery Group
    bat_group = QGroupBox("Battery")
    bat_group.setObjectName("batteryGroup")
    bat_layout = QVBoxLayout()
    battery_bar = QProgressBar()
    battery_bar.setObjectName("battery_bar")
    bat_layout.addWidget(battery_bar)
    bat_group.setLayout(bat_layout)
    layout.addWidget(bat_group)
    
    # Status Group
    status_group = QGroupBox("Flight Status")
    status_group.setObjectName("statusGroup")
    layout.addWidget(status_group)
    
    # Save to .ui file
    # Note: PyQt6 doesn't directly support saving to .ui from code
    # You'll need to recreate this in Qt Designer
    print("Template widget created!")
    print("Now open Qt Designer and recreate this structure:")
    print("1. Main Widget (QWidget)")
    print("2. Vertical Layout")
    print("3. Add QGroupBox widgets for each section")
    print("4. Use QGridLayout inside groups for label/value pairs")
    print("5. Save as 'telemetry_panel.ui'")
    
    widget.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    create_telemetry_template()
