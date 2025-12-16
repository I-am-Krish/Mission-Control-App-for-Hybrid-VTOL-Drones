"""
UAV Settings Panel - Configure custom drone parameters
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QGroupBox,
    QLabel, QLineEdit, QPushButton, QDoubleSpinBox, QMessageBox,
    QScrollArea
)
from PyQt6.QtCore import pyqtSignal, Qt


class UAVSettingsPanel(QWidget):
    """Panel for configuring custom UAV parameters"""
    
    # Signal emitted when settings are updated
    settings_updated = pyqtSignal(dict)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Default values from MATLAB code
        self.settings = {
            'battery_capacity_mah': 1500,
            'battery_voltage': 12.6,
            'cruise_speed': 28.0,
            'vtol_speed': 3.0,
            'max_altitude': 200.0,
            'cruise_altitude': 120.0,
            'drone_weight': 2.5,  # kg
            'payload_weight': 0.5,  # kg
            'home_x': 0.0,
            'home_y': 0.0,
            'delivery_x': 5000.0,
            'delivery_y': 800.0,
            'delivery_z': 120.0
        }
        
        self.init_ui()
    
    def init_ui(self):
        """Initialize UI components"""
        # Main layout for the panel
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Create scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        
        # Create content widget
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(10)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Battery Settings
        battery_group = self.create_battery_group()
        layout.addWidget(battery_group)
        
        # Flight Performance
        flight_group = self.create_flight_group()
        layout.addWidget(flight_group)
        
        # Drone Physical Properties
        drone_group = self.create_drone_group()
        layout.addWidget(drone_group)
        
        # Mission Points
        mission_group = self.create_mission_points_group()
        layout.addWidget(mission_group)
        
        # Apply button
        apply_btn = QPushButton("✅ Apply Settings")
        apply_btn.setStyleSheet("""
            QPushButton {
                background-color: #197278;
                color: #EDDDD4;
                font-size: 14pt;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #1A8289;
            }
        """)
        apply_btn.clicked.connect(self.apply_settings)
        layout.addWidget(apply_btn)
        
        # Reset button
        reset_btn = QPushButton("🔄 Reset to Defaults")
        reset_btn.setStyleSheet("""
            QPushButton {
                background-color: #C44536;
                color: #EDDDD4;
                font-size: 14pt;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #D44F3C;
            }
        """)
        reset_btn.clicked.connect(self.reset_to_defaults)
        layout.addWidget(reset_btn)
        
        layout.addStretch()
        
        # Set content widget to scroll area
        scroll.setWidget(content)
        
        # Add scroll area to main layout
        main_layout.addWidget(scroll)
    
    def create_battery_group(self) -> QGroupBox:
        """Create battery settings group"""
        group = QGroupBox("🔋 Battery Configuration")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Capacity (mAh):"), 0, 0)
        self.battery_capacity = QDoubleSpinBox()
        self.battery_capacity.setRange(500, 10000)
        self.battery_capacity.setValue(self.settings['battery_capacity_mah'])
        self.battery_capacity.setSingleStep(100)
        layout.addWidget(self.battery_capacity, 0, 1)
        
        layout.addWidget(QLabel("Voltage (V):"), 1, 0)
        self.battery_voltage = QDoubleSpinBox()
        self.battery_voltage.setRange(3.0, 48.0)
        self.battery_voltage.setValue(self.settings['battery_voltage'])
        self.battery_voltage.setSingleStep(0.1)
        layout.addWidget(self.battery_voltage, 1, 1)
        
        group.setLayout(layout)
        return group
    
    def create_flight_group(self) -> QGroupBox:
        """Create flight performance group"""
        group = QGroupBox("✈️ Flight Performance")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Cruise Speed (m/s):"), 0, 0)
        self.cruise_speed = QDoubleSpinBox()
        self.cruise_speed.setRange(5.0, 50.0)
        self.cruise_speed.setValue(self.settings['cruise_speed'])
        self.cruise_speed.setSingleStep(1.0)
        layout.addWidget(self.cruise_speed, 0, 1)
        
        layout.addWidget(QLabel("VTOL Speed (m/s):"), 1, 0)
        self.vtol_speed = QDoubleSpinBox()
        self.vtol_speed.setRange(1.0, 10.0)
        self.vtol_speed.setValue(self.settings['vtol_speed'])
        self.vtol_speed.setSingleStep(0.5)
        layout.addWidget(self.vtol_speed, 1, 1)
        
        layout.addWidget(QLabel("Max Altitude (m):"), 2, 0)
        self.max_altitude = QDoubleSpinBox()
        self.max_altitude.setRange(50, 500)
        self.max_altitude.setValue(self.settings['max_altitude'])
        self.max_altitude.setSingleStep(10)
        layout.addWidget(self.max_altitude, 2, 1)
        
        layout.addWidget(QLabel("Cruise Altitude (m):"), 3, 0)
        self.cruise_altitude = QDoubleSpinBox()
        self.cruise_altitude.setRange(50, 300)
        self.cruise_altitude.setValue(self.settings['cruise_altitude'])
        self.cruise_altitude.setSingleStep(10)
        layout.addWidget(self.cruise_altitude, 3, 1)
        
        group.setLayout(layout)
        return group
    

    
    def create_drone_group(self) -> QGroupBox:
        """Create drone physical properties group"""
        group = QGroupBox("🚁 Drone Physical Properties")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Drone Weight (kg):"), 0, 0)
        self.drone_weight = QDoubleSpinBox()
        self.drone_weight.setRange(0.5, 25.0)
        self.drone_weight.setValue(self.settings['drone_weight'])
        self.drone_weight.setSingleStep(0.1)
        layout.addWidget(self.drone_weight, 0, 1)
        
        layout.addWidget(QLabel("Payload Weight (kg):"), 1, 0)
        self.payload_weight = QDoubleSpinBox()
        self.payload_weight.setRange(0.0, 10.0)
        self.payload_weight.setValue(self.settings['payload_weight'])
        self.payload_weight.setSingleStep(0.1)
        layout.addWidget(self.payload_weight, 1, 1)
        
        group.setLayout(layout)
        return group
    
    def create_mission_points_group(self) -> QGroupBox:
        """Create mission points configuration group"""
        group = QGroupBox("📍 Mission Points")
        layout = QGridLayout()
        
        # Home position
        layout.addWidget(QLabel("Home X (m):"), 0, 0)
        self.home_x = QDoubleSpinBox()
        self.home_x.setRange(-1000, 10000)
        self.home_x.setValue(self.settings['home_x'])
        self.home_x.setSingleStep(100)
        layout.addWidget(self.home_x, 0, 1)
        
        layout.addWidget(QLabel("Home Y (m):"), 1, 0)
        self.home_y = QDoubleSpinBox()
        self.home_y.setRange(-1000, 10000)
        self.home_y.setValue(self.settings['home_y'])
        self.home_y.setSingleStep(100)
        layout.addWidget(self.home_y, 1, 1)
        
        # Delivery position
        layout.addWidget(QLabel("Delivery X (m):"), 2, 0)
        self.delivery_x = QDoubleSpinBox()
        self.delivery_x.setRange(-1000, 10000)
        self.delivery_x.setValue(self.settings['delivery_x'])
        self.delivery_x.setSingleStep(100)
        layout.addWidget(self.delivery_x, 2, 1)
        
        layout.addWidget(QLabel("Delivery Y (m):"), 3, 0)
        self.delivery_y = QDoubleSpinBox()
        self.delivery_y.setRange(-1000, 10000)
        self.delivery_y.setValue(self.settings['delivery_y'])
        self.delivery_y.setSingleStep(100)
        layout.addWidget(self.delivery_y, 3, 1)
        
        layout.addWidget(QLabel("Delivery Z (m):"), 4, 0)
        self.delivery_z = QDoubleSpinBox()
        self.delivery_z.setRange(10, 300)
        self.delivery_z.setValue(self.settings['delivery_z'])
        self.delivery_z.setSingleStep(10)
        layout.addWidget(self.delivery_z, 4, 1)
        
        group.setLayout(layout)
        return group
    
    def apply_settings(self):
        """Apply current settings and emit signal"""
        # Update settings dictionary
        self.settings['battery_capacity_mah'] = self.battery_capacity.value()
        self.settings['battery_voltage'] = self.battery_voltage.value()
        self.settings['cruise_speed'] = self.cruise_speed.value()
        self.settings['vtol_speed'] = self.vtol_speed.value()
        self.settings['max_altitude'] = self.max_altitude.value()
        self.settings['cruise_altitude'] = self.cruise_altitude.value()
        self.settings['drone_weight'] = self.drone_weight.value()
        self.settings['payload_weight'] = self.payload_weight.value()
        self.settings['home_x'] = self.home_x.value()
        self.settings['home_y'] = self.home_y.value()
        self.settings['delivery_x'] = self.delivery_x.value()
        self.settings['delivery_y'] = self.delivery_y.value()
        self.settings['delivery_z'] = self.delivery_z.value()
        
        # Emit signal with new settings
        self.settings_updated.emit(self.settings)
        
        QMessageBox.information(
            self,
            "Settings Applied",
            "✅ UAV settings have been updated!\n\n"
            "The simulation will now use your custom parameters for:\n"
            "• Battery calculations\n"
            "• Flight speeds\n"
            "• Mission points"
        )
    
    def reset_to_defaults(self):
        """Reset all settings to default values"""
        self.battery_capacity.setValue(1500)
        self.battery_voltage.setValue(12.6)
        self.cruise_speed.setValue(28.0)
        self.vtol_speed.setValue(3.0)
        self.max_altitude.setValue(200.0)
        self.cruise_altitude.setValue(120.0)
        self.drone_weight.setValue(2.5)
        self.payload_weight.setValue(0.5)
        self.home_x.setValue(0.0)
        self.home_y.setValue(0.0)
        self.delivery_x.setValue(5000.0)
        self.delivery_y.setValue(800.0)
        self.delivery_z.setValue(120.0)
        
        QMessageBox.information(
            self,
            "Settings Reset",
            "🔄 All settings have been reset to default values from MATLAB code."
        )
