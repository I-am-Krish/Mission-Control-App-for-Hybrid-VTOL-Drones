"""
VTOL Mission Control - Main Application Entry Point
Qt6-based Ground Control Station (GCS)
"""

import sys
import argparse
from pathlib import Path

# Add parent directory to path so imports work
sys.path.insert(0, str(Path(__file__).parent.parent))

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt, QTimer
from src.ui.main_window import MissionControlWindow
from src.config import UIConfig

import logging
from datetime import datetime


def setup_logging():
    """Configure application logging"""
    log_dir = Path(__file__).parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = log_dir / f"mission_control_{timestamp}.log"
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(sys.stdout)
        ]
    )
    
    return logging.getLogger("MissionControl")


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="VTOL Drone Mission Control - Ground Control Station"
    )
    
    parser.add_argument(
        '--sim',
        action='store_true',
        help='Start in simulation mode (PX4 SITL)'
    )
    
    parser.add_argument(
        '--ros2',
        action='store_true',
        help='Enable ROS 2 bridge'
    )
    
    parser.add_argument(
        '--port',
        type=int,
        default=14540,
        help='MAVLink connection port (default: 14540)'
    )
    
    parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug mode with verbose logging'
    )
    
    parser.add_argument(
        '--fullscreen',
        action='store_true',
        help='Start in fullscreen mode'
    )
    
    return parser.parse_args()


def main():
    """Main application entry point"""
    
    # Parse command line arguments
    args = parse_arguments()
    
    # Setup logging
    logger = setup_logging()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    logger.info("=" * 60)
    logger.info("VTOL Mission Control System - Starting")
    logger.info("=" * 60)
    logger.info(f"Simulation Mode: {args.sim}")
    logger.info(f"ROS 2 Enabled: {args.ros2}")
    logger.info(f"MAVLink Port: {args.port}")
    
    # Create Qt Application
    app = QApplication(sys.argv)
    app.setApplicationName(UIConfig.WINDOW_TITLE)
    app.setOrganizationName("VTOL Mission Control")
    
    # Set application style
    app.setStyle('Fusion')
    
    # Apply dark theme if configured
    if UIConfig.THEME == "dark":
        apply_dark_theme(app)
    
    # Create main window
    try:
        window = MissionControlWindow(
            simulation_mode=args.sim,
            ros2_enabled=args.ros2,
            mavlink_port=args.port
        )
        
        # Show window
        if args.fullscreen:
            window.showFullScreen()
        else:
            window.show()
        
        logger.info("Main window created successfully")
        
        # Start application event loop
        exit_code = app.exec()
        
        logger.info("Application shutting down")
        logger.info(f"Exit code: {exit_code}")
        
        sys.exit(exit_code)
        
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


def apply_dark_theme(app: QApplication):
    """Apply dark theme to the application"""
    from PyQt6.QtGui import QPalette, QColor
    from PyQt6.QtCore import Qt
    
    dark_palette = QPalette()
    
    # Define colors
    dark_color = QColor(45, 45, 45)
    disabled_color = QColor(127, 127, 127)
    
    dark_palette.setColor(QPalette.ColorRole.Window, dark_color)
    dark_palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
    dark_palette.setColor(QPalette.ColorRole.AlternateBase, dark_color)
    dark_palette.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.Button, dark_color)
    dark_palette.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    dark_palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
    dark_palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, disabled_color)
    dark_palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, disabled_color)
    
    app.setPalette(dark_palette)
    
    # Apply stylesheet for additional styling
    stylesheet = """
    QToolTip {
        color: #ffffff;
        background-color: #2a82da;
        border: 1px solid white;
    }
    
    QPushButton {
        background-color: #3a3a3a;
        border: 1px solid #5a5a5a;
        padding: 5px;
        border-radius: 3px;
    }
    
    QPushButton:hover {
        background-color: #4a4a4a;
    }
    
    QPushButton:pressed {
        background-color: #2a2a2a;
    }
    
    QLineEdit, QTextEdit, QPlainTextEdit {
        background-color: #1e1e1e;
        border: 1px solid #5a5a5a;
        padding: 3px;
        border-radius: 2px;
    }
    
    QTabWidget::pane {
        border: 1px solid #5a5a5a;
    }
    
    QTabBar::tab {
        background-color: #3a3a3a;
        border: 1px solid #5a5a5a;
        padding: 5px 10px;
    }
    
    QTabBar::tab:selected {
        background-color: #2a82da;
    }
    """
    
    app.setStyleSheet(stylesheet)


if __name__ == "__main__":
    main()
