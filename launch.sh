#!/bin/bash
# ========================================
# VTOL Mission Control - Quick Launch
# Linux/Mac Shell Script
# ========================================

echo ""
echo "===================================="
echo " VTOL Mission Control System"
echo " Ground Control Station"
echo "===================================="
echo ""

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3 is not installed"
    echo "Please install Python 3.10 or higher"
    exit 1
fi

echo "[OK] Python found: $(python3 --version)"
echo ""

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    echo "[OK] Virtual environment created"
else
    echo "[OK] Virtual environment found"
fi

echo ""
echo "Activating virtual environment..."
source venv/bin/activate

echo ""
echo "Checking dependencies..."
if ! python -c "import PyQt6" &> /dev/null; then
    echo "Installing dependencies..."
    pip install -r requirements.txt
else
    echo "[OK] Dependencies installed"
fi

echo ""
echo "===================================="
echo " Starting Mission Control App"
echo "===================================="
echo ""
echo "Mode: Simulation (no hardware required)"
echo ""

# Launch the application in simulation mode
python src/main.py --sim --debug

echo ""
echo "Application closed."
