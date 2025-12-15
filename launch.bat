@echo off
REM ========================================
REM VTOL Mission Control - Quick Launch
REM Windows Batch Script
REM ========================================

echo.
echo ====================================
echo  VTOL Mission Control System
echo  Ground Control Station
echo ====================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.10 or higher
    pause
    exit /b 1
)

echo [OK] Python found
echo.

REM Check if virtual environment exists
if not exist "venv\" (
    echo Creating virtual environment...
    python -m venv venv
    echo [OK] Virtual environment created
) else (
    echo [OK] Virtual environment found
)

echo.
echo Activating virtual environment...
call venv\Scripts\activate.bat

echo.
echo Checking dependencies...
pip show PyQt6 >nul 2>&1
if errorlevel 1 (
    echo Installing core dependencies...
    pip install -r requirements-minimal.txt
    if errorlevel 1 (
        echo ERROR: Failed to install dependencies
        pause
        exit /b 1
    )
) else (
    echo [OK] Dependencies installed
)

echo.
echo ====================================
echo  Starting Mission Control App
echo ====================================
echo.
echo Mode: Simulation (no hardware required)
echo.

REM Launch the application in simulation mode
python src\main.py --sim --debug

echo.
echo Application closed.
pause
