@echo off
echo --- Setting up Python Virtual Environment ---

:: Check if Python is installed
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Error: Python is not installed or not in PATH.
    pause
    exit /b
)

:: Create Virtual Environment
if not exist "venv" (
    echo Creating 'venv' folder...
    python -m venv venv
) else (
    echo 'venv' folder already exists.
)

:: Activate Virtual Environment
echo Activating virtual environment...
call venv\Scripts\activate

:: Install Requirements
if exist "requirements.txt" (
    echo Installing dependencies from requirements.txt...
    python -m pip install --upgrade pip
    pip install -r requirements.txt
    echo.
    echo --- Setup Complete! ---
    echo To start the backend, run: venv\Scripts\python backend\main.py
) else (
    echo Error: requirements.txt not found!
)

pause
