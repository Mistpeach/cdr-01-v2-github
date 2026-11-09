@echo off
@rem SPDX-License-Identifier: Apache-2.0
@rem Copyright (c) 2026 Mistpeach
rem CDR01 v2 host launcher (ASCII only: cmd.exe mojibake issue with UTF-8 Chinese text)
cd /d "%~dp0"

set "PY=python"
where python >nul 2>nul
if errorlevel 1 (
  echo [run] Python not found. Install Python 3.10+ and make sure "python" is on PATH.
  pause
  exit /b 1
)

"%PY%" -c "import bleak, PySide6" >nul 2>nul
if errorlevel 1 (
  echo [run] Dependencies missing. Run:
  echo        python -m pip install -r requirements.txt
  pause
  exit /b 1
)

echo [run] Python: "%PY%"
"%PY%" -m cdr01v2_host.main
echo.
echo [run] Host exited with code %errorlevel%.
pause
