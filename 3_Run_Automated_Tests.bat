@echo off
title Smart Pothole Robot - Unit Tests
echo ========================================================
echo   Running 13 Academic Unit & Integration Tests
echo ========================================================
python -m unittest discover -s tests -p "test_*.py" -v
echo.
echo ========================================================
echo   Evaluating 5 Benchmark Cases from Log
echo ========================================================
python tools\summarize_log.py
echo.
pause
