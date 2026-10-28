@echo off
title Rover Explorer - PC Processing Server
echo ========================================================
echo   ROVER EXPLORER - INICIANDO SERVIDOR DE PROCESAMIENTO
echo ========================================================
echo.

echo Verificando dependencias de Python...
python -m pip install -r requirements.txt

echo.
echo Iniciando servidor en http://localhost:8000 ...
python server.py

pause
