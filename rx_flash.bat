@echo off
nrfutil device recover
nrfutil device program --firmware .\build\merged.hex
nrfutil device reset
pause
