@echo off
nrfutil device recover
nrfutil device write --address 0xffd500 --value 0xABaBa
nrfutil device program --firmware .\build\merged.hex
nrfutil device reset
pause
