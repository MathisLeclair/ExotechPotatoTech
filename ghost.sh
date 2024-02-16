#!/bin/bash
if [ -f ".pio/build/simu/program" ]; then
    cpulimit -e program -l 10 &

    systemd-run --scope -p MemoryMax=1024K --user .pio/build/simu/program
else
    echo "Error: Please build the project in simulation mode, before executing ghost"
    exit 1
    
fi