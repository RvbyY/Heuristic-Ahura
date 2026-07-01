#!/bin/bash

git pull
cd coord-drive
./build.sh
cd ..
./run_coord_drive.sh
