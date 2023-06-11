#!/bin/sh
echo "Building firmware..."
/usr/local/bin/cmake --build /Users/samomahony/Documents/code/coding/StepUp-Pico/build --config Debug --target StepUp -j 6 --
echo "Complete!"

echo "Flashing firmware..."
source program.sh
echo "Complete!"