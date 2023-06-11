#!/usr/bin/env bash
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program build/App-StepUp/StepUp.elf verify reset exit"
