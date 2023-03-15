openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program build/Application/App-StepUp/StepUp.elf verify reset exit"
