arm-none-eabi-gdb --tui cmake-build-debug/build_motor -ex "target extended-remote localhost:3232" -ex "load" -ex "b main" -ex "c"
