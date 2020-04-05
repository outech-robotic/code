rm -rf cmake_build
cmake -B cmake_build .
cd cmake_build
make -j16

arm-none-eabi-objcopy -O binary outech_lowlevel ll.bin
cp ll.bin ..
st-flash write ../ll.bin 0x8000000