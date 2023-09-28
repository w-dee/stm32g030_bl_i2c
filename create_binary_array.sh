#!/bin/bash -e

SOURCE=../STM32G030F6/.pio/build/genericSTM32G030F6/firmware.bin
SIZE=`wc -c < $SOURCE`
echo -n "const " > src/binary.c
xxd -i $SOURCE >> src/binary.c
ID=`head -1 src/binary.c | sed -e 's/const unsigned char//' -e 's/\[.*//'`
echo "#include <stddef.h>" > src/binary.h
echo "extern const unsigned char $ID [];" >> src/binary.h
echo "static const size_t ${ID}_size = $SIZE;"  >> src/binary.h
echo "#define BM_FIRMWARE_BINARY ${ID}" >> src/binary.h
echo "#define BM_FIRMWARE_BINARY_SIZE ${ID}_size" >> src/binary.h
