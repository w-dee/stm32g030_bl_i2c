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
echo "#define BM_FIRMWARE_BINARY_SIZE ${SIZE}" >> src/binary.h

TMP_CRC_CALC_DIR=/tmp/calc_crc
TMP_CRC_CALC_NAME=calc_crc
mkdir -p $TMP_CRC_CALC_DIR
TMP_CRC_CALC=$TMP_CRC_CALC_DIR/$TMP_CRC_CALC_NAME.c
TMP_CRC_CALC_BIN=$TMP_CRC_CALC_DIR/$TMP_CRC_CALC_NAME

cat src/binary.c - <<EOF > $TMP_CRC_CALC
#include <stdio.h>
#include <string.h>
#include <stdint.h>


// CRC32 calculation algorithm taken from https://www.cnblogs.com/shangdawei/archive/2013/04/28/3049789.html

// Nibble lookup table for 0x04C11DB7 polynomial
const uint32_t sw_crc32_by_nibble_table[16] = {
  0x00000000,0x04C11DB7,0x09823B6E,0x0D4326D9,
  0x130476DC,0x17C56B6B,0x1A864DB2,0x1E475005,
  0x2608EDB8,0x22C9F00F,0x2F8AD6D6,0x2B4BCB61,
  0x350C9B64,0x31CD86D3,0x3C8EA00A,0x384FBDBD
};

static uint32_t read_le(const unsigned char *ptr)
{
	return ((uint32_t)ptr[0]) +
	((uint32_t)ptr[1] << 8 ) +
	((uint32_t)ptr[2] << 16) +
	((uint32_t)ptr[3] << 24);
}

static uint16_t read_le_16(const unsigned char *ptr)
{
	return ((uint16_t)ptr[0]) +
	((uint16_t)ptr[1] << 8 );
}

static uint32_t stm32_sw_crc32_by_nibble(uint32_t crc32, const unsigned char pBuffer[], uint32_t NumOfByte)  
{
  uint32_t last_data;
  uint32_t NumOfDWord = NumOfByte>>2;
  uint32_t NumOfTailByte =  NumOfByte & 3 ;
  
  while(NumOfDWord--)
  {
    crc32 = crc32 ^ read_le(pBuffer);
    pBuffer += 4;
    crc32 = (crc32 << 4) ^ sw_crc32_by_nibble_table[crc32 >> 28];
    crc32 = (crc32 << 4) ^ sw_crc32_by_nibble_table[crc32 >> 28];
    crc32 = (crc32 << 4) ^ sw_crc32_by_nibble_table[crc32 >> 28];
    crc32 = (crc32 << 4) ^ sw_crc32_by_nibble_table[crc32 >> 28];
    crc32 = (crc32 << 4) ^ sw_crc32_by_nibble_table[crc32 >> 28];
    crc32 = (crc32 << 4) ^ sw_crc32_by_nibble_table[crc32 >> 28];
    crc32 = (crc32 << 4) ^ sw_crc32_by_nibble_table[crc32 >> 28];
    crc32 = (crc32 << 4) ^ sw_crc32_by_nibble_table[crc32 >> 28];
  }
  
  switch ( NumOfTailByte )
  {
    case 0:
      return crc32;     
    case 1:
      last_data = pBuffer[0] << 24;
      break;
    case 2:
      last_data = read_le_16(&pBuffer[0]);
      last_data <<= 16;
      
      break;
    case 3:
      last_data = read_le_16(&pBuffer[0]);
      last_data <<= 8;
      last_data += pBuffer[2]<<24;
      break;
  }
  crc32 = stm32_sw_crc32_by_nibble( crc32, (uint8_t *)&last_data, 4);
  return crc32;
}



EOF


echo "int main() { uint32_t crc = stm32_sw_crc32_by_nibble(0xffffffff, " >> $TMP_CRC_CALC 
echo "$ID, ${ID}_len);" >> $TMP_CRC_CALC 
echo "printf(\"%08x\n\", crc);}"  >> $TMP_CRC_CALC 

(cd $TMP_CRC_CALC_DIR && make $TMP_CRC_CALC_NAME)

CRC=`$TMP_CRC_CALC_BIN`

echo "#define BM_FIRMWARE_BINARY_CRC 0x${CRC}" >> src/binary.h
