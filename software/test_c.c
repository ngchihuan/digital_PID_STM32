#include <stdio.h>
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 
void print_32bit(long m)
{
    printf("m: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\n",
  BYTE_TO_BINARY(m>>32),BYTE_TO_BINARY(m>>16),BYTE_TO_BINARY(m>>8), BYTE_TO_BINARY(m));
}
int main()
{
    __uint8_t* Buf;
    __uint8_t x=4;
    __uint8_t pBuf[4];
    __uint32_t combine=0;
    pBuf[0]=240;
    pBuf[1]=10;
    pBuf[2]=1;
    combine = pBuf[0]|pBuf[1]<<8|pBuf[2]<<16;
    //combine = ((((pBuf[0] << 0x18) | (pBuf[1] << 0x10)) | (pBuf[2] << 8)) | pBuf[3]);
    Buf = &x;
    printf(BYTE_TO_BINARY_PATTERN,BYTE_TO_BINARY(pBuf[2]));
    printf("\n");
    print_32bit(combine);
    ((((pBuf[0] << 0x18) | (pBuf[1] << 0x10)) | (pBuf[2] << 8)) | pBuf[3]);

    return 0;
}