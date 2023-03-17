#include <stdio.h>

typedef unsigned char UCHAR;

// This code copied directly from TMC2300 datasheet - Rev 1.08
void swuart_calcCRC(UCHAR* datagram, UCHAR datagramLength) {
    int i, j;
    UCHAR currentByte = 0; // This line was missing, added by SJFOM
    UCHAR* crc = datagram + (datagramLength - 1); // CRC located in last byte of message UCHAR currentByte;
    *crc = 0;
    for (i = 0; i < (datagramLength - 1); i++)
    {
        currentByte = datagram[i];
        for (j = 0; j < 8; j++)
        {
            if ((*crc >> 7) ^ (currentByte & 0x01))
            {
                *crc = (*crc << 1) ^ 0x07;
            }
            else
            {
                *crc = (*crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
}

int main() {
    const unsigned int dg_len = 4;
    const unsigned int crc_idx = dg_len - 1;

    // Create example datagram
    UCHAR dg[dg_len];
    dg[0] = 0x05;
    dg[1] = 0x03;
    dg[2] = 0x06;

    // Pass datagram into CRC calculation method
    swuart_calcCRC(dg, dg_len);

    // Print message contents - showing nothing has changed
    for (int i = 0; i < dg_len - 1; i++)
    {
        printf("%d: 0x%02x\n", i, dg[i]);
    }

    // Print resultant CRC byte
    printf("%d: 0x%02x\t<--- CRC\n", dg_len, dg[crc_idx]);
    return 0;
}