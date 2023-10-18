uint16_t HAMMING_CODE(uint8_t);
uint8_t HAMMING_DECODE(uint16_t);

uint16_t HAMMING_CODE(uint8_t n) {
    uint8_t x = 0x01 & (n ^ (n>>2) ^ (n>>4) ^ (n>>6));
    x |= 0x02 & (n ^ (n>>1) ^ (n>>4) ^ (n>>5));
    x |= 0x04 & ((n>>1) ^ (n>>2) ^ (n>>3) ^ (n>>4));
    x |= 0x08 & (n>>4);
    return n | ((uint16_t)x<<8);
}
uint8_t HAMMING_DECODE(uint16_t n) {
    uint16_t n2 = HAMMING_CODE(n);
    uint8_t syndrom = (n>>8)^(n2>>8);
    if (syndrom == 0) return n & 0xFF;
    else return n ^ pow(2,(8-syndrom));
}
