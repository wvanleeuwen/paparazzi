extern int xmodemReceive(unsigned char *dest, int destsz);
extern int xmodemTransmit(unsigned char *src, int srcsz);
extern unsigned short crc16tab[];
extern unsigned short crc16_ccitt(const void *buf, int len);

