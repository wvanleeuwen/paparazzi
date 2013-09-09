// SPI transfer mode
#define TIMOD_RX 0x00
#define TIMOD_TX 0x01
#define TIMOD_DMA_RX 0x02
#define TIMOD_DMA_TX 0x03
#define TIMOD_MASK 0x03

//SPI Mode SD commands
#define GO_IDLE_STATE 0
#define SEND_OP_COND 1
#define SEND_CSD 9
#define SEND_CID 10
#define STOP_TRANSMISSION 12
#define SEND_STATUS 13
#define SET_BLOCKLEN 16
#define READ_BLOCK 17
#define READ_MULTIPLE_BLOCK 18
#define WRITE_BLOCK 24
#define WRITE_MULTIPLE_BLOCK 25
#define PROGRAM_CSD 27
#define SET_WRITE_PROT 28
#define CLR_WRITE_PROT 29
#define SEND_WRITE_PROT 30
#define ERASE_WR_BLK_START_ADDR 32
#define ERASE_WR_BLK_END_ADDR 33
#define ERASE 38
#define APP_CMD 55
#define GEN_CMD 56
#define READ_OCR 58
#define CRC_ON_OFF 59

#define DATA_TOKEN 0xFE
#define MDATA_TOKEN 0xFC
#define STOP_TOKEN 0xFD
#define BUSY 0x00

#define DATA_ACCEPTED 0x5
#define DATA_REJ_CRC 0xB
#define DATA_REJ_WRERR 0xD

#define RESPONSE_MASK 0x1F

void InitSD();
void CloseSD();
unsigned char WriteByte(unsigned char byte);
unsigned char SendCommand(unsigned char cmd, unsigned int param);
unsigned char GetToken();
unsigned int CardInit();
unsigned int GetCardParams(unsigned int*secnum);
unsigned int EraseBlocks(unsigned int start_addr, unsigned int end_addr);
unsigned int WriteSector(unsigned int secnum, unsigned char*buf);
unsigned int ReadSector(unsigned int secnum, unsigned char*buf);
unsigned int ReadMultipleBlocks(unsigned int from, unsigned int numsec, unsigned char*buf);
unsigned int WriteMultipleBlocks(unsigned int from, unsigned int numsec, unsigned char*buf);

