/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  sdcard.c - routines to interface with the SD memory card
 *    based on code posted by Dmitry Bodnya on www.blackfin.org/phorum
 *    Copyright (C) 2009  Surveyor Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <cdefBF537.h>
#include "config.h"
#include "uart.h"
#include "print.h"
#include "srv.h"
#include "sdcard.h"

void InitSD(void)
{
    *pPORTF_FER |= PF13 | PF12 | PF11 | PF5; //all pins we need
    //*pSPI_FLG |= FLS5; //my slave select
    *pPORTHIO &= 0xFEFF; // set GPIO-H8 low to activate SPI select
    *pPORT_MUX |= PFS5E;
    // Set baud rate SCK = HCLK/(2*SPIBAUD) SCK = 25MHz HCLK=100MHz
    *pSPI_CTL = MSTR | TIMOD_TX ;
    *pSPI_BAUD = 4;  //25 Mhz - maximum SD card working frequency
}

void CloseSD(void)
{
    *pPORTHIO |= 0x0100; // set GPIO-H8 high to disable SPI select
}

//Write byte through SPI and return incoming byte
unsigned char WriteByte(unsigned char byte)
{
    *pSPI_STAT = TXCOL | RBSY | MODF | TXE;
    *pSPI_CTL = MSTR | TIMOD_TX | SPE;
    *pSPI_TDBR = byte;//starting SPI operation by writing to SPI_TDBR

    while((*pSPI_STAT & TXS)||(*pSPI_STAT & TXS));
    while(((*pSPI_STAT & SPIF)==0));
    return (unsigned char)*pSPI_RDBR;
}

unsigned char SendCommand(unsigned char cmd, unsigned int param)
{
    unsigned int i;
    unsigned char r1;

    for(i=0; i<10; i++)
        if(WriteByte(0xFF)==0xFF)
            break;

    WriteByte(cmd | 0x40);
    WriteByte((unsigned char)(param>>24));
    WriteByte((unsigned char)(param>>16));
    WriteByte((unsigned char)(param>>8));
    WriteByte((unsigned char)param);
    WriteByte(0x95);

    for(i=0;i<8;i++) {
        r1 = WriteByte(0xFF);//receiving an answer to a command
        if (!(r1 & 0x80)) // wait b'0xxxxxxx'
            break;
    }
    WriteByte(0xFF);
    return r1;
}

//This function receives different tokens from SD card
unsigned char GetToken()
{
    unsigned char c;
    unsigned int i;
    for(i=0; i<0x100; i++) {
        c = WriteByte(0xFF);
        if( (c != 0xFF) && (c & 0xFC) )
            return c;
    }
    return c;
}

// Function for initing SD card
unsigned int CardInit()
{
    unsigned int i, ok;
    
    ok=0;
    for(i=0; i<2; i++) {
        if(SendCommand(GO_IDLE_STATE,0) == 1) {
            ok = 1;
            break;
        }
    }
    if(!ok)
        return 0;

    i=0xF000;
    while((SendCommand(SEND_OP_COND,0)!=0)&&(--i));

    return (i>0);
}

// After moment, when SD card inited, you can read and write to SD card
// or you can read SD card params.
// This function returns(through variable) capacity of SD card in 
//sectors(sector=512 bytes).
unsigned int GetCardParams(unsigned int*secnum)
{
    unsigned int i,rbl,dsize,mult,capacity,tmp;

    if(SendCommand(SEND_CSD,0) != 0)
        return 0;
    if(GetToken() != DATA_TOKEN)
        return 0;
    for(i=0;i<5;i++)
        WriteByte(0xff);//1-5
    rbl = WriteByte(0xff) & 0x0f;
    dsize = (WriteByte(0xff) & 0x03) << 10; // 7
    dsize |= WriteByte(0xff) << 2; // 8
    dsize |= (WriteByte(0xff) & 0xC0) >> 6; // 9
    mult = (WriteByte(0xff) & 0x03) << 1;//10
    mult |= (WriteByte(0xff) & 0x80) >> 7;//11

    tmp=1;
    for(i=0; i < 2+mult+rbl; i++)
        tmp <<= 1;
    capacity = (dsize + 1)*tmp;

    for(i=0;i<8;i++)
        WriteByte(0xFF);

    *secnum = (capacity>>9);
    return 1;
}

/* You need to remember that you can write only data portions by 512 bytes(sector). you can read from 1 to 512 bytes. But to read lower then 512 bytes you need to sen data length by individual command, so by default we can read 512 bytes.
Also, SD card have block write and block read operations.
All operations should be aligned by sector boundaries. */

//This function for erasing any area in SD card(aligned!)
//adresses of start byte and end byte of area
unsigned int EraseBlocks(unsigned int start_addr, unsigned int end_addr)
{
    if(SendCommand(ERASE_WR_BLK_START_ADDR, start_addr) != 0)
        return 0;
    if(SendCommand(ERASE_WR_BLK_END_ADDR, end_addr) != 0)
        return 0;
    if(SendCommand(ERASE, 0) != 0)
        return 0;
    while(WriteByte(0xFF)==BUSY);
    return 1;
}

unsigned int WriteSector(unsigned int secnum, unsigned char*buf)
{
    unsigned int i;

    if(SendCommand(WRITE_BLOCK,secnum<<9) != 0)
        return 0;
    while(WriteByte(0xFF) != 0xFF);
    WriteByte(DATA_TOKEN);
    for(i=0;i<512;i++)
        WriteByte(*buf++);
    WriteByte(0xFF);//2 bytes
    WriteByte(0xFF);//of crc - it can be everything except zeros
    if((GetToken() & RESPONSE_MASK) != DATA_ACCEPTED)
        return 0;
    while(WriteByte(0xFF)==BUSY);

    return 1;
}

unsigned int ReadSector(unsigned int secnum, unsigned char*buf)
{
    unsigned int i;
    if(SendCommand(READ_BLOCK,secnum<<9) != 0)
        return 0;

    if(GetToken() != DATA_TOKEN)
        return 0;

    for(i=0;i<512;i++)
        *buf++ = WriteByte(0xFF);
    WriteByte(0xFF);
    WriteByte(0xFF);
    WriteByte(0xFF);

    return 1;
}

/*To write each sector - it is very slowly and it raises big delays after each write. 
  So, it's much better to use block write and read commands.*/


//from - it's start sector to read from
unsigned int ReadMultipleBlocks(unsigned int from, unsigned int numsec, unsigned char*buf)
{
    unsigned int i,k;
    unsigned char *ptr=buf, fill;

    if(SendCommand(READ_MULTIPLE_BLOCK,from<<9) != 0)
        return 0;
    while(WriteByte(0xFF) != 0xFF);

    for(i=0; i<numsec; i++) {
        while((fill = WriteByte(0xFF)) == 0xFF);
        if(fill != DATA_TOKEN)
            return 0;
        for(k=0; k<512; k++)
            *ptr++ = WriteByte(0xFF);
        fill = WriteByte(0xFF);
        fill = WriteByte(0xFF);
    }
    fill = SendCommand(STOP_TRANSMISSION,0);
    return 1;
}

//from - it's start sector to write to
unsigned int WriteMultipleBlocks(unsigned int from, unsigned int numsec, unsigned char*buf)
{
    unsigned int i,k;
    unsigned char *ptr=buf;

    if(SendCommand(WRITE_MULTIPLE_BLOCK,from<<9) != 0)
        return 0;

    for(i=0; i<numsec; i++) {
        WriteByte(MDATA_TOKEN);
        for(k=0; k<512; k++)
            WriteByte(*ptr++);
        WriteByte(0xFF);//crc
        WriteByte(0xFF);//crc
        while(WriteByte(0xFF)==BUSY);
        //here you can add some counter to not loop forever
    }
    WriteByte(STOP_TOKEN);
    while(WriteByte(0xFF)==BUSY);
    return 1;
}

