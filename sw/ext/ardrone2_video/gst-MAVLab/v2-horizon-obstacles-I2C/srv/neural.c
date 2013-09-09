/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  neural.c - simple integer backprop neural network library based on 
 *  floating point code originally written by Baegsch of www.e-m-c.org
 *
 *       Copyright (C) 2007-2009  Surveyor Corporation
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

#include "neural.h"
#include "srv.h"
#include "print.h"


int learn = 300;

unsigned char npattern[NUM_NPATTERNS * 8] = {
    0x18, 0x7E, 0x7E, 0xFF, 0xFF, 0x7E, 0x7E, 0x18,  // solid ball
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // solid square
    0x18, 0x18, 0x3C, 0x3C, 0x7E, 0x7E, 0xFF, 0xFF,  // pyramid
    0x18, 0x18, 0x18, 0xFF, 0xFF, 0x18, 0x18, 0x18,  // cross
    0xFF, 0xFF, 0xC3, 0xC3, 0xC3, 0xC3, 0xFF, 0xFF,  // box
    0x18, 0x7E, 0x66, 0xC3, 0xC3, 0x66, 0x7E, 0x18,  // circle
    0xC3, 0xC3, 0x24, 0x18, 0x18, 0x24, 0xC3, 0xC3,  // xing
    0x18, 0x3C, 0x66, 0xC3, 0xC3, 0x66, 0x3C, 0x18,  // diamond
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,  // horizontal line
    0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C,  // vertical line
    0x03, 0x03, 0x04, 0x18, 0x18, 0x20, 0xC0, 0xC0,  // slash
    0xC0, 0xC0, 0x20, 0x18, 0x18, 0x04, 0x03, 0x03,  // backslash
    0x18, 0x18, 0x3C, 0x3C, 0x66, 0x66, 0xC3, 0xC3,  // up arrow
    0xC3, 0xC3, 0x66, 0x66, 0x3C, 0x3C, 0x18, 0x18,  // down arrow
    0xC0, 0xF0, 0x3C, 0x07, 0x07, 0x3C, 0xF0, 0xC0,  // right arrow
    0x03, 0x0F, 0x3C, 0xE0, 0xE0, 0x3C, 0x0F, 0x03,  // left arrow
};

unsigned char nmask[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

int weights[NUM_INPUT*NUM_HIDDEN + NUM_HIDDEN*NUM_OUTPUT];
int neurons[NUM_INPUT + NUM_HIDDEN + NUM_OUTPUT];
int teach_neurons[NUM_OUTPUT];
int nerror[NUM_OUTPUT + NUM_HIDDEN];

void
nninit_network(void)
{
    int mod = 32, n;
    int w, x = 32;

    for(n=0; n < sizeof(weights) / sizeof(int); n++) {
        w = rand() % mod;
        if(rand() & 0x8000000)
            w = -w;

        weights[n] = w * x;
    }
}

void
nninit_pattern(void)
{
    int pat = rand() % NUM_NPATTERNS;
    nnset_pattern(pat);
}


void
nnset_pattern(int pat)
{
    int i, o, nx;

    for(i=0; i < NUM_INPUT; i++) { 
        nx = (pat * 8) + (i / 8);              // offset into npattern array
        if (npattern[nx] & nmask[i % 8])       // now unpack individual bits
            N_IN(i) = 1024;                    // expand a 1 to 1024
        else
            N_IN(i) = 0;
    }

    for(o=0; o < NUM_OUTPUT; o++)
        if (o == pat)
            N_TEACH(o) = 1024;
        else
            N_TEACH(o) = 0;
}

int
f_logic(int x)
{
    // Sigmoid function approximation using piecewise linear approximation 
    // of a nonlinear function (PLAN) - proposed by Amin, Curtis & Hayes-Gill
    // IEE Proc Circuits, 1997

    int ret;
    int neg;

    neg = 0;
    if (x < 0) {
        neg = 1;
        x = -x;
    }

    if (x > 5120)
        ret = 1024;
    else if (x > 2432)
        ret = (x * 32) / 1024 + 864;
    else if (x > 1024)
        ret = (x * 128) / 1024 + 640;
    else
        ret = (x * 256) / 1024 + 512;
    
    if (neg)
        ret = 1024 - ret;

    return ret;
}


void
nncalculate_network(void)
{
    int i,h,o;

    for(h=0; h < NUM_HIDDEN; h++)
        N_HIDDEN(h) = 0.0;

    for(o=0; o < NUM_OUTPUT; o++)
        N_OUT(o) = 0.0;

    for(h=0; h < NUM_HIDDEN; h++) {
        for(i=0; i < NUM_INPUT; i++)
            N_HIDDEN(h) += (N_IN(i) * W_IN_HIDDEN(i, h)) / 1024;

        N_HIDDEN(h) = f_logic(N_HIDDEN(h));
    }

    for(o=0; o < NUM_OUTPUT; o++) {
        for(h=0; h < NUM_HIDDEN; h++)
            N_OUT(o) +=( N_HIDDEN(h) * W_HIDDEN_OUT(h, o)) / 1024;

        N_OUT(o) = f_logic(N_OUT(o));
    }
}


void
nncalculate_errors(void)
{
    int h, o;
    int err;

    for(o=0; o < NUM_OUTPUT; o++)
        E_OUT(o) = (N_TEACH(o) - N_OUT(o));

    for(h=0; h < NUM_HIDDEN; h++) {
        err = 0.0;
        for(o=0; o < NUM_OUTPUT; o++)
            err += (E_OUT(o) * W_HIDDEN_OUT(h, o)) / 1024;

        E_HIDDEN(h) = (((N_HIDDEN(h) * (1024 - N_HIDDEN(h))) / 1024) * err) / 1024;
    }
}

void
nntrain_network(int num)
{
    int i, h, o;
    unsigned int n;


    for(n = 0; n < num; n++) {

        nninit_pattern();
        nncalculate_network();
        nncalculate_errors();
    
        for(h=0; h < NUM_HIDDEN; h++)
            for(o=0; o < NUM_OUTPUT; o++)
                W_HIDDEN_OUT(h, o) += (((learn * N_HIDDEN(h)) / 1024) * E_OUT(o)) / 1024;

        for(i=0; i < NUM_INPUT; i++)
            for(h=0; h < NUM_HIDDEN; h++)
                W_IN_HIDDEN(i, h) += (((learn * N_IN(i)) / 1024) * E_HIDDEN(h)) / 1024;
    }
}

/* display a stored pattern */
void 
nndisplay(int ix)
{
    int i1, i2;

    for (i1=0; i1<8; i1++) {
        for (i2=0; i2<8; i2++) {
            if (npattern[ix*8 + i1] & nmask[i2])
                printf(" **");
            else
                printf("   ");
        }
        printf("\r\n");
    }
}

/* scale blob in blob_buf[] to 8x8 coordinates of N_IN(0:63)
   note that blob_buf[] only stores data in the even cells  */
void
nnscale8x8(unsigned char *blob_buf, unsigned int blob_ix, unsigned int x1, unsigned int x2, 
    unsigned int y1, unsigned int y2, unsigned int iwidth, unsigned int iheight)
{
    unsigned int ix, x, y, xsize, ysize;
    
    for (x=0; x<64; x++)  // clean the N_IN() input array
        N_IN(x) = 0;
          
    if (x2 <= x1 || y2 <= y1)
        return;

    xsize = (x2 - x1 + 7) / 8;   // careful - we will only get 1/2 as many hits as expected
    ysize = (y2 - y1 + 7) / 8;
    for (y=y1; y<=y2; y++) {
        for (x=x1; x<=x2; x+=2) {
            if (blob_buf[x + (y * iwidth)] == blob_ix) { 
                ix = (((y - y1) / ysize) * 8) + ((x - x1) / xsize);
                N_IN(ix) += 1024;
            }
        }
    }
    
    ix = xsize * ysize / 2;  // adjust for even-only blob_buf[] values
    for (x=0; x<64; x++)
        N_IN(x) /= ix;  // normalize the result
}

/* save data from N_IN(0:63) to packed 8x8 binary pattern in npattern[] */
void
nnpack8x8(int pat)
{
    int i, nx;

    for(i=0; i<8; i++)                    // clear out old pattern
        npattern[pat*8 + i] = 0;
    for(i=0; i < NUM_INPUT; i++) { 
        nx = (pat * 8) + (i / 8);         // offset into npattern array
        if (N_IN(i) > 500) 
            npattern[nx] |= nmask[i % 8]; // now pack individual bits
    }
}

