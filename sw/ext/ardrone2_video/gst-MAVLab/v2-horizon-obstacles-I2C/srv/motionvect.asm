/*******************************************************************************
Copyright(c) 2000 - 2002 Analog Devices. All Rights Reserved.
Developed by Joint Development Software Application Team, IPDC, Bangalore, India
for Blackfin DSPs  ( Micro Signal Architecture 1.0 specification).

By using this module you agree to the terms of the Analog Devices License
Agreement for DSP Software. 
********************************************************************************
Module Name     : motionvect.asm
Label Name      : __motionvect
Version         :   2.0
Change History  :

                Version     Date          Author        Comments
                2.0            01/09/2007    Arjun                Tested with VDSP++4.5
                                                        Compiler 7.2.3.2
                1.2         11/18/2002    Swarnalatha   Tested with VDSP++ 3.0
                                                        compiler 6.2.2 on 
                                                        ADSP-21535 Rev.0.2
                1.1         11/13/2002    Swarnalatha   Tested with VDSP++3.0 
                                                        on ADSP-21535 Rev.0.2
                1.0         07/02/2001    Vijay         Original 

Description     : The assembly function does the full search block matching to
                  compute the motion vectors given a target frame and a 
                  reference frame. By suitably selecting the target frame and 
                  the reference frame, this routine can be used to obtain the 
                  motion vectors of a P or a B frame. The target and reference 
                  frames have to be of the same size and should be an integer 
                  multiple of 16. The input arguments to the function are the 
                  target frame address, the reference frame address, the frame 
                  size, the search area factor and the start addresses of the 
                  motion vectors. The macro block size is fixed to be 16x16. The
                  search area is computed based on the search area factor. It 
                  specifies an integer value between 1 and 3. The search range 
                  is specified as follows:
                        
                        =============================================
                        |   Search_area_factor  |   Search range    |
                        |           1           |     -4 to 3.5     |
                        |           2           |     -8 to 7.5     |       
                        |           3           |    -16 to 15.5    |   
                        =============================================

                  The motion vector gives an integer offset for every macro 
                  block in the target frame with respect to the macro blocks in 
                  the reference frame in both vertical and horizontal 
                  directions. Once the integer offset is computed a half pixel
                 correction is given in either directions (vertical and 
                 horizontal) by interpolating adjacent macro blocks. The 
                 resultant vectors are called half-pel motion vectors. The 
                 motion vectors for all the macro blocks in the target frame 
                 have been computed by grouping the macro blocks into NINE 
                 categories based on the range within which the matching has to 
                 be done, without crossing the frame boundary. Hence there will 
                 be nine code-segments which compute the motion vectors for the
                 macro blocks in that category. These categories are pictorially
                 represented and the ranges allowed for them are explained in a 
                 table :

                       -----------------------------------------
                       |       |       |       |       |       |
                       |  NW   |       | NORTH |       |   NE  |
                       -----------------------------------------
                       |       |       |       |       |       |
                       |       |       |       |       |       |
                       ----W-------------------------------E----
                       |   E   |     C E N T R A L     |   A   |
                       |   S   |       |       |       |   S   |
                       ----T-------------------------------T----
                       |       |       |       |       |       |
                       |       |       |       |       |       |
                       -----------------------------------------
                       |       |       |       |       |       |
                       |  SW   |       | SOUTH |       |   SE  |
                       -----------------------------------------

================================================================================
                         search_area_factor (S)                            
================================================================================
  Category       S = 1                 S = 2                   S = 3               
          (vertical, horizontal) (vertical, horizontal) (vertical, horizontal)  
================================================================================
                                                                                                
NORTH-WEST (0 to 3.5, 0 to 3.5)  (0 to 7.5, 0 to 7.5)  (0 to 15.5, 0 to 15.5)    
NORTH-EAST (0 to 3.5, -4 to 0)   (0 to 7.5, -8 to 0)   (0 to 15.5, -16 to 0)     
SOUTH-WEST (-4 to 0, 0 to 3.5)   (-8 to 0, 0 to 7.5)   (-16 to 0, 0 to 15.5)     
SOUTH-EAST (-4 to 0, -4 to 0)    (-8 to 0, -8 to 0)    (-16 to 0, -16 to 0)      
NORTH      (0 to 3.5, -4 to 3.5) (0 to 7.5, -8 to 7.5) (0 to 15.5, -16 to 15.5)  
SOUTH      (-4 to 0, -4 to 3.5)  (-8 to 0, -8 to 7.5)  (-16 to 0, -16 to 15.5)   
WEST       (-4 to 3.5, 0 to 3.5) (-8 to 7.5, 0 to 7.5) (-16 to 15.5, 0 to 15.5)  
EAST       (-4 to 3.5, -4 to 0)  (-8 to 7.5, -16 to 0) (-16 to 15.5, -16 to 0)   
CENTRAL    (-4 to 3.5, -4 to 3.5)(-8 to 7.5, -8 to 7.5)(-16 to 15.5,-16 to 15.5)
================================================================================

                  As expected by the MPEG-2 standard the routine multiplies the 
                  actual offset between the target macro block and a best 
                  matching reference macro block within the search range by two.
                  It should also be noted that the macro blocks in the boundary 
                  of the frame are not given the half pixel correction, but 
                  still they are scaled by a factor of two. Thus, all target 
                  macro blocks which underwent an integer transition will have 
                  even motion vectors while those blocks that are offset by a 
                  non-integer factor will have odd motion vectors.

Registers Used  : R0-R7, B0-B3, P0, P1, P3-P5.

Prototype       : void __motionvect(&Target_frame, &Reference_Frame,
                  &mv_vertical, &mv_horizontal, rows, columns, search_factor);

                  Input arguments : 
                    Target_frame    - Starting address of the Target frame
                    Reference_Frame - Starting address of the reference frame
                    mv_vertical     - Starting address of the motion vector 
                                      matrix (Vertical direction)
                    mv_horizontal   - Starting address of the motion vector 
                                      matrix (Horizontal direction)
                    rows            - No. of rows in the frame
                    columns         - No. of columns in the frame
                    search_factor   - Search_area_factor

Performance     :
                Code size : 1122 bytes (only motionvect.asm)

    ======================================================================
    Cycle count for the Half-pixel Motion estimation using Mean Absolute 
    Difference (MAD) criterion
    ======================================================================
    S.No. Frame size  Search_factor  Search Range(pixels)  No. of cycles  
    1      80 x 96        1           -4 to 3.5              1,57353    
    2      80 x 96        2           -8 to 7.5              5,27212   
    3      80 x 96        3          -16 to 15.5            19,74577   
    ======================================================================

    Cycle count mentioned above is for the test case given in test_full_search.c
    file.
*******************************************************************************/
.text
.global __motionvect;
.align      8;
.extern __mve_core;
.extern _hpel;

__motionvect:

    [--SP] = RETS;
    [--SP] = (R7:4, P5:3);
/*************************** FETCH INPUT ARGUMENTS ****************************/
    
    B0 = R0;                // Fetch the target frame address
    B1 = R1;                // Fetch the reference frame address
    B2 = R2;                // Fetch the address of motion vector along y 
                            // direction
    P3 = [SP + 44];         // Fetch the address of motion vector along x 
                            // direction
    R4 = [SP + 48];         // No. of rows in the frame
    R5 = [SP + 52];         // No. of columns in the frame
    R6 = [SP + 56];         // Search_area_factor
    B3 = P3;
    
/*************************** PRELIMINARY COMPUTATIONS ************************/
    SP += -52;
    P3 = SP;
    SP += -52;
    R6 += 1;                                    
    R7 = 1;                                     
    R6 = LSHIFT R7 BY R6.L || [SP] = R5;
                            // Compute ...
    R7 = 0x10;
    R6.L = R5.L - R7.L (NS) || [SP + 4] = R6;
                            // [SP + 4]  -> 2^(search_area_factor + 1) 
    R6 = R5 >>> 4 || [SP + 28] = R6;
                            // [SP + 28] -> (No. of columns - 16) 
    R6.L =  R4.L - R7.L(NS) || [SP + 32] = R6;
                            // [SP + 32] -> (No. of columns)/16 
    R6 = R4 >>> 4 || [SP + 36] = R6;
                            // [SP + 36] -> (N0. of rows - 16) 
    [SP + 40] = R6;         // [SP + 40] -> (No. of Rows)/16
    
/*****************************  NORTH WEST ************************************
This code segment computes the motion vectors along the horizontal and vertical 
directions for the top-left corner macro block. The motion vectors along both 
the directions for this macro block cannot take negative values as such a value 
will shift the macro block out of the frame boundary.
*******************************************************************************/
    
    R0 = B1;
    [SP + 16] = R0;         // Starting address of the reference block
    [SP + 12] = R0;         // Location of the target block in the reference 
                            // frame
    R0 = B0;
    [SP + 8] = R0;          // Starting address of the target block
    R0 = [SP + 4];
    [SP + 20] = R0;
    [SP + 24] = R0;
    R0 = SP;                // Pass the address of the structure to the function
    R1 = P3;
    
    
    CALL __mve_core;        // Finds the best matching macro block and computes 
                            // the motion vectors
    P0 = B2;                                    
    P1 = B3;                                    
    B[P0++] = R0;           // Store motion vector along vertical direction
    B[P1++] = R1;           // Store motion vector along horizontal direction
    
/***************************** NORTH EAST *************************************
This code segment computes the motion vectors along the horizontal and vertical 
directions for the top-right corner macro block. For this macro block, the 
motion vector along the vertical direction is always non-negative while along 
the horizontal direction it is negative or zero thus giving a best match for the
target macro block within the frame boundary.
*******************************************************************************/
    
    R6 = [SP + 4];
    R6 += 1;
    R1 = B0;
    R0 = R1 + R5 (NS) || [SP + 20] = R6;
    R0 += -16;
    R0 = R0 - R1 (NS) || [SP + 8] = R0;
                            // Starting address of the target block 
    R1 = B1;                    
    R0 = R0 + R1 (NS) || R6 = [SP + 4]; 
    R0 = R0 - R6 (NS) || [SP + 12] = R0;
                            // Location of the target block in the reference 
                            // frame 
    [SP + 16] = R0;         // Starting address of the reference block
    R0 = SP;
    R1 = P3;
    CALL __mve_core;        // Finds the best matching macro block and computes 
                            // the motion vectors
    R6 = B2;                                    
    R7 = R5 >> 4;
    R7 += -1;
    R6 = R6 + R7;
    P0 = R6;
    R6 = B3;
    R6 = R6 + R7;
    P1 = R6;
    B[P0++] = R0;           // Store motion vector along vertical direction
    R0 = 1;
    R6 = [SP + 4];
    B[P1++] = R1;           // Store motion vector along horizontal direction
    
    
/******************************** SOUTH WEST **********************************
The motion vector along the horizontal and vertical directions for the bottom-
left corner macro block is computed by this code segment. The motion vector 
along the vertical direction is negative or zero while the one along the 
horizontal direction is non-negative for this macro block.
******************************************************************************/
    
    R6 = R6 + R0 (NS) || [SP + 20] = R6;
    R0 = R0 << 4 || [SP + 24] = R6;
    R1 = R4 - R0;
    R0 = B0;
    R3 = B1;
    R1 = R1.L*R5.L (IS);
    R2 = R0 + R1;
    R0 = R3 + R1 (NS) || R1 = [SP + 4];
    R1 = R1.L*R5.L (IS) || [SP + 8] = R2;
                            // Starting address of the target block; 
    R0 = R0 - R1 (NS) || [SP + 12] = R0;
                            // Location of the target block in the reference 
                            // frame 
    [SP + 16] = R0;         // Starting address of the reference block
    R0 = SP;
    R1 = P3;
    CALL __mve_core;        // Finds the best matching macro block and computes 
                            // the motion vectors
    R2 = [SP + 36];
    R2 = R2 >>> 4 || R3 = [SP + 32];
    R6 = B2;
    R7 = R2.L*R3.L (IS);
    R6 = R6 + R7;
    P0 = R6;
    R6 = B3;
    R6 = R6 + R7;
    P1 = R6;
    B[P0++] = R0;           // Store motion vector along vertical direction
    R6 = [SP + 4];
    R6 += 1;
    B[P1++] = R1;           // Store motion vector along horizontal direction
    
/******************************* SOUTH EAST ***********************************
The motion vectors for the bottom-right corner macro block in the target frame 
is computed by this code-segment. Both the vectors, i.e., along the horizontal 
and vertical directions are negative or zero to avoid the crossing of frame 
boundary.
*******************************************************************************/
    
    [SP + 20] = R6;
    [SP + 24] = R6;
    R1 = [SP + 36];
    R1 = R1.L*R5.L (IS) || R0 = [SP + 28];
    R1 = R0 + R1;
    R0 = B0;
    R2 = R0 + R1 (NS) || R3 = [SP + 4];
    R0 = B1;
    R0 = R0 + R1 (NS) || [SP + 8] = R2;
                            // Starting address of the target block 
    R1 = R3.L * R5.L (IS) || [SP + 12] = R0;
                            // Location of the target block in the reference 
                            // frame 
    R0 = R0 - R1;
    R0 = R0 - R3;
    [SP + 16] = R0;         // Starting address of the reference block
    R0 = SP;
    R1 = P3;
    CALL __mve_core;        // Finds the best matching macro block and computes 
                            // the motion vectors
    R6 = [SP + 40];                             
    R7 = [SP + 32];
    R7.L = R7.L*R6.L (IS);
    R7 += -1;
    R6 = B2;
    R6 = R6 + R7;
    P0 = R6;
    R6 = B3;
    R6 = R6 + R7;
    P1 = R6;
    B[P0++] = R0;           // Store motion vector along Y direction
    R3 = 16;
    P4 = B3;
    B[P1++] = R1;           // Store motion vector along X direction
    
/***************************** NORTH *****************************************
The motion vectors for the macro blocks in the first row, except the first and 
last ones are computed by this code segment. The motion vector along the 
vertical direction is non-negative and in the horizontal direction it can be any
integer within the allowed search range which is decided by the 
search_area_factor.
*******************************************************************************/

    P4 += 1;                // Address calculation for storing the X motion 
                            // vector
    P5 = B2;
    P5 += 1;                // Address calculation for storing the Y motion 
                            // vector
    R0 = B0;
    R1 = R0 + R3 (NS) || R6 = [SP + 4];
    R0 = B1;
    R0 = R0 + R3 (NS) || [SP + 8] = R1;
                            // Starting address of the target block 
    R0 = R0 - R6 (NS) || [SP + 12] = R0;
                            // Location of the target block in the reference 
                            // frame 
    [SP + 16] = R0;         // Starting address of the reference block
    R0 = R6 << 1 || R7 = [SP + 32];
    [SP + 20] = R0;
    [SP + 24] = R6;
    R7 += -2;
LOOP1:
    CC = R7 <= 0;           // Loop for the first row target macro blocks
    IF CC JUMP LOOP1_OVER;
    R0 = SP;
    R1 = P3;
    CALL __mve_core;        // Finds the best matching macro block and computes 
                            // the motion vectors
    R2 = [SP + 8];
    R2 += 16;
    [SP + 8] = R2;
    R2 = [SP + 12];
    R2 += 16;
    [SP + 12] = R2;
    R2 = [SP + 16];
    R2 += 16;
    [SP + 16] = R2;
    B[P5++] = R0;           // Store motion vector along Y direction
    B[P4++] = R1;           // Store motion vector along X direction
    R7 += -1;
    JUMP LOOP1;
    
/******************************* SOUTH ****************************************
The motion vectors for the macro blocks in the last row, except the first and 
last ones are computed by this code segment. The motion vector along the 
vertical direction is negative or zero and in the horizontal direction it can be
any integer within the allowed search range which is decided by the 
search_area_factor.
*******************************************************************************/
LOOP1_OVER:
    R0 = [SP + 40];
    R0 += -1;
    R1 = [SP + 32];
    R1 = R1.L*R0.L (IS);
    R1 += 1;
    R0 = B2;
    R0 = R0 + R1;           // Address calculation for storing the Y motion 
                            // vector
    P5 = R0;
    R0 = B3;
    R0 = R0 + R1 (NS) || R1 = [SP + 36];
                            // Address calculation for storing the X motion 
                            // vector 
    P4 = R0;
    R0 = B0;
    R1 = R1.L*R5.L (IS) || R6 = [SP + 4];
    R1 += 16;
    R2 = R0 + R1; 
    R0 = B1;
    R0 = R0 + R1 (NS) || [SP + 8] = R2;
                            // Starting address of the target block 
    R0 = R0 - R6 (NS) || [SP + 12] = R0;
                            // Location of the target block in the reference 
                            // frame 
    R1 = R6.L*R5.L (IS) || R7 = [SP + 32];
    R0 = R0 - R1;
    R0 = R6 << 1 || [SP + 16] = R0;
                            // Starting address of the reference block 
    [SP + 20 ] = R0;
    R6 += 1;
    [SP + 24] = R6;
    R7 += -2;
LOOP2:
    CC = R7 <= 0;           // Loop for the last row target macro blocks
    IF CC JUMP LOOP2_OVER;
    R0 = SP;
    R1 = P3;
    CALL __mve_core;        // Finds the best matching macro block and computes 
                            // the motion vectors
    R2 = [SP + 8];
    R2 += 16;
    [SP + 8] = R2;
    R2 = [SP + 12];
    R2 += 16;
    [SP + 12] = R2;
    R2 = [SP + 16];
    R2 += 16;
    [SP + 16] = R2;
    B[P5++] = R0;           // Store motion vector along Y direction
    B[P4++] = R1;           // Store motion vector along X direction
    R7 += -1;
    JUMP LOOP2;
/************************************ WEST ************************************
The motion vectors for the macro blocks in the first column, except the first 
and last ones are computed by this code segment. The motion vector along the 
horizontal direction is non-negative and in the vertical direction it can be any
integer within the allowed search range which is decided by the 
search_area_factor.
******************************************************************************/

LOOP2_OVER:     
    R0 = B0;
    R1 = R5 << 4 || R6 = [SP + 4];
    R2 = R0 + R1 (NS) || [SP + 20] = R6;
    R0 = B1;
    R0 = R0 + R1 (NS) || [SP + 8] = R2;
                            // Starting address of the target block 
    R3 = R6.L*R5.L (IS) || [SP + 12] = R0;
                            // Location of the target block in the reference 
                            // frame 
    R6 = R6 << 1 || R1 = [SP + 32];
    R2 = R0 - R3 (NS) || [SP + 24] = R6;
    R0 = B2;
    R0 = R0 + R1 (NS) || [SP + 16] = R2;
                            // Starting address of the reference block 
    P5 = R0;                // Address calculation for storing the Y motion 
                            // vector
    R0 = B3;
    R0 = R0 + R1 (NS) || R7 = [SP + 40];
    P4 = R0;                // Address calculation for storing the X motion 
                            // vector
    R7 += -2;
LOOP3:
    CC = R7 <= 0;           // Loop for the first column target macro blocks
    IF CC JUMP LOOP3_OVER;
    R0 = SP;
    R1 = P3;
    CALL __mve_core;        // Finds the best matching macro block and computes
                            // the motion vectors
    R3 = [SP + 32];
    P0 = R3;
    R3 = R5 << 4 || R6 = [SP + 8];
    R2 = R6 + R3 (NS) || R6 = [SP + 12];
    R2 = R6 + R3 (NS) || [SP + 8] = R2;
    R6 = [SP + 16];
    R2 = R6 + R3 (NS) || [SP + 12] = R2;
    [SP + 16] = R2;
    B[P5] = R0;             // Store motion vector along Y direction
    B[P4] = R1;             // Store motion vector along X direction
    P5 = P5 + P0;
    P4 = P4 + P0;
    R7 += -1;
    JUMP LOOP3;
    
/*************************** EAST *********************************************
The motion vectors for the macro blocks in the last column, except the first and
last ones are computed by this code segment. The motion vector along the 
horizontal direction is negative or zero and in the vertical direction it can be
any integer within the allowed search range which is decided by the 
search_area_factor.
*******************************************************************************/
    
LOOP3_OVER:
    R0 = B0;
    R1 = R5 << 4 || R2 = [SP + 28];
    R1 = R1 + R2 (NS) || R6 = [SP + 4];
    R6 += 1;
    R2 = R0 + R1 (NS) || [SP + 20] = R6;
    R6 += -1;
    R0 = B1;
    R0 = R0 + R1 (NS) || [SP + 8] = R2;
                            // Starting address of the target block 
    R3 = R6.L*R5.L (IS) || [SP + 12] = R0;
                            // Location of the target block in the reference 
                            // frame 
    R3 = R3 + R6 (NS);
    R6 = R6 << 1 || R1 = [SP + 32];
    R2 = R0 - R3 (NS) || [SP + 24] = R6;
    R0 = B2;
    R1 <<= 1;
    R1 += -1;
    R0 = R0 + R1 (NS) || [SP + 16] = R2;
                            // Starting address of the reference block 
    P5 = R0;                // Address calculation for storing the Y motion 
                            // vector
    R0 = B3;
    R0 = R0 + R1 (NS) || R7 = [SP + 40];
    P4 = R0;                // Address calculation for storing the X motion 
                            // vector
    R7 += -2;
LOOP4:
    CC = R7 <= 0;           // Loop for the last column target macro blocks
    IF CC JUMP LOOP4_OVER;
    R0 = SP;
    R1 = P3;
    CALL __mve_core;        // Finds the best matching macro block and computes 
                            // the motion vectors
    R3 = [SP + 32];
    P0 = R3;
    R3 = R5 << 4 || R6 = [SP + 8];
    R2 = R6 + R3 (NS) || R6 = [SP + 12];
    R2 = R6 + R3 (NS) || [SP + 8] = R2;
    R6 = [SP + 16];
    R2 = R6 + R3 (NS) || [SP + 12] = R2;
    [SP + 16] = R2;
    B[P5] = R0;             // Store motion vector along Y direction
    B[P4] = R1;             // Store motion vector along X direction
    P5 = P5 + P0;
    P4 = P4 + P0;
    R7 += -1;
    JUMP LOOP4;
/******************************** CENTRAL *************************************
The motion vectors for the macro blocks, other than those in the boundary of the
frame are computed by this code segment. The motion vector along the horizontal 
and verical directions can be any integer within the allowed search range which 
is decided by the search_area_factor.
*******************************************************************************/
    
LOOP4_OVER:     
    R0 = B0;
    R1 = R5 << 4 || R6 = [SP + 4];
    R1 += 16;
    R2 = R0 + R1 (NS) || R4 = [SP + 40];
    R0 = B1;
    R0 = R0 + R1 (NS) || [SP + 8] = R2;
                            // Starting address of the target block 
    R3 = R6.L*R5.L (IS) || [SP + 12] = R0;
                            // Location of the target block in the reference 
                            // frame 
    R3 = R3 + R6 (NS);
    R6 = R6 << 1 || R1 = [SP + 32];
    R2 = R0 - R3 (NS) || [SP + 24] = R6;
    R0 = B2;
    R1 += 1;
    R0 = R0 + R1 (NS) || [SP + 16] = R2;
                            // Starting address of the reference block 
    P5 = R0;                // Address calculation for storing the Y motion 
                            // vector
    R0 = B3;
    R0 = R0 + R1 (NS) || [SP + 20] = R6;
    P4 = R0;                // Address calculation for storing the X motion 
                            // vector
    R4 += -2;
NEXT:
    CC = R4 <= 0;           // Loop for the central target macro blocks
    IF CC JUMP LOOP5_OVER;
    R7 = [SP + 32];
    R7 += -2;
LOOP5:
    CC = R7 <= 0;
    IF CC JUMP ROW_OVER;
    R0 = SP;
    R1 = P3;
    CALL __mve_core;        // Finds the best matching macro block and computes 
                            // the motion vectors
    R0 = R0 - R0 (NS) || [P3 + 32] = R5;
    [P3 + 24] = R0;
    [P3 + 28] = R0;
    R0 = [SP + 8];
    R1 = [SP + 4];
    R1 = R1 << 1 || [P3 + 36] = R0;
    [P3 + 40] = R1;
    R0 = P3;
    CALL _hpel;             // Call the half-pixelation routine to give a half-
                            // pixel correction to the integer MVs
    R2 = [SP + 8];
    R2 += 16;
    [SP + 8] = R2;
    R2 = [SP + 12];
    R2 += 16;
    [SP + 12] = R2;
    R2 = [SP + 16];
    R2 += 16;
    [SP + 16] = R2;
    B[P5++] = R0;           // Store motion vector along Y direction
    B[P4++] = R1 || NOP;    // Store motion vector along X direction
    R7 += -1;
    JUMP LOOP5;
ROW_OVER:
    R3 = R5 << 4 || R0 = [SP + 28];
    R2 = R3 - R0 (NS) || R6 = [SP + 8];
    R2 += 16;
    R6 = R6 + R2 (NS) || R0 = [SP + 12];
    R0 = R0 + R2 (NS) || [SP + 8] = R6;
    R3 = [SP + 16];
    R3 = R3 + R2 (NS) || [SP + 12] = R0;
    [SP + 16] = R3;
    P5 += 2;
    P4 += 2;
    R4 += -1;
    JUMP NEXT;
LOOP5_OVER: 
    SP += 52;
    SP += 52;
    (R7:4, P5:3) = [SP++];
    RETS = [SP++];
    RTS;
    NOP;

__motionvect.end:

.text
.align 8;
.global _hpel;
.extern __hpel_core2;
.extern __hpel_core1;
    
_hpel:

    [--SP] = (R7:4, P5:4);
    P4 = R0;
    [--SP] = RETS;
    P0 = 260;
    SP -= P0;
    R7 = SP;                // Address of the first temporary buffer
    SP -= P0;
    P5 = SP;                // Address of the second temporary buffer
    R0 = R0 - R0 (NS) || R6 = [P4 + 40];
    R6 = R0 - R6 (NS) || R4 = [P4 + 8];
    R5 = [P4 + 12];
    CC = R5 == R6;
    IF CC JUMP BRANCH1;
    
/********************* HOR_OFFSET = -1    VER_OFFSET = 0 *******************/
    
    R0 = -1;
    R0 = R0 - R0 (NS) || [P4 + 20] = R0;
    [P4 + 16] = R0;
    R0 = P4;
    R1 = R7;
    CALL __hpel_core1;      // Averages and stores into buffer
                            // Finds MAD between averaged block(buffer1) and 
                            // target block
BRANCH1:
    CC = R4 == R6;          // If Vertical-offset == R6, skip 2nd,3rd,4th hpel 
                            // estimations(with Y = -1)
    IF CC JUMP BRANCH4;
    CC = R5 == R6;
    IF CC JUMP BRANCH2;
    
/****************************  HOR_OFFSET = -1   VER_OFFSET = -1 **************/
    
    R0 = -1;
    [P4 + 20] = R0;
    [P4 + 16] = R0;
    R0 = P4;
    R1 = R7;
    R2 = P5;
    CALL __hpel_core2;      // Averages and stores into buffer
    
/********************  HOR_OFFSET = 0    VER_OFFSET = -1 **********************/
BRANCH2:
    R0 = -1;
    R0 = R0 - R0 (NS) || [P4 + 16] = R0;
    [P4 + 20] = R0;
    R0 = P4;
    R1 = R7;
    CALL __hpel_core1;      // Averages and stores into buffer
                            // Finds MAD between averaged block(buffer1) and 
                            // target block
    
/************************  HOR_OFFSET = 1    VER_OFFSET = -1 *****************/
    
    R0 = -1;
    R0 = R0.L * R0.L (IS) || [P4 + 16] = R0;
    [P4 + 20] = R0;
    R0 = P4;
    R1 = R7;
    R2 = P5;
    CALL __hpel_core2;      // Averages and stores into buffer
    
/**********************  HOR_OFFSET = 1 VER_OFFSET = 0 ***********************/
    
BRANCH4:
    R0 = 1;
    R0 = R0 - R0 (NS) || [P4 + 20] = R0;
    [P4 + 16] = R0;
    R0 = P4;
    R1 = R7;
    CALL __hpel_core1;      // Averages and stores into buffer
                            // Finds MAD between averaged block(buffer1) and 
                            // target block
    
/**************************  HOR_OFFSET = 1     VER_OFFSET = 1 ****************/
    R0 = 1;
    [P4 + 16] = R0;
    [P4 + 20] = R0;
    R0 = P4;
    R1 = R7;
    R2 = P5;
    CALL __hpel_core2;      // Averages and stores into buffer
    
    
/***************************  HOR_OFFSET = 0    VER_OFFSET = 1 ***************/
    R0 = 1;
    R0 = R0 - R0 (NS) || [P4 + 16] = R0;
    [P4 + 20] = R0;
    R0 = P4;
    R1 = R7;
    CALL __hpel_core1;      // Averages and stores into buffer
                            // Finds MAD between averaged block(buffer1) and 
                            // target block
/*************************  HOR_OFFSET = -1   VER_OFFSET = 1 ******************/
    CC = R5 == R6;          // If Horizontal-offset == R6, skip last hpel 
                            // estimation(X = -1, Y = 1)
    IF CC JUMP BRANCH8;             
    R0 = -1;
    R0 = R0.L * R0.L (IS) || [P4 + 20] = R0;
    [P4 + 16] = R0;
    R0 = P4;
    R1 = R7;
    R2 = P5;
    CALL __hpel_core2;      // Averages and stores into buffer
    
BRANCH8:
    R1 = [P4 + 12];         // Horizontal-vector before half pixel estimation
    R2 = [P4 + 28];
    R1 = R1 + R2 (NS) || R0 = [P4 + 8];
                            // Half pixel estimation for Horizontal-vector added
                            // to X-vector 
    R2 = [P4 + 24];
    R0 = R0 + R2;           // Half pixel estimation for Vertical-vector added 
                            // to Y-vector
    P0 = 260;
    SP = SP + P0;
    SP = SP + P0;
    RETS = [SP++];                  
    (R7:4, P5:4) = [SP++];  // Retrieve call save register
    RTS;
    NOP;
_hpel.end:.text
.align 8;
.global __hpel_core1;
__hpel_core1:

    L0 = 0;
    L1 = 0;
    L3 = 0;
    P1 = R0;
    I3 = R1;
    I2 = R1;
    [--SP] = R7;
    MNOP || R1 = [P1 + 16]; // Current hpel offset for the vertical motion 
                            // vector
    MNOP || R3 = [P1 + 32]; // Horizontal size of the video frame
    R1 = R1.L * R3.L (IS) || R0 = [P1 + 4];
                            // R0 -> Address of the block having least MAD 
    R2 = R0 + R1 (NS) || R1 = [P1 + 20];
                            // R1 -> Current hpel offset for the horizontal 
                            // motion vector 
    I0 = R0;                                
    R2 = R2 + R1;
    I1 = R2;
    P2 = 16;                // Loop ctr(for 16 rows) is initialized
    R3 += -16;
    M1 = R3;                // COLUMNS - 16
    R3 += 4;
    M3 = R3;                // COLUMNS - 12
    
/************************** AVERAGE ADJACENT BLOCKS *************************/
    LSETUP(ST_LOOP,END_LOOP) LC0=P2;
    DISALGNEXCPT || R0 = [I0++] || R2  =[I1++];
                            // Fetch 1st words of the two blocks(if disligned, 
                            // contains partial data
ST_LOOP:
        DISALGNEXCPT || R1 = [I0++] || R3  =[I1++];
                            // Fetch 2nd words(will contain remaining part of 
                            // 1st word) 
        R7 = BYTEOP1P(R1:0,R3:2) || R0 = [I0++] || R2  =[I1++];
                            // Average(R0,R2) and fetch 3rd word 
        R7 = BYTEOP1P(R1:0,R3:2)(R) || R1 = [I0++] || [I3++] = R7 ;
                            // Average(R1,R3), fetch 4th data and store previous
                            // result 
        DISALGNEXCPT  || R3  =[I1++] || [I3++] = R7;
                            // Fetch 4th data and store previous result 
        R7 = BYTEOP1P(R1:0,R3:2) || R0 = [I0++M1] || R2  =[I1++M1];
                            // Average (R0,R2), fetch 5th word and modify 
                            // pointers 
        R7 = BYTEOP1P(R1:0,R3:2)(R) || R0 = [I0++] || [I3++] = R7 ;
                            // Average (R1,R3), fetch 1st word of next row and 
                            // store 
END_LOOP:
        DISALGNEXCPT || R2  =[I1++] || [I3++] = R7;
                            // Fetch 1st word of next row and store previous 
                            // result 
    
/************************* MEAN ABSOLUTE DIFFERENCE ***************************/
    R0 = [P1 + 36];
    I0 = R0;                // Fetch the start address of the target block
    I1 = I2;                // Fetch the start address of the reference block
    A1=A0=0 || R0 = [I0++] || R2 = [I1++];
                            // Initialize accumulators to zero and fetch the 
                            // first data from the two blocks 

    LSETUP (MAD_START, MAD_END) LC0=P2;
MAD_START:
        SAA (R1:0,R3:2) || R1 = [I0++]  || R3 = [I1++];
                            // Compute absolute difference and accumulate 
        SAA (R1:0,R3:2) (R) || R0 = [I0++] || R2 = [I1++];
                            //                  | 
        SAA (R1:0,R3:2) || R1 = [I0 ++ M3] || R3 = [I1++];
                            //  After fetch of 4th word of target blk, pointer 
                            // made to point next row 
MAD_END:
        SAA (R1:0,R3:2) (R) || R0 = [I0++] || R2 = [I1++];
                            //                  | 
    R3=A1.L+A1.H,R2=A0.L+A0.H;    
    R0 = R2 + R3 (NS) || R3 = [P1];
                            // Add the accumulated values in both MACs 
    
/*************************** MINIMUM MAD COMPUTATION ***********************/
    
    CC = R0 <= R3;          // Check if the latest MAD or MSE is less than the 
                            // previous ones
    IF CC JUMP LESS_OR_EQUAL;
                            // If latest MAD is not lesser, then return 
    R7 = [SP++];
    RTS;
LESS_OR_EQUAL:
    CC = R0 < R3;                 
    IF !CC JUMP EQUAL;      // If MAD is lesser jump to 'LESS'
    [P1] = R0;              // If latest MAD is less, then store it as the 
                            // minimum MAD
    R0 = [P1 + 16];
    [P1 + 24] = R0;
    R0 = [P1 + 20];
    [P1 + 28] = R0;
    R7 = [SP++];
    RTS;
EQUAL:
    R2 = [P1 + 12];         // Horizontal-vector before half pixel estimation
    R1 = [P1 + 28];
    R1 = R2 + R1 (NS) || R3 = [P1 + 8];
                            // Half pixel estimation for Horizontal-vector added
                            // to X-vector 
    R0 = [P1 + 24];
    R0 = R3 + R0;           // Half pixel estimation for Vertical-vector added 
                            // to Y-vector
    A1 = R1.L*R1.L (IS) || R1 = [P1 + 20];
    A1 += R0.L*R0.L (IS) || R0 = [P1 + 16];
                            // Distance to previous best match from reference 
                            // block 
    R0 = R0 + R3;
    R1 = R1 + R2;
    A0 = R0.L*R0.L (IS);    // Distance to current best match from reference 
                            // block
    A0 += R1.L*R1.L (IS);
    CC = A0 < A1;
    IF !CC JUMP FINISH;
    R0 = [P1 + 16];
    [P1 + 24] = R0;
    R0 = [P1 + 20];
    [P1 + 28] = R0;
FINISH:
    R7 = [SP++];
    RTS;        
__hpel_core1.end:    

.text
.align 8;
.global __hpel_core2;
__hpel_core2:

    [--SP] = R7;
    L0 = 0;
    L1 = 0;
    L3 = 0;
    P1 = R0;
    M0 = R1;
    I3 = R2;
    I2 = R2;
    R1 = [P1 + 16]; // Current hpel offset for the vertical motion 
                            // vector
    R3 = [P1 + 32]; // Horizontal size of the video frame
    R2 = R1.L * R3.L (IS) || R0 = [P1 + 4];
                            // Address of the block having least MAD 
    R2 = R0 + R2 (NS) || R7 = [P1 + 20];
                            // Current hpel offset for the horizontal motion 
                            // vector 
    R2 = R2 + R7 (S);
    I1 = R2;
    R1 = R1 + R7 (S);           // If (VMV + HMV) == 0
    R2 = 1;                 //     address_offset = curr_hor_mv_off
    CC = R1 == 0;           // else
    IF !CC R2 = R3;         //     address_offset = curr_hor_mv_off*hor_size
    P2 = 16;                // Loop ctr(for 16 rows) is initialized
    R1 = R2.L * R7.L (IS);
    R0 = R0 + R1;
    I0 = R0;
    R3 += -16;
    M1 = R3;
    R3 += 4;
    M3 = R3;
    
/***************** AVERAGE ADJACENT BLOCKS ************************************/
    LSETUP(LOOP_ST,LP_END) LC0=P2;
    DISALGNEXCPT || R0 = [I0++] || R2  =[I1++];
                            // Fetch 1st words of the two blocks(if disligned, 
                            //contains partial data
LOOP_ST:
        DISALGNEXCPT || R1 = [I0++] || R3  =[I1++];
                            // Fetch 2nd words(will contain remaining part of 
                            // 1st word) 
        R7 = BYTEOP1P(R1:0,R3:2) || R0 = [I0++] || R2  =[I1++];
                            // Average(R0,R2) and fetch 3rd word 
        R7 = BYTEOP1P(R1:0,R3:2)(R) || R1 = [I0++] || [I3++] = R7 ;
                            // Average(R1,R3), fetch 4th data and store previous
                            // result 
        DISALGNEXCPT  || R3  =[I1++] || [I3++] = R7;
                            // Fetch 4th data and store previous result 
        R7 = BYTEOP1P(R1:0,R3:2) || R0 = [I0++M1] || R2  =[I1++M1];
                            // Average (R0,R2), fetch 5th word and modify 
                            // pointers 
        R7 = BYTEOP1P(R1:0,R3:2)(R) || R0 = [I0++] || [I3++] = R7 ;
                            // Average (R1,R3), fetch 1st word of next row and 
                            // store 
LP_END: DISALGNEXCPT || R2  =[I1++] || [I3++] = R7;
                            // Fetch 1st word of next row and store previous 
                            // result 
/*********************** AVERAGE CORNER BLOCKS & COMPUTE MAD *****************/
    
    I0 = I2;
    I1 = M0;
    R0 = [P1 + 36];
    I3 = R0;                // Fetch the start address of the target block
    A1=A0=0 || R0 = [I0++] || R2 = [I1++];
                            // Initialize accumulators for MAC and fetch first 
                            // data
    MNOP;
    
    LSETUP(CAVG_ST,CAVG_END) LC0 = P2;
CAVG_ST:
        R1 = BYTEOP1P(R1:0,R3:2) || R3 = [I3++] || R2 = [I1++];
                            // Average and fetch data from tgt blk 
        SAA (R1:0,R3:2) (R) || R0 = [I0++];
                            // MAD and fetch data to be averaged from buffers 
        R1 = BYTEOP1P(R1:0,R3:2) || R3 = [I3++] || R2 = [I1++];
                            // Average and fetch data from tgt blk 
        SAA (R1:0,R3:2) (R) || R0 = [I0++];
                            // MAD and fetch data to be averaged from buffers 
        R1 = BYTEOP1P(R1:0,R3:2) || R3 = [I3++] || R2 = [I1++];
                            // Average and fetch data from tgt blk 
        SAA (R1:0,R3:2) (R) || R0 = [I0++];
                            // MAD and fetch data to be averaged from buffers 
        R1 = BYTEOP1P(R1:0,R3:2) || R3 = [I3++M3] || R2 = [I1++];
                            // Average and fetch data from tgt blk and modify 
                            // pointer 
CAVG_END:
        SAA (R1:0,R3:2) (R) || R0 = [I0++];
                            // MAD and fetch data from next row to be averaged 
                            // from buffers 
    R3=A1.L+A1.H,R2=A0.L+A0.H;    
    R0 = R2 + R3 (NS) || R3 = [P1];
                            // Add the accumulated values in both MACs 
    
/******************** MINIMUM MAD COMPUTATION ******************************/
    
    CC = R0 <= R3;          // Check if the latest MAD or MSE is less than the 
                            // previous ones
    IF CC JUMP LESS_OR_EQUAL1;
                            // If latest MAD is not lesser, then return 
    R7 = [SP++];
    RTS;
LESS_OR_EQUAL1:
    CC = R0 < R3;                 
    IF !CC JUMP EQUAL1;      // If MAD is lesser jump to 'LESS'
    [P1] = R0;              // If latest MAD is less, then store it as the 
                            // minimum MAD
    R0 = [P1 + 16];
    [P1 + 24] = R0;
    R0 = [P1 + 20];
    [P1 + 28] = R0;
    R7 = [SP++];
    RTS;
EQUAL1:
    R2 = [P1 + 12];         // Horizontal-vector before half pixel estimation
    R1 = [P1 + 28];
    R1 = R2 + R1 (NS) || R3 = [P1 + 8];
                            // Half pixel estimation for Horizontal-vector added
                            // to X-vector 
    R0 = [P1 + 24];
    R0 = R3 + R0;           // Half pixel estimation for Vertical-vector added 
                            // to Y-vector
    A1 = R1.L*R1.L (IS) || R1 = [P1 + 20];
    A1 += R0.L*R0.L (IS) || R0 = [P1 + 16];
                            // Distance to previous best match from reference 
                            // block 
    R0 = R0 + R3;
    R1 = R1 + R2;
    A0 = R0.L*R0.L (IS);    // Distance to current best match from reference 
                            // block
    A0 += R1.L*R1.L (IS);
    CC = A0 < A1;
    IF !CC JUMP FINISH1;
    R0 = [P1 + 16];
    [P1 + 24] = R0;
    R0 = [P1 + 20];
    [P1 + 28] = R0;
FINISH1:
    R7 = [SP++];
    RTS;       
__hpel_core2.end:     

.text
.align 8;
.global __mve_core;
__mve_core:

    [--SP] = (R7:5, P5:3);
    L0 = 0;
    L1 = 0;
    L3 = 0;
    P5 = R0;
    I3 = R1;
    P3 = 16;                // Loop ctr(for 16 rows) initialized
    R7.L = -1;              // Initialize R7 to the positive maximum number
    R7.H = 0x7FFF;          // R7 contains the minimum MAD or MSE
    R0 = [P5];              // Horizontal size of the frame
    R1 = R0 << 4 || P1 = [P5 + 16];
                            // Starting address from where subroutine must start
                            // searching for matching blocks 
    M2 = R1;                // 16*Horizontal size
    R0 += -16;
    M1 = R0;                // Horizontal size - 16
    R0 += 4;
    M3 = R0;                // Horizontal size - 12
    MNOP || R1 = [P5 + 8];
    I0 = R1;                // Starting address of target block
    MNOP || R2 = [P5 + 20];
    P4 = R2;                // Horizontal search range
    R6 = [P5 + 24] || NOP;  // Vertical search range
    P2 = 0;
VERTICAL:
    P1 = P1 + P2;           // Update the reference block row address
    P2 = P1;                
    LSETUP(ST_SEARCH, END_SEARCH) LC1 = P4;
                            // Set horizontal span of the search range 
ST_SEARCH:
        I1 = P2;            // Fetch the start address of the reference block
/*********************** MEAN ABSOLUTE DIFFERENCE *****************************/
    
        A1=A0=0;            // Initialize accumulators for accumulation
        DISALGNEXCPT || R0 = [I0++] || R2 = [I1++];
                            // Fetch the first data from the two blocks 
        LSETUP (MAD_START2, MAD_END2) LC0=P3;
MAD_START2:  DISALGNEXCPT || R3 = [I1++];
            SAA (R1:0,R3:2) || R1 = [I0++]  || R2 = [I1++];
                            // Compute absolute difference and accumulate 
            SAA (R1:0,R3:2) (R) || R0 = [I0++] || R3 = [I1++];
                            //                  | 
            SAA (R1:0,R3:2) || R1 = [I0 ++ M3] || R2 = [I1++M1];
                            //  After fetch of 4th word of target blk, pointer 
                            // made to point next row 
MAD_END2:    SAA (R1:0,R3:2) (R) || R0 = [I0++] || R2 = [I1++];

        R3=A1.L+A1.H,R2=A0.L+A0.H || I0 -= M2;    
        R0 = R2 + R3 (NS) || I0 -= 4;
                            // Add the accumulated values in both MACs 
    
    
/*************************** MINIMUM MAD COMPUTATION **************************/
        CC = R0 <= R7;      // Check if the latest MAD or MSE is less than the 
                            // previous ones
        IF !CC JUMP END_SEARCH (BP);
                            // If latest MAD is not lesser, then return 
        CC = R0 < R7;                           
        IF !CC JUMP EQUAL2;  // If MAD is lesser jump to 'LESS'
        R7 = R0;            // If latest MAD is less, latest MAD or MSE is the 
                            // minimum
        P0 = P2;            // and corresponding block is better match
        JUMP END_SEARCH;
EQUAL2:                      // Compute the MVs for the least, previous MAD
        R1 = P0;
        R0 = R0 - R0 (NS) || R2 = [P5 + 12];
        R3 = R1 - R2 (NS) || R5 = [P5];
                            // Find the difference between current block and 
                            // reference block 
        R1 = ABS R3 || R2 = [P5 + 4];
                            // Take the absolute value of the difference 
REPEAT1:
        CC = R1 < R5;
        IF CC JUMP FINISH_MOD1;                 
        R1 = R1 - R5;       // Divide the offset by COLUMNS
        R0 += 2;            // R0 has twice the quotient and R1 has the 
                            // remainder
        JUMP REPEAT1;
FINISH_MOD1:
        CC = R1 <= R2;
        IF CC JUMP NOTRANS1;
        R0 += 2;            // If remainder is greater than that value, 
                            // increment quotient by two
        R1 = R1 - R5;       // and remainder = remainder - COLUMNS
NOTRANS1:
        R1 <<= 1;           // Remainder*2 gives horizontal motion vector
        R2 = -R0;
        R5 = -R1;
        CC = R3 < 0;        // If R3 is negative (diff. negative), negate both 
                            // quotient and remainder
        IF CC R0 = R2;                          
        IF CC R1 = R5;                          
        A1 = R0.L*R0.L (IS);// Distance to previous best match from reference 
                            // block
        A1 += R1.L*R1.L (IS);
    
                            // Compute the MVs for the least, current MAD
        R1 = P2;
        R0 = R0 - R0 (NS) || R2 = [P5 + 12];
        R3 = R1 - R2 (NS) || R5 = [P5];
                            // Find the difference between current block and 
                            // reference block 
        R1 = ABS R3 || R2 = [P5 + 4];
                            // Take the absolute value of the difference
REPEAT2:
        CC = R1 < R5;
        IF CC JUMP FINISH_MOD2;                 
        R1 = R1 - R5;       // Divide the offset by COLUMNS
        R0 += 2;            // R0 has twice the quotient and R1 has the 
                            // remainder
        JUMP REPEAT2;
FINISH_MOD2:
        CC = R1 <= R2;
        IF CC JUMP NOTRANS2;
        R0 += 2;            // If remainder is greater than that value, 
                            // increment quotient by two
        R1 = R1 - R5;       // and remainder = remainder - COLUMNS
NOTRANS2:
        R1 <<= 1;           // Remainder*2 gives horizontal motion vector
        R2 = -R0;
        R5 = -R1;
        CC = R3 < 0;        // If R3 is negative (diff. negative), negate both 
                            // quotient and remainder
        IF CC R0 = R2;                          
        IF CC R1 = R5;                          
        A0 = R0.L*R0.L (IS);
        A0 += R1.L*R1.L (IS);
                            // Distance to current block from reference block 
    
        CC = A0 < A1;
        IF CC P0 = P2;      // Make current block as the best match till now
    
END_SEARCH:
        P2 += 1;            // Update the reference block column address
    P2 = [P5];              // Horizontal size of the video frame
    R6 += -1;
    CC = R6 <= 0;           // Check if the all the rows in the search area are 
                            // completed
    IF !CC JUMP  VERTICAL (BP);
                            // Repeat till the entire search area is matched 
    
                            // Compute the motion vectors
    R1 = P0;
    R0 = R0 - R0 (NS) || R2 = [P5 + 12];
    R3 = R1 - R2 (NS) || R5 = [P5];
                            // Find the difference between current block and 
                            // reference block 
    R1 = ABS R3 || R2 = [P5 + 4];
                            // Take the absolute value of the difference
REPEAT3:
    CC = R1 < R5;
    IF CC JUMP FINISH_MOD3;                 
    R1 = R1 - R5;           // Divide the offset by COLUMNS
    R0 += 2;                // R0 has twice the quotient and R1 has the 
                            // remainder
    JUMP REPEAT3;
FINISH_MOD3:
    CC = R1 <= R2;
    IF CC JUMP NOTRANS3;
    R0 += 2;                // If remainder is greater than that value, 
                            // increment quotient by two
    R1 = R1 - R5;           // and remainder = remainder - COLUMNS
NOTRANS3:
    R1 <<= 1;               // Remainder*2 gives horizontal motion vector
    R2 = -R0;
    R5 = -R1;
    CC = R3 < 0;            // If R3 is negative (diff. negative), negate both 
                            // quotient and remainder
    IF CC R0 = R2;                          
    IF CC R1 = R5;                          
    R3 = P0;
    [I3++] = R7;            // Store the least MAD in the output structure
    [I3++] = R3;            // Store the address of the least MAD in the output 
                            // structure
    [I3++] = R0;            // Store the integer vertical MV in the output 
                            // structure
    [I3++] = R1;            // Store the integer horizontal MV in the output 
                            // structure
    (R7:5, P5:3) = [SP++];          
    RTS;
    NOP;                    //to avoid one stall if LINK or UNLINK happens to be
                            //the next instruction after RTS in the memory.
__mve_core.end:
