.text;
.align 4;
.globl _longjmp;
.type _longjmp, STT_FUNC;
_longjmp:
	P0 = R0;
	R0 = [P0 + 0x00];
	[--SP] = R0;		/* Put P0 on the stack */
	
	P1 = [P0 + 0x04];
	P2 = [P0 + 0x08];
	P3 = [P0 + 0x0C];
	P4 = [P0 + 0x10];
	P5 = [P0 + 0x14];
	
	FP = [P0 + 0x18];
	R0 = [SP++];		/* Grab P0 from old stack */
	SP = [P0 + 0x1C];	/* Update Stack Pointer */
	[--SP] = R0;		/* Put P0 on new stack */
	[--SP] = R1;		/* Put VAL arg on new stack */

	R0 = [P0 + 0x20];	/* Data Registers */
	R1 = [P0 + 0x24];
	R2 = [P0 + 0x28];
	R3 = [P0 + 0x2C];
	R4 = [P0 + 0x30];
	R5 = [P0 + 0x34];
	R6 = [P0 + 0x38];
	R7 = [P0 + 0x3C];

	R0 = [P0 + 0x40];
	ASTAT = R0;

	R0 = [P0 + 0x44];	/* Loop Counters */
	LC0 = R0;
	R0 = [P0 + 0x48];
	LC1 = R0;

	R0 = [P0 + 0x4C];	/* Accumulators */
	A0.W = R0;
	R0 = [P0 + 0x50];
	A0.X = R0;
	R0 = [P0 + 0x54];
	A1.W = R0;
	R0 = [P0 + 0x58];
	A1.X = R0;

	R0 = [P0 + 0x5C];	/* Index Registers */
	I0 = R0;
	R0 = [P0 + 0x60];
	I1 = R0;
	R0 = [P0 + 0x64];
	I2 = R0;
	R0 = [P0 + 0x68];
	I3 = R0;

	R0 = [P0 + 0x6C];	/* Modifier Registers */
	M0 = R0;
	R0 = [P0 + 0x70];
	M1 = R0;
	R0 = [P0 + 0x74];
	M2 = R0;
	R0 = [P0 + 0x78];
	M3 = R0;

	R0 = [P0 + 0x7C];	/* Length Registers */
	L0 = R0;
	R0 = [P0 + 0x80];
	L1 = R0;
	R0 = [P0 + 0x84];
	L2 = R0;
	R0 = [P0 + 0x88];
	L3 = R0;

	R0 = [P0 + 0x8C];	/* Base Registers */
	B0 = R0;
	R0 = [P0 + 0x90];
	B1 = R0;
	R0 = [P0 + 0x94];
	B2 = R0;
	R0 = [P0 + 0x98];
	B3 = R0;

	R0 = [P0 + 0x9C];	/* Return Address (PC) */
	RETS = R0;
	
	R0 = [SP++];
	P0 = [SP++];

	CC = R0 == 0;
	IF !CC JUMP 1f;
	R0 = 1;
1:
	RTS;

.text;
.align 4;
.globl _setjmp;
.type _setjmp, STT_FUNC;

_setjmp:
	[--SP] = P0;		/* Save P0 */
	P0 = R0;
	R0 = [SP++];	
	[P0 + 0x00] = R0;	/* Save saved P0 */
	[P0 + 0x04] = P1;
	[P0 + 0x08] = P2;
	[P0 + 0x0C] = P3;
	[P0 + 0x10] = P4;
	[P0 + 0x14] = P5;

	[P0 + 0x18] = FP;	/* Frame Pointer */
	[P0 + 0x1C] = SP;	/* Stack Pointer */

	[P0 + 0x20] = P0;	/* Data Registers */
	[P0 + 0x24] = R1;
	[P0 + 0x28] = R2;
	[P0 + 0x2C] = R3;
	[P0 + 0x30] = R4;
	[P0 + 0x34] = R5;
	[P0 + 0x38] = R6;
	[P0 + 0x3C] = R7;

	R0 = ASTAT;
	[P0 + 0x40] = R0;

	R0 = LC0;		/* Loop Counters */
	[P0 + 0x44] = R0;
	R0 = LC1;
	[P0 + 0x48] = R0;

	R0 = A0.W;		/* Accumulators */
	[P0 + 0x4C] = R0;
	R0 = A0.X;
	[P0 + 0x50] = R0;
	R0 = A1.W;
	[P0 + 0x54] = R0;
	R0 = A1.X;
	[P0 + 0x58] = R0;

	R0 = I0;		/* Index Registers */
	[P0 + 0x5C] = R0;
	R0 = I1;
	[P0 + 0x60] = R0;
	R0 = I2;
	[P0 + 0x64] = R0;
	R0 = I3;
	[P0 + 0x68] = R0;

	R0 = M0;		/* Modifier Registers */
	[P0 + 0x6C] = R0;
	R0 = M1;
	[P0 + 0x70] = R0;
	R0 = M2;
	[P0 + 0x74] = R0;
	R0 = M3;
	[P0 + 0x78] = R0;

	R0 = L0;		/* Length Registers */
	[P0 + 0x7c] = R0;
	R0 = L1;
	[P0 + 0x80] = R0;
	R0 = L2;
	[P0 + 0x84] = R0;
	R0 = L3;
	[P0 + 0x88] = R0;

	R0 = B0;		/* Base Registers */
	[P0 + 0x8C] = R0;
	R0 = B1;
	[P0 + 0x90] = R0;
	R0 = B2;
	[P0 + 0x94] = R0;
	R0 = B3;
	[P0 + 0x98] = R0;

	R0 = RETS;
	[P0 + 0x9C] = R0;

	R0 = 0;

	RTS;

