
#include "atan2.h"


#define MULTIPLY_FP_RESOLUTION_BITS	15

int atan2_fp_5deg(int y_fp, int x_fp)
{
	int coeff_1 = 45;
	int coeff_2 = 135;

	int angle = 0;

	int r;

	int y_abs_fp = y_fp;
	if (y_abs_fp < 0)
		y_abs_fp = -y_abs_fp;

	if (y_fp == 0)
	{
		if (x_fp >= 0)
		{
			angle = 0;
		}
		else
		{
			angle = 180;
		}
	}
	else if (x_fp >= 0)
	{
		r = (((int)(x_fp - y_abs_fp)) * coeff_1) / ((int)(x_fp + y_abs_fp));
		angle = (int) (  coeff_1 - r   );
	}
	else
	{
		r = (((int)(x_fp + y_abs_fp)) * coeff_1 ) / ((int)(y_abs_fp - x_fp));
		angle = coeff_2 - ((int)	r 	);
	}

	if (y_fp < 0)
		return (-angle);     // negate if in quad III or IV
	else
		return (angle);
}

int atan2_fp_1deg(int y_fp, int x_fp)
{
	int coeff_1 = 45;
	int coeff_1b = -56;	// 56.24;
	int coeff_1c = 11;	// 11.25
	int coeff_2 = 135;

	int angle = 0;

	int r;
	int r3;

	int y_abs_fp = y_fp;
	if (y_abs_fp < 0)
		y_abs_fp = -y_abs_fp;

	if (y_fp == 0)
	{
		if (x_fp >= 0)
		{
			angle = 0;
		}
		else
		{
			angle = 180;
		}
	}
	else if (x_fp >= 0)
	{
		r = (((int)(x_fp - y_abs_fp)) << MULTIPLY_FP_RESOLUTION_BITS) / ((int)(x_fp + y_abs_fp));

		r3 = r * r;
		r3 =  r3 >> MULTIPLY_FP_RESOLUTION_BITS;
		r3 *= r;
		r3 =  r3 >> MULTIPLY_FP_RESOLUTION_BITS;
		r3 *= coeff_1c;
		angle = (int) (  	coeff_1 + ((coeff_1b * r + r3) >> MULTIPLY_FP_RESOLUTION_BITS)   );
	}
	else
	{
		r = (((int)(x_fp + y_abs_fp)) << MULTIPLY_FP_RESOLUTION_BITS) / ((int)(y_abs_fp - x_fp));
		r3 = r * r;
		r3 =  r3 >> MULTIPLY_FP_RESOLUTION_BITS;
		r3 *= r;
		r3 =  r3 >> MULTIPLY_FP_RESOLUTION_BITS;
		r3 *= coeff_1c;
		angle = coeff_2 + ((int)	(((coeff_1b * r + r3) >> MULTIPLY_FP_RESOLUTION_BITS))	);
	}

	if (y_fp < 0)
		return (-angle);     // negate if in quad III or IV
	else
		return (angle);
}


