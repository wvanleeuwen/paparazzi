// generated with MATLAB script in SmartUAV

unsigned int get_prob_color(char* hsv, unsigned int* P, unsigned int sat, unsigned int val, unsigned int threshold_saturation, unsigned int threshold_value_low, unsigned int threshold_value_high)
{
	unsigned int p = 0;
	unsigned int ind;
	if(hsv[1] >= threshold_saturation && hsv[2] >= threshold_value_low && hsv[2] <= threshold_value_high)
	{
		ind = hsv[0];
		p =  P[ind];
		if(sat) p *= hsv[1];
		if(val) p *= hsv[2];
	}
	return p;
}


