#include "rc_potocal.h"
#include "math.h"

#define PI 3.1415926

extern motor_info motor[4];

int16_t remote_value(int16_t x, int16_t y)
{
	int16_t alpha;
	if(y>=0){
		alpha = atan(x/y);
	}
	else if(x<=0 && y<0){
		alpha = atan(x/y) - PI;
	}
	else{
		alpha = atan(x/y) + PI;
	}
	return alpha;
}

int16_t motor_value(int16_t get, int16_t set)
{
	int16_t n = get; //0~8192 当前角度“0”
	int16_t k = set; //0~360 需要调整的角度
	n = n * 360.0 / 8192.0;
	if(k>=0 && k<=PI){
		if(n>=0 && n<(PI+k)){
			n = k - n;
			return n;
		}
		if(n>=(PI+k) && n<=(2*PI)){
			n = 2*PI - n + k;
			return n;
		}
	}
	
	if(k>PI && k<=2*PI){
		if(n>=0 && n<(k-PI)){
			n = -2*PI - n + k;
			return n;
		}
		if(n>=(k-PI) && n<=(2*PI)){
			n = k - n;
			return n;
		}
	}
}