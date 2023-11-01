#include "rc_potocal.h"
#include "math.h"
#include "handle_value.h"
#define PI 3.14

float remote_value(int16_t x, int16_t y) //将获得的x,y值转化为角度后再转化为0~-180和0~180
{
	float alpha;
	if(y>0){
		alpha = atan((float)x/(float)y);
		alpha = alpha * 180 / PI;
		return alpha;
	}
	else if(x<0 && y<0){
		alpha = atan((float)x/(float)y);
		alpha = alpha * 180 / PI;
		alpha = alpha - 180;
		return alpha;
	}
	else if(x>0 && y<0){
		alpha = atan((float)x/(float)y);
		alpha = alpha * 180 / PI;
		alpha = alpha + 180;
		return alpha;
	}

	else if(y<0 && x==0){
		alpha = 180;
		return alpha;
	}
	else if(x<0 && y==0){
		alpha = - 90;
		return alpha;
	}
	else if(x>0 && y==0){
		alpha = 90;
		return alpha;
	}
		else if(x==0 && y==0){
		alpha = 0;
		return alpha;
	}
}

int16_t motor_value(int16_t k, int16_t n) //k:设定的初始角度“0” ； n:想要映射的角度
{
	if(k>=0 && k<4096){
		if(n>=0 && n<(k+4096)){
			n = k - n;
			n = (float)n * 360.f / 8192.f;
			return n;
		}
		else if(n>=(k+4096) && n<8191){
			n = 8192 - n + k;
			n = (float)n * 360.f / 8192.f;
			return n;
		}
	}
	
	else if(k>=4096 && k<8192){
		if(n>=0 && n<(k-4096)){
			n = -8192 + k - n;
			n = (float)n * 360.f / 8192.f;
			return n;
		}
		else if(n>=(k-4096) && n<8191){
			n = k - n;
			n = (float)n * 360.f / 8192.f;
			return n;
		}
	}
}