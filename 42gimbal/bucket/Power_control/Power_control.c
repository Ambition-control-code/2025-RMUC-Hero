#include "Power_control.h"
#include "math.h"

void Slope_Init(slope* Slope,int slope_k)

{
Slope->slope_in=0;
Slope->slope_out=0;
Slope->slope_k=slope_k;

}

int Slope_use(slope* Slope,int in)

{
  Slope->slope_in=in;
	if(Slope->slope_in>=Slope->slope_out)
	Slope->slope_out+=Slope->slope_k;
	else if(Slope->slope_in<Slope->slope_out)
	Slope->slope_out-=Slope->slope_k;	
 
	if ( ( (Slope->slope_out-Slope->slope_in) <=Slope->slope_k) && ( ( Slope->slope_out-Slope->slope_in) >=-Slope->slope_k) )
		Slope->slope_out=Slope->slope_in;
	 return Slope->slope_out;
}

//
// power     电机无功控PID输出值
// use_power 最终输出值
//

float power_limit_k = 1.0f;
void power_limit(float pow_max, double k1, double k2,float *speed_3508,float *power,float *use_power)    //功率控制函数
{
	float velo1, velo2, velo3, velo4;

	velo1 = ((float)speed_3508[0]) / 14.0f / 60.0f * 2.0f * 3.1415926535;
	velo2 = ((float)speed_3508[1]) / 14.0f / 60.0f * 2.0f * 3.1415926535;
	velo3 = ((float)speed_3508[2]) / 14.0f / 60.0f * 2.0f * 3.1415926535;
	velo4 = ((float)speed_3508[3]) / 14.0f / 60.0f * 2.0f * 3.1415926535;
	float cmd1, cmd2, cmd3, cmd4;
	
	cmd1 = ((float)power[0]) / 16384.0f * 20.0f * 3.0f / 8.0f;
	cmd2 = ((float)power[1]) / 16384.0f * 20.0f * 3.0f / 8.0f;
	cmd3 = ((float)power[2]) / 16384.0f * 20.0f * 3.0f / 8.0f;
	cmd4 = ((float)power[3]) / 16384.0f * 20.0f * 3.0f / 8.0f;
	
	float sum_velo_cmd, sum_velo_cmd2;
	sum_velo_cmd = fabs(velo1 * cmd1) + fabs(velo2 * cmd2) + fabs(velo3 * cmd3) + fabs(velo4 * cmd4);
	sum_velo_cmd2 = pow(sum_velo_cmd, 2);
	
	float sum_cmd2, sum_velo2;
	sum_cmd2 = pow(cmd1, 2) + pow(cmd2, 2) + pow(cmd3, 2) + pow(cmd4, 2);
	sum_velo2 = pow(velo1, 2) + pow(velo2, 2) + pow(velo3, 2) + pow(velo4, 2);
	
	float mid_temp;
	mid_temp = sum_velo_cmd2 - 4 * k1 * sum_cmd2 * (k2 * sum_velo2 - pow_max);
	if (mid_temp > 0) {
		power_limit_k = (-sum_velo_cmd + sqrt(mid_temp)) / (2 * k1 * sum_cmd2);
	}
	else {
		power_limit_k = .0;
	}
	if (power_limit_k > 1) {
		power_limit_k = 1;
	}
	use_power[0] = power_limit_k * power[0];
	use_power[1] = power_limit_k * power[1];
	use_power[2] = power_limit_k * power[2];
	use_power[3] = power_limit_k * power[3];
		
}


