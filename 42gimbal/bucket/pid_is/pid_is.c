#include "pid_is.h"
#include "math.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
		
static float sign(float x)
{
	if(x>0)
		return 1;
	else if(x<0)
		return -1;
	else
		return 0;
}
		
float ABS_ABS(float a)
{
if(a<0) a=-a;
return a;
}

float PID_fhan(float x1,float x2,float r,float h)
{
	float a    = 0,
		    a0   = 0,
		    y    = 0,
		    d		 = 0,
		    d0   = 0,
		    absy = 0,
		    absa = 0,
	      fhan = 0;
	absy =  fabsf(y);
  absa =  fabsf(a);
  d    =  r*h;
	d0   =  h*d;
  y    =  x1+h*x2;
  a0   =  sqrtf(d*d+8*r*absy);
	if(absy > d0)
		a = x2 + (a0 - d)/2 * sign(y);
	else
		a = x2 + y / h;
	if(absa > d)
		fhan = -1*r*sign(a);
	else
		fhan = -1*a/d;
  return fhan;
}



void TD_Init(TD_t *td, float r, float h0)
{
    td->r = r;
    td->h0 = h0;

    td->x = 0;
    td->dx = 0;
    td->ddx = 0;
    td->last_dx = 0;
    td->last_ddx = 0;
}
float TD_Calculate(TD_t *td, float input)
{
    static float d, a0, y, a1, a2, a, fhan;

    td->dt=0.001f;

    td->Input = input;

    d = td->r * td->h0 * td->h0;
    a0 = td->dx * td->h0;
    y = td->x - td->Input + a0;
    a1 = sqrt(d * (d + 8 * ABS_ABS(y)));
    a2 = a0 + sign(y) * (a1 - d) / 2;
    a = (a0 + y) * (sign(y + d) - sign(y - d)) / 2 + a2 * (1 - (sign(y + d) - sign(y - d)) / 2);
    fhan = -td->r * a / d * (sign(a + d) - sign(a - d)) / 2 -
           td->r * sign(a) * (1 - (sign(a + d) - sign(a - d)) / 2);

    td->ddx = fhan;
    td->dx += (td->ddx + td->last_ddx) * td->dt / 2;
    td->x += (td->dx + td->last_dx) * td->dt / 2;

    td->last_ddx = td->ddx;
    td->last_dx = td->dx;

    return td->x;
}

float  PID_Position(PID *Pid,float real,float set)
{ Pid->set = set;
	Pid->real = real;
	Pid->error[0] = set - real;
	
	if(ABS_ABS(Pid->error[0])<Pid->interget_seperate){
	Pid->sum_error += Pid->error[0];}
    
	if(Pid->sum_error>Pid->sum_error_max) {Pid->sum_error=Pid->sum_error_max;}
	if(Pid->sum_error<-Pid->sum_error_max) {Pid->sum_error=-Pid->sum_error_max;}

	if(Pid->if_lp==1)
	{
	Pid->error[0]=Pid->error[0]*0.3f+Pid->error[1]*0.7f;
	}
	
	Pid->Pout=Pid->Kp * Pid->error[0] ;
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;

	Pid->Iout=Pid->Ki * Pid->sum_error ;
	Pid->Iout > Pid->IMAX ? Pid->Iout=Pid->IMAX :1;
	Pid->Iout < -Pid->IMAX ?Pid->Iout = -Pid->IMAX :1;

	Pid->Dout=Pid->Kd * ( Pid->error[0] - Pid->error[1] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;
	
	Pid->out=Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OutMAX ? Pid->out = Pid->OutMAX : 1;
	Pid->out < -Pid->OutMAX ? Pid->out = -Pid->OutMAX : 1;

	Pid->error[1] = Pid->error[0];	

	return Pid->out;

}

void PID_IS_Init(PID* Pid,float kp,float ki,float kd,float pmax,float imax,float dmax,float outmax,float sum_error_max,float interget_seperate)
{

	Pid-> set=0;
	Pid-> real=0;
	Pid-> Pout=0;
	Pid-> Iout=0;
	Pid-> Dout=0;
	Pid-> out=0;
	
	Pid-> error[0]=0.0f;
	Pid-> error[1]=0.0f;
	Pid-> error[2]=0.0f;
//	Pid->sum_error=0;
	Pid->interget_seperate=interget_seperate;
	Pid->sum_error_max=sum_error_max;
	
	Pid-> Kp=kp;
	Pid-> Ki=ki;
	Pid-> Kd=kd;
	
	Pid-> PMAX=pmax;
	Pid-> IMAX=imax;
	Pid-> DMAX=dmax;
	Pid->OutMAX=outmax;
}

float  PID_Incremental(PID* Pid,float set ,float real){//增量
	Pid->set = set;
	Pid->real = real;
	Pid->error[0] = Pid->set - Pid->real;
	
	Pid->Pout = Pid->Kp * ( Pid->error[0] - Pid->error[1] );
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;
	
	Pid->Iout =	Pid->Ki * Pid->error[0];
	Pid->Iout > Pid->IMAX ? Pid->Iout = Pid->IMAX : 1;
	Pid->Iout < -Pid->IMAX ? Pid->Iout = -Pid->IMAX : 1;

	Pid->Dout =	Pid->Kd * ( Pid->error[0] - 2 * Pid->error[1] + Pid->error[2] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;

	Pid->error[2] = Pid->error[1];
	Pid->error[1] = Pid->error[0];
	
	Pid->out += Pid->Pout + Pid->Iout + Pid->Dout;
	Pid->out > Pid->OutMAX ? Pid->out = Pid->OutMAX : 1;
	Pid->out < -Pid->OutMAX ? Pid->out = -Pid->OutMAX : 1;

	return Pid->out;	
}

//void gimbal_pid_calculate(gimbal_pid *P)
//{
//	
//}

/**
  * @brief          前馈结构体初始化
	* @param[out]     ff: feedforward结构数据指针
  * @param[in]      k1: 一阶差分环节系数
  * @param[in]      k2：二阶差分环节系数
  * @param[in]      omax：输出上限（绝对值）
  * @retval         none
  */
void feedforward_init(ff_type_def *ff, fp32 k1, fp32 k2, fp32 omax)
{
	if(ff == NULL)
	{
		return;
	}
	ff -> k_ff_1 = k1;
	ff -> k_ff_2 = k2;
	ff -> input_last = 0;
	ff -> output = 0;
	ff -> output_max = omax;
	ff -> dif_1 = ff -> dif_2 = 0.0f;
	ff -> dbuf[0] = ff -> dbuf[1] = 0.0f;
}

/**
  * @brief          前馈数据计算
	* @param[out]     output：前馈环节输出
  * @param[in]      ff: feedforward结构数据指针
  * @param[in]      input：前馈环节输入
  * @retval         前馈输出
  */
fp32 feedforward_calc(ff_type_def *ff, fp32 input)
{
	if(ff == NULL)
	{
		return 0.0f;
	}
	
	ff -> dif_1 = input - ff -> input_last;
	ff -> dbuf[0] = ff -> dif_1;
	ff -> dif_2 = ff -> dbuf[0] - ff -> dbuf[1];
	ff -> output = ff -> dif_1 * ff -> k_ff_1 + ff -> dif_2 * ff -> k_ff_2;
	
	ff -> input_last = input;
	ff -> dbuf[1] = ff -> dbuf[0];
	if (ff -> output > ff -> output_max)
	{                      
			ff -> output = ff -> output_max;
	}
	else if (ff -> output < -(ff -> output_max))
	{
			ff -> output = -(ff -> output_max);
	}
	return ff -> output;
}

//SMC YawSMC(20, 120, 0, 0.001, 25000, 0.8, 0.5); //这是一个yaw轴参考电机参数

	// 符号函数,若有抖动可以换个陡峭的饱和函数
	int8_t Signal(float y)
	{
		if (y > 0)
			return 1;
		else if (y == 0)
			return 0;
		else
			return -1;
	}
	// 饱和函数
	float Sat(float y)
	{
		if (fabs(y) <= 1)
			return y;
		else
			return Signal(y);
	}

void 	SMC_init(SMC_control *SMC,float C,float K,float ref,float error_eps,float u_max,float J,float epsilon)
{
	SMC->C_IN=C;
	SMC->K_IN=K;
	SMC->ref=ref;
	SMC->error_eps=error_eps;
	SMC->u_max=u_max;
	SMC->J=J;
	SMC->epsilon=epsilon;
	
}

float limit_max_min(float in,float min,float max)
{
	in > max ? in = max : 1;
	in < min ? in = min : 1;
	return in;
}


float smc_k_c,smc_k_k;

float SMC_Tick(SMC_control *SMC,float ref,float angle_now,float angle_vel) //anlge为当前位置(°),ang_vel为角速度(°/s)
{
	//读取参数
	SMC->angle = angle_now;
  SMC->ang_vel = angle_vel;
	SMC->ref=ref;
	SMC->error = SMC->angle - SMC->ref;
	
	smc_k_c=0,smc_k_k=0;
	
	smc_k_c=180/(SMC->error-90);
	
	smc_k_c=limit_max_min(smc_k_c,0.4,1.5);
	smc_k_k=limit_max_min(smc_k_k,0.4,1.5);
	
	SMC->C=SMC->C_IN*smc_k_c;
  SMC->K=SMC->K_IN*smc_k_k;
	
	SMC->ddref = (SMC->ref - SMC->refl) - SMC->dref; //这里对前馈进行了处理，没有严格单位统一
	SMC->dref = (SMC->ref - SMC->refl);
	//误差下限处理
	if (fabs(SMC->error) < SMC->error_eps)
	{
		SMC->u = 0;
		return 0;
	}
	//smc surface
	SMC->s = SMC->C * SMC->error + (SMC->ang_vel - SMC->dref);
	SMC->u = SMC->J * (SMC->ddref - SMC->C * (SMC->ang_vel - SMC->dref) - SMC->epsilon * Sat(SMC->s) - SMC->K * SMC->s);
	//控制量限幅
	if (SMC->u > SMC->u_max)
		SMC->u = SMC->u_max;
	if (SMC->u < -SMC->u_max)
		SMC->u = -SMC->u_max;
	//参数更新
	SMC->refl = SMC->ref;
	
	return SMC->u;
}

//void cSMC_Init(Sliding *smc)
//{
//    smc->param.J = 0;
//    smc->param.K = 0;
//    smc->param.c = 0;
//    smc->param.epsilon = 0;
//    smc->flag = EXPONENT;
//    smc->u_max = 0;
//    smc->limit = 0;

//    smc->error.tar_now = 0;
//    smc->error.tar_last = 0;
//    smc->error.tar_differential = 0;

//    smc->error.p_error = 0;
//    smc->error.v_error = 0;
//    smc->error.v_error_integral = 0;
////    smc.error.v_error_integral_max = 0;
//    smc->error.pos_error_eps = 0;
//    smc->error.vol_error_eps = 0;
//    smc->error.pos_get = 0;
//    smc->error.vol_get = 0;
//}

//void cSMC::SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///EXPONENT,POWER,VELSMC 参数设定
//    smc.param.J = J;
//    smc.param.K = K;
//    smc.param.c = c;
//    smc.error.pos_error_eps = pos_esp;
//    smc.flag = flag;
//    smc.param.epsilon = epsilon;
//    smc.u_max = u_max;
//    smc.limit = limit;
//    OutContinuation();
//}

//void cSMC::SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///EISMC 参数设定
//    smc.param.J = J;
//    smc.param.K = K;
//    smc.param.c1 = c1;
//    smc.param.c2 = c2;
//    smc.error.pos_error_eps = pos_esp;
//    smc.flag = flag;
//    smc.param.epsilon = epsilon;
//    smc.u_max = u_max;
//    smc.limit = limit;
//    OutContinuation();
//}

//void cSMC::ErrorUpdate(float target, float pos_now, float vol_now) //位置环误差更新
//{
//    smc.error.tar_now = target;
//    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);

//    smc.error.tar_differential_second = (float)((smc.error.tar_differential- smc.error.tar_differential_last)/SAMPLE_PERIOD); ///二阶导

//    smc.error.p_error = pos_now - target;
//    smc.error.v_error = vol_now - smc.error.tar_differential;
//    smc.error.tar_last = smc.error.tar_now;

//    smc.error.p_error_integral += (float)(smc.error.p_error * SAMPLE_PERIOD); ///位置误差积分项

//    smc.error.tar_differential_last = smc.error.tar_differential; ///二阶导更新

//}

//void cSMC::ErrorUpdate(float target,float vol_now) //速度环误差更新
//{
//    smc.error.tar_now = target;
//    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);

////    smc.error.tar_differential_second = (float)((smc.error.tar_differential- smc.error.tar_differential_last)/SAMPLE_PERIOD); ///二阶导

//    smc.error.v_error = vol_now - smc.error.tar_now;

//    smc.error.v_error_integral += (float)(smc.error.v_error * SAMPLE_PERIOD); ///速度误差积分项
////    if(std::abs(smc.error.v_error_integral) > smc.error.v_error_integral_max) //积分限幅
////    {
////        smc.error.v_error_integral = smc.error.v_error_integral_max;
////    }

//    smc.error.tar_last = smc.error.tar_now;

////    smc.error.tar_differential_last = smc.error.tar_differential; ///二阶导更新

//}
//void cSMC::Clear()
//{
//    smc.error.tar_now = 0;
//    smc.error.tar_last = 0;
//    smc.error.tar_differential = 0;

//    smc.error.p_error = 0;
//    smc.error.v_error = 0;
//    smc.error.v_error_integral = 0;
////    smc.error.v_error_integral_max = 0;
//    smc.error.pos_error_eps = 0;
//    smc.error.vol_error_eps = 0;
//    smc.error.pos_get = 0;
//    smc.error.vol_get = 0;

//    smc.error.tar_differential_second = 0;
//    smc.error.tar_differential_last = 0;
//    smc.error.p_error_integral = 0;
//}
//void cSMC::Integval_Clear()
//{
//    smc.error.v_error_integral = 0;
//    smc.error.p_error_integral = 0;
//}

//float cSMC_SmcCalculate()
//{
//    float u,fun;
//            ///比例积分滑模面，指数趋近律，位置控制
//            if (fabs(smc.error.p_error) - smc.error.pos_error_eps < 0)
//            {
//                smc.error.p_error = 0;
//                return 0;
//            }

//            smc.s = smc.param.c1 * smc.error.p_error + smc.error.v_error + smc.param.c2 * smc.error.p_error_integral; //滑模面
//            fun = Sat(smc.s);//饱和函数消除抖动
//            u =  smc.param.J * ( (-smc.param.c1 * smc.error.v_error)- smc.param.c2 * smc.error.p_error - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算,指数趋近率
//            break;
//    

//    smc.error.error_last = smc.error.p_error; //更新上一步的误差

//    //控制量限幅
//    if (u > smc.u_max)
//    {
//        u = smc.u_max;
//    }
//    if (u < -smc.u_max)
//    {
//        u = -smc.u_max;
//    }
//    smc.u = u;
//    return u;
//}

//void cSMC::OutContinuation()
//{
//    if(smc.param.K != 0 && smc.param.c2 != 0)
//    {
//        smc.error.p_error_integral = (smc.param_last.K / smc.param.K) * (smc.param_last.c2 / smc.param.c2) * smc.error.p_error_integral;
//        smc.error.v_error_integral = (smc.param_last.K / smc.param.K) * (smc.param_last.c / smc.param.c) * smc.error.v_error_integral;
//    }
//    smc.param_last = smc.param;
//}


//// 符号函数
//float cSMC::Signal(float s)
//{
//    if (s > 0)
//        return 1;
//    else if (s == 0)
//        return 0;
//    else
//        return -1;
//}

////饱和函数
//float cSMC_Sat(float s ,)
//{
//    float y;
//    y = s / smc.param.epsilon;
//    if (std::abs(y) <= smc.limit)
//        return y;
//    else
//        return Signal(y);
//}

//const Sliding &cSMC::getSmc() const {
//    return smc;
//}

//float cSMC::Out() {
//    return smc.u;
//}

//void cSMC::SetOut(float out) {
//    smc.u = out;
//}









