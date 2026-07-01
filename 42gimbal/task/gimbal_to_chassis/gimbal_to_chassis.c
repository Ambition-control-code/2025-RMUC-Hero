#include "cmsis_os.h"
#include "stdlib.h"
#include "gimbal_task.h"
#include "string.h"
#include "motor.h"
#include "rc.h"        
#include "ulitility_tools.h"
#include "gimbal_to_chassis.h"

#define pitch_zero 131.1

extern float zimiao_watch[4];
extern u8 refree[9];

extern union{
VT13_data use_data;
u8  temp_data[21];
}VT13_USE;


extern RC rc;
extern bucket_imu imu;
pack_send_mouse  tmd_pdf;
pack_send_gimbal pack_chassic;
u8 send_xy_data[5];
extern u8 live_all,sendchassic[5];
extern dji_motor hero_pitch;
extern Gimbal_all guard_gimbal;
int16_t chasssis_angle;

void gimbal_to_chassis_task(void)   
{	
	  static int count;
	
    pack_chassic.rc.rc_sl=rc.sl;
    pack_chassic.rc.rc_sr=rc.sr;	

    tmd_pdf.rc.mouse_l=rc.mouse_l||VT13_USE.use_data.fn_1||VT13_USE.use_data.mouse_left;		
    tmd_pdf.rc.mouse_r=rc.mouse_r||VT13_USE.use_data.mouse_right;		
		tmd_pdf.rc.c      =rc.C      ||VT13_USE.use_data.C;		
    tmd_pdf.rc.shift  =rc.Shift  ||VT13_USE.use_data.pause||VT13_USE.use_data.SHIFT;	
    tmd_pdf.rc.e      =rc.E      ||VT13_USE.use_data.E;				
    tmd_pdf.rc.q      =rc.Q      ||VT13_USE.use_data.Q;		
    tmd_pdf.rc.r      =rc.R      ||VT13_USE.use_data.R;
    tmd_pdf.rc.f      =rc.F      ||VT13_USE.use_data.F;		
	
		if(tmd_pdf.rc.f==1){count=300;}
		else if(count!=0){count--;}
	
	  if((VT13_USE.use_data.mode_sw==0)||(count!=0))
		{
     pack_chassic.rc.rc_sl=1;		
		}
		else if((VT13_USE.use_data.mode_sw==1)||(VT13_USE.use_data.sof_2==1))
		{
     pack_chassic.rc.rc_sl=2;				
		}
	
		int desplay_x,desplay_y;
		desplay_x=-420*((rc.D||VT13_USE.use_data.D)-(rc.A||VT13_USE.use_data.A));
		desplay_y=-420*((rc.W||VT13_USE.use_data.W)-(rc.S||VT13_USE.use_data.S));
		
		send_xy_data[0]= (-rc.R_x+desplay_x-refree[4]*(VT13_USE.use_data.ch_0-1024))>>8;
		send_xy_data[1]=  -rc.R_x+desplay_x-refree[4]*(VT13_USE.use_data.ch_0-1024);
		send_xy_data[2]= (-rc.R_y+desplay_y-refree[4]*(VT13_USE.use_data.ch_1-1024))>>8;
		send_xy_data[3]=  -rc.R_y+desplay_y-refree[4]*(VT13_USE.use_data.ch_1-1024);
		
		chasssis_angle=imu.pitch*100;
		
		int temp;
		temp=guard_gimbal.PID_HERO_CLIP_ANGLE_sum/72;
		while(temp>5)
		{
		temp-=5;
		}
		pack_chassic.rc.direction=temp;
			
		if(zimiao_watch[2]!=-1)
		{    pack_chassic.rc.zimiao_live=1;}
		else
    {    pack_chassic.rc.zimiao_live=0;}
		Send_to_chassic(send_xy_data[0],send_xy_data[1],send_xy_data[2],send_xy_data[3],&pack_chassic,&tmd_pdf,(u8)(chasssis_angle>>8),(u8)chasssis_angle);
}











