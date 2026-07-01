#include "USE_DRAW.h"
#include "main.h"
#include "motor.h"

#define chongfa 1
u16 R;
u16 CR;
u16 id;

int16_t front_sight_position_x,front_sight_position_y;

extern RC rc;
extern DJI_motor STEER_6020_I_MOTOR[4],WHEEL_3508_I_MOTOR[4],YAW_6020,HERO_CLIP; 
extern int16_t recieve_pitch;
extern PID A_R,B_R,C_R,D_R,PID_YAW_FOLLOW,YAW_SPIN;

u32 text;
picture LINE1[7];      //定义一个图像为直线
picture zhengfang;     //定义一个矩形为zhengfang
picture yuanhu;        //定义一个圆弧
word V1;               //定义一个数据为电压v
picture LINE2[3];      //定义一个图像为直线
word D,F;              //定义一个数据为弹速d
picture yuan;  
//extern capacity D_R;
int hudu;              //圆弧显示角度
struct 
{
	u8 fool:1;
}luoji;
int zhunxingColor =yellow;
int zhunxingZuangTai=ADD_P;
void pictureInit()
{  
  //int pwm=1400;	  
	 // luoji.fool++;
	  LINE1[0].name=            2999;             //直线名
	  LINE1[0].width=           5;             //线宽 
    LINE1[0].start_x=         ScreenX/2;            //直线起点x坐标
	  LINE1[0].start_y=         ScreenY/2-400;            //直线起点y坐标
	  LINE1[0].end_x  =         ScreenX/2;            //直线终点x坐标
	  LINE1[0].end_y  =         ScreenY/2-400;        //直线终点y坐标
    LINE1[0].color=           zhunxingColor;         //直线颜色
    LINE1[0].operate_tpye=    zhunxingZuangTai;            //图形操作添加图形
    LINE1[0].graphic_tpye=    rele;           //图形类型直线
    LINE1[0].layer=           0;              //图层数1
		 
	 static u32 i;
	 u8 a;
	  i++;
	  if((i%2)==0) {a=1;} else {a=2;}
		
	  LINE1[2].name=            288;             //直线名
	  LINE1[2].width=           50;             //线宽 
    LINE1[2].start_x=         ScreenX/2-25;        //直线起点x坐标
	  LINE1[2].start_y=         ScreenY/2-500;            //直线起点y坐标
	  LINE1[2].end_x  =         ScreenX/2+25;        //直线终点x坐标
	  LINE1[2].end_y  =         ScreenY/2-500;            //直线终点y坐标
    LINE1[2].color=           rc.direction+2;         //直线颜色
    LINE1[2].operate_tpye=    a;              //图形操作添加图形
    LINE1[2].graphic_tpye=    line;           //图形类型直线
    LINE1[2].layer=           1;              //图层数1
		
		LINE1[1].name=            3788;             //直线名
	  LINE1[1].width=           4;             //线宽 
    LINE1[1].start_x=         ScreenX/2-280;        //直线起点x坐标
	  LINE1[1].start_y=         ScreenY/2-300;            //直线起点y坐标
	  LINE1[1].end_x  =         ScreenX/2-430;        //直线终点x坐标
	  LINE1[1].end_y  =         ScreenY/2-500;            //直线终点y坐标
    LINE1[1].color=           6;         //直线颜色
    LINE1[1].operate_tpye=    a;            //图形操作添加图形
    LINE1[1].graphic_tpye=    line;           //图形类型直线
    LINE1[1].layer=           1;              //图层数1
		
		LINE1[4].name=            4349;               //直线名
	  LINE1[4].width=           4;                  //线宽 
    LINE1[4].start_x=         ScreenX/2+280;        //直线起点x坐标
	  LINE1[4].start_y=         ScreenY/2-300;       //直线起点y坐标
	  LINE1[4].end_x  =         ScreenX/2+430;        //直线终点x坐标
	  LINE1[4].end_y  =         ScreenY/2-500;       //直线终点y坐标
    LINE1[4].color=           1;      //直线颜色
    LINE1[4].operate_tpye=    a;   //图形操作添加图形
    LINE1[4].graphic_tpye=    line;               //图形类型直线
    LINE1[4].layer=           1;                  //图层数1
		
		LINE1[3].name=            5700;             //直线名
	  LINE1[3].width=           2;             //线宽 
    LINE1[3].start_x=         ScreenX/2+20;        //直线起点x坐标
	  LINE1[3].start_y=         ScreenY/2+40;            //直线起点y坐标
	  LINE1[3].end_x  =         ScreenX/2-30;        //直线终点x坐标
	  LINE1[3].end_y  =         ScreenY/2-10;            //直线终点y坐标
    LINE1[3].color=           rc.if_zimiao+3;         //直线颜色
    LINE1[3].operate_tpye=    a;            //图形操作添加图形
    LINE1[3].graphic_tpye=    rele;           //图形类型直线
    LINE1[3].layer=           1;              //图层数1

//	  LINE2[0].name=            1111;             //直线名
//	  LINE2[0].width=           5;             //线宽 
//    LINE2[0].start_x=         ScreenX/2+600;            //直线起点x坐标
//	  LINE2[0].start_y=         ScreenY/2+200;            //直线起点y坐标
//	  LINE2[0].end_x  =         50;            //直线终点x坐标
//	  LINE2[0].end_y  =         50;        //直线终点y坐标
//		LINE2[0].start_angle  =   YAW_6020.relative_zero_angle-YAW_6020.round_cnt-30;
//		LINE2[0].end_angle  =     YAW_6020.relative_zero_angle-YAW_6020.round_cnt;
//    LINE2[0].color=           cyan;         //直线颜色
//    LINE2[0].operate_tpye=    a;            //图形操作添加图形
//    LINE2[0].graphic_tpye=    ciar;           //图形类型直线
//    LINE2[0].layer=           2;              //图层数1
		 
//	  LINE2[1].name=            8989;             //直线名
//	  LINE2[1].width=           2;             //线宽 
//    LINE2[1].start_x=         ScreenX/2+50;        //直线起点x坐标
//	  LINE2[1].start_y=         ScreenY/2-50;            //直线起点y坐标
//	  LINE2[1].end_x  =         ScreenX/2-50;        //直线终点x坐标
//	  LINE2[1].end_y  =         ScreenY/2+50;            //直线终点y坐标
//    LINE2[1].color=           white;         //直线颜色
//    LINE2[1].operate_tpye=    zhunxingZuangTai;            //图形操作添加图形
//    LINE2[1].graphic_tpye=    line;           //图形类型直线
//    LINE2[1].layer=           2;              //图层数1
  	
		zhengfang.name=         1275;
		zhengfang.width=        2;
		zhengfang.start_x=      ScreenX/2-50;
		zhengfang.start_y=      ScreenY/2-100;
		zhengfang.end_x=        ScreenX/2-50;
		zhengfang.end_y=        ScreenY/2+100;
		zhengfang.color=        3;
		zhengfang.operate_tpye= a;
		zhengfang.graphic_tpye= line;
		zhengfang.layer       = 1;
		
		yuanhu.name =           7878;
		yuanhu.start_angle   =  330;
		yuanhu.end_angle     =  30;
		yuanhu.width         =  5;
		yuanhu.start_x       =  ScreenX/2;
		yuanhu.start_y       =  ScreenY/2-450;
		yuanhu.end_x         =  70;
		yuanhu.end_y         =  70;
		yuanhu.color         =  green;
		yuanhu.operate_tpye  =  ADD_P;
		yuanhu.graphic_tpye  =  ciar; 
		yuanhu.layer         =  4;
		
//		yuan.name =  2523;
//		yuan.width=5;
//		yuan.start_x=ScreenX/2-15;
//		yuan.start_y=ScreenY/2-50;
////		yuan.end_x         =  70;
////		yuan.end_y         =  70;
//		yuan.color =        green;
//		yuan.operate_tpye= ADD_P;
//		yuan.graphic_tpye = _round;
//		yuan.radius = 10;
//		yuan.layer = 4;
		
		
	  V1.name=9999;
		V1.start_angle =20;              //字体大小
		V1.end_angle   = 3;              //小数有效位数
		V1.width       = 3;              //线宽
		V1.start_x     = ScreenX/2+300;
		V1.start_y     = ScreenY/2+300;
		V1.W.zhengshu   = recieve_pitch;
		V1.color       =  white;
		V1.operate_tpye = a;
		V1.graphic_tpye = ZS;
		V1.layer        = 3;
		
		

		D.start_angle =   40;              //字体大小
		D.end_angle   =   2;              //小数有效位数
		D.width       =   4;              //线宽
		D.start_x     =   ScreenX/2+340;
		D.start_y     =   ScreenY/2+270;
		D.W.zhengshu   =  CAP_U*100;
	  D.color        =  green;
		D.operate_tpye =  a;
		D.graphic_tpye =  ZS;
		D.layer        =  3;
		
		F.start_angle =  305;              //字体大小
		F.end_angle   =   0;              //小数有效位数
		F.width       =   4;              //线宽
		F.start_x     =   ScreenX/2+340;
		F.start_y     =   ScreenY/2+270;
		F.W.zhengshu   =  robot.bullet_remaining_num;
	  F.color        =  green;
		F.operate_tpye =  a;
		F.graphic_tpye =  ZS;
		F.layer        =  3;
}
u8 zuce=0;


//大疆ui显示图像步骤 ，添加显示图像和修改显示图像，通过添加显示图像实现显示，修改图像实现刷新
void drawPictureDemo(void)
{
	  pictureInit(); 
		id=robot.robot_id;
    if(id<0X100)
		{
		   R=id;
			 CR=id+0x100;
		}
		
		
		else if(id>=0x100)
		{
		   R=id;
			 CR=R-100+0X164;
		}

	
		
		switch(zuce)
		{
			case 0:
					drawPicture(draw1P,R,CR,(u8*)&V1);          //绘图函数
					break;
		  case 1:			
					drawPicture(draw1P,R,CR,(u8*)&D);           //绘图函数
					break;
			case 2:
					drawPicture(draw5P,R,CR,(u8*)LINE1);        //绘图函数
					break;
			case 3:		
					drawPicture(draw1P,R,CR,(u8*)&yuanhu);      //绘图函数
					break;
			case 4:
					drawPicture(draw2P,R,CR,(u8*)LINE2);        //绘图函数
					break;
			case 5:
			    drawPicture(draw1P,R,CR,(u8*)&zhengfang);   //绘图函数
					break;
			case 6:
					drawPicture(0x308,R,CR-2,(u8*)&F);   
					break;
			
			default: break;
		}
		zuce++;
		if(zuce>6){zuce=0;} 
}

    
 u8 ji=0;
int abc=0;
void drawPictureDemoppxu(void)
{
	
	  pictureInit(); 
    D.operate_tpye =  CHANGE_P;
	  zhengfang.operate_tpye= CHANGE_P;
		yuanhu.operate_tpye  =  CHANGE_P;
		V1.operate_tpye = CHANGE_P;
	  D.W.zhengshu   =  CAP_U*100;
		V1.W.zhengshu=abc++%100;

	 
		id=robot.robot_id;
    if(id<0X100)
		{
		   R=id;
			 CR=id+0x100;
		}
		else if(id>=0x100)
		{
		   R=id;
			 CR=R-100+0X164;
		}		  
	     yuanhu.start_angle   =  (hudu-30)<0?(hudu+360-30):(hudu-30);
		   yuanhu.end_angle     =  (hudu+30)>360?(hudu-360+30):(hudu+30);

		
//		if (robot.power_management_shooter_output==1)//发射机构
//		{
//			 LINE2[1].start_x=         ScreenX/2;            //直线起点x坐标
//			 LINE2[1].start_y=         ScreenY/2;            //直线起点y坐标
//			 LINE2[1].end_x  =         ScreenX/2;            //直线终点x坐标
//			 LINE2[1].end_y  =         ScreenY/2;          //直线终点y坐标

//			 LINE2[0].start_x=         ScreenX/2;            //直线起点x坐标			
//			 LINE2[0].start_y=         ScreenY/2;            //直线起点y坐标
//			 LINE2[0].end_x  =         ScreenX/2;            //直线终点x坐标
//			 LINE2[0].end_y  =         ScreenY/2;          //
//			 LINE2[1].operate_tpye=    CHANGE_P;            //图形操作添加图形
//			 LINE2[0].operate_tpye=    CHANGE_P;            //图形操作添加图形
//		}
//		else
//		{
//			 LINE2[0].start_x=         ScreenX/2-50;            //直线起点x坐标
//			 LINE2[0].start_y=         ScreenY/2-50;            //直线起点y坐标
//			 LINE2[0].end_x  =         ScreenX/2+50;            //直线终点x坐标
//			 LINE2[0].end_y  =         ScreenY/2+50;          //直线终点y坐标
//		
//			 LINE2[1].start_x=         ScreenX/2+50;        //直线起点x坐标
//			 LINE2[1].start_y=         ScreenY/2-50;            //直线起点y坐标
//			 LINE2[1].end_x  =         ScreenX/2-50;        //直线终点x坐标
//			 LINE2[1].end_y  =         ScreenY/2+50;            //直线终点y坐标
//			 LINE2[1].operate_tpye=    CHANGE_P;            //图形操作添加图形
//			 LINE2[0].operate_tpye=    CHANGE_P;            //图形操作添加图形    
//		}
		switch(ji)
		{
			case 0:
					drawPicture(draw1P,R,CR,(u8*)&V1);          //绘图函数
					break;                                                                                                                           
		  case 1:				
				  drawPicture(draw1P,R,CR,(u8*)&D);            //绘图函数
					break;
			case 2:
					drawPicture(draw2P,R,CR,(u8*)LINE2);        //绘图函数
					break;
			case 3:						
					drawPicture(draw1P,R,CR,(u8*)&yuanhu);      //绘图函数
					break;
			case 4:
				  drawPicture(draw1P,R,CR,(u8*)&zhengfang);   //绘图函数
					break;
				
			default: break;
		}
		ji++;
		if(ji>4){ji=0;}   
}

extern RC rc;

uint8_t DP_time_set=0;
uint8_t select_P;
void run_demo(void)
{
	DP_time_set++;
	//30ms 一个数据帧
	if(DP_time_set>=3)
	{
		DP_time_set=0;
		if(rc.V==1)
		{
			drawPictureDemoppxu();
			select_P=1;			
		}
		else
		{
			drawPictureDemo();
			select_P=0;
		}
	}
}


