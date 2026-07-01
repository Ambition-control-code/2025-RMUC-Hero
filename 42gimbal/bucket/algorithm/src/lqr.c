/* Includes ------------------------------------------------------------------*/
#include "lqr.h"
#include "stdint.h"
#include "VMC_calc.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const float k[12][3] = {
    175.9120,  -170.6678, -17.1254, 35.0040,  -31.1551, -3.6713,
    20.3988,   -21.8621,  -6.6372,  16.7956,  -19.7366, -10.9749,
    101.1676,  -109.3629, 43.8122,  18.3027,  -19.2733, 8.2075,
    -161.4978, 56.6788,   27.0371,  -1.5529,  -14.1453, 11.7354,
    -36.9927,  11.8450,   6.5878,   -31.0895, 2.3105,   10.9002,
    -188.9519, 156.6850,  49.1255,  -31.0208, 26.7038,  7.4055,
};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Lqr_Calc(vmc_leg_t* usedata,balance_usedata *output) 
{
  float lsqr = usedata->L0 * usedata->L0;

  for (uint8_t i = 0; i < 2; ++i) {
    uint8_t j = i * 6;
    output->T_K_[i][0] =
        (k[j+0][0]*lsqr+k[j+0][1]*usedata->L0+k[j+0][2])* -usedata->theta; 
    output->T_K_[i][1] =
        (k[j+1][0]*lsqr+k[j+1][1]*usedata->L0+k[j+1][2])*-usedata->d_theta;
    output->T_K_[i][2] =
        (k[j+2][0]*lsqr+k[j+2][1]*usedata->L0+k[j+2][2])* output->location;
    output->T_K_[i][3] =
		    (k[j+3][0]*lsqr+k[j+3][1]*usedata->L0+k[j+3][2])*(output->target_speed-output->speed);
    output->T_K_[i][4] =
        (k[j+4][0]*lsqr+k[j+4][1]*usedata->L0+k[j+4][2])* output->pitch;
    output->T_K_[i][5] =
        (k[j+5][0]*lsqr+k[j+5][1]*usedata->L0+k[j+5][2])* -output->w_pitch;
  }

//  if (output->f_support < 20.0f) {
//    for (uint8_t i = 0; i < 6; ++i) {
//      output->T_K_[0][i] = 0.0f;
//    };
//    output->T_K_[1][2] = output->T_K_[1][3] = output->T_K_[1][4] = output->T_K_[1][5] = 0.0f;
//  }

  for (uint8_t i = 0; i < 2; ++i) {
    output->T_O[i] = output->T_K_[i][0] + output->T_K_[i][1] + output->T_K_[i][2]
		                +output->T_K_[i][3] + output->T_K_[i][4] + output->T_K_[i][5];
  }
}