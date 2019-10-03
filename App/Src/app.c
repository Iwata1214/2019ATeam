#include "app.h"
#include "DD_Gene.h"
#include "DD_RCDefinition.h"
#include "SystemTaskManager.h"
#include <stdlib.h>
#include "MW_GPIO.h"
#include "MW_IWDG.h"
#include "message.h"
#include "MW_flash.h"
#include "constManager.h"
#include <trapezoid_ctrl.h>
#include <math.h>
#include <stdlib.h>

#define AB_TIME 80

#define IDX0_QUAD1 1.0
#define IDX0_QUAD2 1.0
#define IDX0_QUAD3 2.0
#define IDX0_QUAD4 1.0

#define IDX1_QUAD1 1.3
#define IDX1_QUAD2 0.5
#define IDX1_QUAD3 1.5
#define IDX1_QUAD4 0.5

#define IDX2_QUAD1 0.7
#define IDX2_QUAD2 1.0
#define IDX2_QUAD3 0.5
#define IDX2_QUAD4 1.0

#define IDX3_QUAD1 1.0
#define IDX3_QUAD2 1.0
#define IDX3_QUAD3 1.0
#define IDX3_QUAD4 1.0





#define ARM_POSI __RC_ISPRESSED_RIGHT(g_rc_data)
#define ARM_REVERSE __RC_ISPRESSED_LEFT(g_rc_data)
#define ARM_AB_INJE __RC_ISPRESSED_DOWN(g_rc_data)
#define ARM_AB_RETURN __RC_ISPRESSED_L2(g_rc_data) && __RC_ISPRESSED_DOWN(g_rc_data)

#define HAND_POSI __RC_ISPRESSED_L1(g_rc_data) && __RC_ISPRESSED_L2(g_rc_data)
#define HAND_REVERSE __RC_ISPRESSED_L1(g_rc_data)
#define FIRST_POSI __RC_ISPRESSED_START(g_rc_data)
#define LOADING_POSI __RC_ISPRESSED_R1(g_rc_data) && __RC_ISPRESSED_CROSS(g_rc_data)
#define LOADING_REVERSE __RC_ISPRESSED_R1(g_rc_data)
 

  


static
int SusSystem(void);

static
int ArmSystem(void);

static
int Arm_ABSystem(void);

static
int HandSystem(void);

static
int LoadingSystem(void);

static
int ReleaseSystem(void);

static
int CollectSystem(void);

/*メモ
 *g_ab_h...ABのハンドラ
 *g_md_h...MDのハンドラ
 *
 *g_rc_data...RCのデータ
 */

int appInit(void){

  ad_init();

  /*GPIO の設定などでMW,GPIOではHALを叩く*/
  return EXIT_SUCCESS;
}

/*application tasks*/
int appTask(void){
  int ret=0;


  /*それぞれの機構ごとに処理をする*/
  /*途中必ず定数回で終了すること。*/
    
    
  ret = SusSystem();
  if(ret){
    return ret;
  }

  ret = ArmSystem();
  if(ret){
    return ret;
  }

  ret = Arm_ABSystem();
  if(ret){
    return ret;
  }

  ret = HandSystem();
  if(ret){
    return ret;
  }

  ret = LoadingSystem();
  if(ret){
    return ret;
  }

  ret = ReleaseSystem();
  if(ret){
    return ret;
  }

  ret = CollectSystem();
  if(ret){
    return ret;
  }
  
  return EXIT_SUCCESS;
}


  
    

static
int SusSystem(void){
  const int num_of_motor = 4;/*モータの個数*/
  //  int rc_analogdata;/*アナログデータ*/
  unsigned int idx=0;/*インデックス*/
  int i;
  int quad = 0;//象限判定
  double x,y,w;
  double upd_x=0,upd_y=0;
  double m=0;
  double degree = 0;//角度
  int adjust;
  
  const tc_const_t tc ={
    .inc_con = 200,
    .dec_con = 2000,
  };
  
  x = DD_RCGetLY(g_rc_data);
  y = DD_RCGetLX(g_rc_data);
  w = DD_RCGetRX(g_rc_data);

  upd_x=x;//x*cos(M_PI_4)-y*sin(M_PI_4);
  upd_y=y;//y*cos(M_PI_4)+x*sin(M_PI_4);
  /* if(__RC_ISPRESSED_R2(g_rc_data)){
     w = 60;
     }else if(__RC_ISPRESSED_L2(g_rc_data)){
     w = -60;
     } else {
     w = 0;
     }*/

  /*if(upd_x > 0 && upd_y <= 0){
    quad = 3;
  }else if(upd_x <= 0 && upd_y < 0){
    quad = 2;
  }else if(upd_x < 0 && upd_y >= 0){
    quad = 1;
  }else if(upd_x >= 0 && upd_y > 0){
    quad = 4;
    }*/

  /*x座標、y座標で象限の判定*/
  if(x > 0 && y <= 0){
    quad = 3;
  }else if(x <= 0 && y < 0){
    quad = 2;
  }else if(x < 0 && y >= 0){
    quad = 1;
  }else if(x >= 0 && y > 0){
    quad = 4;
  }

  /*xとyの絶対値でarctanを取り角度を求める*/
  degree=atan2(fabs(x),fabs(y))*180/M_PI; 
    


  /*for each motor*/
  for(i=0;i<num_of_motor;i++){
    /*それぞれの差分*/
    switch(i){
    case 0:
      idx = MECHA1_MD0; 
      m = -upd_x - upd_y + w; //x + y - w;
      m *= 75;
      if(abs(m) <= 2100) {//dutyが低かったら引き上げ
	m *= 2;
      } else if(abs(m) >= 4200) {//dutyが6500を超えたら6500以下になるよう調整
	adjust = abs(m) - 4200;
	if(m > 0) {
	  m -= adjust;
	} else if(m < 0) {
	  m += adjust;
	}
      }
      break;
      
    case 1:
      idx = MECHA1_MD1; 
      m = - upd_x + upd_y + w; //x - y - w;
      m *= 75;
      if(abs(m) <= 2100) {//dutyが低かったら引き上げ
	m *= 2;
      } else if(abs(m) >= 4200) {//dutyが6500を超えたら6500以下になるよう調整
	adjust = abs(m) - 4200;
	if(m > 0) {
	  m -= adjust;
	} else if(m < 0) {
	  m += adjust;
	}
      }
      break;
      
    case 2:
      idx = MECHA1_MD2; 
      m = upd_x + upd_y + w; //- x - y - w;
      m *= 75;
      if(abs(m) <= 2100) {//dutyが低かったら引き上げ
	m *= 2;
      } else if(abs(m) >= 4200) {//dutyが6500を超えたら6500以下になるよう調整
	adjust = abs(m) - 4200;
	if(m > 0) {
	  m -= adjust;
	} else if(m < 0) {
	  m += adjust;
	}
      }
      break;
      
    case 3:
      idx = MECHA1_MD3; 
      m = upd_x - upd_y + w; //- x + y - w;
      m *= 75;
      if(abs(m) <= 2100) {//dutyが低かったら引き上げ
	m *= 2;
      } else if(abs(m) >= 4200) {//dutyが6500を超えたら6500以下になるよう調整
	adjust = abs(m) - 4200;
	if(m > 0) {
	  m -= adjust;
	} else if(m < 0) {
	  m += adjust;
	}
      }
      break;      
      return EXIT_FAILURE;
    }

    if(idx==1){
      if((quad == 2 || quad == 3) && degree < 45 && idx == 1){
	trapezoidCtrl((int)(m*0.5),&g_md_h[idx],&tc);
      }else if(((quad == 1 || quad == 4) && degree < 45) && idx==1){
	trapezoidCtrl((int)(m*0.5),&g_md_h[idx],&tc);
      }else{
	trapezoidCtrl((int)m*2,&g_md_h[idx],&tc);
      }
    }else if((idx==2 && w==0) || (idx==3 && w==0)){
      if((quad == 1 || quad == 4) && degree < 45 && idx == 2){
	trapezoidCtrl((int)m*0.8,&g_md_h[idx],&tc);
      }else if((quad == 2 || quad == 3) && degree < 45 && idx == 2){
	trapezoidCtrl((int)m*0.8,&g_md_h[idx],&tc);
      }else if((quad == 1 || quad == 4) && degree < 45 && idx == 3){
	trapezoidCtrl((int)(m*0.7),&g_md_h[idx],&tc);
      }else if((quad == 2 || quad == 3) && degree < 45 && idx == 3){
	trapezoidCtrl((int)(m*0.7),&g_md_h[idx],&tc);
      }else{
	trapezoidCtrl((int)m,&g_md_h[idx],&tc);
      }
    }else{
      trapezoidCtrl((int)m,&g_md_h[idx],&tc);
    }
  }
    
  
  MW_printf("Quadrant:%d degree:%d\n°",quad,(int)degree);
  
  return EXIT_SUCCESS;
}

static
int ArmSystem(void){
  const tc_const_t arm_tcon = {
    .inc_con = 250,
    .dec_con = 250,
  };
  
  int arm_duty = 0;


  if(ARM_POSI){
    arm_duty = 8000;
    trapezoidCtrl(arm_duty,&g_md_h[MECHA1_MD4],&arm_tcon);
  }else if(ARM_REVERSE){
    arm_duty = -8000;
    trapezoidCtrl(arm_duty,&g_md_h[MECHA1_MD4],&arm_tcon);
  }else{
    trapezoidCtrl(0,&g_md_h[MECHA1_MD4],&arm_tcon);
  }

    
  return EXIT_SUCCESS;
}

static
int Arm_ABSystem(void){

  static int check = 0;
  
  if(ARM_AB_INJE && check == 0){
    check = 1; 
  }
  
  if(check == 1){
    g_ab_h[0].dat |= ON_AB0;
    if(ARM_AB_RETURN){
      check = 0;
      g_ab_h[0].dat &= ~ON_AB0;
    }
  }
  
  return EXIT_SUCCESS;
}
  

static
int HandSystem(void){
  const tc_const_t hand_tcon = {
    .inc_con = 250,
    .dec_con = 250,
  };

  int hand_duty = 0;
  static int hand_check = 0;
  static int count=0;

  if(HAND_POSI){
    hand_check = 1;
  }
  
  /*if(HAND_POSI){
    hand_duty = -5000;
    trapezoidCtrl(hand_duty,&g_md_h[MECHA1_MD5],&hand_tcon);
    }else*/
  if(hand_check == 1 && count < 30){
    hand_duty = -5000;
    trapezoidCtrl(hand_duty,&g_md_h[MECHA1_MD5],&hand_tcon);
    count++;
  }else if(HAND_REVERSE){
    hand_duty =  5000;
    trapezoidCtrl(hand_duty,&g_md_h[MECHA1_MD5],&hand_tcon);
  }else{
    count = 0;
    hand_check = 0;
    trapezoidCtrl(0,&g_md_h[MECHA1_MD5],&hand_tcon);
  }
  
  MW_printf("count_hand = %2d\n",count);
  
  return EXIT_SUCCESS;
}

static
int LoadingSystem(void){
  const tc_const_t loading_tcon = {
    .inc_con = 250,
    .dec_con = 250,
  };

  const tc_const_t go_tcon = {
    .inc_con = 500,
    .dec_con = 1000,
  };

  int loading_duty = 0;
  static int check_first = 0;
  static int check_load = 0;

  if(FIRST_POSI && check_first == 0){
    check_first = 1;
  }

  if(LOADING_POSI && check_load == 0){
    check_load =1;
  }

  MW_printf("PI1[%d] PI2[%d]\n",PI1(),PI2());
  MW_printf("check_first[%d] check_load[%d]\n",check_first,check_load);
  MW_printf("START_PUSH=%d\n",FIRST_POSI);
	    
  

  if(check_first == 1 && PI2() == 1){
    loading_duty = 4000;
    trapezoidCtrl(loading_duty,&g_md_h[MECHA1_MD6],&loading_tcon);
  }else if(check_load == 1 && PI1() == 0){
    check_load = 2;
  }else if(check_load == 2 && PI2() == 1){
    loading_duty = 3000;
    trapezoidCtrl(loading_duty,&g_md_h[MECHA1_MD6],&loading_tcon);
  }else if(check_load == 1 && PI1() == 1){
    loading_duty = -3000;
    trapezoidCtrl(loading_duty,&g_md_h[MECHA1_MD6],&go_tcon);
  }else if(__RC_ISPRESSED_SELECT(g_rc_data)){
    loading_duty = 2000;
    trapezoidCtrl(loading_duty,&g_md_h[MECHA1_MD6],&loading_tcon);
  }else{
    check_load = 0;
    check_first = 0;
    trapezoidCtrl(0,&g_md_h[MECHA1_MD6],&loading_tcon);
  }
      

  return EXIT_SUCCESS;
}

static
int ReleaseSystem(void){

  static int count = 0;
  static int check = 0;
  
  if(__RC_ISPRESSED_R2(g_rc_data) && __RC_ISPRESSED_CIRCLE(g_rc_data)){
    check = 1;
  }
  if(check == 1 && (count < 40 && count > 10)){
      g_ab_h[0].dat |= ON_AB1;
      count++;
  }else if(check == 1 && count < 11){
    count++;
  }else{
    g_ab_h[0].dat &= ~ON_AB1;
    count=0;
    check = 0;
  }
  MW_printf("count_release=%2d\n",count);
  
  return EXIT_SUCCESS;
}

int CollectSystem(void){

  static int count = 0;
  static int check_collect = 0;
  static int check_release = 0;

  if( (__RC_ISPRESSED_TRIANGLE(g_rc_data)) ){
    check_collect = 1;
  }else if((__RC_ISPRESSED_R2(g_rc_data) && __RC_ISPRESSED_CIRCLE(g_rc_data))){
    check_release = 1;
  }
  
  if(check_collect == 1 && count < 60){
    if(count < 37){ 
      g_ab_h[0].dat |= ON_AB2;
    }else{
      g_ab_h[0].dat &= ~ON_AB2;
    }
    if(count > 19 && count < 56){
      g_ab_h[0].dat |= ON_AB3;
    }else if(count > 56){
      g_ab_h[0].dat &= ~ON_AB3;
    }
    count++;
  }else if(check_release == 1 && count < 60){
    if(count < 37){
      g_ab_h[0].dat |= ON_AB2;
    }else{
      g_ab_h[0].dat &= ~ON_AB3;
    }
    if(count > 10 && count < 56){
      g_ab_h[0].dat |= ON_AB3;
    }else if(count > 36){
      g_ab_h[0].dat &= ~ON_AB2;
    }
    count++;
  }else{
    g_ab_h[0].dat &= ~ON_AB3;
    check_release = 0;
    count = 0;
    check_collect = 0;
  }
  
  return EXIT_SUCCESS;
}

