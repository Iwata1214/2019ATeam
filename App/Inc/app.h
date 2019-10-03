#ifndef __APP_H
#define __APP_H

/*NO Device mode*/
#define _NO_DEVICE 0

int appTask(void);
int appInit(void);

#define DD_NUM_OF_MD 7
#define DD_NUM_OF_AB 1

#define DD_NUM_OF_SS 1
#define DD_USE_ENCODER1 0
#define DD_USE_ENCODER2 0
#define DD_NUM_OF_SV 0

#define PI1_GPIOxID GPIOBID
#define PI1_GPIOPIN GPIO_PIN_13
#define PI1() ((MW_GPIORead(PI1_GPIOxID,PI1_GPIOPIN)))

#define PI2_GPIOxID GPIOBID
#define PI2_GPIOPIN GPIO_PIN_14
#define PI2() ((MW_GPIORead(PI2_GPIOxID,PI2_GPIOPIN)))

#define PI3_GPIOxID GPIOBID
#define PI3_GPIOPIN GPIO_PIN_15
#define PI3() ((MW_GPIORead(PI3_GPIOxID,PI3_GPIOPIN)))

#define PI4_GPIOxID GPIOBID
#define PI4_GPIOPIN GPIO_PIN_1
#define PI4() ((MW_GPIORead(PI1_GPIOxID,PI1_GPIOPIN)))

/*#define PI5_GPIOxID GPIOCID
#define PI5_GPIOPIN GPIO_PIN_4
#define PI5() ((MW_GPIORead(PI2_GPIOxID,PI2_GPIOPIN)))

#define PI6_GPIOxID GPIOBID
#define PI6_GPIOPIN GPIO_PIN_4
#define PI6() ((MW_GPIORead(PI3_GPIOxID,PI3_GPIOPIN))*/




/*
#define _SW_CENTER_LIMIT_GPIOxID
#define _SW_CENTER_LIMIT_GPIOPIN
#define _IS_PRESSED_CENTER_LIMIYSW()*/

#include "DD_RC.h"
#include "DD_MD.h"
#include "DD_SV.h"

#define MECHA1_MD0 0 //駆動右前側のモータ
#define MECHA1_MD1 1 //駆動右後側のモータ 
#define MECHA1_MD2 2 //駆動左後側のモータ
#define MECHA1_MD3 3 //駆動左前側のモータ
#define MECHA1_MD4 4 //腕用のモータ
#define MECHA1_MD5 5 //手用のモータ
#define MECHA1_MD6 6 //装填機構用のモータ


  
#define CENTRAL_THRESHOLD 4


#define ON_AB0 (1<<0) //0x01
#define ON_AB1 (1<<1) //0x02
#define ON_AB2 (1<<2) //0x04
#define ON_AB3 (1<<3) //0x08
//#define ON_AB4 (1<<4) //0x10

#define NO_BLOW 1000

#define MD_GAIN ( DD_MD_MAX_DUTY / DD_RC_ANALOG_MAX )

#endif
