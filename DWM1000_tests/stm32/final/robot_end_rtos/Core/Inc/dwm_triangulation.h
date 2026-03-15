/*
 * dwm_triangulation.h
 *
 *  Created on: Mar 15, 2026
 *      Author: Aneek Mubarak
 */

#ifndef INC_DWM_TRIANGULATION_H_
#define INC_DWM_TRIANGULATION_H_

#include "main.h"
#include "dwm1000.h"
#include <math.h>
//#include "stm32f4xx_hal_spi.h"


void DWM_Triangulation_Init(SPI_HandleTypeDef *hspi);
void DWM_Triangulation_TaskStep(void);
bool DWM_Triangulation_GetPosition(RemotePosition *pos);

bool get_standby_state();


#endif /* INC_DWM_TRIANGULATION_H_ */
