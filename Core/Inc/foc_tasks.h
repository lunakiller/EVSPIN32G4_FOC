/*
 * foc_tasks.h
 *
 *  Created on: Mar 16, 2024
 *      Author: lunakiller
 *
 */

#ifndef __FOC_TASKS_H
#define __FOC_TASKS_H


void FOC_Modulator(float Valpha, float Vbeta, uint32_t* Va, uint32_t* Vb, uint32_t* Vc);
void FOC_Task(void);


#endif /* __FOC_TASKS_H */
