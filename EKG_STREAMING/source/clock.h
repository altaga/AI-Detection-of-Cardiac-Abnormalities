/******************************************************************************
* File Name:   clock.h
*
* Description: This file provides a clock.
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <stdint.h>

/*******************************************************************************
* Types
*******************************************************************************/

/* uint32_t will wrap around every 12 hour if CLOCK_TICK_PER_SECOND equals 100000.
 * To avoid this change clock_tick_t to uint64_t.
 */
typedef uint64_t clock_tick_t;

/*******************************************************************************
* Defines
*******************************************************************************/

/* Number of counts per second */
#define CLOCK_TICK_PER_SECOND 100000

/* Interrupt Priority Level  */
#define CLOCK_INTERRUPT_PRIORITY  3

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

bool clock_init(void);
clock_tick_t clock_get_tick();

#endif /* _CLOCK_H_ */

/* [] END OF FILE */
