/*
 * GPIO_defs.h
 *
 *  Created on: Feb 11, 2022
 *      Author: 240040842
 */
#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H

// basic light switch
#define LED1_POS (1)	// on port A
#define LED2_POS (2)	// on port A
#define SW1_POS (5)		// on port A

#define MASK(x) (1UL << (x))

// Speaker output
#define SPKR_POS (0) 	// on port C

#endif
