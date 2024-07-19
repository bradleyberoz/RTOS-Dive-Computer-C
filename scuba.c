/** \file scuba.c

 $NOTES: $
 -	This code has been formatted for Doxygen generation

 $REVISIONS: $
 - 01 Jan 2008	
	- 	Original writing.

 $COPYRIGHT: $
 -	Copyright 2014 Barr Group, LLC. All rights reserved.
**/

#include <stdint.h>

#include "os.h"

#include "scuba.h"


#define	RMV	    (1200)		/**< Respiratory minute volume = 1200 centiLitres / minute */
#define	RHSV	(RMV / 120)	/**< Respiratory half second volume = 10 centiLitres / half_second */


/**
 FUNCTION: gas_rate_in_cl

 DESCRIPTION:
 This computes how much gas is consumed in a half second at a certain depth.

 PARAMETERS:
 -	The current depth in meters

 RETURNS:
 -	The number of centilitres in gas.

 NOTES:


**/
uint32_t
gas_rate_in_cl(uint32_t depth_in_mm)
{
    uint32_t  depth_in_m = depth_in_mm / 1000;	

    /* 10m of water = 1 bar = 100 centibar */
	uint16_t ambient_pressure_in_cb = 100 + (10 * depth_in_m);	
			
	/* Gas consumed at STP = RHSV * ambient pressure / standard pressure */
	return ((RHSV * ambient_pressure_in_cb) / 100);
}

/**
 FUNCTION: gas_to_surface_in_cl

 DESCRIPTION:
 This computes how much gas at STP it would take to surface from the current
 depth, assuming no decompression stops and an ascent rate of ASCENT_RATE_LIMIT.

 It does this via numerical integration. The step size is 1 m.

 PARAMETERS:
 -	The current depth in meters

 RETURNS:
 -	The number of centilitres of gas at STP required to make it to the surface.

 NOTES:


**/
uint32_t
gas_to_surface_in_cl(uint32_t depth_in_mm)
{
	uint32_t	gas = 0;
	uint16_t	halfsecs_to_ascend_1m = (2 * 60) / ASCENT_RATE_LIMIT;;
	uint16_t	ambient_pressure_in_cb;		/* Ambient pressure in centiBar */


	for (uint32_t depth_in_m = depth_in_mm / 1000; depth_in_m > 0; depth_in_m--)
	{
		ambient_pressure_in_cb = 100 + (depth_in_m * 100 / 10);
		gas += (RHSV * halfsecs_to_ascend_1m * ambient_pressure_in_cb) / 100;
	}

	return (gas);
}
