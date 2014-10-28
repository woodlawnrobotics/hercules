/*
 * HMC5883L_inc.h
 *
 *  Created on: Jan 11, 2014
 *      Author: simon
 */

#ifndef HMC5883L_INC_H_
#define HMC5883L_INC_H_
// ---------------------------------------------------------------------[ BoF ]

//math.h extensions:
///* Constants rounded for 21 decimals. */
//#define M_E 2.71828182845904523536
//#define M_LOG2E 1.44269504088896340736
//#define M_LOG10E 0.434294481903251827651
//#define M_LN2 0.693147180559945309417
//#define M_LN10 2.30258509299404568402
//#define M_PI 3.14159265358979323846
//#define M_PI_2 1.57079632679489661923
//#define M_PI_4 0.785398163397448309616
//#define M_1_PI 0.318309886183790671538
//#define M_2_PI 0.636619772367581343076
//#define M_1_SQRTPI 0.564189583547756286948
//#define M_2_SQRTPI 1.12837916709551257390
//#define M_SQRT2 1.41421356237309504880
//#define M_SQRT_2 0.707106781186547524401



typedef enum HMC5883L_enum_tag
{
	// --- measurement type
	hmc5883L_mea_continuous			= 0,
	hmc5883L_mea_singleShot			= 0x01,
	hmc5883L_mea_iddle				= 0x03,

	// --- registers
	hmc5883L_address				= 0x1E,
	hmc5883L_reg_configA			= 0x00,
	hmc5883L_reg_configB			= 0x01,
	hmc5883L_reg_mode				= 0x02,
	hmc5883L_reg_dataBegin			= 0x03,
	hmc5883L_reg_id					= 0x0A,
	hmc5883L_reg_id_value			= 0x48,

	// --- error code
	hmc5883L_err_none				= 0,
	hmc5883L_err_code1				= 1,
	hmc5883L_err_code2				= 2,

	hmc5883L_gaussScale_088			= 0x0088,
	hmc5883L_gaussScale_130			= 0x0130,
	hmc5883L_gaussScale_190			= 0x0190,
	hmc5883L_gaussScale_250			= 0x0250,
	hmc5883L_gaussScale_400			= 0x0400,
	hmc5883L_gaussScale_470			= 0x0470,
	hmc5883L_gaussScale_560			= 0x0560,
	hmc5883L_gaussScale_810			= 0x0810,

} hmc5883L_enum_t;


typedef struct MagnetometerScaled_tag
{
	float XAxis;
	float YAxis;
	float ZAxis;

} magnetometerScaled_t;

typedef struct MagnetometerRaw_tag
{
	int16_t XAxis;
	int16_t YAxis;
	int16_t ZAxis;

} magnetometerRaw_t;

typedef struct MagnetometerData_tag
{
	float 					heading;
	float					headingDeg;

	magnetometerRaw_t 		raw;
	magnetometerScaled_t	scaled;

} magnetometerData_t;


// ---------------------------------------------------------------------[ EoF ]
#endif /* HMC5883L_INC_H_ */
