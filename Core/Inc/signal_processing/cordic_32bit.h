#ifndef INC_SIGNAL_PROCESSING_CORDIC_32BIT_H_
#define INC_SIGNAL_PROCESSING_CORDIC_32BIT_H_

#include <math.h>
//Cordic in 32 bit signed fixed point math
//Function is valid for arguments in range -pi/2 -- pi/2
//for values pi/2--pi: value = half_pi-(theta-half_pi) and similarly for values -pi---pi/2
//
// 1.0 = 1073741824
// 1/k = 0.6072529350088812561694
// pi = 3.1415926535897932384626
//Constants
#define cordic_1K 0x26DD3B6A
#define two_pi M_PI*2
#define half_pi M_PI/2
#define MUL 1073741824.000000
#define CORDIC_NTAB 32
const int cordic_ctab [] = {0x3243F6A8, 0x1DAC6705, 0x0FADBAFC, 0x07F56EA6, 0x03FEAB76, 0x01FFD55B,
0x00FFFAAA, 0x007FFF55, 0x003FFFEA, 0x001FFFFD, 0x000FFFFF, 0x0007FFFF, 0x0003FFFF,
0x0001FFFF, 0x0000FFFF, 0x00007FFF, 0x00003FFF, 0x00001FFF, 0x00000FFF, 0x000007FF,
0x000003FF, 0x000001FF, 0x000000FF, 0x0000007F, 0x0000003F, 0x0000001F, 0x0000000F,
0x00000008, 0x00000004, 0x00000002, 0x00000001, 0x00000000, };

inline void cordic(int theta, int *s, int *c, int n)
{
  int k, d, tx, ty, tz;
  int x=cordic_1K,y=0,z=theta;
  n = (n>CORDIC_NTAB) ? CORDIC_NTAB : n;
  for (k=0; k<n; ++k)
  {
    d = z>>31;
    //get sign. for other architectures, you might want to use the more portable version
    //d = z>=0 ? 0 : -1;
    tx = x - (((y>>k) ^ d) - d);
    ty = y + (((x>>k) ^ d) - d);
    tz = z - ((cordic_ctab[k] ^ d) - d);
    x = tx; y = ty; z = tz;
  }
 *c = x; *s = y;
}

inline float shiftAngle180(float phi)
{
	while(phi < -M_PI || phi > M_PI)
	{
		if (phi < -M_PI)
			phi += two_pi;
		else if (phi > M_PI)
			phi -= two_pi;
	}

	return phi;
}

inline int shiftAngle90(float* phi)
{
    if (*phi < -half_pi)
    {
        *phi = *phi + M_PI;
        return -1;
    }
    else if (*phi > half_pi)
    {
        *phi = *phi - M_PI;
        return -1;
    }
    else
    {
        return 1;
    }
}

inline void sinCordic(float theta, float* sinC)
{
	int s, c, quadAdj;
	float p;
	p = shiftAngle180(theta);
    quadAdj = shiftAngle90(&p);
	cordic((p*MUL), &s, &c, 32);
	*sinC = quadAdj*s/MUL;
}

inline void cosCordic(float theta, float* cosC)
{
	int s, c, quadAdj;
	float p;
	p = shiftAngle180(theta);
    quadAdj = shiftAngle90(&p);
	cordic((p*MUL), &s, &c, 32);
	*cosC = quadAdj*c/MUL;
}

inline void sincosCordic(float theta, float* sinC, float* cosC)
{
	int s, c, quadAdj;
	float p;
	p = shiftAngle180(theta);
    quadAdj = shiftAngle90(&p);
	cordic((p*MUL), &s, &c, 32);
	*sinC = quadAdj*s/MUL;
	*cosC = quadAdj*c/MUL;
}

#endif /* INC_SIGNAL_PROCESSING_CORDIC_32BIT_H_ */
