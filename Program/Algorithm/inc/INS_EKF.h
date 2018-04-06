/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _INS_EKF_H_
#define _INS_EKF_H_

#include "Matrix.h"

//16-state q0 q1 q2 q3 Pn Pe Alt Vn Ve Vd bwx bwy bwz bax bay baz
#define INS_EKF_STATE_DIM 16

//9-measurement mx my mz (3D magnetometer) Pn Pe Alt Vn Ve Vd
//unit vector pointing to MagNorth in body coords
//north pos, east pos, altitude
//north vel, east vel, down velocity
#define INS_EKF_MEASUREMENT_DIM 9

#define INS_EKF_HALFPI 1.5707963267948966192313216916398f
#define INS_EKF_PI 3.1415926535897932384626433832795f
#define INS_EKF_TWOPI 6.283185307179586476925286766559f
#define INS_EKF_TODEG(x) ((x) * 57.2957796f)
void EKF(double *xk,double *q,double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
#endif
