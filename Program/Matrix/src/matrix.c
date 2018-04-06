/* $XConsortium: matrix.c,v 5.2 94/04/17 20:44:40 rws Exp $ */
/*

Copyright (c) 1989, 1990, 1991  X Consortium

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of the X Consortium shall
not be used in advertising or otherwise to promote the sale, use or
other dealings in this Software without prior written authorization
from the X Consortium.

*/
/***********************************************************
Copyright(c) 1989,1990, 1991 by Sun Microsystems, Inc.

						All Rights Reserved

Permission to use, copy, modify, and distribute this software and its
documentation for any purpose and without fee is hereby granted,
provided that the above copyright notice appear in all copies and that
both that copyright notice and this permission notice appear in
supporting documentation, and that Sun Microsystems
not be used in advertising or publicity
pertaining to distribution of the software without specific, written
prior permission.

SUN MICROSYSTEMS DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT
SHALL SUN MICROSYSTEMS BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL
DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
SOFTWARE.

******************************************************************/

/*--------------------------------------------------------------------*\
|
|  Copyright (C) 1989,1990, 1991, National Computer Graphics Association
|
|  Permission is granted to any individual or institution to use, copy, or
|  redistribute this software so long as it is not sold for profit, provided
|  this copyright notice is retained.
|
|                         Developed for the
|                National Computer Graphics Association
|                         2722 Merrilee Drive
|                         Fairfax, VA  22031
|                           (703) 698-9600
|
|                                by
|                 SimGraphics Engineering Corporation
|                    1137 Huntington Drive  Unit A
|                      South Pasadena, CA  91030
|                           (213) 255-0900
|---------------------------------------------------------------------
|
| Author   	:	nde / SimGraphics Engineering Corportation
|
| File     	:	matrix.c
| Date     	:	3/16/89
| Project  	:	PLB
| Description	:	Matrix formation utilities
| Status   	:	Version 1.0
|
| Revisions	:	
|
|	6/28/89		Added matcat.
|
\*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*\
|	Table of Contents
|
|	mx_identity(matrix4)
|		:	Generate an identity matrix
|	void mx_rot_x(float, *float)
|		:	Generate an X rotation matrix
|	mx_rot_y(float, *float)
|		:	Generate a Y rotation matrix
|	mx_rot_z(float, *float)
|		:	Generate a Z rotation matrix
|	mx_scale(float, float, float, *float)
|		:	Generate a scale matrix
|	mx_translate(float, float, float, *float)
|		:	Generate a translation matrix
|	mx_euler_matrix(float, float, float, *float)
|		:	Generate an euler angles  matrix
|	mx_mult(*float, *float, *float)
|		:	Multiply two 4x4 matrices
|	mx_44mult(*float, *float, *float)
|		:	Multiply two 4x4 matrices
|	mx_invert(*float, *float)
|		:	do a GEBS invert of a 4x4 with pivoting
|	int matcat(mat_from, mat_to, concat)
|		:	Concatenate the mat_from and mat_to according to
|	int copyvec3f(a,b)
|		:	copy a into b
|	double dotvec3f(a,b)
|		:	Compute and return the dot product of a and b
|	int crossvec3f(a,b,c)
|		:	Cross a with b return in c
|	int unitvec(vector3)
|		:	Normalize the given vector.
|	int copy44f(mata,matb)
|		:	copy mata into matb (4x4 matrix of type float)
|	print44f(mat)
|		:	print a 4x4 matrix of type float
|	mx_comp(mat1,mat2)
|		:	compares two 4x4 matrix returns TRUE if identical.
|
\*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*\
|	Include files
\*--------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include "brftypes.h"
//#include "bifbuild.h"
#include "bifmacro.h"
#include "ph_map.h"

/*--------------------------------------------------------------------*\
|	Local #define
\*--------------------------------------------------------------------*/
#define RADIANS(x) 0.01745329 * (x)
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

/*--------------------------------------------------------------------*\
|	Local global variables
\*--------------------------------------------------------------------*/
typedef double Vector4[4];

typedef double Pmatrix3[4][4] ;

/* ---------------------------------------------------------------------*\
| BEGIN PROCEDURE CODE                                                   |
\*--------------------------------------------------------------------- */

/*--------------------------------------------------------------------*\
| Procedure	:	mx_identity(matrix4)
|----------------------------------------------------------------------
| Description	:	Generate an identity matrix
|----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/
void mx_identity (double (*mx)[4])
{
	mx[ 0 ][ 0 ] =  1.0;
	mx[ 0 ][ 1 ] =  0.0;
	mx[ 0 ][ 2 ] =  0.0;
	mx[ 0 ][ 3 ] =  0.0;

	mx[ 1 ][ 0 ] =  0.0;
	mx[ 1 ][ 1 ] =  1.0;
	mx[ 1 ][ 2 ] =  0.0;
	mx[ 1 ][ 3 ] =  0.0;

	mx[ 2 ][ 0 ] =  0.0;
	mx[ 2 ][ 1 ] =  0.0;
	mx[ 2 ][ 2 ] =  1.0;
	mx[ 2 ][ 3 ] =  0.0;

	mx[ 3 ][ 0 ] =  0.0;
	mx[ 3 ][ 1 ] =  0.0;
	mx[ 3 ][ 2 ] =  0.0;
	mx[ 3 ][ 3 ] =  1.0;      
}/* End mx_identity */


/*--------------------------------------------------------------------*\
| Procedure	:	void mx_rot_x(float, *float)
|-----------------------------------------------------------------------
| Description	:	Generate an X rotation matrix
|-----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/
void mx_rot_x (double angle,double (*mx)[4] )
{/* mx_rot_x */

	float r_angle;
	float cos_rho, sin_rho;

	r_angle = RADIANS( angle );
	cos_rho = cos( r_angle );
	sin_rho = sin( r_angle );

	mx[ 0 ][ 0 ] =  1.0;
	mx[ 0 ][ 1 ] =  0.0;
	mx[ 0 ][ 2 ] =  0.0;
	mx[ 0 ][ 3 ] =  0.0;

	mx[ 1 ][ 0 ] =  0.0;
	mx[ 1 ][ 1 ] =  cos_rho;
	mx[ 1 ][ 2 ] =  sin_rho;
	mx[ 1 ][ 3 ] =  0.0;

	mx[ 2 ][ 0 ] =  0.0;
	mx[ 2 ][ 1 ] = -sin_rho;
	mx[ 2 ][ 2 ] =  cos_rho;
	mx[ 2 ][ 3 ] =  0.0;

	mx[ 3 ][ 0 ] =  0.0;
	mx[ 3 ][ 1 ] =  0.0;
	mx[ 3 ][ 2 ] =  0.0;
	mx[ 3 ][ 3 ] =  1.0;

}/* mx_rot_x */

/*--------------------------------------------------------------------*\
| Procedure	:	mx_rot_y(float, *float)
|-----------------------------------------------------------------------
| Description	:	Generate a Y rotation matrix
|-----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/
void mx_rot_y (double angle,double (*mx)[4] )
{/* mx_rot_y */

	float r_angle;
	float cos_rho, sin_rho;

	r_angle = RADIANS( angle );
	cos_rho = cos( r_angle );
	sin_rho = sin( r_angle );

	mx[ 0 ][ 0 ] =  cos_rho;
	mx[ 0 ][ 1 ] =  0.0;
	mx[ 0 ][ 2 ] = -sin_rho;
	mx[ 0 ][ 3 ] =  0.0;

	mx[ 1 ][ 0 ] =  0.0;
	mx[ 1 ][ 1 ] =  1.0;
	mx[ 1 ][ 2 ] =  0.0;
	mx[ 1 ][ 3 ] =  0.0;

	mx[ 2 ][ 0 ] =  sin_rho;
	mx[ 2 ][ 1 ] =  0.0;
	mx[ 2 ][ 2 ] =  cos_rho;
	mx[ 2 ][ 3 ] =  0.0;

	mx[ 3 ][ 0 ] =  0.0;
	mx[ 3 ][ 1 ] =  0.0;
	mx[ 3 ][ 2 ] =  0.0;
	mx[ 3 ][ 3 ] =  1.0;

}/* mx_rot_y */

/*--------------------------------------------------------------------*\
| Procedure	:	mx_rot_z(float, *float)
|-----------------------------------------------------------------------
| Description	:	Generate a Z rotation matrix
|-----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/
void mx_rot_z (double angle,double (*mx)[4] )
{/* mx_rot_z */

	float r_angle;
	float cos_rho, sin_rho;

	r_angle = RADIANS( angle );
	cos_rho = cos( r_angle );
	sin_rho = sin( r_angle );

	mx[ 0 ][ 0 ] =  cos_rho;
	mx[ 0 ][ 1 ] =  sin_rho;
	mx[ 0 ][ 2 ] =  0.0;
	mx[ 0 ][ 3 ] =  0.0;

	mx[ 1 ][ 0 ] = -sin_rho;
	mx[ 1 ][ 1 ] =  cos_rho;
	mx[ 1 ][ 2 ] =  0.0;
	mx[ 1 ][ 3 ] =  0.0;

	mx[ 2 ][ 0 ] =  0.0;
	mx[ 2 ][ 1 ] =  0.0;
	mx[ 2 ][ 2 ] =  1.0;
	mx[ 2 ][ 3 ] =  0.0;

	mx[ 3 ][ 0 ] =  0.0;
	mx[ 3 ][ 1 ] =  0.0;
	mx[ 3 ][ 2 ] =  0.0;
	mx[ 3 ][ 3 ] =  1.0;

}/* mx_rot_z */

/*--------------------------------------------------------------------*\
| Procedure	:	mx_scale(float, float, float, *float)
|-----------------------------------------------------------------------
| Description	:	Generate a scale matrix
|-----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/
void mx_scale (double sx,double sy,double sz,double (*mx)[4] )
{/* mx_scale */

	mx[ 0 ][ 0 ] =  sx;
	mx[ 0 ][ 1 ] =  0.0;
	mx[ 0 ][ 2 ] =  0.0;
	mx[ 0 ][ 3 ] =  0.0;

	mx[ 1 ][ 0 ] =  0.0;
	mx[ 1 ][ 1 ] =  sy;
	mx[ 1 ][ 2 ] =  0.0;
	mx[ 1 ][ 3 ] =  0.0;

	mx[ 2 ][ 0 ] =  0.0;
	mx[ 2 ][ 1 ] =  0.0;
	mx[ 2 ][ 2 ] =  sz;
	mx[ 2 ][ 3 ] =  0.0;

	mx[ 3 ][ 0 ] =  0.0;
	mx[ 3 ][ 1 ] =  0.0;
	mx[ 3 ][ 2 ] =  0.0;
	mx[ 3 ][ 3 ] =  1.0;

}/* mx_scale */

/*--------------------------------------------------------------------*\
| Procedure	:	mx_translate(float, float, float, *float)
|-----------------------------------------------------------------------
| Description	:	Generate a translation matrix
|-----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/
void mx_translate (double tx,double ty,double tz,double (*mx)[4] )
{/* mx_translate */

	mx[ 0 ][ 0 ] =  1.0;
	mx[ 0 ][ 1 ] =  0.0;
	mx[ 0 ][ 2 ] =  0.0;
	mx[ 0 ][ 3 ] =  0.0;

	mx[ 1 ][ 0 ] =  0.0;
	mx[ 1 ][ 1 ] =  1.0;
	mx[ 1 ][ 2 ] =  0.0;
	mx[ 1 ][ 3 ] =  0.0;

	mx[ 2 ][ 0 ] =  0.0;
	mx[ 2 ][ 1 ] =  0.0;
	mx[ 2 ][ 2 ] =  1.0;
	mx[ 2 ][ 3 ] =  0.0;

	mx[ 3 ][ 0 ] =  tx;
	mx[ 3 ][ 1 ] =  ty;
	mx[ 3 ][ 2 ] =  tz;
	mx[ 3 ][ 3 ] =  1.0;

}/* mx_translate */

/*--------------------------------------------------------------------*\
| Procedure	:	mx_euler_matrix(float, float, float, *float)
|-----------------------------------------------------------------------
| Description	:	Generate an euler angles  matrix
|-----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/
void mx_euler_matrix (double ax,double ay,double az,double (*mat)[4] )
{/* mx_euler_matrix */

	float rax, ray, raz;
	float sin_ax, sin_ay, sin_az;
	float cos_ax, cos_ay, cos_az;

	rax = RADIANS( ax );
	ray = RADIANS( ay );
	raz = RADIANS( az );
	sin_ax = sin( rax );
	sin_ay = sin( ray );
	sin_az = sin( raz );
	cos_ax = cos( rax );
	cos_ay = cos( ray );
	cos_az = cos( raz );

	mat[ 0 ][ 0 ] =  cos_ay*cos_az;
	mat[ 0 ][ 1 ] =  cos_ay*sin_az;
	mat[ 0 ][ 2 ] = -sin_ay;
	mat[ 0 ][ 3 ] =  0.;

	mat[ 1 ][ 0 ] =  sin_ax*sin_ay*cos_az - cos_ax*sin_az;
	mat[ 1 ][ 1 ] =  sin_ax*sin_ay*sin_az + cos_ax*cos_az;
	mat[ 1 ][ 2 ] =  sin_ax*cos_ay;
	mat[ 1 ][ 3 ] =  0.;

	mat[ 2 ][ 0 ] =  cos_ax*sin_ay*cos_az + sin_ax*sin_az;
	mat[ 2 ][ 1 ] =  cos_ax*sin_ay*sin_az - sin_ax*cos_az;
	mat[ 2 ][ 2 ] =  cos_ax*cos_ay;
	mat[ 2 ][ 3 ] =  0.;

	mat[ 3 ][ 0 ] =  0.;
	mat[ 3 ][ 1 ] =  0.;
	mat[ 3 ][ 2 ] =  0.;
	mat[ 3 ][ 3 ] =  1.;

}/* mx_euler_matrix */


/*--------------------------------------------------------------------*\
| Procedure	:	mx_44mult(*float, *float, *float)
|-----------------------------------------------------------------------
| Description	:	Multiply two 4x4 matrices
|-----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/

void mx_44mult(double (*matrix)[4],Vector4 vector,Vector4 result)
{
	double *row1,*row2,*row3,*row4;
	
	row1 = matrix[0];
	row2 = matrix[1];
	row3 = matrix[2];
	row4 = matrix[3];

	result[0] =row1[0] * vector[0] + row2[0] * vector[1] +
	            row3[0] * vector[2] + row4[0] * vector[3];

	result[1] = row1[1] * vector[0] + row2[1] * vector[1] +
	            row3[1] * vector[2] + row4[1] * vector[3];

	result[2] = row1[2] * vector[0] + row2[2] * vector[1] +
	            row3[2] * vector[2] + row4[2] * vector[3];

	result[3] = row1[3] * vector[0] + row2[3] * vector[1] +
	            row3[3] * vector[2] + row4[3] * vector[3];
}

void mx_4MVmult(double (*matrix)[4],double *vector,double * result)
{

	result[0] =matrix[0][0] * vector[0] + matrix[0][1]  * vector[1] +
	           matrix[0][2]  * vector[2] + matrix[0][3]  * vector[3];

	result[1] =matrix[1][0]  * vector[0] +matrix[1][1]  * vector[1] +
	            matrix[1][2]  * vector[2] + matrix[1][3]  * vector[3];

	result[2] =matrix[2][0]  * vector[0] +matrix[2][1]  * vector[1] +
	            matrix[2][2]  * vector[2] + matrix[2][3]  * vector[3];

	result[3] = matrix[3][0]  * vector[0] + matrix[3][1]  * vector[1] +
	            matrix[3][2] * vector[2] + matrix[3][3]  * vector[3];
}
/*--------------------------------------------------------------------*\
| Procedure	:	mx_mult(*float, *float, *float)
|-----------------------------------------------------------------------
| Description	:	Multiply two 4x4 matrices
|-----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/
//void mx_mult(Pmatrix3 matrix1,Pmatrix3 matrix2,Pmatrix3 result) 
void mx_mult(double (*matrix1)[4],double (*matrix2)[4],double (*result)[4]) 
{
	double *row11,*row12,*row13,*row14;
	double *row21,*row22,*row23,*row24;

	row11 = matrix1[0];
	row12 = matrix1[1];
	row13 = matrix1[2];
	row14 = matrix1[3];

	row21 = result[0];
	row22 = result[1];
	row23 = result[2];
	row24 = result[3];

	mx_44mult(matrix2,row11,row21);
	mx_44mult(matrix2,row12,row22);
	mx_44mult(matrix2,row13,row23);
	mx_44mult(matrix2,row14,row24);
}

/*--------------------------------------------------------------------*\
| Procedure	:	mx_invert(*float, *float)
|-----------------------------------------------------------------------
| Description	:	do a GEBS invert of a 4x4 with pivoting
|-----------------------------------------------------------------------
| Return	:	
\*--------------------------------------------------------------------*/
/* Invert the Matrix */
int mx_invert(double (*matrix)[4],double (*inverse)[4])
{
	float work_space[4][8], a;
	register int j, jr, k, pivot;
	int i, row[4];

	/* Initialize */
	for (j=0; j<4; j++)
	{
		for (k=0; k<4; k++)
		{
			work_space[j][k] = matrix[j][k];
			work_space[j][4+k] = 0.0;
		}
		work_space[j][4+j] = 1.0;
		row[j] = j;
	}

	/* Eliminate columns */
	for (i=0; i < 4; i++)
	{

		/* Find pivot */
		k = i;
		a = fabs(work_space[row[k]][k]);
		for (j=i+1; j < 4; j++)
		{
			jr = row[j];
			if (a < fabs(work_space[jr][i]))
			{
				k = j;
				a = fabs(work_space[jr][i]);
			}
		}

		/* Swap PIVOT row with row I */
		pivot = row[k];
		row[k] = row[i];
		row[i] = pivot;

		/* Normalize pivot row */
		a = work_space[pivot][i];
		if (a == 0.0) return(0); /* Singular */
		work_space[pivot][i] = 1.0;
		for (k=i+1; k < 8; k++) work_space[pivot][k] /= a;

		/* Eliminate pivot from all remaining rows */
		for (j = i+1; j < 4; j++)
		{
			jr = row[j];
			a = - work_space[jr][i];
			if (a == 0.0) continue;
			work_space[jr][i] = 0.0;
			for (k = i+1; k < 8; k++)
				work_space[jr][k] += (a * work_space[pivot][k]);
		}
	}

	/* Back solve */
	for (i = 3; i > 0; --i)
	{
		pivot = row[i];
		for (j = i-1; j >= 0; --j)
		{
			jr = row[j];
			a = work_space[jr][i];
			for (k=i; k<8; k++)
				work_space[jr][k] -= (a * work_space[pivot][k]);
		}
	}

	/* Copy inverse back into I */
	for (j=0; j<4; j++)
	{
		jr = row[j];
		for (k=0; k<4; k++)
		{
			inverse[j][k] = work_space[jr][k+4];
		}
	}

	return(1);
}
