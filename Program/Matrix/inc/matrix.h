

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

void mx_identity (double (*mx)[4]);
void mx_rot_x (double angle,double (*mx)[4] );
void mx_rot_y (double angle,double (*mx)[4] );
void mx_rot_z (double angle,double (*mx)[4] );
void mx_scale (double sx,double sy,double sz,double (*mx)[4] );
void mx_translate (double tx,double ty,double tz,double (*mx)[4] );
void mx_euler_matrix (double ax,double ay,double az,double (*mat)[4] );
//void mx_44mult(double  *matrix[],double *vector,double *result);
void mx_44mult(double (*matrix)[4],Vector4 vector,Vector4 result);
//void mx_mult(double  *matrix1[],double  *matrix2[],double  *result[]) ;
void mx_mult(double (*matrix1)[4],double (*matrix2)[4],double (*result)[4]) ;
int mx_invert(double (*matrix)[4],double (*inverse)[4]);

void mx_4MVmult(double (*matrix)[4],double *vector,double * result);