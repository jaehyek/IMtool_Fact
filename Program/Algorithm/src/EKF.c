#include <math.h>


double dt=0.01;

 double P[7][7]= { { 10000, 0, 0, 0,0,0,0 }, { 0, 10000, 0, 0,0,0,0 }, { 0, 0, 10000, 0,0,0,0 }, { 0, 0, 0, 10000,0,0,0 },{ 0, 0, 0, 0,10000,0,0 },{ 0, 0, 0,0,0, 10000,0 },{ 0, 0, 0,0,0,0, 10000 }};
 double I[7][7] = { { 1, 0, 0, 0 ,0,0,0}, { 0, 1, 0, 0,0,0,0}, { 0, 0, 1, 0 ,0,0,0}, { 0, 0, 0, 1,0,0,0 },{ 0, 0, 0, 0,1,0,0 },{ 0, 0, 0,0,0, 1,0 },{ 0, 0, 0, 0,0,0,1 }    };

     
 double Q[7][7]={ {0,0,0,0,0,0,0 },{0,0,0,0,0,0,0 }, {0,0,0,0,0,0,0 }, {0,0,0,0,0,0,0 },{0,0,0,0,0.2,0,0 },{0,0,0,0,0,0.2,0 },{0,0,0,0,0,0,0.2 } };
 double R[6][6] =  { {0.00001,0, 0, 0, 0, 0}, { 0, 0.00001, 0, 0, 0, 0 }, { 0, 0, 0.00001, 0, 0, 0 }, { 0, 0, 0, 0.00001, 0, 0 }, { 0,  0, 0, 0,0.00001, 0 },{ 0, 0, 0, 0, 0.000001 } };
     double constant;      
int i,j,k;
    int n,m,l;

#define ERROR 1.0e-10
    
void MMulti( double *A,  double *B,   double *result, int A_col,int A_row,int B_col)
{
    n = A_col;//no of columns A
    m = A_row;//no of rows A
    l = B_col;// number of colums B
    

     for(i=0;i<n;i++)
        for(j=0;j<l;j++)
        {
            *(result+i*l+j) = 0;
            for(k=0;k<m;k++)
                *(result+i*l+j) += *(A+i*m+k) * *(B+k*l+j);
        }    
}

        
void MTrans( double *A, double *result ,int A_col, int A_row)
{
    n = A_col;//no of columns A
    m = A_row;//no of rows A
    

    for(i=0;i<m;i++)
        for(j=0;j<n;j++)         
            *(result+i*m+j) = *(A+j+m*i) ;            
}


void MSum( double* A,  double* B,  double* result, int A_col, int A_row, int B_col ) 
{
    n = A_col;//no of columns A
    m = A_row;//no of rows A
    l = B_col;// number of colums B
    
 
    for(i=0;i<n;i++)
        for(j=0;j<m;j++) 
            *(result+i*m+j) = *(A+i*m+j) + *(B+i*m+j);
}

void MSub( double* A,  double* B,  double* result, int A_col, int A_row, int B_col ) 
{
    n = A_col;//no of columns A
    m = A_row;//no of rows A
    l = B_col;// number of colums B
    
    for(i=0;i<n;i++)
        for(j=0;j<m;j++) 
            *(result+i*m+j) = *(A+i*m+j) - *(B+i*m+j);
}

int CopyArray2D( double *source,  double *target, int n) 
{
   if (n < 1) return (-1);

   for (i=0; i<n; i++)
       target[i] = source[i];

   return (0); 
} 
int InverseMatrix( double *work, double *tmpWork,  double *result, int n) 
{
   // 2차원 배열 복사
   CopyArray2D(work, tmpWork, (n*n));
//*(result+i*m+j) = *(A+i*m+j) + *(B+i*m+j);
    
   // 계산 결과가 저장되는 result 행렬을 단위행렬로 초기화
   for (i=0; i<n; i++)
      for (j=0; j<n; j++)
         result[i*n+j] = (i == j) ? 1 : 0;
    
   
   /* 대각 요소를 0 이 아닌 수로 만듦 */
   for (i=0; i<n; i++)
      if (-ERROR <  tmpWork[i*n+i] && tmpWork[i*n+i]< ERROR) {
         for (k=0; k<n; k++) {
            if (-ERROR <tmpWork[k*n+i] &&tmpWork[k*n+i]< ERROR) continue;
            for (j=0; j<n; j++) {
                tmpWork[i*n+j] += tmpWork[k*n+j];
                result[i*n+j] += result[k*n+j];
            }
            break;
         }
         if (-ERROR < tmpWork[i*n+i] && tmpWork[i*n+i] < ERROR) return 0;
      }

      
   /* Gauss-Jordan elimination */
   for (i=0; i<n; i++) {
      // 대각 요소를 1로 만듦 
      constant =tmpWork[i*n+i];      
      for (j=0; j<n; j++) {
         tmpWork[i*n+j] /= constant;   // tmpWork[i][i] 를 1 로 만드는 작업 
         result[i*n+j] /= constant;   // i 행 전체를 tmpWork[i][i] 로 나눔 
      }

      // i 행을 제외한 k 행에서 tmpWork[k][i] 를 0 으로 만드는 단계 
      for (k=0; k<n; k++) {
         if (k == i) continue;      // 자기 자신의 행은 건너뜀 
         if (tmpWork[k*n+i]   == 0) continue;   // 이미 0 이 되어 있으면 건너뜀 

         // tmpWork[k][i] 행을 0 으로 만듦 
         constant = tmpWork[k*n+i];
         for (j=0; j<n; j++) {
            tmpWork[k*n+j] = tmpWork[k*n+j] - tmpWork[i*n+j] * constant;
            result[k*n+j] = result[k*n+j] - result[i*n+j] * constant;
         }
      }
   }

   return (0);
}

double HP[6][7];
                double HPH[6][6];
                double S[6][6];
                double inv_S[6][6];
  double PH[7][6];                
                double K[7][6];    

   double F[7][7];
extern double norm;
 double FP[7][7];
double Trans_F[7][7];
double FPF[7][7];
 double Rm[3];
double bx, bz;
  double h[6];
  double z[6];
 double y[1][6];
 
double Trans_H[4][6];


 double H[6][7];
 

 double tmp_S[6][6];

double Ky[7];
double KH[7][7];
double IKH[7][7];
        
double gxb=0,gyb=0,gzb=0;
 double xx[20];
 void EKF( double *xk, double *q, double gx,  double gy,  double gz,  double ax,  double ay,  double az,  double mx,  double my,  double mz)
{

    
   if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)))
            if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
            {
                 q[0] = q[0] + (dt / 2) * (-q[1] * gx - q[2] * gy - q[3] * gz);
                q[1] = q[1] + (dt / 2) * ( q[0] * gx + q[3] * gy - q[2] * gz);
                q[2] = q[2] + (dt / 2) * (-q[3] * gx + q[0] * gy + q[1] * gz);
                q[3] = q[3] + (dt / 2) * ( q[2] * gx - q[1] * gy + q[0] * gz);


                norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
                q[0] = q[0] / norm;
                q[1] = q[1] / norm;
                q[2] = q[2] / norm;
                q[3] = q[3] / norm;

                //q[0] = xk[0]; q[1] = xk[1]; q[2] = xk[2]; q[3] = xk[3];

                F[0][0] = 1;
                F[0][1] = -dt / 2 * gx;
                F[0][2] = -dt / 2 * gy;
                F[0][3] = -dt / 2 * gz;

                F[1][0] = dt / 2 * gx;
                F[1][1] = 1;
                F[1][2] = dt / 2 * gz;
                F[1][3] = -dt / 2 * gy;

                F[2][0] = dt / 2 * gy;
                F[2][1] = -dt / 2 * gz;
                F[2][2] = 1;
                F[2][3] = dt / 2 * gx;

                F[3][0] = dt / 2 * gz;
                F[3][1] = dt / 2 * gy;
                F[3][2] = -dt / 2 * gx;
                F[3][3] = 1;

                // FP = MMulti(F, P, 4, 4, 4);
                // Trans_F = MTransPose(F, 4, 4);
                //FPF = MMulti(FP, Trans_F, 4, 4, 4);
                //P = MSum(FPF, Q, 4, 4, 4);
                
                MMulti(*F, *P,*FP, 4, 4, 4);
//                MTrans(*F,*Trans_F, 4, 4);
                for ( i = 0; i < 4; i++)
                    for ( j = 0; j < 4; j++)
                        Trans_F[i][ j] = F[j][ i];    
                MMulti(*FP, *Trans_F,*FPF, 4, 4, 4);
                MSum(*FPF, *Q,*P, 4, 4, 4);


                norm = sqrt(ax * ax + ay * ay + az * az);
                ax = ax / norm;
                ay = ay / norm;
                az = az / norm;

                norm = sqrt(mx * mx + my * my + mz * mz);
                mx = mx / norm;
                my = my / norm;
                mz = mz / norm;               

                Rm[0] = mx * (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) + 2 * my * (q[1] * q[2] + q[0] * q[3]) + 2 * mz * (q[1] * q[3] - q[0] * q[2]);
                Rm[1] = 2 * mx * (q[1] * q[2] - q[0] * q[3]) + my * (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]) + 2 * mz * (q[2] * q[3] + q[0] * q[1]);
                Rm[2] = 2 * mx * (q[1] * q[3] + q[0] * q[2]) + 2 * my * (q[2] * q[3] - q[0] * q[1]) + mz * (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

                bx = sqrt(Rm[0] * Rm[0] + Rm[1] * Rm[1]);
                bz = Rm[2];

               
                h[0] = -2 * (q[1] * q[3] - q[0] * q[2]);
                h[1] = -2 * (q[2] * q[3] + q[0] * q[1]);
                h[2] = -(q[0] * q[0]) + q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
                h[3] = bx * (q[0] * q[0] + q[1] * q[1] - q[2]* q[2] - q[3] * q[3]) + 2 * bz * (q[1] * q[3] - q[0] * q[2]);
                h[4] = 2 * bx * (q[1] * q[2] - q[0] * q[3]) + 2 * bz * (q[2] * q[3] + q[0] * q[1]);
                h[5] = 2 * bx * (q[1] * q[3] + q[0] * q[2]) + bz * (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
                
                z[0] = ax;
                z[1] = ay;
                z[2] = az;
                z[3] = mx;
                z[4] = my;
                z[5] = mz;
                
                for ( i = 0; i < 6; i++)
                    y[0][i] = z[i] - h[i];

                
             H[0][0] = 2 * q[2];
                H[0][1] = -2 * q[3];
                H[0][2] = 2 * q[0];
                H[0][3] = -2 * q[1];

                H[1][0] = -2 * q[1];
                H[1][1] = -2 * q[0];
                H[1][2] = -2 * q[3];
                H[1][3] = -2 * q[2];

                H[2][0] = -2 * q[0];
                H[2][1] = 2 * q[1];
                H[2][2] = 2 * q[2];
                H[2][3] = -2 * q[3];

                H[3][0] = 2 * ( q[0] * bx - q[2] * bz);
                H[3][1] = 2 * ( q[1] * bx + q[3] * bz);
                H[3][2] = 2 * (-q[2] * bx - q[0] * bz);
                H[3][3] = 2 * (-q[3] * bx + q[1] * bz);

                H[4][0] = 2 * (-q[3] * bx + q[1] * bz);
                H[4][1] = 2 *  (q[2] * bx + q[0] * bz);
                H[4][2] = 2 * ( q[1] * bx + q[3] * bz);
                H[4][3] = 2 * (-q[0] * bx + q[2] * bz);

                H[5][0] = 2 * (q[2] * bx + q[0] * bz);
                H[5][1] = 2 * (q[3] * bx - q[1] * bz);
                H[5][2] = 2 * (q[0] * bx - q[2] * bz);
                H[5][3] = 2 * (q[1] * bx + q[3] * bz);


                

                MMulti(*H, *P, *HP,6, 4, 4);        
                //MTrans(*H,*Trans_H, 6, 4);
                for ( i = 0; i < 4; i++)
                    for ( j = 0; j < 6; j++)
                        Trans_H[i][ j] = H[j][ i];                                                                 
                MMulti(*HP, *Trans_H,*HPH, 6, 4, 6);        
                MSum(*HPH, *R,*S, 6, 6, 6);
                               
                InverseMatrix(*S,*tmp_S,*inv_S,6);
                MMulti(*P, *Trans_H,*PH, 4, 4, 6);
                MMulti(*PH, *inv_S,*K, 4, 6, 6);

                
                //Ky = MMulti_s(K, y, 4, 6, 6);
            //   MMulti(*K, *y,*Ky, 4, 6, 6);
                Ky[0] = K[0][0]*y[0][0] + K[0][1]*y[0][1]+ K[0][2]*y[0][2]+ K[0][3]*y[0][3]+ K[0][4]*y[0][4]+ K[0][5]*y[0][5];
                Ky[1] = K[1][0]*y[0][0] + K[1][1]*y[0][1]+ K[1][2]*y[0][2]+ K[1][3]*y[0][3]+ K[1][4]*y[0][4]+ K[1][5]*y[0][5];
                Ky[2] = K[2][0]*y[0][0] + K[2][1]*y[0][1]+ K[2][2]*y[0][2]+ K[2][3]*y[0][3]+ K[2][4]*y[0][4]+ K[2][5]*y[0][5];
                Ky[3] = K[3][0]*y[0][0] + K[3][1]*y[0][1]+ K[3][2]*y[0][2]+ K[3][3]*y[0][3]+ K[3][4]*y[0][4]+ K[3][5]*y[0][5];
  
                for (i = 0; i < 4; i++)
                    q[i] = q[i] + Ky[i];

                norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
                q[0] = q[0] / norm;
                q[1] = q[1] / norm;
                q[2] = q[2] / norm;
                q[3] = q[3] / norm;


 
                 MMulti(*K, *H,*KH, 4, 6, 4);
                MSub(*I, *KH,*IKH, 4, 4, 4);
                MMulti(*IKH, *P,*P, 4, 4, 4);
                
            }       
}
