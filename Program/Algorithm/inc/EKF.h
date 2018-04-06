






void MMulti(double *A, double *B,  double *result, int A_col,int A_row,int B_col);
void MMulti_s(double *A, double *B,double *result, int A_col, int A_row, int B_col);
void MTrans(double *A,double *result ,int A_col, int A_row);
void MSum(int* A, int* B, int* result, int A_col, int A_row, int B_col ) ;
void EKF(double *xk,double *q,double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
void MSub(int* A, int* B, int* result, int A_col, int A_row, int B_col ) ;
int InverseMatrix(double *work,double *tmpWork, double *result, int n) ;
int CopyArray2D(double *source, double *target, int n) ;

