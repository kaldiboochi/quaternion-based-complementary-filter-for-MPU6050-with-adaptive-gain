

#include "complementary_filter.h"


void updateQuaternionFromRate(struct quaternion* global_quaternion,float rate[3], float deltaT){
  struct quaternion omega;
  struct quaternion qDot;
  pureQuat(rate,&omega);
  quatMul(&omega,global_quaternion,&qDot);
  scalarMul(-0.5*deltaT,&qDot,&qDot);
  quatAdd(global_quaternion,&qDot,global_quaternion);
}

void updateQuaternionFromAcc(struct quaternion* global_quaternion, float acc[3]){
  struct quaternion inverse_global_quaternion;
  float g_p[]={0,0,0};
  inverse(global_quaternion,&inverse_global_quaternion);
  quatRotate(inverse_global_quaternion,acc,g_p);
  struct quaternion delta_acc;
  delta_acc->w = sqrtf((g_p[2]+1)/2);
  float temp = 1/sqrtf((g_p[2]+1)*2);
  delta_acc->x = -1.0*g_p[1]*temp;
  delta_acc->y = g_p[0]*temp;
  delta_acc-> = 0;
  float eps = quatDot(&delta_acc,&identity_quaternion);
  if (eps>=0.9){
    quatNormalise(&delta_acc);
  }
  else{
    float alpha = 0.1;
    struct quaternion A;
    struct quaternion B;
    scalarMul(1-alpha,&identity_quaternion,&A);
    scalarMul(alpha,&delta_acc,&B);
    quatAdd(&A,&B,&delta_acc);
  }
  quatMul(global_quaternion,&delta_acc,global_quaternion);
}



