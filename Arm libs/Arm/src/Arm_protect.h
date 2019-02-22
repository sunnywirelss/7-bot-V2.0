#ifndef  ARM_PROTECT_H
#define ARM_PROTECT_H

#define PRINTF_DATA

#include "Arduino.h"
#include "Arm.h"
#include "Serial_arm.h"

#define BUZZER_PIN                37                                                           //����������

//ע�⣺�����нǶȵ��������ȫ�����Ի�����Ϊ׼

const double STEER_0_MIN_LIMT   = PI/18;                                          //��е�۶��0�Ļ�����Сֵ
const double STEER_0_MAX_LIMT   = 17*PI/18;                                    //��е�۶��0�Ļ������ֵ����������
const double STEER_1_MIN_LIMT   = PI/18;
const double STEER_1_MAX_LIMT   = PI;
const double STEER_2_MIN_LIMT   = PI/18;
const double STEER_2_MAX_LIMT   = 17*PI/18;

const double STEER_3_MIN_LIMT   = 0;                                          
const double STEER_3_MAX_LIMT   = PI;                                     
const double STEER_4_MIN_LIMT   = 0;                                          
const double STEER_4_MAX_LIMT   = PI;                                     
const double STEER_5_MIN_LIMT   = 0;                                          
const double STEER_5_MAX_LIMT   = 2*PI;                                     

/*************������ϸ˵�������CPP�ļ�*************/

class Arm_protect{
  private:
  
  public:     
  boolean angle_limt_protect(double theta[], int num = 0);
  
  boolean position_line_calculate(double theta1, double theta2);
  boolean Positon_line2_limit(double theta1, double theta2);
  
  boolean Position_Protect( double *theta, int num = 0);
  
  /****ǰ�ߵĶ��Ƕ��012��λ�ñ���������Ƕ��345����ת�Ƕȵı���*****/
  boolean steer_345_angle_protect(double theta[], int num = 0);
  
  void warning();
};

extern Arm_protect MyArm_Protect;

#endif
