#include "Arm_protect.h"

Arm_protect MyArm_Protect;
 /**
 * @brief   		                   �Ƕ����Ʊ�������::���÷�Χ���012
 * @note                         Լ����е�۵ĽǶ���һ����Χ�ڣ������򱨾��������Լ��0
 * @code                        ʾ��     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.angle_limt_protect(theta, sizeof(theta)/sizeof(theta[0]));
 * @endcode
 * @param[in]                  theta   �洢���012�Ļ�����Ϣ
 * @param[in]                  num    theta����Ĵ�С
 * @return                       true������Ƕȶ���Լ����Χ��   false: ����Ƕȳ�������        
 */
boolean Arm_protect::angle_limt_protect(double theta[], int num = 0)
{      
      if(num > 2)
      {
        if( (theta[0] >= STEER_0_MIN_LIMT)  && (theta[0] <= STEER_0_MAX_LIMT)  \
            && (theta[1] >= STEER_1_MIN_LIMT) && (theta[1] <= STEER_1_MAX_LIMT) \
            && (theta[2] >= STEER_2_MIN_LIMT) && (theta[2] <= STEER_2_MAX_LIMT) )
            {
              Serial.println("in limit");
              return true;
            }
          else
          {
              Serial.println("Eeror : out of position limit0");
              return false;
          }
      }
      else 
      {
        Serial.println("Eeror : there is not enough data");
        return false;
      }
}

 /**
 * @brief   		                   ��е��ֱ��λ�ó����б���::���÷�Χ���012
 * @note                         Լ����е�۵�λ�÷���һ�����˶����ɣ�theta1��theta2��һ����̬��Լ�����̣�����Լ��1
 * @code                        ʾ��     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.position_line_calculate(theta[1],  theta[2]);
 * @endcode
 * @param[in]                  theta1    ���1�Ļ���ֵ
 * @param[in]                  theta2    ���2�Ļ���ֵ
 * @return                       true������Ƕȶ���Լ����Χ��   false: ����Ƕȳ�������        
 */
boolean Arm_protect::position_line_calculate(double theta1, double theta2)
{
   double jud = -58*tan(theta1) + 40*tan(theta1)*sin(theta2) - 40*cos(theta2) + 18.3;
   #ifdef PRINTF_DATA
   Serial.print("-58*");
   Serial.print(tan(theta1)); 
   Serial.print(" + 40*");
   Serial.print(tan(theta1)*sin(theta2)); 
   Serial.print(" -40*"); 
   Serial.print(cos(theta2));
   Serial.print(" + 18  = ");
   Serial.println(jud);
   
   Serial.print("theta1 =  "); Serial.print(theta1); Serial.print("   tan(theta1) = "); Serial.println(tan(theta1));
   Serial.print("theta2 =  "); Serial.print(theta2); Serial.print("   sin(theta2) = "); Serial.println(sin(theta2));
   Serial.print("  cos(theta2)= "); Serial.println(sin(cos(theta2)));
   #endif
   
   if(jud > 0)
   {
      return true;
   }
    else 
   {
        Serial.println("Eeror : out of position limit1");
        return false;
   }
}

 /**
 * @brief   		                   ��е��ֱ��λ�ó����б���::���÷�Χ���012
 * @note                         Լ����е�۵�λ�÷���һ�����˶����ɣ�theta1��theta2��һ����̬��Լ�����̣������Լ��2
 * @code                        ʾ��     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.position_line_calculate(theta[1],  theta[2]);
 * @endcode
 * @param[in]                  theta1    ���1�Ļ���ֵ
 * @param[in]                  theta2    ���2�Ļ���ֵ
 * @return                       true������Ƕȶ���Լ����Χ��   false: ����Ƕȳ�������        
 */
  boolean Arm_protect::Positon_line2_limit(double theta1, double theta2)
  {
  #ifdef PRINTF_DATA
  Serial.print(theta1); Serial.print(" + "); Serial.print(theta2); Serial.print(" - "); Serial.print( 13*PI/9) ; Serial.print(" = "); Serial.println(theta1 + theta2 - 13*PI/9);
  #endif
    if(theta1 + theta2 < 13*PI/9)
    {
      return true;
    }
    else 
    {
      Serial.println("Eeror : out of position limit2");
      return false;
    }
  }

 /**
 * @brief   		                   ��е��λ�ñ�������::���÷�Χ���012
 * @note                         ��Լ��012��һ����Լ��
 * @code                        ʾ��     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.Position_Protect(theta,  sizeof(theta)/sizeof(theta[0]));
 * @endcode
 * @param[in]                  theta   �洢���012�Ļ�����Ϣ
 * @param[in]                  num    theta����Ĵ�С
 * @return                       true������Ƕȶ���Լ����Χ��   false: ����Ƕȳ�������        
 */
boolean Arm_protect::Position_Protect( double *theta, int num = 0)
{
        if(num > 2)
        {
          boolean judge1 = angle_limt_protect( theta, num);
          boolean judge2 = position_line_calculate(theta[1], theta[2]);
          boolean judge3 = Positon_line2_limit(theta[1], theta[2]);
          if( judge1 && judge2&&judge3){return true;}
          else{return false;}
        }
        else
        {
          return false;
        }
}

 /**
 * @brief   		                   �Ƕ����Ʊ�������::���÷�Χ���345
 * @note                         Լ����е�۵ĽǶ���һ����Χ�ڣ������򱨾�������Ƕ��345��Լ��0
 * @code                        ʾ��     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.steer_345_angle_protect(theta, sizeof(theta)/sizeof(theta[0]));
 * @endcode
 * @param[in]                  theta   �洢���012�Ļ�����Ϣ
 * @param[in]                  num    theta����Ĵ�С
 * @return                       true������Ƕȶ���Լ����Χ��   false: ����Ƕȳ�������        
 */
boolean Arm_protect::steer_345_angle_protect(double theta[], int num = 0)
{
      if(num > 2)
      {
        if( (theta[3] >= STEER_3_MIN_LIMT)  && (theta[3] <= STEER_3_MAX_LIMT)  \
            && (theta[4] >= STEER_4_MIN_LIMT) && (theta[4] <= STEER_4_MAX_LIMT) \
            && (theta[5] >= STEER_5_MIN_LIMT) && (theta[5] <= STEER_5_MAX_LIMT) )
            {
              Serial.println("angle of 345 in limit");
              return true;
            }
          else
          {
              Serial.println("Eeror : out of position limit3");
              return false;
          }
      }
      else 
      {
        Serial.println("Eeror : there is not enough data");
        return false;
      }

}

 /**
 * @brief   		                   ��е��λ�ñ������ޱ�������::���÷�Χ���012345
 * @code                        ʾ��     
 *      				                       MyArm_Protect.warning();
 * @endcode
 * @param[in]                  void
 * @return                       void     
 */
void Arm_protect::warning()
{
  for(int i = 0; i < 3; i++)
  {
    tone(BUZZER_PIN, 3500, 200);
    delay(200);
    noTone(BUZZER_PIN);
    delay(300);
  }
}

