#include <Arm.h>

Arm MyArm;                                                                   //��ʼ��һ����е�۶���

Steer_protocol  steer_tmp(&Serial1, 10);                         //��ʼ��һ������ײ���󣬸��������


 /**
 * @brief   		                  ��е�����ó�ʼ��
 * @code                        ʾ��      ����USBͨ��
 *      				                       MyArm.begin(USB_SER);
 * @endcode
 * @param[in]                 desireSer       ����Ĵ��ں�
 *          @arg                  USB_SER            ʹ��USBͨ��
 *          @arg                  WIFI_SER            ʹ��WiFiͨ��
 * @return                       void        
 */
 
void Arm::begin(HardwareSerial *desireSer)         
{
    comSer = desireSer;
    comSer->begin(115200);
    comSer->setTimeout(5);

    Steer_Detect();
    //Steer_Num = 7;
    Para_Init();
    Get_Offset();
}


 /**
 * @brief   		                  ��е��λ�ó�ʼ��
 * @note                        �����ʼ�������λ��
 * @code                        ʾ��      ��ʼ�����λ�õ���ָ��λ��
 *      				                       MyArm.position_init();
 * @endcode
 * @param[in]                 void
 * @return                      void        
 */
void Arm::position_init(void)
{
  Get_Offset();
  for(int i = 0; i < Steer_Num; i++)
  {
    steer[i] ->Set_Steer_position_runtime(2047+offPos[i] , 0x06b8);
    pos_goal[i] = 2047+offPos[i];
  }
}

 /**
 * @brief   		                  ��е������λ����⺯�����ɣ�x, y, z��������������е�۵Ķ��0��1��2�Ļ���ֵ
 * @code                        ʾ��      ���㣨0,188,188�����˶�ѧ��⣬�õ�IDΪ0,1,2,�Ķ���ĽǶ�ֵ
 *      				                       MyArm.inverse_movement(0, 188, 188);
 * @endcode
 * @param[in]                  x_    ��е�ۻ���x�������λ�ã���λ�Ǻ��ף�
 * @param[in]                  y_    ��е�ۻ���y�������λ��
 * @param[in]                  z_    ��е�ۻ���z�������λ��
 * @return                       void        
 */
void Arm::inverse_movement(double x_ , double y_, double z_)
{
  double x = x_, y = y_, z = z_;

    theta[0] = atan2(y, x);
    x -= d * cos(theta[0]);
    y -= d * sin(theta[0]);
    z -= e;
    
    double lengthA = sqrt(x * x + y * y + z * z);
    double lengthC = sqrt(h * h + c * c);
    double offsetAngle = atan(h / c);
    double angleA = acos( (a * a + lengthC * lengthC - lengthA * lengthA) / (2 * a * lengthC) );
    double angleB = atan( z / sqrt(x * x + y * y) );
    double angleC = acos( (a * a + lengthA * lengthA - lengthC * lengthC) / (2 * a * lengthA) );
    theta[1] = angleB + angleC;
    theta[2] = PI - angleA - angleB - angleC + offsetAngle;
    theta[2] += HALF_PI;
}


 /**
 * @brief   		                  ��е������λ����������inverse_movement(double x_ , double y_, double z_)�������أ�ֻ�ǰ�����ֵ�ŵ��˵���������
 * @code                        ʾ��     PVector MyDot;     MyDot.set_xyz(0, 188, 188);
 *      				                       MyArm.inverse_movement(MyDot);
 * @endcode
 * @param[in]                  pt        Ϊ������������Ϣ�ĵ�����
 * @return                       void        
 */
void Arm::inverse_movement(PVector pt)
{
  double x = pt.x, y = pt.y, z = pt.z;

    theta[0] = atan2(y, x);
    x -= d * cos(theta[0]);
    y -= d * sin(theta[0]);
    z -= e;
    
    double lengthA = sqrt(x * x + y * y + z * z);
    double lengthC = sqrt(h * h + c * c);
    double offsetAngle = atan(h / c);
    double angleA = acos( (a * a + lengthC * lengthC - lengthA * lengthA) / (2 * a * lengthC) );
    double angleB = atan( z / sqrt(x * x + y * y) );
    double angleC = acos( (a * a + lengthA * lengthA - lengthC * lengthC) / (2 * a * lengthA) );
    theta[1] = angleB + angleC;
    theta[2] = PI - angleA - angleB - angleC + offsetAngle;
    theta[2] += HALF_PI;
}

 /**
 * @brief   		                  ��е���˶����ƺ���   ��Ҫ��������������Ͽ��Կ��ƻ�е�۵��˶�
 * @note                         �ѻ�е�۵�ĩ�����е�����(x_, y_, z_)������ʱ��Ϊruntime
 * @code                        ʾ��     ���е��ĩ���ɵ�ǰλ�����е���0�� 188�� 188������Ҫ��ʱ��Ϊ2s
 *      				                       MyArm.move_to_position(0, 188, 188, 2000);
 * @endcode
 * @param[in]                  x_             ��е�ۻ���x�������λ�ã���λ�Ǻ��ף�
 * @param[in]                  y_             ��е�ۻ���y�������λ��
 * @param[in]                  z_             ��е�ۻ���z�������λ��
 * @param[in]                  runtime    ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
 * @return                       void        
 */

void Arm::move_to_position( double x_ , double y_, double z_ , word runtime)
{    
    inverse_movement(x_ , y_, z_);
    
    #ifdef START_PROTECT
        double thet[3];
        thet[0] = theta[0];
        thet[1] = theta[1];
        thet[2] = theta[2];
        boolean judge = MyArm_Protect.Position_Protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
    #endif

    for(byte i = 0 ; i < 3; i++)   
    {
      pos_goal[i] = Rad2Pos(theta[i]) + offPos[i];
      steer[i] ->Set_Steer_position_runtime(pos_goal[i] , runtime);
      Serial.println(pos_goal[i] );
    }
}

 /**
 * @brief   		                  ��Ҫ��������������Ͽ��Կ��ƻ�е�۵��˶�
 * @note                         �ѻ�е�۵�ĩ�����е���������������ʱ��Ϊruntime
 * @code                        ʾ��     ���е��ĩ���ɵ�ǰλ�����е���0�� 188�� 188������Ҫ��ʱ��Ϊ2s
*                                      PVector MyDot;     MyDot.set_xyz(0, 188, 188);
 *      				                       MyArm.move_to_position(MyDot, 2000);
 * @endcode
 * @param[in]                  pt            Ϊ������������Ϣ�ĵ�����
 * @param[in]                  runtime    ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
 * @return                       void        
 */
void Arm::move_to_position( PVector pt , word runtime)
{    
    inverse_movement(pt);
    
    #ifdef START_PROTECT
        double thet[3];
        thet[0] = theta[0];
        thet[1] = theta[1];
        thet[2] = theta[2];
        boolean judge = MyArm_Protect.Position_Protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
    #endif

    for(byte i = 0 ; i < 3; i++)   
    {
      pos_goal[i] = Rad2Pos(theta[i]) + offPos[i];
      steer[i] ->Set_Steer_position_runtime(pos_goal[i] , runtime);
    }
}

 /**
 * @brief   		                  ��Ҫ��������������Ͽ��Կ��ƻ�е�۵��˶�
 * @note                         ֱ�����û�е���ж��0,1,2��λ�ã�����ʱ��Ϊruntime
 * @code                        ʾ��     ʹ����������е��м�λ��
 *      				                       MyArm.move_to_position(2047, 2047, 2047, 2000);
 * @endcode
 * @param[in]                  pos0             ���0��λ�ã���Χ��0~4095������ʾ��0~360�㣩ע�⣺����������������Ա����е�λ��Ϊ��
 * @param[in]                  pos1             ���1��λ�ã���Χ��0~4095������ʾ��0~360�㣩
 * @param[in]                  pos2             ���2��λ�ã���Χ��0~4095������ʾ��0~360�㣩
 * @param[in]                  runtime         ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
 * @return                       void        
 */
void Arm::move_to_position( word pos0 , word pos1, word pos2 , word runtime)
{
    #ifdef START_PROTECT
        double thet[3];
        thet[0] = Pos2Rad(pos0);
        thet[1] = Pos2Rad(pos1);
        thet[2] = Pos2Rad(pos2);
        boolean judge = MyArm_Protect.Position_Protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
    #endif
    
      steer[0] ->Set_Steer_position_runtime(pos0 + offPos[0] , runtime);
      steer[1] ->Set_Steer_position_runtime(pos1 +  offPos[1], runtime);
      steer[2] ->Set_Steer_position_runtime(pos2 +  offPos[2], runtime);
}

 /**
 * @brief   		                  �����Ӧ��⺯��
 * @note                         ��⵽��Ӧ���������
 * @code                        ʾ��     
 *      				                       MyArm.Steer_Detect();
 * @endcode
 * @return                       void        
 */
byte Arm::Steer_Detect()
{
    byte state;
    byte num = 0;

    for (byte i = 0; i < 7; i++) 
    {
        if (steer_tmp.ping(i, &state)) 
        {
            num ++;
        }
    }
    Steer_Num = num;
    return num;
}


 /**
 * @brief   		                  ��е�۲�����ʼ������
 * @code                        ʾ��     
 *      				                       MyArm.Para_Init();
 * @endcode
 * @return                       void        
 */
 void Arm::Para_Init()
 {
    if(Steer_Num > 0)
    {
          offPos = (short *)malloc(Steer_Num * sizeof(short));
          theta = (double *)malloc(Steer_Num * sizeof(double));
          
          pos_goal = (int *)malloc(Steer_Num * sizeof(int));
          
          for(byte i  = 0; i < Steer_Num; i++ )
          {
            steer[i] = new Steer( i, &Serial1); 
          }
    }
 }
 
  /**
 * @brief   		                  ��е��Ť�ؿ�������  ���û�е�۵�Ť��Ϊ����ʹ��е�ۻָ�Ť��
 * @code                        ʾ��     
 *      				                       MyArm.Set_Arm_Torque_On();
 * @endcode
 * @return                       void        
 */
void Arm::Set_Arm_Torque_On(void)
{
  for( int i = 0; i < Steer_Num; i++)
  {
    steer[i] -> Set_Steer_Torque_On();
  }
}

  /**
 * @brief   		                  ��е��Ť�عرպ���  ���û�е�۵�Ť��Ϊ�أ�ʹ��е�۴�������״̬
 * @code                        ʾ��     
 *      				                       MyArm.Set_Arm_Torque_Off();
 * @endcode
 * @return                       void        
 */
void Arm::Set_Arm_Torque_Off(void)
{
  for( int i = 0; i < Steer_Num; i++)
  {
    steer[i] -> Set_Steer_Torque_Off();
  }
}

 /**
 * @brief   		                   ��3����4����5�Ŷ����ת�˶�����    ��Ҫ�����345��״̬���ú���
 * @note                         ǰ��Ļ�е�۵�λ�ú�����Ҫ������ȷ�������ĩ��λ�ã���������С���������ʹ���345�˶�����תĩ�˻���
 * @code                        ʾ��     ʹ3��4��5����������е��м�λ�ã��˶�ʱ��Ϊ����
 *      				                       MyArm.turn_steer_345_to_positon(2047, 2047, 2047, 2000);
 * @endcode
 * @param[in]                  pos3             ���3��λ�ã���Χ��0~4095������ʾ��0~360�㣩ע�⣺����������������Ա����е�λ��Ϊ��
 * @param[in]                  pos4             ���4��λ�ã���Χ��0~4095������ʾ��0~360�㣩
 * @param[in]                  pos5             ���5��λ�ã���Χ��0~4095������ʾ��0~360�㣩
 * @param[in]                  runtime         ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
 * @return                       boolean         �Ƕȳ������Ʒ���false        
 */
boolean Arm::turn_steer_345_to_positon(word pos3 , word pos4, word pos5 , word runtime)
{
      #ifdef START_PROTECT
        double thet[3];
        thet[0] = Pos2Rad(pos3);
        thet[1] = Pos2Rad(pos4);
        thet[2] = Pos2Rad(pos5);
        boolean judge = MyArm_Protect.steer_345_angle_protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
      #endif

      steer[3] ->Set_Steer_position_runtime(pos3 + offPos[3] , runtime);
      steer[4] ->Set_Steer_position_runtime(pos4 +  offPos[4], runtime);
      steer[5] ->Set_Steer_position_runtime(pos5 +  offPos[5], runtime);
}

 /**
 * @brief   		                   ��Ҫ�����345��״̬�������غ���
 * @note                         ǰ��Ļ�е�۵�λ�ú�����Ҫ������ȷ�������ĩ��λ�ã���������С���������ʹ���345�˶�����תĩ�˻���
 * @code                        ʾ��     ʹ3��4��5����������е��м�λ�ã��˶�ʱ��Ϊ����
 *      				                       MyArm.turn_steer_345_to_positon(90, 90, 90, 2000);
 * @endcode
 * @param[in]                  angle3             ���3�ĽǶȣ���Χ��0~360�� ע�⣺����������������Ա����е�λ��Ϊ��
 * @param[in]                  angle4             ���4�ĽǶȣ���Χ��0~360�� 
 * @param[in]                  angle5             ���5�ĽǶȣ���Χ��0~360�� 
 * @param[in]                  runtime           ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
 * @return                       boolean           ���������޶ȷ���false       
 */
boolean Arm::turn_steer_345_to_positon(double angle3 , double angle4, double angle5 , word runtime)
{
      #ifdef START_PROTECT
        double thet[3];
        thet[0] = Angle2Rad(angle3);
        thet[1] = Angle2Rad(angle4);
        thet[2] = Angle2Rad(angle5);
        boolean judge = MyArm_Protect.steer_345_angle_protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
      #endif
      steer[3] ->Set_Steer_position_runtime(Angle2Pos(angle3) +  offPos[3] , runtime);
      steer[4] ->Set_Steer_position_runtime(Angle2Pos(angle4) +  offPos[4], runtime);
      steer[5] ->Set_Steer_position_runtime(Angle2Pos(angle5) +  offPos[5], runtime);
}


  /**
 * @brief   		                  �õ���е�۵�ƫ�ú���
 * @code                        ʾ��     
 *      				                       MyArm.Get_Offset();
 * @endcode
 * @return                       void        
 */
void Arm::Get_Offset()
{
    for (byte i = 0; i < Steer_Num; i++) 
    {
        EEPROM.get(i * sizeof(short), offPos[i]);
    }
}

  /**
 * @brief   		                  ͨ����е�۵�ֱ��λ�ã����û�е��ƫ��    ͨ����е�۵�ֱ���������û�е��ƫ�ú���
 * @code                        ʾ��     ����IDΪ1�Ķ����ƫ��Ϊ100
 *      				                       MyArm.offset_by_pos(1�� 100);
 * @endcode
 * @param[in]                  id              �����ID��
 * @param[in]                  offset        ���ö����ƫ�ã�ֵ�ķ�Χ��-2046 ~ +2046��
 * @return                       void        
 */
void Arm::offset_by_pos(byte id, short offset)
{
      if(id < Steer_Num)
      {
        EEPROM.put(id * sizeof(short), offset);
      }
      else 
      {
        comSer -> println("error: id > Steer_Num");
        comSer -> println("Please enter a correct id");
      }
}

  /**
 * @brief   		                  ͨ����е�۵ĽǶȣ����û�е��ƫ��
 * @code                        ʾ��     ����IDΪ1�Ķ����ƫ��Ϊ10��
 *      				                       MyArm.offset_by_pos(1�� 10);
 * @endcode
 * @param[in]                  id             �����ID��
 * @param[in]                  angle        ���ö����ƫ�ýǶȣ�ֵ�ķ�Χ��-90 ~ +90��
 * @return                       void        
 */
void Arm::offset_by_angle(byte id, double angle)
{
  short tmp = mapFloat(angle, -90, 90, -1024, 1024);
  offset_by_pos(id, tmp);
}

 /**
 * @brief   		                  ��ֵӳ�亯��    
 * @param[in]                 val                     ��Ҫӳ���ֵ
 * @param[in]                 in_min                ԭ�������Сֵ            
 * @param[in]                 in_max               ԭ��������ֵ
 * @param[in]                 out_min              ��Ҫӳ�䵽���������Сֵ
 * @param[in]                 out_max             ��Ҫӳ�䵽����������ֵ
 * @return                                               ����ֵ������ӳ����ֵ
 */
double mapFloat(double val, double in_min, double in_max, double out_min, double out_max)
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

 /**
 * @brief   		                  ����תλ�����ݣ�λ������Ϊ�����ֱ�ӿ������ݣ�    
 * @code                        ʾ��  
 *      		                          MyArm.Rad2Pos( 2.12);		                        
 * @endcode
 * @param[in]                 rad                    ����ֵ
 * @return                                               ����ֵ�������λ������
 */
short Arm::Rad2Pos(double rad)
{
    return short(mapFloat(rad, 0, PI, 1024, 3071));
}

 /**
 * @brief   		                  �Ƕ�תλ�����ݣ�λ������Ϊ�����ֱ�ӿ������ݣ�    
 * @code                        ʾ��  
 *      		                          MyArm.Angle2Pos( 90 );		                        
 * @endcode
 * @param[in]                 angle                 �Ƕ�ֵ
 * @return                                               ����ֵ�������λ������
 */
short Arm::Angle2Pos(double angle)
{
    return short(mapFloat(angle, 0, 180, 1024, 3071));
}

/**
 * @brief   		                  λ������ת�Ƕ�����   
 * @code                        ʾ��  
 *      		                          MyArm.Pos2Angle( 90 );		                        
 * @endcode
 * @param[in]                 pos                   λ������
 * @return                                               ����ֵ���Ƕ�����
 */
double Arm::Pos2Angle(double pos)
{
    return double(mapFloat(pos, 1024, 3071, 0, 180));
}

/**
 * @brief   		                  ����ֵת�Ƕ�ֵ����  
 * @code                        ʾ��  
 *      		                          MyArm.Rad2Angle( 2.12 );		                        
 * @endcode
 * @param[in]                 rad                    ��������
 * @return                                               ����ֵ���Ƕ�����
 */
double Arm::Rad2Angle(double rad)
{
    return double(mapFloat(rad, 0, PI, 0, 180));
}

/**
 * @brief   		                  �Ƕ�ֵת����ֵ   
 * @code                        ʾ��  
 *      		                          MyArm.Angle2Rad( 98 );		                        
 * @endcode
 * @param[in]                 angle                 �Ƕ�����
 * @return                                               ����ֵ����������
 */
double Arm::Angle2Rad(double angle)
{
    return double(mapFloat(angle, 0, 180, 0, PI));
}

/**
 * @brief   		                  ֱ��λ������ת������ֵ����
 * @code                        ʾ��  
 *      		                          MyArm.Rad2Angle( 2.12 );		                        
 * @endcode
 * @param[in]                 rad                    ��������
 * @return                                               ����ֵ���Ƕ�����
 */
double Arm::Pos2Rad(word pos)
{
   return double(mapFloat(pos, 1024, 3071, 0, PI));
}

//�����ٶ�ȥ���������