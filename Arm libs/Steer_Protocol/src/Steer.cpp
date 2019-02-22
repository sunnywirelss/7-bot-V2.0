/**
  *************************************************************************************************
  * @file                 Steer.cpp
  * @author            
  * @qq                  269426626
  * @qq_group       604556405
  * @version           V2.0
  * @date               2017.08.13
  * @note    ��          �˳���Ϊ���ͨ��Ӧ�ò㺯��
  *************************************************************************************************
  */

#include "Steer.h"


 /**
 * @brief   		                  ���ͨѶӦ�ò㹹�캯������ʼ��ID��ͨѶ����
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1
 *      				                      Steer   steer1( 0, &Serial1 );
 * @endcode
 * @param[in]                 id               �����ID��
 * @param[in]                Serial          ����ѡ��(��Ҫ�����Լ��Ŀ�����ʵ��ȷ��)
 *                                   @arg           Serial1
 *                                   @arg           Serial2
 *                                   @arg           Serial3
 */
Steer::  Steer(byte ID=0 , HardwareSerial *serial = &Serial1)
{
  id = ID;
  Set_Serial_init(serial);
}

/**
 * @brief   		                  �õ������������Ϣ
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1           ����һ���������Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Get_Steer_All_Inf();
 * @endcode
 * @note                        һ����Ҫ��ȡ��Ϣʱ����ʹ�øú���������ʹ�ö�ʱ�жϵ���ע�⣺�ú�������һ�κ�ʱ13ms����
 */
void Steer::Get_Steer_All_Inf()
{
  Get_Steer_Electric_Current_Inf();
  Get_Steer_voltage_Inf();
  Get_Steer_Temperature_Inf();
  Get_Steer_Position_Current_Inf();
  Get_Steer_Position_Target_Inf();
  Get_Steer_RunTime_Inf();
  Get_Steer_Speed_Current_inf();
  Get_Steer_Angle_Limit_inf();
}

/**
 * @brief   		                  ���Ӧ���������Զ����ͨ���Ƿ�����
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1           ����һ���������Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Steer_Ping();
 * @endcode
 * @return                      tureΪӦ��ɹ��� falseΪʧ��
 */
boolean Steer::Steer_Ping()
{
   byte dat;
   return (ping(id, &dat));
}
    
/**
 * @brief   		                  ���ö�����������ƣ�����������4096����360��֮��
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1           ����һ���������Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Max_Angle_Limit(3072);
 * @endcode
 * @param[in]                 max_angle               ������ƽǣ��������ת�������ֵ��4094Ϊ360��
 */ 
void Steer::Set_Steer_Max_Angle_Limit(word max_angle)
{
      byte wbuf[2];
      wbuf[0] = highByte(max_angle);
      wbuf[1] = lowByte(max_angle);
      write(id , 0X0b , wbuf , 2);
}

/**
 * @brief   		                  ���ö������С�����ƣ�����������4096����360��֮��
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1           ����һ���������Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Min_Angle_Limit(100);
 * @endcode
 * @param[in]                 min_angle               ��С���ƽǣ��������ת������Сֵ��0Ϊ0�㣬 4094Ϊ360��
 */ 
void Steer::Set_Steer_Min_Angle_Limit(word min_angle)
{
      byte wlbuf[2];
      wlbuf[0] = highByte(min_angle);
      wlbuf[1] = lowByte(min_angle);
      write(id , 0X09 , wlbuf , 2);    
}

/**
 * @brief   		                  ���ö�����Ť�ؿ���Ϊ��״̬����������Ť��
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1           ����һ���������Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Torque_On();
 * @endcode
 */   
void Steer::Set_Steer_Torque_On()
{
      byte buf[2];
      buf[0] = 0x01;
      buf[1] = 0x01;
      write(id , 0X28 , buf , 1);    
}

/**
 * @brief   		                  ���ö�����Ť�ؿ���Ϊ��״̬�������ʧȥŤ��
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1           ����һ���������Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Torque_Off();
 * @endcode
 */       
void Steer::Set_Steer_Torque_Off()
{
      byte buf[2];
      buf[0] = 0x00;
      buf[1] = 0x00;
      write(id , 0X28 , buf , 1);    
}
  
 /**
 * @brief   		                  ������к��������øú���ʹ��������˶�
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1           ����һ���������Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_position_runtime(2047�� 1000);    //һ�������е�2047���Ŀ��λ��
 * @endcode
 * @param[in]                 pos               �����λ�ã�ֵ��ΧΪ���Ƕ���������С�Ƕ�����֮��
 * @param[in]                 runtime         ���������ʱ�䣬1000Ϊ1��
 */        
void Steer::Set_Steer_position_runtime(word pos, word runtime)
{
      byte dat[5];
      dat[0] = id;
      dat[1] = highByte(pos);
      dat[2] = lowByte(pos);
      dat[3] = highByte(runtime);
      dat[4] = lowByte(runtime); 
      sync_write(0x2a, dat, 1, 5);      
}
 
  /**
 * @brief   		                  �ı�����ID
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1           ����һ���������Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Change_Steer_ID(2);           //�Ѹö����id����Ϊ2
 * @endcode
 * @param[in]                 new_id           �������ID
 */        
void Steer::Change_Steer_ID( byte new_id)
{
        byte new_id_ = new_id;
        write(id, 0x05, &new_id_, sizeof(new_id_));
        id = new_id;
}

  /**
 * @brief   		                  ����ָ���������
 * @code                        ʾ��
 *      				                      // ����id = 0����ʼ������1           ����һ���������Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Reset();           
 * @endcode
 * @note                         ��λ������ID�ᱻ��Ϊ0X01
 */        
void Steer::Set_Steer_Reset()
{
   reset(id);
   id = 1;
}

  /**
 * @brief   		                  ��ö���ĵ�ǰ����
 * @note                        ��õ�ֵ�洢�� Electric_Current[]�����������뿴�ඨ������Ĺ��г�Ա����
 * @note                        ���ڶ������ԭ�򣬸ù�����δ���ţ���ȡ����ȫΪ0X00
 */        
void Steer::Get_Steer_Electric_Current_Inf()
{
    read(id , 0X2e , Electric_Current , 2); 
}

  /**
 * @brief   		                  ��ö���ĵ�ǰ��ѹ
 * @note                        ��õ�ֵ�洢�� voltage�������뿴�ඨ������Ĺ��г�Ա����
 * @note                        ���ڶ������ԭ�򣬸ù�����δ���ţ���ȡ����ȫΪ0X00
 */        
 void Steer::Get_Steer_voltage_Inf()
 {
    byte tmp[2] = {0};
    read(id , 0X3e , tmp , 1);  
    voltage = tmp[0];
 }
 
   /**
 * @brief   		                  ��ö���ĵ�ǰ�¶�
 * @note                        ��õ�ֵ�洢�� Temperature�������뿴�ඨ������Ĺ��г�Ա����
 * @note                        ���ڶ������ԭ�򣬸ù�����δ���ţ���ȡ����ȫΪ0X00
 */        
  void Steer::Get_Steer_Temperature_Inf()
  {
      byte tmpe[2] = {0};
      read(id , 0X3f , tmpe , 1); 
      Temperature = tmpe[0];
  }
  
  /**
 * @brief   		                  ��ö���ĵ�ǰλ��
 * @note                        ��õ�ֵ�洢�� Position_Current[]�����������뿴�ඨ������Ĺ��г�Ա����
 */        
  void Steer::Get_Steer_Position_Current_Inf()
  {
      read(id , 0X38 , Position_Current , 2); 
  }
  
 /**
 * @brief   		                  ��ö����Ŀ��λ��
 * @note                        ��õ�ֵ�洢�� Position_Target[]�����������뿴�ඨ������Ĺ��г�Ա����
 */        
  void Steer::Get_Steer_Position_Target_Inf()
  {
      read(id , 0X2a , Position_Target , 2); 
  }
  
 /**
 * @brief   		                  ��ö��������ʱ��
 * @note                        ��õ�ֵ�洢�� RunTime[]�����������뿴�ඨ������Ĺ��г�Ա����
 */        
  void Steer::Get_Steer_RunTime_Inf()
  {
      read(id , 0X2c , RunTime , 2); 
  }
  
 /**
 * @brief   		                  ��ö���ĵ�ǰ�ٶ�
 * @note                        ��õ�ֵ�洢�� Speed_Current[]�����������뿴�ඨ������Ĺ��г�Ա����
 * @note                        ���ڶ������ԭ�򣬸ù�����δ���ţ���ȡ����ȫΪ0X00
 */        
  void Steer::Get_Steer_Speed_Current_inf()
  {
      read(id , 0X3a , Speed_Current , 2); 
  }
  
    /**
 * @brief   		                  ��ö�������/��С�Ƕ�����
 * @note                        ��õ�ֵ�洢�� Min_Angle_Limit[]��Max_Angle_Limit[]���������������뿴�ඨ������Ĺ��г�Ա����
 */        
  void Steer::Get_Steer_Angle_Limit_inf()
  {
      read(id , 0X09 , Min_Angle_Limit , 2); 
      read(id , 0X0b , Max_Angle_Limit , 2); 
  }
  
  