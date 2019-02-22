/**
  *************************************************************************************************
  * @file                 Steer.h
  * @author            
  * @qq                  269426626
  * @qq_group       604556405
  * @version           V2.0
  * @date               2017.08.11
  * @note    ��          �˳���Ϊ���ͨ��Ӧ�ò㺯��
  *************************************************************************************************
  */


#ifndef STEER_H
#define STEER_H

#include <Arduino.h>
#include <Steer_protocol.h>

/*class���溯���ľ���˵������� Steer.cpp */
/*��δ���ŵĹ�����ָ�����ʱ��֧�ָù��ܣ��������ݶ���������0X00*/

class Steer:public Steer_protocol
{
  private:
  byte id;
  
  public:
  
  byte Electric_Current[2] = {0};                                                               //��δ����
  byte voltage = 0;                                                                                  //��δ����
  byte Temperature = 0;                                                                          //��δ����
  byte Position_Current[2] = {0};                                                              //��ǰλ��
  byte Position_Target[2] = {0};                                                                //Ŀ��λ��
  byte RunTime[2] = {0};                                                                          //����ʱ�䣨һ����Ŀ��λ��һ��д�룬�γ��ٶȿ��ƣ�
  byte Speed_Current[2] = {0};                                                                //��δ����
  byte Max_Angle_Limit[2] = {0};                                                             //���Ƕ�����
  byte Min_Angle_Limit[2] = {0};                                                              //��С�Ƕ�����
  
  Steer(byte ID=0 , HardwareSerial *serial = &Serial1);                            //���캯����ʼ��
  
  
  boolean Steer_Ping();                                                                            //���Ӧ����
     
  void Set_Steer_Max_Angle_Limit(word max_angle);                              //���ö�������Ƕ����ƣ�Ĭ����4096����360��
  void Set_Steer_Min_Angle_Limit(word min_angle);                                //���ö������С�Ƕ����ƣ�Ĭ����0����0��   //����е��Ĭ��2047Ϊ90�㣬��ô0����-90��
  
  void Set_Steer_Torque_On();                                                               //���ö����Ť�ؿ��ش�
  void Set_Steer_Torque_Off();                                                                //���ö����Ť�ؿ��عر�
 
  void Set_Steer_position_runtime(word pos, word runtime);                  //���ö����Ŀ��λ���뵽���λ�õ�����ʱ��
  void Change_Steer_ID( byte new_id);                                                   //�ı�����ID
  void Set_Steer_Reset();                                                                        //�����λ����λ��ID = 0x01
    
  void Get_Steer_All_Inf();                                                                       //�õ������������Ϣ
    
  /*��ȡ�����Ϣ����API����������������Ҫ����Щ�����ŵ�private��ֻ�ǲ���һ�£��������һ��Get_Steer_All_Inf()��
    Ҫ��ʱ13ms��������ֻ��Ҫ��ȡ������Ϣ�����������ĺ���Get_Steer_Position_Current_Inf()������ʱ1.3ms����������
    Ϊ�˸���Щ����Ӧ�ٶ�Ҫ��ȽϸߵĿ�����ʹ�ã�����ѡ�񿪷���Щ����*/
    
  void Get_Steer_Electric_Current_Inf();                                                   //��ȡ��������е������ù�����δ����
  void Get_Steer_voltage_Inf();                                                                //��ȡ����ĵ�ѹ���ù�����δ����
  void Get_Steer_Temperature_Inf();                                                       //��ȡ������¶ȣ��ù�����δ����
  void Get_Steer_Position_Current_Inf();                                                 //��ȡ����ĵ�ǰλ��
  void Get_Steer_Position_Target_Inf();                                                   //��ȡ�����Ŀ��λ��
  void Get_Steer_RunTime_Inf();                                                             //��ȡ���������ʱ��
  void Get_Steer_Speed_Current_inf();                                                    //�ù�����δ����
  void Get_Steer_Angle_Limit_inf();                                                        //��ȡ��������/��С�Ƕ�����
};

#endif






