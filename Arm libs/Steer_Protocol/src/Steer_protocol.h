/**
  *************************************************************************************************
  * @file                 Steer_protocol.h
  * @author            
  * @qq                  269426626
  * @qq_group       604556405
  * @version           V1.0
  * @date               2017.07.28
  * @note    ��          �˳���Ϊ���ͨ�ŵײ㺯��
  *************************************************************************************************
  */


#ifndef STEER_PROTOCOL_H
#define STEER_PROTOCOL_H

#include<arduino.h>


#include <Arduino.h>

#define BROADCAST_ADDR                        0xFE                            //�㲥��ַ

//the macros below is instructions              ָ��
#define INSTRUCTION_PING                      0x01                            //��ѯָ��
#define INSTRUCTION_READ_DATA           0x02                             //��ָ��
#define INSTRUCTION_WRITE_DATA          0x03                             //дָ��
#define INSTRUCTION_REG_WRITE             0x04                             //�첽дָ��
#define INSTRUCTION_ACTION                  0x05                            //�����첽дָ��
#define INSTRUCTION_RESET                     0x06                             //��λָ��
#define INSTRUCTION_SYNC_WRITE          0x83                             //ͬ��дָ��


//the macros below is storaged in EEPROM      ��д��ַ��EEPROM�����µ�ַ�Ĵ洢���ݶ��ǿɶ�д��
#define ID_REG			                                     0x05                             //ID�ĵ�ַ��ͨ���޸ĸ�λ�õ�ֵ�������޸Ķ���ĵ�ַ����ַ��Χ:: 0 ~253��
#define MIN_ANGLE_LIMIT_H	                   0x09                             //��С�Ƕ����Ƶĸ��ֽڴ洢λ��
#define MIN_ANGLE_LIMIT_L	                   0x0A                             //��С�Ƕ����Ƶĵ��ֽڴ洢λ��
#define MAX_ANGLE_LIMIT_H	                   0x0B                             //���Ƕ����Ƶĸ��ֽڴ洢λ��
#define MAX_ANGLE_LIMIT_L	                   0x0C                             //���Ƕ����Ƶĵ��ֽڴ洢λ��
#define MAX_TORQUE_H		                         0x10                             //���Ť�ظ��ֽڴ洢λ��
#define MAX_TORQUE_L		                         0x11                             //���Ť�ص��ֽڴ洢λ��
#define DEFAULT_SPEED		                         0x12                             //�����ٶ�λ��                         
#define MIDDLE_POSITION_H	                   0x14                             //��λ�������ֽڴ洢λ��
#define MIDDLE_POSITION_L	                   0x15                             //��λ�������ֽڴ洢λ��

//the macros below is storaged in RAM       RAM��ַ(���ֿ�д)
#define TORQUE_SWITCH		                       0x28                              //Ť�ؿ��أ���д��  ��1����0�أ�
#define TARGET_POSITION_H	                   0x2A                              //Ŀ��λ�ø��ֽڴ洢λ��(��д)
#define TARGET_POSITION_L	                     0x2B                              //Ŀ��λ�õ��ֽڴ洢λ��(��д)
#define CURRENT_POSITION_H                  0x38                              //��ǰλ�ø��ֽڴ洢λ��(ֻ��)
#define CURRENT_POSITION_L                   0x39                              //��ǰλ�õ��ֽڴ洢λ��(ֻ��)
#define TARGET_SPEED		                           0x41                              //�ٶȵ���(��д

//дģʽ

#define REG_WRITE                                     0x01                               //�첽дָ��
#define SYNC_WRITE                                  0x02                               //ͬ��дָ��


/*class���溯���ľ���˵������� Steer_protocol.cpp */

class Steer_protocol
{
  private:
    HardwareSerial *svSer;

  public:
     Steer_protocol() = default;
     Steer_protocol(HardwareSerial *serial , long timeout);
    void Set_Serial_init(HardwareSerial *serial );                                                                     //��Ҫ������һ���޸Ĵ��ڵĽӿ�
     
    void begin(HardwareSerial *serial , long timeout );                                                          //�����û��������Զ��ʱʹ�ã������ʹ��ڼ�����ͨ��
    
    byte Check_Sum(byte *buf, byte len);
     
    boolean ping(byte id, byte *data);
    boolean read(byte id, byte regStartAddr, byte *data, byte readlen);
    void write(byte id, byte regStartAddr, byte *buf, byte bufLen);
    void sync_write(byte regStartAddr, byte *buf, byte svNum, byte perDataLen);
    //void reg_write(byte regStartAddr, byte *buf, byte svNum, byte perDataLen);                //�첽д
    //void act();                                                                                                                     //�����첽д
    void reset(byte id);
};

#endif






