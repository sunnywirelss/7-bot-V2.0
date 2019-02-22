/**
  *************************************************************************************************
  * @file                 Steer_protocol.cpp
  * @author            
  * @qq                  269426626
  * @qq_group       604556405
  * @version           V1.0
  * @date               2017.07.28
  * @note    ��          �˳���Ϊ���ͨ�ŵײ㺯��
  *************************************************************************************************
  */

#include "Steer_protocol.h"


 /**
 * @brief   		                  ���ͨѶЭ�鹹�캯������ʼ��ͨѶ����
 * @code                        ʾ��
 *      				                      // ��ʼ������1����ʱ10
 *      				                      Steer_protocol   Steer1( &Serial1 , 10 );
 * @endcode
 * @param[in]                Serial          ����ѡ��
 * @param[in]                 timeout      �ȴ���ʱ
 */
Steer_protocol::Steer_protocol(HardwareSerial *serial, long timeout)
{
  svSer = serial;
  svSer -> begin(115200);
  svSer -> setTimeout(timeout);
}

 /**
 * @brief   		                  ��ʼ��ͨѶ����
 * @param[in]                Serial          ����ѡ��
 */
void Steer_protocol::Set_Serial_init(HardwareSerial *serial )
{
   svSer = serial;
  svSer -> begin(115200);
  svSer -> setTimeout(10);
}

 /**
 * @brief   		                  �û����ڲ��Զ����ʱ��ʹ��
 * @code                        ʾ��      Steer_protocol   Steer1( &Serial1 , 10 );
 *      				                      Steer1.begin();
 * @endcode
 * @param[in]                Serial          ����ѡ��
 *                                   @arg 
 *                                   @arg 
 *                                   @arg 
 * @param[in]                 timeout      �ȴ���ʱ
 */
void Steer_protocol::begin(HardwareSerial *serial, long timeout)
{
  serial ->begin (115200);
  serial ->setTimeout(timeout);
}

 /**
 * @brief   		                  У��ͺ�����У��͹�ʽ��
 *                                    Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
 * @code                        ʾ��
 *      				                      // Steer_protocol Steer1; �������buf�����һ������ΪУ���   byte buf[] = {1,2,3,4,5,6}    ��ʱ len = 5
 *      				                      Steer1.Check_Sum(buf , 5);           
 *                                    return  ~��3+4+5��
 * @endcode
 * @param[in]                 buf          ��ҪУ��������ַ
 * @param[in]                 len           ���鳤�ȣ� ��ʾ���ĳ���Ϊ5
 * @return                      sum         ����ֵ��У���
 */
byte Steer_protocol::Check_Sum(byte *buf, byte len)
{
    int    tmp = 0;
    byte  cut = 0;
    byte  sum = 0;
    
    for(cut = 2; cut < len; cut++)
    {
        tmp += buf[cut];
    }
    sum = (byte)((~tmp) & 0xff );
    return sum;
}

 /**
 * @brief   		                  ����״̬��ѯ����    
 * @code                        ʾ��  
 *      				                      //Steer_protocol Steer1; byte *dat; ��ѯ ID Ϊ 1 �Ķ�� �Ĺ���״̬
 *      				                      Steer1.ping(0x01 , dat);       
 * @endcode
 * @param[in]                 id             �����ID
 * @param[in]                 data         ��������Ϊbyte��ָ�룬��ö������״̬
 * @return                      true:         ����ֵ�������Ӧ��ͨ������
 *                                    false:        ����ֵ�������Ӧ��ͨ�Ų�����
 */
 boolean Steer_protocol::ping(byte id, byte *data)
 {
    byte buf[6] = {0xff , 0xff, id , 0x02 , 0x01, (byte)( (~(id+3)) & (0xff) )};

    while(svSer -> availableForWrite() < 6); 
    svSer -> write(buf, 6);
    svSer -> flush();
             
     byte cnt = svSer -> readBytes(buf , 6);
    
     if((cnt == 6)  && (buf[0] == 0xff) && (buf[1] == 0xf5) && (buf[2] == id) 
          && (buf[3] == 0x06) && (buf[5] == Check_Sum( buf, 5 )) )
     {
            *data = buf[4];
            return true;
     }
     else
     {
        return false;
     }         
  }

 /**
 * @brief   		                  ��ȡ���״̬����    print
 * @code                        ʾ��  
 *      				                      //���ID = 1���ڿ��Ʊ���ӵ�ַ 0X38 ����ȡ�����ֽ�,Steer_protocol Steer1; byte dat[2];
 *      				                      Steer1.read(0x01 , 0x38 , dat , 0x02);       
 * @endcode
 * @param[in]                 id                        �����ID
 * @param[in]                 regStartAddr       ���ȡ��Ϣ���ڴ濪ʼ��ַ
 * @param[in]                 data                    ��������Ϊbyte��ָ�룬�洢��ȡ����Ϣ
 * @return                      true:                    ����ֵΪ�棺��ȡ�ɹ�
 *                                    false:                   ����ֵΪ�٣���ȡʧ��
 */
boolean Steer_protocol::read(byte id, byte regStartAddr, byte *data, byte readlen)
{
      byte writebuf[8] = {0xff, 0xff, id, 0x04, 0x02, regStartAddr, readlen, (byte)( (~(id + 6 +regStartAddr +readlen)) & 0xff ) };
      
      while(svSer -> availableForWrite() < 8);
  
      svSer -> write(writebuf , 8);
      svSer -> flush();


      byte readbuf[readlen + 6]; 
      
      byte cnt = svSer -> readBytes( readbuf , readlen + 6);

       if( (cnt == ( readlen + 6)) && ( readbuf[0] == 0xff) && ( readbuf[1] == 0xf5) && ( readbuf[2] == id) && (readbuf[3] == readlen + 2) 
       && ( readbuf[readlen + 5] == (byte)( Check_Sum( readbuf , readlen + 5) )))
      {
          memcpy(data, &readbuf[5], readlen);
            return true;
  }
  return false;

}

 /**
 * @brief   		                  �����λ����    
 * @code                        ʾ��  
 *      				                      //��ID = 1 �Ķ�����и�λ , Steer_protocol Steer1 ;
 *      				                      Steer1.reset(0x01) :       
 * @endcode
 * @param[in]                 id            �����ID
 */
void Steer_protocol::reset(byte id)
{
  byte buf[6] = { 0xff, 0xff, id, 0x2, 0x6, (byte)((~(id+8)) & 0xff ) };

  while(svSer -> availableForWrite() < 6);
  svSer -> write(buf, 6);
  svSer -> flush();
}

 /**
 * @brief   		                  д����    
 * @code                        ʾ��  
 *      				                      //���ID = 1���ڿ��Ʊ���ӵ�ַ 0X2A ��д������ֽڣ�ΪĿ��λ��,Steer_protocol Steer1; byte dat[2] = { 0x00 , 0xff };
 *      				                      Steer1.write( 0x01 , 0X2A , dat , 0x02 );       
 * @endcode
 * @param[in]                 id                         �����ID
 * @param[in]                 regStartAddr        ��д����Ϣ���ڴ濪ʼ��ַ
 * @param[in]                 buf                       ��������Ϊbyte������ָ�룬�洢��Ҫд����Ϣ
 * @param[in]                 bufLen                 buf�����鳤��
 */
void Steer_protocol::write(byte id, byte regStartAddr, byte *buf, byte bufLen)
{
  byte writeBufLen = bufLen + 7;
  byte writeBuf[writeBufLen] = {0xff, 0xff, id, (byte)(bufLen+3), INSTRUCTION_WRITE_DATA, regStartAddr};
  memcpy(&writeBuf[6], buf, bufLen);
  writeBuf[writeBufLen - 1] = Check_Sum(writeBuf, writeBufLen - 1);

  while(svSer -> availableForWrite() < writeBufLen);
  svSer -> write(writeBuf, writeBufLen);
  svSer -> flush();
  /*
  Serial.println( "writeBuf");
  for( int i = 0; i < writeBufLen; i++)
  {
    Serial.print( "   0X");
    Serial.print( writeBuf[ i ] , HEX);
  }
    Serial.println( "");
    Serial.println( "writeBuf over");*/
  }


 /**
 * @brief   		                  ͬ��д�������㲥��:������ִ����ͬ���ܣ�������ͬ�����ݿ��Բ�ͬ���������ͬʱ�趨Ŀ��λ�ã�    
 * @code                        ʾ��  
 *      				                      //�ڿ��Ʊ���ӵ�ַ 0X2A ��д������ֽڣ�ΪĿ��λ��,Steer_protocol Steer1; byte dat[] = { 0x01,0x00 , 0xff........0x06,0x00,0xff  };
 *      				                      Steer1.sync_write( 0X2A , buf , 6 , 3);       
 * @endcode
 * @param[in]                 regStartAddr        ��д����Ϣ���ڴ濪ʼ��ַ
 * @param[in]                 buf                       ��������Ϊbyte������ָ�룬�洢��Ҫд����Ϣ
 * @param[in]                 svNum                 ��������Ϊbyte������ָ�룬�洢��Ҫд����Ϣ
 * @param[in]                 perDataLen           д�����ݵĳ���+1��ΪID�ţ�
 */
void Steer_protocol::sync_write(byte regStartAddr, byte *buf, byte svNum, byte perDataLen)
{
  byte writeBufLen = perDataLen * svNum + 8;
  
  byte writeBuf[writeBufLen] = {0xff, 0xff, BROADCAST_ADDR, (byte)((writeBufLen - 4) & 0xff),
                                INSTRUCTION_SYNC_WRITE, regStartAddr, (byte)((perDataLen - 1) & 0xff )};
                                
  memcpy(&writeBuf[7], buf, writeBufLen - 8);
  writeBuf[writeBufLen - 1] = Check_Sum(writeBuf, writeBufLen - 1);

  while(svSer -> availableForWrite() < writeBufLen);
  svSer -> write(writeBuf, writeBufLen);
  svSer -> flush();
}










