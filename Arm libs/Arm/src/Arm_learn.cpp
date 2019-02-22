#include <Arm_learn.h>

Arm_learn MyArm_learn;                                                //��ʼ��һ����е��ѧϰ����


 /**
 * @brief   		                  ��е�۹��캯��
 * @note                         ������ʼ����е�۵�ѧϰ����    
 * @code                        ʾ��  
 *      				                      Arm_learn MyArm_learn;  
 * @endcode
 */
void Arm_learn::Arm_learn_()
{
    button0_time_cnt = 0;
    button1_time_cnt = 0; 
    rom_offset = 8* sizeof(short);
    EEPROM.get(rom_offset + 1*sizeof(word), record_point_num);
    start_learn_flag = false;
    record_flag = false;
    
    pinMode(BTN0_PIN, INPUT);
    pinMode(BTN1_PIN, INPUT);
    
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);                   
}

 /**
 * @brief   		                  ������⺯��
 * @code                        ʾ��  
 *      				                      MyArm_learn.button_learn_detect();    
 * @endcode
 */
void Arm_learn::button_learn_detect(void)
{
  if(digitalRead(BTN0_PIN) == LOW)
  {
    button0_time_cnt = 0;
    for(int i = 0; (i < 3000) && (digitalRead(BTN0_PIN) == LOW) ; i++)
    {
      button0_time_cnt = i;
      delay(10);
    }
  }
  
  if(digitalRead(BTN1_PIN) == LOW)
  {
    button1_time_cnt = 0;
    for(int i = 0; (i < 3000) && (digitalRead(BTN1_PIN) == LOW) ; i++)
    {
      button1_time_cnt = i;
      delay(10);
    }
  }
  
  if(button0_time_cnt > 101)  { start_learn_flag = !start_learn_flag;  record_flag = false; button0_time_cnt=0;
                                                if(start_learn_flag == true){record_point_num = 0;MyArm.Set_Arm_Torque_Off();} tone(37, 3000, 2000);}
  if( button1_time_cnt > 18) { record_flag = true; button1_time_cnt=0;}
}

 /**
 * @brief   		                  ��е��ѧϰ����
 * @code                        ʾ��  
 *      				                      MyArm_learn.start_learn();    
 * @endcode
 */
void Arm_learn::start_learn(void)
{
  button_learn_detect();

  while(start_learn_flag == true)
  {
    //Serial.println("test222!!!");
    button_learn_detect();
    if( record_flag == true)
    {
      record_point_num++;
      record_one_point();
      record_flag = false;
      EEPROM.put(rom_offset + 1*sizeof(word), record_point_num);
      tone(37, 3000, 500);
      Serial.println("test!!!");
      delay(10);
    }
  }
}

 /**
 * @brief   		                  λ���¼����
 * @code                        ʾ��  
 *      				                      MyArm_learn.start_learn();    
 * @endcode
 */
void Arm_learn::record_one_point(void)
{
  for(int i = 0 ; i < 7; i ++)
  {
      word pos_tmpe = get_steer_positon(i);
      Serial.print("get from steer     ");Serial.println(pos_tmpe);
      EEPROM.put(rom_offset + 2*sizeof(word) + record_point_num*7*sizeof(word) + i*sizeof(word), pos_tmpe);
  }
  Serial.println(record_point_num);
  for(int i = 0; i < 7; i++)
  {
      word pos_tmpe = 0;
      EEPROM.get(rom_offset + 2*sizeof(word) + record_point_num*7*sizeof(word) + i*sizeof(word), pos_tmpe);
      Serial.print("get for eeprom    "); Serial.println(pos_tmpe);
  }
}

 /**
 * @brief   		                  ��е��ѧϰ��̬���ֺ���
 * @code                        ʾ��  
 *      				                      MyArm_learn.reappear_learn();    
 * @endcode
 */
void Arm_learn::reappear_learn()
{
  button_learn_detect();
  
  if((start_learn_flag == false) && (record_flag == true))
  {  
    word tmpe_pos[7] = {0};

   // MyArm.Set_Arm_Torque_On();
    EEPROM.get(rom_offset + 1*sizeof(word), record_point_num);
    Serial.print("record_point_num:   ");Serial.println(record_point_num);
    for(int i = 1; i <= record_point_num; i++)
    {
      for(int j = 0; j < 7; j++)
      {
          EEPROM.get(rom_offset + 2*sizeof(word) + i*7*sizeof(word) + j*sizeof(word), tmpe_pos[j]);
          Serial.println(tmpe_pos[j]);
      }
          MyArm.move_to_position(  tmpe_pos[0] ,  tmpe_pos[1],  tmpe_pos[2] ,  RUN_TIME);
          MyArm.turn_steer_345_to_positon(tmpe_pos[3] ,  tmpe_pos[4],  tmpe_pos[5] ,  RUN_TIME);
          delay(RUN_TIME);
    }
  }
}

 /**
 * @brief   		                  �õ���е��λ�ˣ����������ξ�ֵ�˲���
 * @code                        ʾ��  �õ�һ�Ŷ����λ������
 *      				                      MyArm_learn.get_steer_positon(1);    
 * @endcode
 * @param[in]                 i            �����ID��
 */
word Arm_learn::get_steer_positon(byte i)
{
  word pos_tmp[3] = {0};
  for(int j = 0; j < 3; j++)
  {
    MyArm.steer[i]->Get_Steer_Position_Current_Inf();
    pos_tmp[j] = MyArm.steer[i] ->Position_Current[0];
    pos_tmp[j] = (pos_tmp[j] << 8) + MyArm.steer[i] ->Position_Current[1];
  }
  return (pos_tmp[0] + pos_tmp[1] + pos_tmp[2])/3;
}

//�����ٶ�ȥ���������