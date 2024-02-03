//#######################################################################################################################
//#
//# ��������� �������������� 00.00 - 50.00�,  0.000 - 5.000�   ������� 4�2
//# Chip - ATtiny13A 
//# F_CPU = 9,6 MHz
//#
//#	(programmed by SHADS)
//#
//#######################################################################################################################
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

//��������� 
#define F_CPU 9600000

//#define LED_DISP_STROBE_INVERT //������ ������� ������������� (����� ����������������)
#define LED_DISP_DATA_INVERT //������ ��������� ������������� (����� ����������������)
//��� ������� � �� ������������� �������� - ���������������� LED_DISP_STROBE_INVERT
//��� ������� � �� ������������� �������� - ���������������� LED_DISP_DATA_INVERT
//��� ������� � OA � � ��������� ������� - ����������������� LED_DISP_STROBE_INVERT � LED_DISP_DATA_INVERT
//��� ������� � OK � � ��������� ������� - ����������������  LED_DISP_STROBE_INVERT � LED_DISP_DATA_INVERT

#define SERIAL_PORT PORTB
#define SERIAL_DDR DDRB
#define SERIAL_CLK_LINE (1<<1) //����� ������������
#define SERIAL_DATA_LINE (1<<2) //����� ������
#define SERIAL_STROBE_LINE (1<<0) //����� �������������

// ��������� ���� � PROGMEM
uint8_t SimbMass [] PROGMEM = 
{   
	0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,0b01101101,0b01111101,0b00000111,0b01111111,0b01101111
};

//���������� ����������
uint8_t DispBuffer [8]; //����� ������� (8 ������)
uint16_t AdcBuffer [2]; //����� �������� ADC (2 ������)

//���������� �������
void DispIntShow (uint16_t Integer, uint8_t DispNum);

//������� �������
int main (void)
{
  //������������� ADC
  ADMUX = (1<<REFS0)|//� ���-�� ��� ���������� �������� 1,1�
    (1<<MUX1)| (0<<MUX0)|//����� ���2 
    (0<<ADLAR);//��������� ������
  ADCSRA =(1<<ADEN)|//��� ���
    (1<<ADPS2)| (1<<ADPS1)| (1<<ADPS0);//������������

  //������������� ������ ���������� ��������
  SERIAL_DDR |= SERIAL_CLK_LINE; 
  SERIAL_DDR |= SERIAL_DATA_LINE;
  SERIAL_DDR |= SERIAL_STROBE_LINE;

  //��������� ���������� �� ������� 1000 ��
  TCCR0A = (1<<WGM01)| (0<<WGM00);//����� ������� - CTC 
  TCCR0B = (0<<WGM02)|
    (0<<CS02)| (1<<CS01)| (1<<CS00);//������������� ������ ������������ ������� (������� �� 64)
  TIMSK0 = (1<<OCIE0A);//������������� �������\�������� �� ���������� � OCR0A
  OCR0A = 150 - 1;//��������� �������� ��������� (f ���������� =1 khz)

  //������� ����
  sei ();
  while (1)
  {
    DispIntShow (AdcBuffer[0] * 8,1);//������� � ����� ������� �������� ADC ���������� �� 8 (������ �� 81,91V)
    DispIntShow (AdcBuffer[1] * 8,2);//������� � ������ ������� �������� ADC ���������� �� 8 (������ �� 8,191A)
  }
}

//������� ������ ����� � ����� ������� 
//��������1 - ����� � ��������� 0 - 9999
//��������2 - 1-����� �������, 2-������ �������
void DispIntShow (uint16_t Integer, uint8_t DispNum)
{
  uint8_t *pBuf = DispBuffer+3;//��������� ��� ������ ����� (� �����) � �����
  if (DispNum == 2)	pBuf += 4;//��� ������� �������

  for (uint8_t i=0; i<4; i++)
  {//������� ����� � �����
    *pBuf-- = pgm_read_byte (SimbMass + (Integer %10));//�������� ������� �����
    Integer /= 10;//������� �� 10
  }
}


//��������� ���������� (������� ������ 1000 ��)
ISR (TIM0_COMPA_vect)
{
  //������������ ���������� ������� � ������� ������ ����� ��������� ������� 74HC595
  //���� 15,14,13,12,11,10,9,8 ����� Word - ������������� �������� �������  8,7,6,5,4,3,2,1
  //���� 7,6,5,4,3,2,1,0       ����� Word - ������������� ��������� ������� H,G,F,E,D,C,B,A
  static uint8_t *pBufPointer = DispBuffer;//��������� �� ������ ������
  static uint8_t SimbOnMask;//����� ��������� ������� �������
  if (! SimbOnMask)
  {//�������� �� ��������� �����
    SimbOnMask = 0x01;//������ �������
    pBufPointer = DispBuffer;
  }
 
  uint8_t Byte = *pBufPointer;
  if ((SimbOnMask == (1<<1)) || (SimbOnMask == (1<<4))) Byte |= 0b10000000;//������������ ����� � 1 � 4 �����������
  uint16_t Word = (SimbOnMask<<8) + Byte;

  #ifdef LED_DISP_STROBE_INVERT
    Word ^= 0xff00;	
  #endif
  #ifdef  LED_DISP_DATA_INVERT
    Word ^= 0x00ff;
  #endif

  for (uint8_t i=0; i<16; i++)
  {//���� �������� 16 ��� ������
    if (Word & 0x8000)  SERIAL_PORT |=  SERIAL_DATA_LINE;//��������� ����� ������
    else                SERIAL_PORT &= ~SERIAL_DATA_LINE;//����� ����� ������ 

    SERIAL_PORT |= SERIAL_CLK_LINE;//������� �������������
    asm ("nop");
    SERIAL_PORT &= ~SERIAL_CLK_LINE;
    Word <<= 1;
  }

  //������� ������������� ���������� ���� �� �����
  SERIAL_PORT |= SERIAL_STROBE_LINE;
  asm ("nop");
  SERIAL_PORT &= ~SERIAL_STROBE_LINE;

  //����� ����� � ���������
  SimbOnMask <<= 1;
  pBufPointer++;

  //����� ADC �������
  static uint16_t AdcTemp[2];//���������� ��� ����������
  static uint8_t AdcCounter;//������� ���������� (������� ��� ���������� ����� ������ ADC) 

  if (++AdcCounter == 128)
  {//�.�. ������ 128 ��������, �� �� ������ ����� ������ ��������, �.�. 64
    AdcCounter = 0;
    AdcBuffer[0] = AdcTemp[0] / 64;//��������� � ������ ������
    AdcBuffer[1] = AdcTemp[1] / 64;
    AdcTemp[0] = 0;//�������� ����������
    AdcTemp[1] = 0;
  }

  if (AdcCounter & 0x01)
  {//���� TRUE �� ��������� ADC2, FALSE - ��������� ADC3 (�� ��� �������� :)...)
    AdcTemp[0] += ADC;//��������� 64 ��� ��� ����������
    ADCSRA |= (1<<ADSC);//��������� ��������������
    ADMUX &= ~(1<<MUX0);//�������� ������ �����
  }
  else 
  {
    AdcTemp[1] += ADC;//��������� 64 ��� ��� ����������
    ADCSRA |= (1<<ADSC);//��������� �������������� 
    ADMUX |= (1<<MUX0);//�������� ������ �����
  }
}

//# THE END!
