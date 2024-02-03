//#######################################################################################################################
//#
//# Программа Милливольтметр 00.00 - 50.00В,  0.000 - 5.000А   Дисплей 4х2
//# Chip - ATtiny13A 
//# F_CPU = 9,6 MHz
//#
//#	(programmed by SHADS)
//#
//#######################################################################################################################
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

//настройки 
#define F_CPU 9600000

//#define LED_DISP_STROBE_INVERT //данные стробов инвертированы (иначе закомментировать)
#define LED_DISP_DATA_INVERT //данные сегментов инвертированы (иначе закомментировать)
//для дисплея с ОА подключенного напрямую - закомментировать LED_DISP_STROBE_INVERT
//для дисплея с ОК подключенного напрямую - закомментировать LED_DISP_DATA_INVERT
//для дисплея с OA и с инверсией стробов - раскомментировать LED_DISP_STROBE_INVERT и LED_DISP_DATA_INVERT
//для дисплея с OK и с инверсией стробов - закомментировать  LED_DISP_STROBE_INVERT и LED_DISP_DATA_INVERT

#define SERIAL_PORT PORTB
#define SERIAL_DDR DDRB
#define SERIAL_CLK_LINE (1<<1) //линия тактирования
#define SERIAL_DATA_LINE (1<<2) //линия данных
#define SERIAL_STROBE_LINE (1<<0) //линия стробирования

// константы цифр в PROGMEM
uint8_t SimbMass [] PROGMEM = 
{   
	0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,0b01101101,0b01111101,0b00000111,0b01111111,0b01101111
};

//глобальные переменные
uint8_t DispBuffer [8]; //буфер дисплея (8 знаков)
uint16_t AdcBuffer [2]; //буфер значений ADC (2 канала)

//объявления функций
void DispIntShow (uint16_t Integer, uint8_t DispNum);

//Главная функция
int main (void)
{
  //инициализация ADC
  ADMUX = (1<<REFS0)|//в кач-ве ИОН внутренний источник 1,1в
    (1<<MUX1)| (0<<MUX0)|//канал АЦП2 
    (0<<ADLAR);//выровнять вправо
  ADCSRA =(1<<ADEN)|//вкл АЦП
    (1<<ADPS2)| (1<<ADPS1)| (1<<ADPS0);//предделитель

  //инициализация портов сдвигового регистра
  SERIAL_DDR |= SERIAL_CLK_LINE; 
  SERIAL_DDR |= SERIAL_DATA_LINE;
  SERIAL_DDR |= SERIAL_STROBE_LINE;

  //настройка прерываний по таймеру 1000 Гц
  TCCR0A = (1<<WGM01)| (0<<WGM00);//режим таймера - CTC 
  TCCR0B = (0<<WGM02)|
    (0<<CS02)| (1<<CS01)| (1<<CS00);//инициализация режима предделителя таймера (деление на 64)
  TIMSK0 = (1<<OCIE0A);//инициализация таймера\счетчика по совпадению с OCR0A
  OCR0A = 150 - 1;//настройка регистра сравнения (f прерывания =1 khz)

  //Главный цикл
  sei ();
  while (1)
  {
    DispIntShow (AdcBuffer[0] * 8,1);//выводим в левый дисплей значение ADC умноженное на 8 (вольты до 81,91V)
    DispIntShow (AdcBuffer[1] * 8,2);//выводим в правый дисплей значение ADC умноженное на 8 (амперы до 8,191A)
  }
}

//Функция вывода числа в буфер дисплея 
//АРГУМЕНТ1 - число в диапазоне 0 - 9999
//АРГУМЕНТ2 - 1-левый дисплей, 2-правый дисплей
void DispIntShow (uint16_t Integer, uint8_t DispNum)
{
  uint8_t *pBuf = DispBuffer+3;//указатель для печати числа (с конца) в буфер
  if (DispNum == 2)	pBuf += 4;//для правого дисплея

  for (uint8_t i=0; i<4; i++)
  {//выводим цифры в цикле
    *pBuf-- = pgm_read_byte (SimbMass + (Integer %10));//печатаем младшую цифру
    Integer /= 10;//деление на 10
  }
}


//обработка прерывания (частота вызова 1000 Гц)
ISR (TIM0_COMPA_vect)
{
  //ДИНАМИЧЕСКОЕ ОБНОВЛЕНИЕ ДИСПЛЕЯ с выводом данных через сдвиговый регистр 74HC595
  //биты 15,14,13,12,11,10,9,8 байта Word - соответствуют символам дисплея  8,7,6,5,4,3,2,1
  //биты 7,6,5,4,3,2,1,0       байта Word - соответствуют сегментам символа H,G,F,E,D,C,B,A
  static uint8_t *pBufPointer = DispBuffer;//указатель на начало буфера
  static uint8_t SimbOnMask;//маска активного разряда дисплея
  if (! SimbOnMask)
  {//проверка на окончание цикла
    SimbOnMask = 0x01;//начать сначала
    pBufPointer = DispBuffer;
  }
 
  uint8_t Byte = *pBufPointer;
  if ((SimbOnMask == (1<<1)) || (SimbOnMask == (1<<4))) Byte |= 0b10000000;//дорисовываем точку в 1 и 4 знакоместах
  uint16_t Word = (SimbOnMask<<8) + Byte;

  #ifdef LED_DISP_STROBE_INVERT
    Word ^= 0xff00;	
  #endif
  #ifdef  LED_DISP_DATA_INVERT
    Word ^= 0x00ff;
  #endif

  for (uint8_t i=0; i<16; i++)
  {//цикл передачи 16 бит данных
    if (Word & 0x8000)  SERIAL_PORT |=  SERIAL_DATA_LINE;//установка линии данных
    else                SERIAL_PORT &= ~SERIAL_DATA_LINE;//сброс линии данных 

    SERIAL_PORT |= SERIAL_CLK_LINE;//импульс проталкивания
    asm ("nop");
    SERIAL_PORT &= ~SERIAL_CLK_LINE;
    Word <<= 1;
  }

  //импульс стробирования записанной инфы на выход
  SERIAL_PORT |= SERIAL_STROBE_LINE;
  asm ("nop");
  SERIAL_PORT &= ~SERIAL_STROBE_LINE;

  //сдвиг маски и указателя
  SimbOnMask <<= 1;
  pBufPointer++;

  //ОПРОС ADC КАНАЛОВ
  static uint16_t AdcTemp[2];//накопители для усреднения
  static uint8_t AdcCounter;//счетчик накоплений (младший бит определяет номер канала ADC) 

  if (++AdcCounter == 128)
  {//т.к. крутим 128 значений, то на каждый канал дается половина, т.е. 64
    AdcCounter = 0;
    AdcBuffer[0] = AdcTemp[0] / 64;//усредняем и выдаем наверх
    AdcBuffer[1] = AdcTemp[1] / 64;
    AdcTemp[0] = 0;//обнуляем накопители
    AdcTemp[1] = 0;
  }

  if (AdcCounter & 0x01)
  {//если TRUE то обработка ADC2, FALSE - обработка ADC3 (ну или наоборот :)...)
    AdcTemp[0] += ADC;//суммируем 64 раз для усреднения
    ADCSRA |= (1<<ADSC);//запустить преобразование
    ADMUX &= ~(1<<MUX0);//включаем нужный канал
  }
  else 
  {
    AdcTemp[1] += ADC;//суммируем 64 раз для усреднения
    ADCSRA |= (1<<ADSC);//запустить преобразование 
    ADMUX |= (1<<MUX0);//включаем нужный канал
  }
}

//# THE END!
