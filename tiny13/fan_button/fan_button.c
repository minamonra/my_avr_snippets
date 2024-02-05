#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define PWMPIN  PB0
#define LEDPIN  PB1
#define PWRPIN  PB2
#define SWPIN   PB3
#define	ADCPIN  PB4
#define	SAMPLES_MAX (64) // max number of samples


static uint32_t samples_sum = 0;
static volatile uint8_t samples_cnt = 0;

volatile uint32_t ttms = 0, lastms = 0;
volatile uint8_t pressed = 0, released = 0, overheat = 0, pwrstate = 0; // shortpress = 0, longpress = 0;
volatile uint8_t lastbtcnt = 0, btcnt = 0;
volatile uint16_t duty = 0, value = 0;



static void adc_init(void)
{
  DDRB   &= ~(1 << ADCPIN);
  PORTB  &= ~(1 << ADCPIN);
  ADCSRA |=  (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADPS2-0 (ADC Prescaler Select Bits)
  ADCSRA |=  (1 << ADEN ) | (1 << ADIE); // ADEN (ADC Enable)   ADIE (ADC Interrupt Enable)
  ADMUX   =  (1 << MUX1); // MUX1-0 (Analog Channel Selection Bits) 
                          // REFS0 не трогаем, опора от VCC
  sei(); // разрешаем прерывания
  ADCSRA |= _BV(ADSC); // ADC Start Conversion

}
static void pwm_init()
{
  DDRB   |= (1 << PWMPIN);
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  TCCR0A |= (1 << COM0A1);
  TCCR0B  = (TCCR0B & ~((1<<CS02)|(1<<CS01)|(1<<CS00))) | 1;
}

static void pwm_set_duty(uint8_t duty)
{
  OCR0A = duty; // уcтaнoвить OCRnx
}

void poweroff()
{
  PORTB |=  (1 << LEDPIN);
  PORTB &= ~(1 << PWRPIN);
  pwrstate = 0;
}

void poweron()
{
  PORTB &= ~(1 << LEDPIN);
  PORTB |=  (1 << PWRPIN);
  pwrstate = 1;
}

void button_process()
{
  if (overheat) { poweroff(); return; }

  if (lastms > ttms || ttms - lastms > 100)
  {
    lastms = ttms;
	
	  if (((PINB & (1 << SWPIN)) >> SWPIN) == 0) 
	  { 
	  	btcnt++; 
	  	lastbtcnt = btcnt;
	  }  
	  else btcnt = 0;
	
	  if (btcnt) pressed = 1; 
	
	  if ((pressed) & (btcnt == 0)) released = 1;

	  if ((lastbtcnt > 2) & (released)) // Короткое нажатие -> выключение
	  {
	    pressed = 0;
	    released = 0; 
	    lastbtcnt = 0;
      poweroff();
	  }
		
	  if (lastbtcnt > 20) // Длинное нажатие (удержание)
	  {
	    pressed = 0; 
	    // released = 0; 
	    lastbtcnt = 0;
      poweron();
      // if (pwrstate == 0) poweron(); else poweroff(); // Если БП выключен -> включим
	  }
  }
}

void adc_process(void)
{
  if (samples_cnt < SAMPLES_MAX) {return;}
	value = samples_sum / (uint32_t)samples_cnt;
	samples_sum = samples_cnt = 0;
	ADCSRA |= _BV(ADSC);
	
	if (value > 800) 
	{ 
	  overheat = 1; 
	  pwm_set_duty(254);
	}
	else 
	{
      overheat = 0;
	  if (value < 565) value = 565;
	  duty = value - 560;          //
	  if (duty < 30)  duty =   0;  //
	  if (duty > 240) duty = 254;
	  pwm_set_duty(duty);
	}
}

int main(void)
{
  DDRB  |=  (1 << LEDPIN);
  DDRB  |=  (1 << PWRPIN);
  poweroff();
  adc_init();
  pwm_init();
  pwm_set_duty(254);
  _delay_ms(2000);
  pwm_set_duty(0);

  while(1)
  {
    adc_process();
    button_process();

  }
  return 0;
}

ISR(ADC_vect)
{
  uint8_t low, high;
  if (samples_cnt < SAMPLES_MAX)
  {
    low = ADCL;
    high = ADCH;
    samples_sum += (high << 8) | low;
    samples_cnt++;
    ADCSRA |= _BV(ADSC);
    ttms++;
  }
}

// EOF