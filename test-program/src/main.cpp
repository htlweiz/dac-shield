#include <Arduino.h>

/**
 * @brief Test DAC methds
 * 
 * PWM on PD3/OC2B
 * Single bit DAC on PD4
 * R2R DAC on PB0..PB5
 * 
 */


static uint16_t sb_target;

ISR(ADC_vect) {
  if(ADC > sb_target) {
    PORTD &= ~(1 << PORTD4);
  } else {
    PORTD |= (1 << PORTD4);
  }
}

void setup() {
  // PB0 to PB5 is output
  DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5);

  // PD4 is output
  DDRD |= (1 << DDD4);
  // ADC channel is ADC1, AVcc is ref
  ADMUX = (1 << REFS0) | (1 << MUX0);
  // ADC Free Running Mode
  ADCSRB = 0;
  // ADC enable, start conversion, auto trigger, enable interrupt, prescale by 128 (2**7)
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (7 << ADPS0);

  // PD3 is output
  DDRD |= (1 << DDD3);

  // Serial console
  Serial.begin(230400);

  // Timer 2
  // Fast PWM (WGM22:0 = 7)
  // with OCRA (TOP) set to 255 for 8 bit operation
  // no prescaling
  // clocked from clk_IO
  ASSR = 0; // clock from clk_IO
  OCR2A = 255; // count from 0 to 255
  OCR2B = 128; // initial compare value
  TCCR2A = (2 << COM2B0) | (3 << WGM20);
  TCCR2B = (1 << WGM22) | (1 << CS20);
  
  // enable interrupts globally
  sei();
}

void loop() {
  static uint8_t pwm_target = 0;
  static uint8_t r2r_target = 0;

  if(sb_target < 1022) {
    sb_target++;
  } else {
    sb_target = 0;
  }

  OCR2B = pwm_target++;
  PORTB = (r2r_target++) & 0x3F;

  delay(10);
}
