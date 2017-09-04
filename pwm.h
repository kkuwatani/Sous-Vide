#define PWM_PIN     PD5
#define PWM_DDR     DDRD

void pwm_init() {
	//set OC1A pin as output
	PWM_DDR |= (1<<PWM_PIN);

	//set registers to Fast PWM mode, prescaler = 8 (2Hz cycle)
	TCCR1A |= (1<<COM1A1)|(0<<COM1A0)|(1<<WGM11);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11);
	ICR1 = 62500;

	//turn on PWM at 0% duty, 62500 = 100%
	OCR1A = 0;
	TCNT1 = 0;

	//enable TOV1 interrupt
	TIMSK |= (1<<TOIE1);
}

void pwm_set(int percent) {
    OCR1A = percent * 625;
}
