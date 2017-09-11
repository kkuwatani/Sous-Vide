void adc_init() {
	//configure for 10bit ADC, right shifted
	ADCSRA |= 1<<ADPS2;
	ADMUX |= (1<<REFS0) | (1<<REFS1);
	ADCSRA |= 1<<ADEN;
	//start first adc conversion with interrupt enabled
//	adc_interrupt_enable();
//	adc_start_conversion();

}

void adc_interrupt_enable() {
    ADCSRA |= (1<<ADIE);
}

void adc_interrupt_disable() {
    ADCSRA &= ~(1<<ADIE);
}

void adc_start_conversion() {
    ADCSRA |= 1<<ADSC;
}

uint16_t adc_receive() {
    uint8_t low = ADCL;
	uint16_t result = ADCH<<8 | low;
	return result;
}
