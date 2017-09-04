#define LED_PORT    PORTB
#define LED_PIN     PB0
#define LED_DDR     DDRB

void button_init() {
	MCUCR |= (1<<ISC01) | (1<<ISC00);
	LED_DDR |= 1 << LED_PIN;
	button_interrupt_enable();
}

void button_interrupt_enable {
    GICR |= (1<<INT0);
}

void button_interrupt_disable {
    GICR &= ~(1<<INT0);
}

void running_led_enable() {
    LED_PORT |= (1 << LED_PIN);
}

void running_led_disable() {
    LED_PORT &= ~(1 << LED_PIN);
}