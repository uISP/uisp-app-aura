#include <arch/antares.h>
#include <avr/boot.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <generated/usbconfig.h>
#include <arch/vusb/usbportability.h>
#include <arch/vusb/usbdrv.h>

const int prescaler_lookup[] = { 0, 1, 8, 64, 256, 1024 };

struct avr_pwm16 {
	uint16_t icr; /* icr */
	uint8_t cs; /* prescaler value */ 
};
static struct avr_pwm16 conf;

struct avr_pwm16 avr_timer_conf(uint64_t period_us)
{
	int i; 
	uint64_t tmp;
	struct avr_pwm16 pwm;
	for (i=1; i<ARRAY_SIZE(prescaler_lookup); i++)
	{
		tmp = (F_CPU / prescaler_lookup[i]) * period_us / 1000000 ;
		if (tmp < 65536) 
			break;
	}	
	pwm.icr = (uint16_t) tmp;
	pwm.cs = i;
	return pwm;
} 

static void pwm_init(struct avr_pwm16* conf)
{
	TCCR1A = 0;
	TCCR1B = 0;
	ICR1 = conf->icr;
	TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B = (1 << WGM13) | (1<<WGM12) | conf->cs;
	TCCR1A |= 2<<6;
	DDRB |= 1<<1 | 1<<2;
}

static uint16_t adc_read(uint8_t reference, uint8_t channel)
{
	ADMUX &= ~(0xf | (1<<6) | (1<<7));
	ADMUX |= channel & 0xf;
	ADMUX |= reference;
	ADCSRA |= (1 << ADSC);  
	while(ADCSRA & (1<<ADSC)); 
	uint16_t ret = ADCL;
	ret |= ((uint16_t) (ADCH << 8));
	return ret;
}


char msg[128];
uchar   usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (void *)data;
	switch (rq->bRequest){
	case 0: /* get_last_message */
		usbMsgPtr = msg;
		return strlen(msg) + 1;
		break;
	case 1: /* led */
		PORTC&=~(1<<2);
		PORTC|=(!!(rq->wValue.bytes[0]) << 2);
		break;
	case 2: { /* pwm_init */
		conf = avr_timer_conf(rq->wValue.word);
		pwm_init(&conf);
		sprintf(msg, "ICR: %u  | Prescaler: %u", conf.icr, conf.cs);
		break;
	}
	case 3: { /* pwm_set */
		uint64_t hticks = ((uint64_t) (F_CPU / prescaler_lookup[conf.cs])) * rq->wIndex.word / 1000000 ;
		if (rq->wValue.word == 0) 
			OCR1A = (uint16_t) hticks;
		else
			OCR1B = (uint16_t) hticks;
		sprintf(msg, "Channel %d ticks %u", rq->wValue.word, (uint16_t) hticks);
		break;
	}

	case 4:
		return USB_NO_MSG;
		
	}
	return 0;
}

uchar usbFunctionWrite(uchar *data, uchar len)
{
	if (strcmp(data, msg)==0)
		PORTC^=1<<2;
}

inline void usbReconnect()
{
	DDRD=0xff;
	_delay_ms(250);
	DDRD=0;
}

ANTARES_INIT_LOW(io_init)
{
	DDRC=1<<2;
	PORTC=0xff;
 	usbReconnect();
}

ANTARES_INIT_HIGH(uinit)
{
  	usbInit();
}


ANTARES_APP(usb_app)
{
	usbPoll();
}
