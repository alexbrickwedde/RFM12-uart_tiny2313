#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <avr/eeprom.h>
//#include <avr/wdt.h>
#include <util/delay.h>

#ifndef cbi
#define cbi(sfr, bit)     (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit)     (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define RF12FREQ868(freq)       ((unsigned short)((freq-860.0)/0.005))
#define RF_PORT   PORTB
#define RF_DDR    DDRB
#define RF_PIN    PINB

#define SDI   4
#define SCK   3
#define CS    2
#define SDO   1
#define FSK   0

int uuart_putc(unsigned char c) {
	while (!(UCSRA & (1 << UDRE))) {
	}

	UDR = c;
	return 0;
}

void uuart_puts(char *s) {
	while (*s) {
		uuart_putc(*s);
		s++;
	}
}

unsigned short rf12_trans(unsigned short wert) {
	unsigned short werti = 0;
	unsigned char i;

	cbi(RF_PORT, CS);
	for (i = 0; i < 16; i++) {
		if (wert & 32768)
			sbi(RF_PORT, SDI);
		else
			cbi(RF_PORT, SDI);
		werti <<= 1;
		if (RF_PIN & (1 << SDO))
			werti |= 1;
		sbi(RF_PORT, SCK);
		wert <<= 1;
		_delay_us(0.3);
		cbi(RF_PORT, SCK);
	}sbi(RF_PORT, CS);
	return werti;
}

void rfm12_setbandwidth(unsigned char bandwidth, unsigned char gain,
		unsigned char drssi) {
	rf12_trans(
			0x9400 | ((bandwidth & 7) << 5) | ((gain & 3) << 3) | (drssi & 7));
}

void rfm12_setfreq(unsigned short freq) {
	if (freq < 96) // 430,2400MHz
		freq = 96;
	else if (freq > 3903) // 439,7575MHz
		freq = 3903;
	rf12_trans(0xA000 | freq);
}
void rfm12_setbaud(unsigned short baud) {
	if (baud < 663)
		return;
	if (baud < 5400) // Baudrate= 344827,58621/(R+1)/(1+CS*7)
		rf12_trans(0xC680 | ((43104 / baud) - 1));
	else
		rf12_trans(0xC600 | ((344828UL / baud) - 1));
}

void rfm12_setpower(unsigned char power, unsigned char mod) {
	rf12_trans(0x9800 | (power & 7) | ((mod & 15) << 4));
}

void rf12_init(void) {
	RF_DDR = (1 << SDI) | (1 << SCK) | (1 << CS) | (1 << FSK);

	sbi(RF_PORT, CS);
	sbi(RF_PORT, FSK);

	for (unsigned char i = 0; i < 10; i++)
		_delay_ms(10); // wait until POR done

	rf12_trans(0xC0E0); // AVR CLK: 10MHz
	rf12_trans(0x80E7); // Enable FIFO and 868
	rf12_trans(0xC2AB); // Data Filter: internal
	rf12_trans(0xCA81); // Set FIFO mode
	rf12_trans(0xE000); // disable wakeuptimer
	rf12_trans(0xC800); // disable low duty cycle
	rf12_trans(0xC4F7); // AFC settings: autotuning: -10kHz...+7,5kHz

	rfm12_setfreq(RF12FREQ868(868.3));
	rfm12_setbandwidth(4, 1, 4);
	rfm12_setbaud(666);
	rfm12_setpower(0, 6);

}

void rfm12_init(void) {
	rf12_init();
}
unsigned char rf12_ready(unsigned char bTimeout) {
	int timeout = 10000;

	cbi(RF_PORT, SDI);
	cbi(RF_PORT, CS);
	asm( "nop" );
	while (!(RF_PIN & (1 << SDO)) && timeout) {
		if (bTimeout) {
			timeout--;
			_delay_us(1); // wait until FIFO ready
		}
	}sbi(PORTB, CS);
	if (timeout == 0)
		return 0;
	else
		return 1;
}

unsigned rf12_data(void) {
	cbi(RF_PORT, CS);
	cbi(RF_PORT, SDI);
	asm("nop");
	if (RF_PIN & (1 << SDO))
		return 1;
	else
		return 0;
}
void rf12_rxdata(char *data, unsigned char number) {
	unsigned char i;
	rf12_trans(0x82C8); // RX on
	rf12_trans(0xCA81); // set FIFO mode
	rf12_trans(0xCA83); // enable FIFO
	for (i = 0; i < number; i++) {
		rf12_ready(0);
		*data++ = rf12_trans(0xB000);
	}
	rf12_trans(0x8208); // RX off
}

unsigned char rf12_rxbyte(char *data) {
	int t = rf12_ready(1);
	if (!t) {
		return 0;
	}
	*data = rf12_trans(0xB000);
	return 1;
}

void enable_rx() {
	rf12_trans(0x82C8); // RX on
	_delay_ms(10);
	rf12_trans(0xCA81); // set FIFO mode
	_delay_ms(10);
	rf12_trans(0xCA83); // enable FIFO
	_delay_ms(10);
}

void disable_rx() {
	rf12_trans(0x8208); // RX off
}

unsigned int rf12_rxdata_if_available(char *data, int *res) {
	*res = rf12_trans(0x0000);
	char bOk = ((*res) & 0x0200) == 0;
	if (bOk) {
		*data = rf12_trans(0xB000);
	}
	return (bOk);
}

void rfm12_int_process(void) {
	uint16_t status = rf12_trans(0x0000);

	if (status & 0x4000) {
		uuart_puts("rfm12/por - init");
		rfm12_init();
		return;
	}

	if (status & 0x2000) {
		uuart_puts("rfm12/overflow - init");
		rfm12_init();
		return;
	}

	if ((status & 0x8000) == 0) {
		//    return;
	}

	char bOk = (status & 0x0200) == 0;
	//  bOk = rf12_ready(1);
	if (bOk) {
		char buf[64];
		unsigned int x = 0;
		unsigned int size = 5;
		while (1) {
			buf[x] = 0;
			int res = rf12_rxbyte(buf + x);
			if (!res) {
			}
			switch (buf[4]) {
			case 'e':
				size = 9;
				break;
			case 'f':
				size = 19;
				break;
			case 'T':
				size = 25;
				break;
			case 'g':
				size = 19;
				break;
			default:
				size = 6;
				break;
			}
			x++;
			if (x >= size)
				break;
		}

		disable_rx();
		_delay_us(10);
		enable_rx();

		if (size > 6) {

			uuart_puts("\r\n>");
			char s[20];

			for (int c = 0; c < size; c++) {
				itoa(buf[c], s, 16);
				if (buf[c] < 16) {
					uuart_putc('0');
				}
				uuart_puts(s);
			}
			uuart_puts("\r\n#");

		}
	}
	return;

}

uint16_t rfm12_get_status(void) {
	uint16_t r;
	r = rf12_trans(0x0000); /* read status word */
	return r;
}

#define BAUD 38400

/********************** main code below *******************/

int main(void) {
	unsigned int xxxx = ((14745600+BAUD*8)/(BAUD*16)-1);
	UBRRH = xxxx >> 8;
	UBRRL = xxxx & 0xFF;

	UCSRB |= (1 << TXEN); // UART TX einschalten
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0); // Asynchron 8N1

	uuart_puts("#Uart-Init... ready\r\n#");

	uuart_puts("#RFM12-Init... ");
	rf12_init ();
	uuart_puts("ready\r\n#");

	DDRD = ~(1 << 2);
	PORTD = 0;

	sei();

	for (;;) {
		rfm12_int_process ();
	}
}
