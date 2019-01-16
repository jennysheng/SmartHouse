/*
* SmartHouse.c
*
* Created: 2018-10-18 13:47:23
* Author : Jenny
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define F_CPU 16000000
#define BaudRate 9600
#define MYUBRR ( F_CPU/16/BaudRate)-1




void serialInit(void);
void uart_putc(unsigned char c);
void uart_puts (char *s);


char o[6];
char m[6];
char n[6];
char t[6];
char p[6];



uint16_t X, Y, Z, T, P; //unsigned 16-bit integer

int main(void)
{
	//Input signal panel: 1. window alarm. 2.fire alarm. 3. stove/spis. 4. water leakage.
	//                    1.inbrottsalarm, 2.utomhusTemp, 3.inomhusTemp.
	//                    1.elförbrukning, 2.skymning, 3. elavbrott.
	// Output signal: Timer1, Time2, 1.lighting indoor,2.inbrottsalarm ljudet, 3.inbrottsalarm lamp. 4. Fan 5. element/radiator.
	
	
	char c;
	serialInit();
	//1. window alarm. 2.fire alarm. 3. stove/spis. 4. water leakage.
	//fire alarm PD2----------------------------------
	DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
	PIND |=1 << PIND2;
	//---------------------------------------------------------------interruptSetup();
	EIMSK = 1<<INT0 | 1<<INT1 ;					// Enable External interrupt INT0 and INT1
	EICRA = 1<<ISC01 | 1<<ISC00;	            // Trigger INT0 on rising edge
	//---------------------------------------------------------------------
	PCICR |= (1 << PCIE0);     // set PCIE0 to enable PCMSK0 scan // PCICR |= (1 << PCIE0)|(1<<PCIE1)|(1<<PCIE2);
	PCMSK0 |= (1 << PCINT20)|(1<<PCINT4)|(1<<PCINT5)|(1<<PCINT6);   // set PCINT0 to trigger an interrupt on state change // PD4 waterlackage
	
	//vattenlackage PD4----------------------------
	DDRD |= 0<<PIND4;// input
	PIND |= 1<<PIND4;// pull up-------------------
	//spis   PD5----------------------------
	DDRD |= 0<<PIND5;
	PIND |= 1<<PIND5;// pull up-------------------
	//fonster PD6----------------------------
	DDRD |= 0<<PIND6;
	PIND |= 1<<PIND6;// pull up-------------------
	  
	//inbrottslarm PD3---------------------------
	DDRD |= 0<<PIND3;
	PIND |= 0<<PIND3;// pull down
	
	
	sei();
	initADC();
	TempRum();
	TempUtomhus();
	Tempvind();
	//LDR();
	
	
	
	while (1)
	{		;;
	}
}

void uart_putc(unsigned char c)
{

	while (!(UCSR0A & (1<<UDRE0)))  // wait until sending is possible
	{
	}

	UDR0 = c;                       // sending signs

}
void uart_puts (char *s)
{
	while (*s)
	{                               // sending char until "/0"
		uart_putc(*s);
		s++;
	}
}
int initADC(){
	ADMUX = 0b01000000;//0 1 AVCC with external capacitor at AREF pin
	ADCSRA = 0;
	ADCSRA |= (1<<ADEN);//Writing this bit to one enables the ADC.
	ADCSRA |= (1<<ADPS0);//Timer/Counter1 Capture Event
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);
	while (ADCSRA & (1<<ADSC));          //wait for end of conversion
	return ADCW;
}

void TempRum(){
	ADMUX = (ADMUX & 0xf0 | 0x01);//satt den vanster 4 bit som utgång och adc for x
	ADCSRA |= 0x40;
	while (ADCSRA & (1<<ADSC)){}
	Y=ADC;
	itoa(Y, m, 10);// integer to string
	uart_putc("Tempout");
	uart_puts(m);
	_delay_ms(1000);
	uart_puts("\t\t");
	
}

void Tempvind(){
	ADMUX = (ADMUX & 0xf0 | 0x02);//satt den vanster 4 bit som utgång och adc for x
	ADCSRA |= 0x40;
	while (ADCSRA & (1<<ADSC)){}
	Z=ADC;
	itoa(Z, n, 10);//integer to string
	uart_putc("vind");
	uart_puts(n);
	_delay_ms(1000);
	uart_puts("\t\t");
}
void elforbukning(){
	ADMUX = (ADMUX & 0xf0 | 0x00);//satt den vanster 4 bit som utgång och adc for x
	ADCSRA |= 0x40;
	while (ADCSRA & (1<<ADSC)){}
	P=ADC;
	itoa(P, p, 10);//integer to string
	uart_putc("el");
	uart_puts(p);
	_delay_ms(1000);
	uart_puts("\t\t");
}

void TempUtomhus(){
	DDRB=0<<PINB1;// input signal digital PB1
	T=PINB&1<<PINB2;
	itoa(T, t, 10);
	uart_puts(t);
}


ISR LDR(){
	ADMUX = (ADMUX & 0xf0 | 0x03);//satt den vanster 4 bit som utgång och adc for x
	ADCSRA |= 0x40;
	while (ADCSRA & (1<<ADSC)){}
	X=ADC;
	if(X>200){
		DDRB=0xf0;
		PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything
		PINB |=(0<<PINB4)|(1<<PINB5)|(1<<PINB3)|(1<<PINB0);	// turn on the outdoor light
		
	}else{
		PINB |=(1<<PINB4)|(1<<PINB5)|(1<<PINB3)|(1<<PINB0);	// turn off the outdoor light
		
	}
	itoa(X, o, 10);// interger to string
	uart_putc("LDR");
	uart_puts(o);
	_delay_ms(1000);
	uart_puts("\t\t");	
}

void serialInit() {
	/* Set the baud rate */
	UBRR0H = (unsigned char) ((MYUBRR)>>8);
	UBRR0L = (unsigned char) MYUBRR;
	/* Frame format: 8data, No parity, 1stop bit */
	UCSR0C = (3 << UCSZ00);
	/* Enable receiver and transmitter to engage in communication! */
	UCSR0B = (1 << RXEN0) | (1 << RXCIE0) | (1 << TXCIE0)| (1<<TXEN0);
	
}

ISR (PCINT20){
	DDRB=0xf0;
	PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything
	PINB |=(1<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	//sound on -----------------------------------
	PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0); //sound off-----------------------------------
	
	uart_puts("water lackage");
	uart_puts ("\n\r");
	
}
ISR (PCINT21){
	DDRB=0xf0;
	PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything
	PINB |=(1<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	//sound on -----------------------------------
	PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0); //sound off-----------------------------------
	
	uart_puts("spis");
	uart_puts ("\n\r");
	
}
ISR (PCINT6){
	DDRB=0xf0;
	PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything
	PINB |=(1<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	//sound on -----------------------------------
	PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0); //sound off-----------------------------------
	
	uart_puts("windowOn");
	uart_puts ("\n\r");
	
}
//Receive interrupt
ISR (INT0_vect){		
	DDRB=0xf0;
	PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything
	PINB |=(1<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	//sound on -----------------------------------
	PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0); //sound off-----------------------------------
	
	uart_puts("Fire");
	uart_puts ("\n\r");
}


ISR (INT1_vect){
	
	DDRB=0xf0;
    PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything	
	PINB |=(0<<PINB4)|(0<<PINB5)|(1<<PINB3)|(1<<PINB0);	//inbrottalarm  on -----------------------------------
	PINB |=(1<<PINB4)|(0<<PINB5)|(1<<PINB3)|(1<<PINB0); //inbrottsalarm off-----------------------------------
	
	uart_puts("inbrottsalarm");
	uart_puts ("\n\r");	
}


//Receive interrupt
ISR(USART_RX_vect){
	char sendData = UDR0;
	if(sendData == 'F'){
		DDRB= 1<<PINB2;//output fan working
		PINB|=1<<PINB2;// fan
		
		}else if(sendData == 'L'){
		DDRB=0xf0;
		PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything
		PINB |=(0<<PINB4)|(1<<PINB5)|(1<<PINB3)|(1<<PINB0);	// turn on the outdoor light
		PINB |=(1<<PINB4)|(1<<PINB5)|(1<<PINB3)|(1<<PINB0);	// turn off the outdoor light
		
		
		}else if(sendData == 'O'){
		DDRB=0xf0;
		PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything
		PINB |=(0<<PINB4)|(0<<PINB5)|(1<<PINB3)|(0<<PINB0);	// turn on the indoor light
		PINB |=(1<<PINB4)|(0<<PINB5)|(1<<PINB3)|(0<<PINB0);	// turn off the indoor light
		
		
		}else if(sendData == 'E'){
		DDRB=0xf0;
		PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything
		PINB |=(0<<PINB4)|(1<<PINB5)|(1<<PINB3)|(0<<PINB0);	// turn on the element vind
		PINB |=(1<<PINB4)|(1<<PINB5)|(1<<PINB3)|(0<<PINB0);	// turn off the element vind
		
		
		}	else if(sendData == 'G'){
		DDRB=0xf0;
		PINB |=(0<<PINB4)|(0<<PINB5)|(0<<PINB3)|(0<<PINB0);	// empty everything
		PINB |=(0<<PINB4)|(1<<PINB5)|(0<<PINB3)|(1<<PINB0);	// turn on the element 
		PINB |=(1<<PINB4)|(1<<PINB5)|(0<<PINB3)|(1<<PINB0);	// turn off the element 


		}else{// temperature inmatning
			if()
			
		}
	UDR0 = sendData;
}
























void toggleAbutton(){
	if(PIND & (1 << PIND5)) {  // button pressed working code
		uart_putc('J');
		}else{
		uart_putc('I');
	}
	
};


