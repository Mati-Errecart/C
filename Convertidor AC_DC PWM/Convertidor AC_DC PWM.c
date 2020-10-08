/*
 * Convertidor_AC_DC_PWM.c
 *
 * Created: 6/10/2020 10:43:34 a. m.
 */ 

//######################################################### SISTEMAS DIGITALES IV ##################################################################################################################
// ALUMNOS: Correa Bruno, De Battista Cristian; Errecart Matias
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000ul
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int modoant;
int Contoff = 0;
int i,k;
int indice = 0;
int cData;
int DSPI;
int Tension_Deseada;
int modo;
long ADC_Pote;
long ADC_Volt;
int TENSIONmax = 1;
int CANAL = 0;
int CONFIG = 0x40;
int estado; 
int numero = 0;

char valor[5]="\0\0\0\0\n";;									// String para armar cadena de caracteres
char datoRX;

volatile unsigned char bPB8=1;					// Bandera estado de PB0
volatile unsigned char bTX=0;					// Bandera para transmitir dato

void CONTROL(void);						// Funcion control PWM por puerto serie
void onoff(void);								// Funicion para mostrar estado PWM
void SPI_MasterTransmit(int cData);				// Funcion para transimitir al display
void SPI_initial(void);							// Funcion inicializar display
uint8_t OnOffSwitch(void);						// Funcion de lectura de PD7 int On_Off_PWM
uint8_t SwitchModo(void);
void dato_recibido(char);
void Mostrar_Tensiones(void);

int main(void)
{	
	DDRD |=(1<<DDD5)|(0<<DDD7)|(0<<DDD6);			// PD5 como salida PWM (OC0B) y PD7(7) como entrada (On/Off_PWM en modo local) y PD6(6) como entrada "MODO_CONTROL"
	DDRB |=(1<<DDB5)|(1<<DDB3)|(1<<DDB2)|(1<<DDB1)|(1<<DDB0);	// Config como salida PB5(13,CSK), PB3(11,MOSI), PB2(10,SS), PB0(8)semiciclo positivo, PB1(9) semiciclo negativo
	PORTD |= (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD7)|(1<<PORTD6);			// Configuro Restistencias pull up INT0, INT1, On_Off_PWM
	PORTB |= (1<<PORTB2);									// Resistencia Pull-UP

	TCCR0A= (0<<COM0B1)|(0<<COM0B0)|(0<<WGM01)|(1<<WGM00);	// PWM fase correcta,
	TCCR0B= (0<<CS02)|(1<<CS01)|(0<<CS00)|(1<<WGM02);		// Selecciono prescaler en 8

	OCR0A= 199;												// Define la frecuencia pwm 5KHz
	OCR0B = 100;											// Define el ancho de pulso

	EIMSK= (1<<INT1)|(1<<INT0);								// Config INT0 INT1
	EICRA= (1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(1<<ISC00);		// Config INT1 flanco descendente, INT0 flanco ascendente

	TCCR1A=0;
	TCCR1B=4;												// Selector de reloj CKl/256
	TCNT1=30360;											// 1s
	TIMSK1=(1<<TOIE1);										// Habilitación de interrupción por desbordamiento

	UCSR0A=0;
	UBRR0=103;												// Velociadad de transmicion en 9600
	UCSR0C=(0<<UMSEL01)|(0<<UMSEL00)|(1<<UCSZ01)|(1<<UCSZ00)|(0<<USBS0)|(0<<UPM01)|(0<<UPM00);	// Modo Asincrono, size caracter 8-bit, 1-bit de stop, paridad OFF
	UCSR0B=(1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);			// Habilito receptor y transceptor, interrupcion recepcion completa

	ADMUX= (1<<REFS0);										// Tension de referencia con capacitor externo
	ADCSRA=(1<<ADEN)|(1<<ADSC)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADATE)|(1<<ADIE);		// Habilito el ADC, Configuro prescaler en 8
	ADCSRB=(1<<ADTS2)|(1<<ADTS1);							//Autodisparo por desbordamiento del timer 1
	
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);					// Configuracion del SPI Modo Maestro, frecuencia SCK 1Mhz
	SPI_initial();
	
	sei();
	
	while(1)
	{	
		if(SwitchModo() == 0)
		{
			modo = 0;			//Modo Local
		}
		if(SwitchModo() == 1)
			{
				modo = 1;		//Modo remoto
			}											
	
		if ((modo==0)&&(OnOffSwitch()==1))					//Pregunto por "MODO LOCAL" y si el PIN7 fue pulsado
		{
			TCCR0A ^= (1<<COM0B1);						   //Cambio de estado la salida PWM OC0B (PD5)
			onoff();
		}
	
		if ((modo==1)&&(estado==1))							//Pregunto por "MODO REMOTO" y si estado es "O"
		{
			TCCR0A ^= (1<<COM0B1);							//Cambio de estado la salida PWM OC0B (PD5)
			estado=0;										//Cambio el estado para que no vuelva a ingresar
			onoff();										//Llama a funcion para mostrar estado por puerto serial y OFF en el MAX
		}

	CONTROL();
		
		if (numero==5)
		{	
			Mostrar_Tensiones();
			numero=0;				
		}
			
	_delay_ms(25);
	}
	}
//######################################################### FUNCION PARA ESTABLECER CONTROL LOCAL/REMOTO #######################################################################################

void CONTROL(void)
{
	char mremoto[]="Modo Remoto\n";
	char mlocal[]="Modo Local\n";
	int decena = 0, unidad = 0, decimal = 0, resultado = 0;
	
	//<<<<<<<<<<<<<<<<<SI EL MODO ES REMOTO>>>>>>>>>>>>>>>>>
	if (modo==1)		
	{
		if (modoant!=1)		
		{
			modoant=1;
			for (i=0;i<=strlen(mremoto);i++)
			{
				while ( !(UCSR0A &(1<<UDRE0)));
				UDR0=mremoto[i];				//Mostramos "MODO REMOTO"
			}
		}

		decena = (valor[0]-48) * 100;
		unidad = (valor[1]-48) * 10;
		decimal = (valor[2]-48);
		resultado = decena + unidad + decimal;

		
		if(resultado<=150)					//+++++ ACA HAY QUE VER QUE PUTA PREGUNTAMOS!!!+++ =( =P
			{
				Tension_Deseada = resultado;
			}
	}
	
	//<<<<<<<<<<<<<<SI EL MODO ES LOCAL>>>>>>>>>>>>>>>>>>>>>>
	if (modo==0)
	{
		if (modoant!=2)
		{
			modoant=2;
			for (i=0;i<=strlen(mlocal);i++)
			{
				while ( !(UCSR0A &(1<<UDRE0)));
				UDR0=mlocal[i];					//Mostramos "MODO LOCAL"
			}
		}									
	}
}

//######################################################### FUNCION PARA ENCENDER/APAGAR PWM EN MODO LOCAL ##################################################################################################################

uint8_t OnOffSwitch(void)
{
	if((PIND & (1<<PIND7)) == 0)					// Si el botón esta presionado.
	{
		_delay_ms(50);								// Retardo de entrada para el valor leído.
	}
	if((PIND & (1<<PIND7)) == 0)					// Verifica que la la lectura sea correcta.
	{
		return 1;									// Si todavía es 0 es porque si teníamos pulsado el botón
	}
	else
	{
		return 0;									// Si el valor cambio la lectura es incorrecta.
	}
}

uint8_t SwitchModo(void)
{
	if((PIND & (1<<PIND6)) == 0)					// Si el botón esta presionado.
	{
		return 0;								// Si devuelve 1 es modo local
	}
	else
	{
		return 1;									// Si devuellve 0 es modo remoto
	}
}

//######################################################### VECTOR DE INTERRUPCION RECEPCION DE DATOS ##################################################################################################################

ISR(USART_RX_vect)
{
	datoRX=UDR0;	// Leemos el buffer de recepcion
	
	if(datoRX=='O')
	{
		estado=1;	// si se cumple pone variable indice a 0
	}
	
	if(datoRX=='$')
	{
		indice=0;	// si se cumple pone variable indice a 0
	}
	
	if(((datoRX>='0') && (datoRX<='9')))	// Si es un numero lo guarda en DATO
	{
		valor[indice]=datoRX;
		indice++;
	}
		
	return;
}

//######################################################### RUTINA DE TRATAMIENTO DE INTERRUPCION EXTERNA ##################################################################################################################

ISR (INT1_vect) // PD3(pin3)
{
	PORTB &= ~ (1<<PORTB0); //~ En 0 PINB0
	_delay_us(100);
	PORTB |= (1<<PORTB1); // En 1 PB1(9) Semiciclo negativo
}

ISR (INT0_vect) // PD2(pin2)
{
	PORTB &= ~ (1<<PORTB1); // En 0 PB1
	_delay_us(100);
	PORTB |= (1<<PORTB0); // En 1 PB0(8) Semiciclo positivo
}

//######################################################### RUTINA DE TRATAMIENTO DE INTERRUPCION DEL CAD ##################################################################################################################

ISR(ADC_vect)
{	
	if(CANAL==0)
	{
		ADC_Pote = ADC;
		ADMUX = CONFIG+1;		//Config para leer ADC1 
		CANAL = 1;
	}
	else 
	{	
		ADC_Volt = ADC;
		ADMUX = CONFIG;		//Config para leer ADC0
		CANAL = 0;
	}
}
//######################################################### RUTINA DE TRATAMIENTO DE INTERRUPCION DEL TIMER1_OVF ##################################################################################################################

ISR(TIMER1_OVF_vect)
{
	numero++;
	
	if (modo==0)
	{
		Tension_Deseada = ADC_Pote*100/1023;	 // Guardamos el valor del ADC0 	
		OCR0B = Tension_Deseada*199/100;			// PD5 salida PWM OC0B
	}
	if (modo==1)
	{
		OCR0B = Tension_Deseada*199/150;			// PD5 salida PWM OC0B
	}
	
	
}
//######################################################### FUNCION DE MOSTRAR ON/OFF ##################################################################################################################

void onoff ()
{
	char on[]="PWM on\n";
	char off[]="PWM off\n";
	
	Contoff++;
	
	if ((Contoff>1))
	{
		Contoff=0;
	}
	
	if ((Contoff == 1))
	{
		for (i=0;i<=strlen(on);i++)
		{
			while ( !(UCSR0A &(1<<UDRE0)));
			UDR0=on[i];
			SPI_initial();
		}
	}
	
	if ((Contoff == 0))
	{
		for (i=0;i<=strlen(off);i++)
		{
			while ( !(UCSR0A &(1<<UDRE0)));
			UDR0=off[i];
		}
		
	}
	return;
}
//######################################################### SPI_MasterTransmit ##################################################################################################################

void SPI_MasterTransmit(int cData)
{
	SPDR = cData;
	while(!(SPSR & (1<<SPIF)))
	;
}

//################################################################ SPI ##################################################################################################################

void SPI_initial()
{
	////////////////////////////////////////// Intensidad de brillo /////////////////////////////////////////
	
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x0A;						//Intensidad minima
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = 0x00;						//Decodificación de código B para dígitos 7-0
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	//////////////////////////////////////// Modo de decodificacion /////////////////////////////////////////
	
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x09;						//Decodificación de código B para dígitos 7-0
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = 0xFF;						//Decodificación de código B para dígitos 7-0
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	////////////////////////////////////////// Cantidad de digitos /////////////////////////////////////////
	
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x0B;						//Mostrar 3 digitos
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = 0x02;						//Decodificación de código B para dígitos 7-0
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	////////////////////////////////////////// Modo de operacion /////////////////////////////////////////
	
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x0C;						//Opercion normal
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = 0x01;						//Decodificación de código B para dígitos 7-0
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	////////////////////////////////////////// Modo de testeo /////////////////////////////////////////
	
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x0F;						//Opercion normal
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = 0x00;						//Decodificación de código B para dígitos 7-0
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	////////////////////////////////////////// Inician todos apagados /////////////////////////////////////////
	
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x01;						//Bit 0
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = 0xFF;
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x02;						//Bit 1
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = 0xFF;
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x03;						//Bit 2
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = 0xFF;
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
}

void Mostrar_Tensiones()
{
	int cen = 0, dec = 0, uni = 0;
	int cenD = 0, decD = 0, uniD = 0;
	unsigned char Resto;
	unsigned char RestoD;
	
	int Tension_Real = ADC_Volt*(10*10)/1023;
	char Medicion_msj[]="Tension Medida:";
	char UDeseada_msj[]="Tension Deseada:";
	
	for (i=0;i<=strlen(Medicion_msj);i++)
	{
		while ( !(UCSR0A &(1<<UDRE0)));
		UDR0=Medicion_msj[i];
	}
	
	cen = (Tension_Real/100);
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x03;						//Bit 2
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = (cen);						//Envio decena al MAX
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	while(!(UCSR0A & (1<<UDRE0)));		// Espera a que se envíe el dato
	UDR0 = cen + 48;					//Envio por puerto serie la decena
	while(!(UCSR0A & (1<<UDRE0)));		// Espera a que se envíe el dato
	
	Resto = (Tension_Real%100);
	dec = (Resto/10);
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x02;						//Bit 1
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = (0xF0 + dec);						//Envio la unidad al MAX
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	UDR0 = dec + 48;					//Envio por puerto serie la unidad
	while(!(UCSR0A & (1<<UDRE0)));
	
	UDR0 = 0x2C;						//Envio ","
	while(!(UCSR0A & (1<<UDRE0)));
	
	uni = (Resto%10);
	PORTB = (0<<PORTB2);				//Indico inicio de transferencia
	_delay_us(1);
	DSPI = 0x01;						//Bit 0
	SPI_MasterTransmit(DSPI);
	_delay_us(1);
	DSPI = (uni);						//Envio primer decimal al MAX
	SPI_MasterTransmit(DSPI);
	PORTB = (1<<PORTB2);				//Indico fin de transferencia
	_delay_us(1);
	
	UDR0 = (uni + 48);					//Envio por puerto serie primer decimal
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = 10;
	while(!(UCSR0A & (1<<UDRE0)));
	
	for (i=0;i<=strlen(UDeseada_msj);i++)
	{
		while ( !(UCSR0A &(1<<UDRE0)));
		UDR0=UDeseada_msj[i];
	}
	
	cenD = (Tension_Deseada/100);
	while(!(UCSR0A & (1<<UDRE0)));		// Espera a que se envíe el dato
	UDR0 = cenD + 48;					//Envio por puerto serie la decena
	
	RestoD = (Tension_Deseada%100);
	decD = (RestoD/10);
	while(!(UCSR0A & (1<<UDRE0)));		// Espera a que se envíe el dato
	UDR0 = decD + 48;					//Envio por puerto serie la unidad
	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = 0x2C;						//Envio ","
	
	uniD = (RestoD%10);
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = (uniD + 48);					//Envio por puerto serie primer decimal
	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = 10;
	
}