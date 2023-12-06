#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#define brate 9600
#define DEBOUNCE_TIME 200
volatile uint32_t lastButtonPress1 = -DEBOUNCE_TIME;
volatile uint32_t lastButtonPress2 = -DEBOUNCE_TIME;
volatile uint8_t button1_pressed = 0;
#define LCD_BACKLIGHT 0x08
#define LCD_EN 0x04
#define LCD_RW 0x02
#define LCD_RS 0x01

#define TW_START (1 << TWSTA) | (1 << TWEN) | (1 << TWINT)
#define TW_STOP (1 << TWSTO) | (1 << TWEN) | (1 << TWINT)
#define TW_ACK (1 << TWEA) | (1 << TWEN) | (1 << TWINT)
#define TW_NACK (1 << TWEN) | (1 << TWINT)
#define TW_SEND (1 << TWEN) | (1 << TWINT)
#define BUTTON1_PIN PD2
#define BUTTON2_PIN PD3

#define LCD_I2C_ADDR 0x27 // Dirección I2C del adaptador PCF8574
#define I2C_SCL_PIN 5     // Pin SCL del bus I2C
#define I2C_SDA_PIN 4     // Pin SDA del bus I2C

typedef enum {
	ENERGIZADO,
	ANDANDO,
	BLOQUEADO,
	DESCONOCIDO
} Estado;

Estado estado_actual=ENERGIZADO;

volatile uint32_t timer1_millis = 0;
unsigned int indcom = 0;
unsigned int cmd = 0;
int Xvalue, Yvalue;
int centro = 1;
int prevX = 0;
int prevY = 0;
uint8_t nuevo_estado;
char comando[5];
volatile uint8_t actualizar_display = 0;
void mi_UART_Init(unsigned int);
int mi_putc(char);
int mi_getc(void);
void mi_puts(const char *);
void EnviarDatos(void);
void ADC_init(void);
int ADC_GetData(int);
int shouldSend(int, int);
uint32_t millis();
#define fgetc() mi_getc()
#define fputc(x) mi_putc(x)
void initTimer1();
void configure_buttons();
void interpretarcomando();
void lcd_init();
void i2c_init();
void display_machine_state();
void lcd_send_command(uint8_t);


void set_estado_actual(Estado);

int main(void) {
	cli();  // Deshabilita todas las interrupciones.
	lcd_init();  // Inicializa el LCD.
	i2c_init();  // Inicializa la comunicación I2C.
	mi_UART_Init(brate);  // Inicializa la UART con una tasa de baudios especificada.
	ADC_init();  // Inicializa el Conversor Analógico a Digital.
	configure_buttons();  // Configura botones.
	initTimer1();  // Inicializa el Timer1.
	sei();  // Habilita las interrupciones.
	set_estado_actual(DESCONOCIDO);  // Establece el estado actual a DESCONOCIDO.
	mi_puts("E");  // Envía una cadena de caracteres por UART.
	mi_putc('\n');  // Envía un salto de línea por UART.
	while (1) {
		EnviarDatos();  // Llama a la función para enviar datos.
		_delay_ms(100);  // Delay de 100 ms.
	}
	return 0;
}

void initTimer1() {
	// Configura el Timer1 en modo CTC (Clear Timer on Compare Match).
	TCCR1B |= (1 << WGM12);

	// Establece el valor de comparación para una interrupción cada 1ms.
	OCR1A = 250;  // (16,000,000 / (64 * 1000)) - 1

	// Habilita interrupciones por comparación en Timer1.
	TIMSK1 |= (1 << OCIE1A);

	// Establece el prescaler del Timer1 a 64.
	TCCR1B |= (1 << CS11) | (1 << CS10);
}


int shouldSend(int Xvalue, int Yvalue) {
	// Verifica si los valores X e Y están cerca de un valor central (520, 500) con una tolerancia.
	if (((Xvalue - 520 >= -20) && (Xvalue - 520 <= 20)) && ((Yvalue - 500 >= -20) && (Yvalue - 500 <= 20))) {
		if (centro == 0) {
			centro = 1;
			return 1;  // Retorna 1 si es la primera vez que está en el centro.
		}
		return 0;  // Retorna 0 si ya estaba en el centro anteriormente.
	}
	centro = 0;
	return 1;  // Retorna 1 si los valores X e Y no están en el centro.
}


void mi_puts(const char *str) {
	while (*str) {  // Mientras el carácter actual no sea nulo ('\0').
		mi_putc(*str++);  // Envía el carácter actual y avanza al siguiente.
	}
}


int mi_putc(char c) {
	while (!(UCSR0A & (1 << UDRE0)));  // Espera hasta que el buffer de transmisión esté vacío.
	UDR0 = c;  // Coloca el carácter en el registro de datos de UART.
	return 0;
}


int mi_getc() {
	while (!(UCSR0A & (1 << RXC0)));  // Espera hasta que haya datos para leer en UART.
	return UDR0;  // Devuelve el carácter recibido.
}


void mi_UART_Init(unsigned int brate0) {
	UBRR0 = F_CPU / 16 / brate0 - 1;  // Configura la tasa de baudios para UART.
	UCSR0A = 0;  // Configura el registro de control y estado A.
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);  // Habilita la recepción y transmisión UART.
	UCSR0C = (1 << USBS0) | (3 << UCSZ00);  // Configura el formato del frame UART.
	UCSR0B |= (1 << RXCIE0);  // Habilita la interrupción de recepción completa de UART.
}


void ADC_init() {
	ADMUX &= ~(1 << ADLAR);  // Ajusta la alineación del resultado del ADC a la derecha.

	ADMUX |= (1 << REFS0);  // Selecciona AVcc como referencia de voltaje para el ADC.
	ADMUX &= ~(1 << REFS1);  // Configura el bit adecuado para la referencia de voltaje.

	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // Configura el prescaler del ADC.
}

int ADC_GetData(int canal) {
	ADMUX &= (0b11110000 << MUX0);  // Reinicia los bits de selección de canal.
	ADMUX |= (canal << MUX0);  // Selecciona el canal de ADC.

	ADCSRA |= (1 << ADEN);  // Habilita el ADC.

	ADCSRA |= (1 << ADSC);  // Inicia una conversión ADC.

	while (!(ADCSRA & (1 << ADIF)));  // Espera a que se complete la conversión.
	ADCSRA |= (1 << ADIF);  // Limpia la bandera de interrupción.

	ADCSRA &= ~(1 << ADEN);  // Deshabilita el ADC.
	
	return ADC;  // Devuelve el resultado de la conversión.
}

void EnviarDatos() {
	Xvalue = ADC_GetData(0);  // Lee el valor de ADC del canal 0.
	Yvalue = ADC_GetData(1);  // Lee el valor de ADC del canal 1.

	// Calcula las diferencias absolutas con los valores previos.
	int diffX = abs(Xvalue - prevX);
	int diffY = abs(Yvalue - prevY);

	// Verifica si hay un cambio significativo en los valores.
	if (diffX > 10 || diffY > 10) {
		// Actualiza los valores previos.
		prevX = Xvalue;
		prevY = Yvalue;

		// Convierte los valores a cadenas de texto.
		char XvalueStr[6];
		char YvalueStr[6];
		sprintf(XvalueStr, "%d", Xvalue);
		sprintf(YvalueStr, "%d", Yvalue);

		// Envia los valores por UART.
		mi_puts("X:");
		mi_puts(XvalueStr);
		mi_puts(" Y:");
		mi_puts(YvalueStr);
		mi_putc(' ');
		mi_putc('\n');
	}
}


void configure_buttons() {
	// Configura los pines como entradas.
	DDRD &= ~(1 << BUTTON1_PIN);
	DDRD &= ~(1 << BUTTON2_PIN);

	// Habilita las resistencias pull-up internas.
	PORTD |= (1 << BUTTON1_PIN);
	PORTD |= (1 << BUTTON2_PIN);

	// Configura las interrupciones externas para los botones.
	EICRA |= (1 << ISC00) | (1 << ISC01); // Cambios en INT0.
	EICRA |= (1 << ISC10) | (1 << ISC11); // Cambios en INT1.

	EIMSK |= (1 << INT0);   // Habilita INT0.
	EIMSK |= (1 << INT1);   // Habilita INT1.
}

ISR(INT0_vect) {
	uint32_t currentMillis = millis();  // Obtiene el tiempo actual en milisegundos.
	if (currentMillis - lastButtonPress1 > DEBOUNCE_TIME) {  // Verifica el debounce.
		// Realiza acciones basadas en el estado actual.
		if (estado_actual == ENERGIZADO || estado_actual==DESCONOCIDO) {
			mi_puts("B");
			mi_putc('\n');
			} else if (estado_actual == BLOQUEADO || estado_actual == ANDANDO) {
			mi_puts("E");
			mi_putc('\n');
		}
		lastButtonPress1 = currentMillis;  // Actualiza el tiempo del último botón presionado.
	}
}

// Rutina de servicio para INT1
ISR(INT1_vect) {
	uint32_t currentMillis = millis();  // Obtiene el tiempo actual.
	// Verifica el debounce.
	if (currentMillis - lastButtonPress2 > DEBOUNCE_TIME) {
		if (estado_actual == BLOQUEADO) {
			mi_puts("A");
			mi_putc('\n');
			} else if (estado_actual == ANDANDO) {
			mi_puts("B");
			mi_putc('\n');
		}
		lastButtonPress2 = currentMillis;  // Actualiza la última pulsación.
	}
}

// Establece el estado actual y actualiza la pantalla si es necesario
void set_estado_actual(Estado nuevo_estado) {
	if (estado_actual != nuevo_estado) {
		estado_actual = nuevo_estado;
		display_machine_state();
	}
}

// Rutina de servicio para el Timer1
ISR(TIMER1_COMPA_vect) {
	timer1_millis++;  // Incrementa el contador de milisegundos.
}

// Devuelve el valor actual del contador de milisegundos
uint32_t millis() {
	return timer1_millis;
}

// Interpreta el comando recibido
void interpretarcomando() {
	if (strcmp(comando, "E") == 0) {
		set_estado_actual(ENERGIZADO);
		} else if (strcmp(comando, "A") == 0) {
		set_estado_actual(ANDANDO);
		} else if (strcmp(comando, "B") == 0) {
		set_estado_actual(BLOQUEADO);
		} else if (strcmp(comando, "K") == 0) {
		mi_puts("K");
		mi_putc('\n');
	}
}

// Rutina de servicio para recepción UART
ISR(USART_RX_vect) {
	char dato;
	dato = mi_getc();  // Lee el carácter recibido.
	switch (dato) {
		case ':':
		indcom = 0;
		break;
		case '\r':
		interpretarcomando();
		indcom = 0;
		break;
		default:
		if (indcom < 5)
		comando[indcom++] = dato;
		break;
	}
	UCSR0A |= (1 << RXC0);  // Limpia la bandera de recepción.
}

// Inicializa el bus I2C
void i2c_init(void) {
	TWSR = 0x00; // Preescalador en 1
	TWBR = ((F_CPU / 100000) - 16) / 2; // Configura el reloj I2C a 100kHz
}

// Genera una condición de inicio en I2C
void i2c_start(void) {
	TWCR = TW_START;
	while (!(TWCR & (1 << TWINT)));  // Espera a que se complete.
}

// Genera una condición de parada en I2C
void i2c_stop(void) {
	TWCR = TW_STOP;
	while (TWCR & (1 << TWSTO));  // Espera a que se limpie TWSTO.
}

// Escribe un dato en I2C
void i2c_write(uint8_t data) {
	TWDR = data;  // Carga el dato.
	TWCR = TW_SEND;
	while (!(TWCR & (1 << TWINT)));  // Espera a que se complete.
}

// Lee un dato con ACK en I2C
uint8_t i2c_read_ack(void) {
	TWCR = TW_ACK;
	while (!(TWCR & (1 << TWINT)));  // Espera a que se complete.
	return TWDR;  // Retorna el dato leído.
}

// Lee un dato con NACK en I2C
uint8_t i2c_read_nack(void) {
	TWCR = TW_NACK;
	while (!(TWCR & (1 << TWINT)));  // Espera a que se complete.
	return TWDR;  // Retorna el dato leído.
}

void lcd_send_nibble(uint8_t data) {
	i2c_start();  // Inicia la comunicación I2C.
	i2c_write(LCD_I2C_ADDR << 1);  // Envía la dirección del LCD con bit de escritura.
	i2c_write(data | 0x04);  // Envía el nibble con el bit E (Enable) en alto.
	i2c_write(data & 0xFB);  // Envía el nibble con el bit E en bajo.
	i2c_stop();  // Finaliza la comunicación I2C.
}

void i2c_send_byte(uint8_t data) {
	i2c_start();  // Inicia la comunicación I2C.
	i2c_write(LCD_I2C_ADDR << 1);  // Envía la dirección del LCD.
	i2c_write(data);  // Envía los datos.
	i2c_write(data | LCD_EN);  // Envía el dato con el bit Enable en alto.
	i2c_write(data & ~LCD_EN);  // Envía el dato con el bit Enable en bajo.
	i2c_stop();  // Finaliza la comunicación I2C.
}

void lcd_send_command(uint8_t command) {
	// Prepara los nibbles alto y bajo del comando con la luz de fondo encendida.
	uint8_t high_nibble = (command & 0xF0) | LCD_BACKLIGHT;
	uint8_t low_nibble = ((command << 4) & 0xF0) | LCD_BACKLIGHT;

	// Envía el nibble alto del comando.
	i2c_send_byte(high_nibble);
	_delay_us(1);  // Pequeña pausa.
	i2c_send_byte(high_nibble);
	_delay_us(50);  // Espera a que el LCD procese el comando.

	// Envía el nibble bajo del comando.
	i2c_send_byte(low_nibble);
	_delay_us(1);
	i2c_send_byte(low_nibble);
	_delay_us(50);  // Espera a que el LCD procese el comando.
}


void lcd_send_data(uint8_t data) {
	// Prepara los nibbles alto y bajo de los datos con la luz de fondo y el bit RS (Register Select) encendidos.
	uint8_t high_nibble = (data & 0xF0) | LCD_BACKLIGHT | LCD_RS;
	uint8_t low_nibble = ((data << 4) & 0xF0) | LCD_BACKLIGHT | LCD_RS;

	// Envía los nibbles alto y bajo de los datos.
	i2c_send_byte(high_nibble);
	i2c_send_byte(low_nibble);
}


void lcd_init(void) {
	_delay_ms(50);  // Espera inicial para estabilización del LCD.
	// Secuencia de reinicio del LCD.
	lcd_send_command(0x03);
	_delay_ms(5);
	lcd_send_command(0x03);
	_delay_us(150);
	lcd_send_command(0x03);
	lcd_send_command(0x02);  // Configura el LCD en modo de 4 bits.

	// Configuraciones adicionales del LCD.
	lcd_send_command(0x28);  // Modo de 2 líneas, caracteres de 5x8.
	lcd_send_command(0x0C);  // Display ON, Cursor OFF.
	lcd_send_command(0x01);  // Limpia el display.
	_delay_ms(2);
	lcd_send_command(0x06);  // Modo de incremento del cursor.
}


// Función para mostrar el estado en el LCD
void display_machine_state() {
	// Limpia la pantalla
	lcd_send_command(0x01);

	// Establece la posición del cursor en la primera línea
	lcd_send_command(0x80);

	// Muestra el estado en función de estado_actual
	switch (estado_actual) {
		case ENERGIZADO:
		lcd_send_data(' ');
		lcd_send_data(' ');
		lcd_send_data('E');
		lcd_send_data('N');
		lcd_send_data('E');
		lcd_send_data('R');
		lcd_send_data('G');
		lcd_send_data('I');
		lcd_send_data('Z');
		lcd_send_data('A');
		lcd_send_data('D');
		lcd_send_data('O');

		break;
		case BLOQUEADO:
		lcd_send_data(' ');
		lcd_send_data(' ');
		lcd_send_data('B');
		lcd_send_data('L');
		lcd_send_data('O');
		lcd_send_data('Q');
		lcd_send_data('U');
		lcd_send_data('E');
		lcd_send_data('A');
		lcd_send_data('D');
		lcd_send_data('O');
		break;
		case ANDANDO:
		lcd_send_data(' ');
		lcd_send_data(' ');
		lcd_send_data('A');
		lcd_send_data('N');
		lcd_send_data('D');
		lcd_send_data('A');
		lcd_send_data('N');
		lcd_send_data('D');
		lcd_send_data('O');
		break;
		case DESCONOCIDO:
		lcd_send_data(' ');
		lcd_send_data(' ');
		lcd_send_data('D');
		lcd_send_data('E');
		lcd_send_data('S');
		lcd_send_data('C');
		lcd_send_data('O');
		lcd_send_data('N');
		lcd_send_data('O');
		lcd_send_data('C');
		lcd_send_data('I');
		lcd_send_data('D');
		lcd_send_data('O');
		break;
	}
}