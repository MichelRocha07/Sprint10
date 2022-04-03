// Aluno: Michel da Rocha Chagas
// Matrícula: 118111422

#define F_CPU 16000000UL //Frequência de trabalho da CPU
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include "SSD1306.h"
#include "Font5x8.h"
#include "Shift_reg.h"

#define tst_bit(y,bit) (y&(1<<bit))//Retorna 0 ou 1 conforme leitura do bit
// Variáveis globais
int RPM = 0;
uint16_t distancia_percorrida = 0;
uint64_t distancia1_percorrida = 0;
uint16_t distancia_objeto = 0;
int periodo = 0;
uint32_t  Velocidade_veiculo = 0;
uint32_t tempoX = 0;
int diametro = 0;
uint32_t tempo_us = 0;
uint16_t contVoltas = 0;
float temperatura = 0;
uint8_t temperatura_max = 0;
float temperatura1 = 0;
float RT = 0;
uint64_t bateria = 0;
uint64_t bateria1 = 0;
uint32_t aceleracao = 0;
uint32_t aceleracao1 = 0;
uint8_t flag5ms = 0;
uint8_t flag500ms = 0;
uint8_t flag_estouro = 0;
uint32_t tempo_subida = 0;
uint32_t tempo_delta = 0;
uint8_t Cont = 0;
uint16_t Lux = 0;
uint16_t Lux_LDR= 0;
uint16_t tempo_buzzer = 0;

ISR(USART_RX_vect)
{
	char recebido; // Variável local
	recebido = UDR0;
	if(recebido =='l') // Limpar o valor da temperatura máxima para pegar a próxima leitura
	temperatura_max = 0;
	if(recebido =='d') // Mostrar o valor da temperatura máxima
	USART_Transmit(eeprom_read_byte(4));
}
// Função para inicialização da USART
void USART_Init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr>>8); //Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 2 de parada
}
// Função para envio de um frame de 5 a 8bits
void USART_Transmit(unsigned char data)
{
	while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
	UDR0 = data; //Coloca o dado no registrador e o envia
}

// Função para recepção de um frame de 5 a 8bits
unsigned char USART_Receive(void)
{
	while(!(UCSR0A & (1<<RXC0))); //Espera o dado ser recebido
	return UDR0; //Lê o dado recebido e retorna
}

ISR(TIMER0_COMPA_vect)
{
	tempoX++;  // Acressento 1 ms
	if((tempoX%5) == 0) // Verdade a cada 5 ms
		flag5ms = 1;
	if((tempoX%500) == 0) // Verdade a cada 500 ms
		flag500ms = 1;
	if(tempoX%tempo_buzzer == 0) // Inverter o estado do pino PC6 para gerar o som do buzzer
		PORTC ^= (1<<6); 
}

ISR(TIMER1_OVF_vect) // Interrupção para tratar o estouro
{
	flag_estouro = 1;
}
// Interrupção por captura do valor do IC1
ISR(TIMER1_CAPT_vect)
{
	if(TCCR1B & (1<<ICES1)) // Ler o valor de contagem do TC1 na borda de subida do sinal
		tempo_subida = ICR1; // Salva a primeira contagem para determinar a largura do pulso
	else // Ler o valor de contagem do TC1 na borda de descida do sinal
		tempo_delta = (ICR1 - tempo_subida)*16; // incremento a cada 16us
	TCCR1B ^=(1<<ICES1); // Inverter a borda de captura
	// Condição para tratar o estouro
	if(flag_estouro == 0) // Quando não temos o estouro
		distancia_objeto = tempo_delta/58; // Determinando a distância do objeto
	else // Quando temos o estouro
		flag_estouro = 0;
	
	if(distancia_objeto < 200 && (PIND & (1<<7)) && (PIND & (1<<6))) // Calculando o tempo do buzzer ligado/desligado
		tempo_buzzer = ((float)distancia_objeto/200)*200;
	else{
		PORTC &= !(1<<6); 
		tempo_buzzer = 0;
	}
}
// Tacômetro
ISR(INT0_vect)
{
	static uint8_t contVoltas = 0; // Variável local para o número de voltas
	if (contVoltas == 5) // Fazer a velocidade a cada 5 voltas
	{
		periodo = tempoX - tempo_us; // Determinando o período
		tempo_us = tempoX;
		Velocidade_veiculo = (565.49*diametro)/periodo; // ]Determinando a velocidade
		RPM = 300000/periodo; // Calculando o RPM
		contVoltas = 0;
	}
	contVoltas++;
	// Determinando a distancia percorrida
	distancia1_percorrida += ((float)3.1415*diametro); // Distancia em cm
	distancia_percorrida = distancia1_percorrida/100000; // Distancia em km
	//Temperatura
	RT = ((double)((temperatura*5000)/(5115 - (temperatura*5))));
	temperatura1 = ((double)((RT - 100)*2.6));
	
	// LDR
	Lux_LDR = ((float)1023000/Lux) - 1000; 
	
	// Condição para o farol ligar
	if(Lux_LDR < 400)
	  PORTB |= (1<<3);
	else
	   PORTB &= !(1<<3);
	
	if(temperatura1 - ((int)temperatura1)>= (1/2))	// Arredondando
	temperatura1++;
	if(temperatura1 > temperatura_max) // Pegando o valor maximo da temperatura
	temperatura_max = temperatura1;
	if(temperatura1 < 0) // Não permite temperatura negativa
	temperatura1 = 0;
	// Bateria
	bateria1 = ((float)((bateria*100)/1023));
	// Transformar o valor do potênciometro de analógico para digital
	if(Velocidade_veiculo > 20 && distancia_objeto < 300) // Restringindo a aceleração do motor a 20%
	{
		aceleracao1 = (aceleracao*255)/10230;
	}
	else
	 aceleracao1 = (aceleracao*255)/1023;
}

//Interrupção externa para os pinos D
ISR(PCINT2_vect)
{
	if(!tst_bit(PIND, PD4))
	{
		if(!(PIND & (1<<4))) //Botão (+)
		if(diametro < 250) // Limite superior do diametro do pneu
		diametro++; //Aumenta o diâmetro do pneu
	}
	if(!tst_bit(PIND, PD5))
	{
		if(!(PIND & (1<<5)))//Botão (-)
		if(diametro > 0)// Limite inferior do diametro do pneu
		diametro--; //Diminui o diâmetro do pneu
	}
}
// Interrupção para o ADC
ISR(ADC_vect){
	switch(Cont){ // Selecionar a entrada do ACD
		case 0: // Aceleração
		aceleracao = ADC;
		ADMUX = 0b01000001;
		break;
		case 1: // Temperatura da Bateeria
		temperatura = ADC;
		ADMUX = 0b01000010;
		break;
		case 2: // Porcentagem da Bateria
		bateria = ADC;
		ADMUX = 0b01000011;
		break;
		case 3: // SENSOR LDR
		Lux = ADC;
		ADMUX = 0b01000000;
		Cont = -1;
		break;
	}
	Cont++;
}
int main(void)
{
	//Configuração ADC
	ADMUX = 0b01000000; // Vcc com referencia canal PC0
	ADCSRA= 0b11101111; // Habilita o AD, habilita interrupção, modo de conversão continua, prescaler = 128
	ADCSRB= 0b00000000; // Modo de conversão contínua
	DIDR0 = 0b00000000; // habilita pino PC como entrada digitais
	DDRC = 0b11110000; // Habilitando o PC0 para o potenciômetro, o PC2 para temperatura e PC1 para a bateria
	// O PC4 e PC5 para o Display
	DDRB = 0b11111110; // Saidas para o LCD
	TCCR0A = 0b00000010; // Habilitar o modo CTC do CT0
	TCCR0B = 0b00000011; // liga TC0 com prescaler = 64
	OCR0A = 249; // ->  1ms = (249 + 1)*8/16MHz
	TIMSK0 = 0b00000010; //Interrupção na igualdade
	DDRD &= (1<<2); // Habilita o pino PD2 como entrada
	PORTD |= (1<<2); // Habilita o resistor de pull-up do pino PD2
	DDRD &= (1<<3); // Habilita o pino PD3 como entrada
	DDRD &= (1<<4); // Habilita o pino PD4 como entrada
	PORTD |= (1<<4); // Habilita o resistor de pull-up do pino PD4
	DDRD &= (1<<5); // Habilita o pino PD4 como entrada
	PORTD |= (1<<5); // Habilita o resistor de pull-up do pino PD4
	EIMSK = 0x01;  // Habilita a intertupção externa INT0 e INT0
	EICRA |= 0x02; // Intertupção externa INT0
	EICRA |= 0x08; // Intertupção externa INT1
	//intertupção externa
	PCICR  = 0b00000100; // Habilita a interrupção pin change 2 (porta D)
	PCMSK2 = 0b00110000; // Habilita a interrupção PnChange PD4 e PD5
	//Configuração do timer T2 para o modo PWM com prescaler
	TCCR2A = 0b00100011;// Habilita o PWM para o pino PD3
	TCCR2B = 0b00000100;// Configura o prescale para 64
	TCCR1B = (1<<ICES1)|(1<<CS12); // Captura na borda de subida, TC1 com prescaler = 256
	TIMSK1 = (1 << ICIE1)|(1<<TOIE1); // Habilita a interrupção por captura
	USART_Init(MYUBRR); // Chamada da função para inicialização da USART
	sei(); // Intertupção global
	// Função da bibilioteca nokia
	GLCD_Setup();
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
	// Chamada da função para ler a gravação da EEPROM
	Ler_EEPROM();
	while (1)
	{
		OCR2B = aceleracao1; // PWM
		Mostrar_Velocidade(&flag5ms); // Mostrar a velocidade no LCD
		Display(&flag500ms); // Mostrar as informações no DISPLAY
		// Gravar os dados na EEPROM
		eeprom_write_word(0,distancia_percorrida);
		eeprom_write_word(2,diametro);
		eeprom_write_byte(4,temperatura_max);
	}
}
void Ler_EEPROM() // Gravar os dados na EEPROM
{
	distancia_percorrida = eeprom_read_word(0);
	diametro = eeprom_read_word(2);
	temperatura_max = eeprom_read_byte(4);
	
	distancia1_percorrida = distancia_percorrida*100000; // Recalculando o valor da velocidade em cm
}

void Mostrar_Velocidade(uint8_t *flag5ms1){ // Função para mostrar a velocidade

	if(*flag5ms1){
		Shift_reg(Velocidade_veiculo);
		}
		*flag5ms1 = 0;
}
void Display (uint8_t *flag500ms1){ // Função para mostrar no display
	
	if(*flag500ms1 && !(PIND & (1<<6)) || !(PIND & (1<<7)) ){
		int8_t inicio = 1; //posição vertical do cursor
		GLCD_Clear();
		GLCD_GotoXY(1,inicio);
		GLCD_PrintString("LASD Car");
		// Porcentagem da batria
		GLCD_GotoXY(87,inicio+3);
		GLCD_PrintInteger(bateria1);
		GLCD_PrintString(" %");
		// RPM
		GLCD_GotoXY(1,inicio+=12);
		GLCD_PrintInteger(RPM);
		GLCD_PrintString(" RPM");
		// Temperatura da bateria em °C
		GLCD_GotoXY(87,inicio+5);
		GLCD_PrintInteger(temperatura1);
		GLCD_PrintString(" C");
		// Sonar
		GLCD_GotoXY(1,inicio+=12);
		GLCD_PrintString("Sonar: ");
		GLCD_PrintInteger(distancia_objeto);
		// Diametro do Pneu
		GLCD_GotoXY(1,inicio+=12);
		GLCD_PrintString("D. Pneu: ");
		GLCD_PrintInteger(diametro);
		GLCD_PrintString(" CM");
		// Distância Percorrida
		GLCD_GotoXY(18,inicio+=14);
		GLCD_PrintInteger(distancia_percorrida);
		GLCD_PrintString("KM");
		GLCD_GotoXY(115,inicio);
		// Indicar a marcha
		if(!(PIND & (1<<7))) // Quando o pino D7 recebe 0, coloca o carro em modo Park
			GLCD_PrintString("P");
		if(PIND & (1<<7)) // Quando o pino D7 recebe 1, ativa o modo R ou D
		{
		   // quando recebe 0, ativa o modo D
		   GLCD_PrintString("D");
		}
		// Envolver os dados do Display com o retângulo
		GLCD_DrawLine(1,10,48,10,GLCD_Black);
		GLCD_DrawRectangle(10,inicio-3,60,inicio+10,GLCD_Black);
		GLCD_DrawRectangle(110,inicio-3,125,inicio+10,GLCD_Black);
		GLCD_DrawRectangle(80,1,124,30,GLCD_Black); // Não deu certo colocar com a variável inicio
		*flag500ms1 = 0;
	}
	if(*flag500ms1 && (PIND & (1<<7)) && (PIND & (1<<6))) // Condição para quando a marcha ré for engatada 
	{
		GLCD_Clear();
		GLCD_GotoXY(0,0);
		GLCD_GotoXY(29,12);
		GLCD_PrintString("SENSOR DE RE");
		GLCD_GotoXY(45,40);
		GLCD_PrintInteger(distancia_objeto); // Sonar
		GLCD_PrintString(" CM");
		GLCD_DrawRectangle(40,33,84,54,GLCD_Black);
		*flag500ms1 = 0;
	}
	GLCD_Render();
}
