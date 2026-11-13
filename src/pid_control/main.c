#define F_CPU 4000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

// === MODBUS AYARLARI ===
#define SLAVE_ID 2
#define BUFFER_SIZE 32

volatile uint8_t rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t frame_ready = 0;
volatile uint16_t modbus_timer = 0;

// === YENİ REGISTER HARİTASI (5 Adet) ===
// [0]=Set_Value, [1]=Kp, [2]=Ki, [3]=Kd, [4]=Current_Temp
volatile uint16_t holding_register[5] = {50, 8, 1, 1, 20}; // Başlangıç değerleri

// === ZAMANLAMA VE PID DEĞİŞKENLERİ ===
volatile uint8_t pid_tick = 0;
volatile uint16_t pid_timer_counter = 0; // 100ms sayacı

double integral = 0.0;
double previous_error = 0.0;

// === J-TİPİ SENSÖR VE ADC AYARLARI ===
#define ADC_REF       5.0
#define FILTER_SIZE   8
#define GAIN          100.0
#define NIST_MV_MIN   0.0
#define NIST_MV_MAX   42.919
#define HEATER_PWM_PIN PB3  // Isıtıcı (OC2) Pini

const double a[] = {
	0.0000000E+00, 1.9528268E+01, -1.2286185E+00, 1.0752178E-01,
	-5.9086933E-03, 1.7256713E-04, -2.8131513E-06, 2.3963370E-08, -8.3823321E-11
};

// 1. DONANIM BAŞLATMA FONKSİYONLARI

void Timer2_PWM_Init() {
	DDRB |= (1 << HEATER_PWM_PIN); // PB3 Çıkış
	// Fast PWM Modu, Non-Inverting, Prescaler = 64
	TCCR2 = (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS22);
	OCR2 = 0; // Isıtıcı başlangıçta kapalı
}

void ADC_Init() {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0); // Prescaler 32
}

uint16_t ADC_Read(uint8_t channel) {
	channel &= 0b00000111;
	ADMUX = (ADMUX & 0xF8) | channel;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return (ADC);
}

uint16_t readFilteredADC(uint8_t channel) {
	uint32_t sum = 0;
	for (int i = 0; i < FILTER_SIZE; i++) {
		sum += ADC_Read(channel);
		_delay_ms(1);
	}
	return (uint16_t)(sum / FILTER_SIZE);
}

double calculateTemperaturePrecise(double mv) {
	double temp = 0.0;
	double mv_pow = 1.0;
	for (int i = 0; i < 9; i++) {
		temp += a[i] * mv_pow;
		mv_pow *= mv;
	}
	return temp;
}

// 2. MODBUS VE HABERLEŞME FONKSİYONLARI

uint16_t Modbus_CRC16(uint8_t *data, uint8_t len) {
	uint16_t crc = 0xFFFF;
	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
			else crc >>= 1;
		}
	}
	return crc;
}

void USART_Init(uint16_t baud) {
	uint16_t ubrr = (F_CPU / (16UL * baud)) - 1;
	UBRRH = (ubrr >> 8);
	UBRRL = ubrr;
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
	UCSRC = (1 << URSEL) | (3 << UCSZ0);
}

void USART_Transmit(uint8_t data) {
	while (!(UCSRA & (1 << UDRE)));
	UDR = data;
}

// Timer1: 1ms ayarlı (Hem Modbus sessizliğini hem de PID'i sayar)
void Timer1_Init() {
	TCCR1B = (1 << WGM12) | (1 << CS11);
	OCR1A = 499;
	TIMSK |= (1 << OCIE1A);
}

ISR(USART_RXC_vect) {
	uint8_t data = UDR;
	if (rx_index < BUFFER_SIZE) {
		rx_buffer[rx_index++] = data;
		modbus_timer = 0;
	}
}

ISR(TIMER1_COMPA_vect) {
	// 1. Görev: Modbus 4ms Sessizlik Dedektörü
	if (rx_index > 0) {
		if (++modbus_timer > 4) {
			frame_ready = 1;
			modbus_timer = 0;
		}
	}
	
	// 2. Görev: PID 100ms Tetikleyicisi
	if (++pid_timer_counter >= 100) {
		pid_tick = 1; // 100ms doldu, hesaplama yap!
		pid_timer_counter = 0;
	}
}

void process_modbus() {
	if (!frame_ready) return;
	frame_ready = 0;
	
	if (rx_index < 8) { rx_index = 0; return; }

	uint8_t addr = rx_buffer[0];
	uint8_t func = rx_buffer[1];
	uint16_t crc_recv = rx_buffer[rx_index - 2] | (rx_buffer[rx_index - 1] << 8);
	uint16_t crc_calc = Modbus_CRC16((uint8_t*)rx_buffer, rx_index - 2);

	if (addr != SLAVE_ID || crc_recv != crc_calc) { rx_index = 0; return; }

	uint8_t tx_buf[32];
	uint8_t len = 0;
	
	uint16_t reg_addr = (rx_buffer[2] << 8) | rx_buffer[3];

	// --- 0x03 OKUMA (READ HOLDING REGISTERS) ---
	if (func == 0x03) {
		uint16_t qty = (rx_buffer[4] << 8) | rx_buffer[5];
		if ((reg_addr + qty) <= 5) {
			tx_buf[0] = SLAVE_ID;
			tx_buf[1] = 0x03;
			tx_buf[2] = qty * 2;
			len = 3;
			for (uint16_t i = 0; i < qty; i++) {
				tx_buf[len++] = (holding_register[reg_addr + i] >> 8) & 0xFF;
				tx_buf[len++] = holding_register[reg_addr + i] & 0xFF;
			}
			uint16_t tx_crc = Modbus_CRC16(tx_buf, len);
			tx_buf[len++] = tx_crc & 0xFF;
			tx_buf[len++] = (tx_crc >> 8) & 0xFF;
			for (uint8_t i = 0; i < len; i++) USART_Transmit(tx_buf[i]);
		}
	}
	// --- 0x06 TEKİL YAZMA (WRITE SINGLE REGISTER) ---
	else if (func == 0x06) {
		uint16_t val = (rx_buffer[4] << 8) | rx_buffer[5];
		if (reg_addr < 5) {
			holding_register[reg_addr] = val; // Yeni veriyi RAM'e kaydet
			// Cevap olarak gelen paketin aynısını (Echo) geri yolla
			for (len = 0; len < 8; len++) USART_Transmit(rx_buffer[len]);
		}
	}
	// --- 0x10 ÇOKLU YAZMA (WRITE MULTIPLE REGISTERS) ---
	else if (func == 0x10) {
		uint16_t qty = (rx_buffer[4] << 8) | rx_buffer[5];
		uint8_t byte_count = rx_buffer[6];
		if ((reg_addr + qty) <= 5 && byte_count == (qty * 2)) {
			for (uint16_t i = 0; i < qty; i++) {
				holding_register[reg_addr + i] = (rx_buffer[7 + (i * 2)] << 8) | rx_buffer[8 + (i * 2)];
			}
			// Cevap paketi: ID + Func + Addr_H/L + Qty_H/L + CRC
			for (len = 0; len < 6; len++) tx_buf[len] = rx_buffer[len];
			uint16_t tx_crc = Modbus_CRC16(tx_buf, 6);
			tx_buf[6] = tx_crc & 0xFF;
			tx_buf[7] = (tx_crc >> 8) & 0xFF;
			for (uint8_t i = 0; i < 8; i++) USART_Transmit(tx_buf[i]);
		}
	}
	rx_index = 0;
}

// ==========================================
// 3. ANA PROGRAM (MAIN)
// ==========================================
int main(void) {
	USART_Init(19200);
	Timer1_Init();
	Timer2_PWM_Init();
	ADC_Init();
	sei();

	double cold_junction_temp = 20.0;
	double dt = 0.1; // 100ms = 0.1 saniye
	
	while (1) {
		// --- 100ms PID VE SENSÖR DÖNGÜSÜ ---
		if (pid_tick == 1) {
			pid_tick = 0;
			
			// 1. Sıcaklık Okuma
			uint16_t adcValue = readFilteredADC(2);
			double voltage = ((double)adcValue * ADC_REF) / 1023.0;
			double mv_thermocouple = (voltage * 1000.0) / GAIN;
			double temperature = 0.0;
			
			if (mv_thermocouple >= NIST_MV_MIN && mv_thermocouple <= NIST_MV_MAX) {
				temperature = calculateTemperaturePrecise(mv_thermocouple) + cold_junction_temp;
				} else {
				temperature = 999.0; // Hata
			}
			
			// Okunan sıcaklığı Register'a (Python'un göreceği yere) yaz
			holding_register[4] = (uint16_t)temperature;
			
			// 2. Modbus'tan Gelen PID Parametrelerini Değişkene Çek
			double setpoint = (double)holding_register[0];
			double kp = (double)holding_register[1];
			double ki = (double)holding_register[2];
			double kd = (double)holding_register[3];
			
			// 3. PID Algoritması
			if (temperature != 999.0) {
				double error = setpoint - temperature;
				integral += (error * dt);
				
				// Windup Koruması
				if (integral > 1000) integral = 1000;
				if (integral < -1000) integral = -1000;
				
				double derivative = (error - previous_error) / dt;
				previous_error = error;
				
				double pid_output = (kp * error) + (ki * integral) + (kd * derivative);
				
				if (pid_output > 255) pid_output = 255;
				else if (pid_output < 0) pid_output = 0;
				
				OCR2 = (uint8_t)pid_output; // Isıtıcıya Güç Ver
				} else {
				OCR2 = 0; // Sensör bozuksa ısıtıcıyı hemen kapat!
			}
		}
		
		// --- MODBUS DİNLEME (SÜREKLİ) ---
		process_modbus();
	}
}