#include <stm32f4xx.h>
#include <stdbool.h>
#include "../gpio.h"
#include "../adc.h"


error adc_init(adc *self) {
	if (_adc_is_adc(self->pin) != OK) {
		return ADC_ERR_PIN_NOT_ADC;
	}
	if (self->mode > ADC_MODE_CONTINUOUS) {
		return ADC_ERR_WRONG_MODE;
	}
	if (self->resolution > ADC_RESOLUTION_6BIT) {
		return ADC_ERR_WRONG_RESOLUTION;
	}
	if (self->_order > 0) {
		return ADC_ERR_PORT_ALREADY_INIT;
	}
	self->_order = 0;
	GPIO_enable(self->pin, GPIO_ANALOG);
	if (self->port == ADC1) {
		RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
		self->port->CR2 &= ~(1U << ADC_CR2_ADON_OFFSET);
	}
	self->port->CR2 &= ~(1 << ADC_CR2_SWSTART_OFFSET);
	volatile i32 num_conv = 0;
	for (i32 i = 1, reg_sub = 0x1F, reg = 3, bit = 0; i <= ADC_NUM_CHANNELS; i++, reg_sub<<=5, bit+=5) {
		if (reg == 3) {
			if ((ADC1->SQR3 & reg_sub) == 0 && _adc_fetch_channel(self) != -1) {
				ADC1->SQR3 |= (_adc_fetch_channel(self) << bit);
				self->_order = i;
				break;
			}
		} else if (reg == 2) {
			if ((ADC1->SQR2 & reg_sub) == 0 && _adc_fetch_channel(self) != -1) {
				ADC1->SQR2 |= (_adc_fetch_channel(self) << bit);
				self->_order = i;
				break;
			}
		} else if (reg == 1) {
			if ((ADC1->SQR1 & reg_sub) == 0 && _adc_fetch_channel(self) != -1) {
				ADC1->SQR1 |= (_adc_fetch_channel(self) << bit);
				self->_order = i;
				break;
			}
		}
		if ((i % ADC_NUM_CHANNELS_SQR3+1) == 0) {
			reg--;
			reg_sub = 0x1F;
			bit = 0;
		}
		num_conv = i;
		if (i == ADC_NUM_CHANNELS) {
			return ADC_ERR_NO_FREE_CHANNEL;
		}
	}
	self->port->SQR1 &= ~(0x0F << ADC_SQR1_LBIT_OFFSET);
	if (self->_order != 0) {
		self->port->SQR1 |= ((self->_order-1) << ADC_SQR1_LBIT_OFFSET);
	} else {
		self->port->SQR1 |= ((num_conv) << ADC_SQR1_LBIT_OFFSET);
	}
	if (self->mode == ADC_MODE_SINGLE) {
		self->port->CR2 &= ~ADC_CONTINUOUS_CONT;
	} else if (self->mode == ADC_MODE_CONTINUOUS) {
		self->port->CR2 |= ADC_CONTINUOUS_CONT;
	}

	self->port->CR2 |= (1U << ADC_CR2_ADON_OFFSET);
	self->port->CR2 |= (1U << ADC_CR2_SWSTART_OFFSET);
	return OK;
}


error adc_deinit(adc *self) {
	if (_adc_is_adc(self->pin) != OK) {
		return ADC_ERR_PIN_NOT_ADC;
	}
	if (self->_order < 1) {
		return OK;
	}

	if (self->_order < 7) {
		self->port->SQR3 &= ~(0x1F << (self->_order - 1) * 5);
	} else if (self->_order < 13) {
		self->port->SQR2 &= ~(0x1F << ((self->_order % ADC_NUM_CHANNELS_SQR2) - 1) * 5);
	} else if (self->_order < 16) {
		self->port->SQR1 &= ~(0x1F << ((self->_order % ADC_NUM_CHANNELS_SQR2) - 1) * 5);
	}
	//uint8_t l_bit = (self->port->SQR1 & (0x0F << ADC_SQR1_LBIT_OFFSET)) == 0 ? 0 : (self->port->SQR1 & (0x0F << ADC_SQR1_LBIT_OFFSET)) - 1;
	volatile i32 l_bit = ((self->port->SQR1 & (0xF << ADC_SQR1_LBIT_OFFSET)) >> ADC_SQR1_LBIT_OFFSET) - 1;
	//volatile int l_bit = self->port->SQR1 & ;
	self->port->SQR1 &= ~(0x0F << ADC_SQR1_LBIT_OFFSET);
	self->port->SQR1 |= (l_bit << ADC_SQR1_LBIT_OFFSET);
	return OK;
}


error _adc_is_adc(pinnum_t pin) {
	if ((pin >= PA1 && pin <= PA7)
			|| (pin >= PC0 || pin <= PC5)
			|| pin == PB0 || pin == PB1) {
		return OK;
	}
	return ADC_ERR_PIN_NOT_ADC;
}

i8 _adc_fetch_channel(adc *self) {
	if (_ADC_is_adc(self->pin) != OK) {
		return -1;
	}
	switch(self->pin) {
		case ADC1_1:
			return 1;
		case ADC1_2:
			return 2;
		case ADC1_3:
			return 3;
		case ADC1_4:
			return 4;
		case ADC1_5:
			return 5;
		case ADC1_6:
			return 6;
		case ADC1_7:
			return 7;
		case ADC1_8:
			return 8;
		case ADC1_9:
			return 9;
		case ADC1_10:
			return 10;
		case ADC1_11:
			return 11;
		case ADC1_12:
			return 12;
		case ADC1_13:
			return 13;
		case ADC1_14:
			return 14;
		case ADC1_15:
			return 15;
		default:
			return -1;
	}
}
