/***************************************************************************
 *   Copyright (C) 10/2020 by Olaf Rempel                                  *
 *   razzor@kopf-tisch.de                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; version 2 of the License,               *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

#define BOOTLOADER_START		0x7C00 

#define VERSION_STRING          "TWIBOOT v3.2"
#define EEPROM_SUPPORT          0
#define LED_SUPPORT             0

#ifndef USE_CLOCKSTRETCH
#define USE_CLOCKSTRETCH        0
#endif

#ifndef VIRTUAL_BOOT_SECTION
#define VIRTUAL_BOOT_SECTION    0
#endif

#ifndef TWI_ADDRESS
#define TWI_ADDRESS             0x5d
#endif

#define F_CPU                   8000000ULL
#define TIMER_DIVISOR           1024
#define TIMER_IRQFREQ_MS        25
#define TIMEOUT_MS              5000

#define TIMER_MSEC2TICKS(x)     ((x * F_CPU) / (TIMER_DIVISOR * 1000ULL))
#define TIMER_MSEC2IRQCNT(x)    (x / TIMER_IRQFREQ_MS)

#if !defined(TWCR) && defined(USICR)
#define USI_PIN_INIT()          { PORTB |= ((1<<PORTB0) | (1<<PORTB2)); \
                                  DDRB |= (1<<PORTB2); \
                                }
#define USI_PIN_SDA_INPUT()     DDRB &= ~(1<<PORTB0)
#define USI_PIN_SDA_OUTPUT()    DDRB |= (1<<PORTB0)
#define USI_PIN_SCL()           (PINB & (1<<PINB2))

#define USI_STATE_MASK          0x0F
#define USI_STATE_IDLE          0x00    /* wait for Start Condition */
#define USI_STATE_SLA           0x01    /* wait for Slave Address */
#define USI_STATE_SLAW_ACK      0x02    /* ACK Slave Address + Write (Master writes) */
#define USI_STATE_SLAR_ACK      0x03    /* ACK Slave Address + Read (Master reads) */
#define USI_STATE_NAK           0x04    /* send NAK */
#define USI_STATE_DATW          0x05    /* receive Data */
#define USI_STATE_DATW_ACK      0x06    /* transmit ACK for received Data */
#define USI_STATE_DATR          0x07    /* transmit Data */
#define USI_STATE_DATR_ACK      0x08    /* receive ACK for transmitted Data */
#define USI_WAIT_FOR_ACK        0x10    /* wait for ACK bit (2 SCL clock edges) */
#define USI_ENABLE_SDA_OUTPUT   0x20    /* SDA is output (slave transmitting) */
#define USI_ENABLE_SCL_HOLD     0x40    /* Hold SCL low after clock overflow */
#endif /* !defined(TWCR) && defined(USICR) */

/* SLA+R */
#define CMD_WAIT                0x00
#define CMD_READ_VERSION        0x01
#define CMD_ACCESS_MEMORY       0x02
#define CMD_GET_VERSION			0x03
/* internal mappings */
#define CMD_ACCESS_CHIPINFO     (0x10 | CMD_ACCESS_MEMORY)
#define CMD_ACCESS_FLASH        (0x20 | CMD_ACCESS_MEMORY)
#define CMD_ACCESS_EEPROM       (0x30 | CMD_ACCESS_MEMORY)
#define CMD_WRITE_FLASH_PAGE    (0x40 | CMD_ACCESS_MEMORY)
#define CMD_WRITE_EEPROM_PAGE   (0x50 | CMD_ACCESS_MEMORY)

/* SLA+W */
#define CMD_SWITCH_APPLICATION  CMD_READ_VERSION
/* internal mappings */
#define CMD_BOOT_BOOTLOADER     (0x10 | CMD_SWITCH_APPLICATION) /* only in APP */
#define CMD_BOOT_APPLICATION    (0x20 | CMD_SWITCH_APPLICATION)

/* CMD_SWITCH_APPLICATION parameter */
#define BOOTTYPE_BOOTLOADER     0x00    /* only in APP */
#define BOOTTYPE_APPLICATION    0x80

/* CMD_{READ|WRITE}_* parameter */
#define MEMTYPE_CHIPINFO        0x00
#define MEMTYPE_FLASH           0x01
#define MEMTYPE_EEPROM          0x02

/*
 * LED_GN flashes with 20Hz (while bootloader is running)
 * LED_RT flashes on TWI activity
 *
 * bootloader twi-protocol:
 * - abort boot timeout:
 *   SLA+W, 0x00, STO
 *
 * - show bootloader version
 *   SLA+W, 0x01, SLA+R, {16 bytes}, STO
 *
 * - start application
 *   SLA+W, 0x01, 0x80, STO
 *
 * - read chip info: 3byte signature, 1byte page size, 2byte flash size, 2byte eeprom size
 *   SLA+W, 0x02, 0x00, 0x00, 0x00, SLA+R, {8 bytes}, STO
 *
 * - read one (or more) flash bytes
 *   SLA+W, 0x02, 0x01, addrh, addrl, SLA+R, {* bytes}, STO
 *
 * - read one (or more) eeprom bytes
 *   SLA+W, 0x02, 0x02, addrh, addrl, SLA+R, {* bytes}, STO
 *
 * - write one flash page
 *   SLA+W, 0x02, 0x01, addrh, addrl, {* bytes}, STO
 *
 * - write one (or more) eeprom bytes
 *   SLA+W, 0x02, 0x02, addrh, addrl, {* bytes}, STO
 */

const static uint8_t info[16] = VERSION_STRING;
const static uint8_t chipinfo[8] = {
    SIGNATURE_0, SIGNATURE_1, SIGNATURE_2,
    SPM_PAGESIZE,

    (BOOTLOADER_START >> 8) & 0xFF,
    BOOTLOADER_START & 0xFF,

    0x00, 0x00
};

static uint8_t boot_timeout = TIMER_MSEC2IRQCNT(TIMEOUT_MS);
static uint8_t bcnt = 0u;
static uint8_t bcnt_read = 16u;
static uint8_t cmd = CMD_WAIT;

/* flash buffer */
static uint8_t buf[SPM_PAGESIZE];
static uint16_t addr;
static uint16_t read_addr = 0;



/* *************************************************************************
 * write_flash_page
 * ************************************************************************* */
static void write_flash_page(void)
{
	
    uint16_t pagestart = addr;
    uint8_t size = SPM_PAGESIZE;
    uint8_t *p = buf;
	
    if (pagestart < BOOTLOADER_START)
    {
        boot_page_erase(pagestart);
        boot_spm_busy_wait();
		
        do {
            uint16_t data = *p++;
            data |= *p++ << 8;
            boot_page_fill(addr, data);

            addr += 2;
            size -= 2;
        } while (size);


        boot_page_write(pagestart);
        boot_spm_busy_wait();
		



#if defined (ASRE) || defined (RWWSRE)
        /* only required for bootloader section */
        boot_rww_enable();
#endif
		
		
		
		// Reset for the next page for 0x80
		bcnt = 4u; 
		cmd = CMD_ACCESS_FLASH;
		
    }
} /* write_flash_page */


/* *************************************************************************
 * TWI_data_write
 * ************************************************************************* */
static uint8_t TWI_data_write(uint8_t data)
{
    uint8_t ack = 0x01;

    switch (bcnt)
    {
        case 0:
            switch (data)
            {
                case CMD_SWITCH_APPLICATION:
                case CMD_ACCESS_MEMORY:
				case CMD_GET_VERSION:
                    /* no break */

                case CMD_WAIT:
                    /* abort countdown */
                    boot_timeout = 0;
                    cmd = data;
					//This will ensure Bootloader version read to work (cmd "Show bootloader version").
					//bcnt = 0u;
					// This will ensure that go to read chipingo
					bcnt ++ ;
                    break;

                default:
                    /* boot app now */
                    cmd = CMD_BOOT_APPLICATION;
                    ack = 0x00;
                    break;
            }
            break;

        case 1:
            switch (cmd)
            {
                case CMD_SWITCH_APPLICATION:
                    if (data == BOOTTYPE_APPLICATION)
                    {
                        cmd = CMD_BOOT_APPLICATION;
						bcnt++;
                    }

                    ack = 0x00;
                    break;

                case CMD_ACCESS_MEMORY:
                    if (data == MEMTYPE_CHIPINFO)
                    {
                        cmd = CMD_ACCESS_CHIPINFO;
						bcnt++;
						
                    }
                    else if (data == MEMTYPE_FLASH)
                    {
                        cmd = CMD_ACCESS_FLASH;
						bcnt++;
                    }
                    else
                    {
                        ack = 0x00;
                    }
                    break;
				
					
				case CMD_GET_VERSION:
					cmd = CMD_READ_VERSION;
					ack = 0x00;
					bcnt++;
					break;
				
				
                default:
                    ack = 0x00;
                    break;
            }
            break;

        case 2:
        case 3:
            addr <<= 8;
            addr |= data;
			bcnt++;
			break;

        default:
            switch (cmd)
            {
                case CMD_ACCESS_FLASH:			
					{
					//cmd = CMD_WRITE_FLASH_PAGE;
                    uint8_t pos = bcnt - 4;
                    buf[pos] = data;
                    if (pos >= (SPM_PAGESIZE -1))
                    {
                        if (cmd == CMD_ACCESS_FLASH)
                        {
                            cmd = CMD_WRITE_FLASH_PAGE;
                        }

                        ack = 0x00;
                    }
					
					break;
					}					
                

                default:
                    ack = 0x00;
					break;
            }
			bcnt++;
            break;
    }

	//Increment bcnt.
	//bcnt++;
    return ack;
} /* TWI_data_write */


/* *************************************************************************
 * TWI_data_read
 * ************************************************************************* */
static uint8_t TWI_data_read(void)
{
    uint8_t data;

    switch (cmd)
    {
        case CMD_READ_VERSION:
            bcnt_read %= sizeof(info);
            data = info[bcnt_read];
            break;

        case CMD_ACCESS_CHIPINFO:
            bcnt_read %= sizeof(chipinfo);
            data = chipinfo[bcnt_read];
            break;

        case CMD_ACCESS_FLASH: 
						
			data = pgm_read_byte_near(read_addr);          
			read_addr++;			
            break;

        default:
            data = 0xFF;
            break;
    }

	//Increment bcnt.
	bcnt_read++;

	/*
	//This will ensure Bootloader version read to work (cmd "Show bootloader version").
	if(16u == bcnt)
	{
		//Reset bcnt in case it reached 16 bytes.
		bcnt = 0u;
	}
	else
	{
		//Do nothing with bcnt, keep sending FBL version data.
	}
	*/

    return data;
} /* TWI_data_read */


#if defined (TWCR)
/* *************************************************************************
 * TWI_vect
 * ************************************************************************* */
static void TWI_vect(void)
{
    uint8_t control = TWCR;


    switch (TWSR & 0xF8)
    {
        /* SLA+W received, ACK returned -> receive data and ACK */
        case 0x60:
            break;

        /* prev. SLA+W, data received, ACK returned -> receive data and ACK */
        case 0x80:
            if (TWI_data_write(TWDR) == 0x00)
            {
                /* the ACK returned by TWI_data_write() is not for the current
                 * data in TWDR, but for the next byte received
                 */
                control &= ~(1<<TWEA);
            }
            break;

        /* SLA+R received, ACK returned -> send data */
        case 0xA8:
            /* fall through */

        /* prev. SLA+R, data sent, ACK returned -> send data */
        case 0xB8:
            TWDR = TWI_data_read();
            break;

        /* prev. SLA+W, data received, NACK returned -> IDLE */
        case 0x88:
            TWI_data_write(TWDR);
            /* fall through */

        /* STOP or repeated START -> IDLE */
        case 0xA0:
		
		if (cmd == CMD_WRITE_FLASH_PAGE)
			{
			/* disable ACK for now, re-enable after page write */
			control &= ~(1<<TWEA);
			TWCR = (1<<TWINT) | control;			
			write_flash_page();			
			}

		
        /* fall through */
			

        /* prev. SLA+R, data sent, NACK returned -> IDLE */
        case 0xC0:
			control |= (1<<TWEA);
            break;

        /* illegal state(s) -> reset hardware */
        default:
            control |= (1<<TWSTO);
            break;
    }

    TWCR = (1<<TWINT) | control;
} /* TWI_vect */
#endif /* defined (TWCR) */

#if defined (USICR)
/* *************************************************************************
 * usi_statemachine
 * ************************************************************************* */
static void usi_statemachine(uint8_t usisr)
{
    static uint8_t usi_state;
    static uint8_t bcnt;

    uint8_t data = USIDR;
    uint8_t state = usi_state & USI_STATE_MASK;

    /* Start Condition detected */
    if (usisr & (1<<USISIF))
    {
        /* wait until SCL goes low */
        while (USI_PIN_SCL());

        usi_state = USI_STATE_SLA | USI_ENABLE_SCL_HOLD;
        state = USI_STATE_IDLE;
    }

    /* Stop Condition detected */
    if (usisr & (1<<USIPF))
    {
        usi_state = USI_STATE_IDLE;
        state = USI_STATE_IDLE;
    }

    if (state == USI_STATE_IDLE)
    {
        /* do nothing */
    }
    /* Slave Address received => prepare ACK/NAK */
    else if (state == USI_STATE_SLA)
    {
        bcnt = 0;

        /* SLA+W received -> send ACK */
        if (data == ((TWI_ADDRESS<<1) | 0x00))
        {
            usi_state = USI_STATE_SLAW_ACK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x00;
        }
        /* SLA+R received -> send ACK */
        else if (data == ((TWI_ADDRESS<<1) | 0x01))
        {
            usi_state = USI_STATE_SLAR_ACK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x00;
        }
        /* not addressed -> send NAK */
        else
        {
            usi_state = USI_STATE_NAK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x80;
        }
    }
    /* sent NAK -> go to idle */
    else if (state == USI_STATE_NAK)
    {
        usi_state = USI_STATE_IDLE;
    }
    /* sent ACK after SLA+W -> wait for data */
    /* sent ACK after DAT+W -> wait for more data */
    else if ((state == USI_STATE_SLAW_ACK) ||
             (state == USI_STATE_DATW_ACK)
            )
    {
        usi_state = USI_STATE_DATW | USI_ENABLE_SCL_HOLD;
    }
    /* data received -> send ACK/NAK */
    else if (state == USI_STATE_DATW)
    {
        if (TWI_data_write(bcnt++, data))
        {
            usi_state = USI_STATE_DATW_ACK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x00;
        }
        else
        {
            usi_state = USI_STATE_NAK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x80;
        }
    }
    /* sent ACK after SLA+R -> send data */
    /* received ACK after DAT+R -> send more data */
    else if ((state == USI_STATE_SLAR_ACK) ||
             ((state == USI_STATE_DATR_ACK) && !(data & 0x01))
            )
    {
        USIDR = TWI_data_read(bcnt++);
        usi_state = USI_STATE_DATR | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
    }
    /* sent data after SLA+R -> receive ACK/NAK */
    else if (state == USI_STATE_DATR)
    {
        usi_state = USI_STATE_DATR_ACK | USI_WAIT_FOR_ACK | USI_ENABLE_SCL_HOLD;
        USIDR = 0x80;
    }
    /* received NAK after DAT+R -> go to idle */
    else if ((state == USI_STATE_DATR_ACK) && (data & 0x01))
    {
        usi_state = USI_STATE_IDLE;
    }
    /* default -> go to idle */
    else
    {
        usi_state = USI_STATE_IDLE;
    }

    /* set SDA direction according to current state */
    if (usi_state & USI_ENABLE_SDA_OUTPUT)
    {
        USI_PIN_SDA_OUTPUT();
    }
    else
    {
        USI_PIN_SDA_INPUT();
    }

    if (usi_state & USI_ENABLE_SCL_HOLD)
    {
        /* Enable TWI Mode, hold SCL low after counter overflow, count both SCL edges */
        USICR = (1<<USIWM1) | (1<<USIWM0) | (1<<USICS1);
    }
    else
    {
        /* Enable TWI, hold SCL low only after start condition, count both SCL edges */
        USICR = (1<<USIWM1) | (1<<USICS1);
    }

    /* clear start/overflow/stop condition flags */
    usisr &= ((1<<USISIF) | (1<<USIOIF) | (1<<USIPF));
    if (usi_state & USI_WAIT_FOR_ACK)
    {
        /* count 2 SCL edges (ACK/NAK bit) */
        USISR = usisr | ((16 -2)<<USICNT0);
    }
    else
    {
        /* count 16 SCL edges (8bit data) */
        USISR = usisr | ((16 -16)<<USICNT0);
    }
} /* usi_statemachine */
#endif /* defined (USICR) */


/* *************************************************************************
 * TIMER0_OVF_vect
 * ************************************************************************* */
static void TIMER0_OVF_vect(void)
{
    /* restart timer */
    TCNT0 = 0xFF - TIMER_MSEC2TICKS(TIMER_IRQFREQ_MS);

    /* count down for app-boot */
    if (boot_timeout > 1)
    {
        boot_timeout--;
    }
    else if (boot_timeout == 1)
    {
        /* trigger app-boot */
        cmd = CMD_BOOT_APPLICATION;
    }
} /* TIMER0_OVF_vect */


static void (*jump_to_app)(void) __attribute__ ((noreturn)) = (void*)0x0000;


/* *************************************************************************
 * init1
 * ************************************************************************* */
void init1(void) __attribute__((naked, section(".init1")));
void init1(void)
{
  /* make sure r1 is 0x00 */
  asm volatile ("clr __zero_reg__");

  /* on some MCUs the stack pointer defaults NOT to RAMEND */
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8515__) || \
    defined(__AVR_ATmega8535__) || defined (__AVR_ATmega16__) || \
    defined (__AVR_ATmega32__) || defined (__AVR_ATmega64__)  || \
    defined (__AVR_ATmega128__) || defined (__AVR_ATmega162__)
  SP = RAMEND;
#endif
} /* init1 */


/*
 * For newer devices the watchdog timer remains active even after a
 * system reset. So disable it as soon as possible.
 * automagically called on startup
 */
#if defined (__AVR_ATmega88__) || defined (__AVR_ATmega168__) || \
    defined (__AVR_ATmega328P__)
/* *************************************************************************
 * disable_wdt_timer
 * ************************************************************************* */
void disable_wdt_timer(void) __attribute__((naked, section(".init3")));
void disable_wdt_timer(void)
{
    MCUSR = 0;
    WDTCSR = (1<<WDCE) | (1<<WDE);
    WDTCSR = (0<<WDE);
} /* disable_wdt_timer */
#endif


/* *************************************************************************
 * main
 * ************************************************************************* */
int main(void) __attribute__ ((OS_main, section (".init9")));
int main(void)
{

    /* timer0: running with F_CPU/1024 */
#if defined (TCCR0)
    TCCR0 = (1<<CS02) | (1<<CS00);
#elif defined (TCCR0B)
    TCCR0B = (1<<CS02) | (1<<CS00);
#else
#error "TCCR0(B) not defined"
#endif

#if defined (TWCR)
    /* TWI init: set address, auto ACKs */
    TWAR = (TWI_ADDRESS<<1);
    TWCR = (1<<TWEA) | (1<<TWEN);
#elif defined (USICR)
    USI_PIN_INIT();
    usi_statemachine(0x00);
#else
#error "No TWI/USI peripheral found"
#endif

    while (cmd != CMD_BOOT_APPLICATION)
    {
#if defined (TWCR)
        if (TWCR & (1<<TWINT))
        {
            TWI_vect();
        }
#elif defined (USICR)
        if (USISR & ((1<<USISIF) | (1<<USIOIF) | (1<<USIPF)))
        {
            usi_statemachine(USISR);
        }
#endif

#if defined (TIFR)
        if (TIFR & (1<<TOV0))
        {
            TIMER0_OVF_vect();
            TIFR = (1<<TOV0);
        }
#elif defined (TIFR0)
        if (TIFR0 & (1<<TOV0))
        {
            TIMER0_OVF_vect();
            TIFR0 = (1<<TOV0);
        }
#else
#error "TIFR(0) not defined"
#endif
    }

#if defined (TWCR)
    /* Disable TWI but keep address! */
    TWCR = 0x00;
#elif defined (USICR)
    /* Disable USI peripheral */
    USICR = 0x00;
#endif

    /* disable timer0 */
#if defined (TCCR0)
    TCCR0 = 0x00;
#elif defined (TCCR0B)
    TCCR0B = 0x00;
#else
#error "TCCR0(B) not defined"
#endif


    jump_to_app();
} /* main */
