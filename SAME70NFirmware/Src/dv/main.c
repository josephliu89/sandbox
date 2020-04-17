/* Atmel library includes. */
#include "asf.h"
#include "board.h"
#include "conf_project.h"
#include "dv/conf/conf_uart_serial.h"
#include "telem/dv_handler.h"
#include "peripherals/dv_spi.h"
#include "peripherals/dv_i2c.h"
#include "peripherals/dv_adc.h"
#include "peripherals/dv_tc.h"
#include "peripherals/dv_efc.h"
#include "peripherals/dv_plc.h"
#include "peripherals/dv_ext_irq.h"
#include "dv/telem/dv_handler.h"
#include "sleep.h"
#include "conf_pplc_if.h"
#include "dv/sensors/DV13131B/DV13131B.h"
#include <assert.h>
#include "dv/sensors/LDC1614.h"

/* Test defines */
//#define WRITE_TO_FLASH

/* String shown at initialization */
#define STRING_HEADER	"\r\n\n----------------------------------------------" \
						"\r\n-> DarkVision PL360 G3-PLC MAC RT Telemetry" \
						"\r\n-> " "DV13341A/DV13381A" \
						"\r\n-> Compiled: "__DATE__ " "__TIME__ "\r\n" \
						"----------------------------------------------\r\n"

/* LED blink rate in ms */
#define COUNT_MS_SWAP_LED			500

/* 1 ms timer definitions for LED management */
#define ID_TC_1MS					ID_TC1
#define TC_1MS						TC0
#define TC_1MS_CHN					1
#define TC_1MS_IRQn					TC1_IRQn
#define TC_1MS_Handler				TC1_Handler

/* LED management variables */
static uint32_t sul_count_ms = COUNT_MS_SWAP_LED;
uint32_t sul_ind_count_ms = 0;
static uint8_t suc_led_swap = 0;
static bool sb_ind_led_swap = false;

uint32_t one_ms_cnt = 0;					// Used to track time passed for transaction

/* Flash user space */
extern uint32_t __shared__;
extern uint32_t __temp__;
extern uint32_t __flwrite__;
extern uint32_t flash_writer_start;
extern uint32_t flash_writer_end;

/* Global variables declaration and initialization */
uint8_t role = ROLE_SLAVE;
bool first_boot = true;

static uint32_t getFlashWriterBinaryAndSize(uint32_t* address) {
    *address = (uint32_t)&flash_writer_start;
    uint32_t endPos = (uint32_t)&flash_writer_end;
    uint32_t startPos = (uint32_t)&flash_writer_start;
    return (endPos - startPos);
}

static void progFlashWriter(void) {
    uint32_t binAddress = 0;
    uint32_t ul_rc;
    
    /* Get flash writer binary address and size */
    uint32_t binSize = getFlashWriterBinaryAndSize(&binAddress);
    DV_info("Flash writer binary size: %lu", binSize);
    
    /* Program only if flash writer is different or missing */
    ul_rc = validate_data(&__flwrite__, (uint8_t *)binAddress, binSize);
    
    if (ul_rc != FLASH_RC_ERROR) {
        DV_info("--- Flash writer missing or out-of-date, programming started ---");
        
        /* Erase 'flash writer' partition */
        ul_rc = flash_erase_sector((uint32_t)&__flwrite__);
    
        if (ul_rc == FLASH_RC_OK) {
            DV_info("Flash writer sector erased successfully!");
        }
        else {
            DV_error("Cannot erase flash writer sector!");
        }
    
        /* Write to flash */
        ul_rc = flash_write((uint32_t)&__flwrite__, (uint8_t *)binAddress, binSize, 0);

        if (ul_rc == FLASH_RC_OK) {
            DV_info("Programming content written to flash!");
        }
        else {
            DV_error("Cannot write to flash!");
        }
    
        /* Validate data written */
        ul_rc = validate_data(&__flwrite__, (uint8_t *)binAddress, binSize);
    
        if (ul_rc == FLASH_RC_OK) {
            DV_info("Programming content validated!");
        }
        else {
            DV_error("Cannot validate contents written to flash!");
        }        
    }
    else {
        DV_info("Skipping flash writer programming.");
    }    
}

/**
 *  Configure Serial Console.
 */
static void configure_dbg_console(void)
{
#ifdef CONF_BOARD_UDC_CONSOLE
	/* Configure console UDC (USB, SAMG55). */
	stdio_udc_init(UDP, (void *)usb_wrp_udc_putchar, (void *)usb_wrp_udc_getchar, (void *)usb_wrp_udc_start);
	/* Wait to open terminal */
	while (!usb_wrp_cdc_get_dtr()) {
		/* Reset watchdog */
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;

		/* Blink led 0 */
		if (suc_led_swap == 2) {
			suc_led_swap = 0;
            ioport_toggle_pin_level(NGLED_PIN);
		}
	}
#else
	/* Configure console UART. */
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};
	sysclk_enable_peripheral_clock(CONF_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
#endif
}

/**
 * \brief Configure the hardware.
 */
static void prvSetupHardware(void)
{
	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();
}

/**
 * \brief Initialize 1mSec timer 3 interrupt
 */
static void initTimer1ms(void)
{
	uint32_t ul_div, ul_tcclks;

	/* Configure PMC */
	pmc_enable_periph_clk(ID_TC_1MS);

	/* MCK = 150000000 -> tcclks = 3 : TCLK = MCK/128 = 1171875 = 853.33ns ->
	 * ul_div = 1ms/853.33ns = 1172 */
	ul_tcclks = 3;
	ul_div = 1172;
	tc_init(TC_1MS, TC_1MS_CHN, ul_tcclks | TC_CMR_CPCTRG);

	tc_write_rc(TC_1MS, TC_1MS_CHN, ul_div);

	/* Configure and enable interrupt on RC compare */
	NVIC_SetPriority((IRQn_Type)ID_TC_1MS, 0);
	NVIC_EnableIRQ((IRQn_Type)ID_TC_1MS);
	tc_enable_interrupt(TC_1MS, TC_1MS_CHN, TC_IER_CPCS);

	/** Start the timer. TC0, channel 1 */
	tc_start(TC_1MS, TC_1MS_CHN);
}

/**
 * \brief Interrupt handler for Timer 3
 */
void TC_1MS_Handler(void)
{
	/* Clear status bit to acknowledge interrupt */
	tc_get_status(TC_1MS, TC_1MS_CHN);

	/* Update ms counter to blink led 0 */
	if (!sul_count_ms--) {
		sul_count_ms = COUNT_MS_SWAP_LED;
		suc_led_swap++;
	}

	/* Update ms counter to blink led 1 when a PLC message is received */
	if (sul_ind_count_ms) {
		if (!--sul_ind_count_ms) {
			sb_ind_led_swap = true;
		}
	}

	/* One ms counter, used to track transmission speeds */
	one_ms_cnt++;
}

/**
 * \brief SysTick_Handler implementation
 */
volatile uint32_t g_ul_ms_ticks = 0;
void SysTick_Handler(void)
{
    g_ul_ms_ticks++;
}

#define PPLC_PCS    spi_get_pcs(PPLC_CS)

/**
 * \brief Main code entry point.
 */
int main(void)
{
	/* Prepare the hardware */
    prvSetupHardware();
   
    /* Determine hardware role via hardware pin strapping */
	role = ioport_get_pin_level(CTRL_NDEV);

	/* Initialize 1ms timer. Used to manage LEDs */
	initTimer1ms();

	/* Initialize SysTick timer to do coarse timing */
    if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
        puts("-F- Systick configuration error\r");
        while (1);
    }

	/* Initialize UART debug */
	configure_dbg_console();
	puts(STRING_HEADER);
       	
    /* Initialize SPI master for testing */
    static uint32_t gs_ul_pplc_clock = PPLC_CLOCK;
    uint32_t ul_cpuhz;
    uint8_t uc_div;

    ul_cpuhz = sysclk_get_cpu_hz();
    uc_div = ul_cpuhz / gs_ul_pplc_clock;

    if (ul_cpuhz % gs_ul_pplc_clock) {
        uc_div++;
    }

    sampleSpiInit();
    
    uint8_t incData, incData3 = 0;
    uint8_t incData2 = 0x80;
    
    while (1) {
        char charVal = getchar();
             
        switch(charVal) {
            case 'x':
                enableIr();
                break;
                
            case 'y':
                disableIr();
                break;
                
            case 'w':
                DV_info("IR Value: %.2f", (3.3*getIrAnalogVal())/4095);
                break;
                
            case '0':
                getAnalogVal(0);
                break;
                
            case '1':
                getAnalogVal(1);                
                break;
                
            case 'a':
                getSpineBoardAdc(0);
                break;                    
                
            case 's':
                getSpineBoardAdc(1);
                break;
                
            case 'd':
                getSpineBoardAdc(2);
                break;
                
            case 'f':
                getSpineBoardAdc(3);
                break;   
                
            case 'r':
                // read RTC
                readZBoardMulti(I2C0_COM, 0x6F, 0x00, &incData, 1);
                DV_info("RTC Data: 0x%02X\r\n", incData);
                break;  
                
            case 't':
                // start RTC
                writeZBoardMulti(I2C0_COM, 0x6F, 0x00, &incData2, 1);
                break;                
                
            case 'q':
                // stop RTC
                writeZBoardMulti(I2C0_COM, 0x6F, 0x00, &incData3, 1);
                break;                

            case 'u':
                enableGLed();
                break;

            case 'i':
                disableGLed();
                break;

            case 'o':
                enableRLed();
                break;

            case 'p':
                disableRLed();
                break;                                                    
                
            case '8':
                if (testZBoardIntf() != SPI_SUCCESS) {
                    DV_info("SPI communication interface failed!");
                }
                else {
                    DV_info("SPI communication interface successful!");
                }
                break;   
                
            case 'l':
                initLDC();
                DV_info("Finished executing LDC1614 initialization routine");
                
                uint32_t testArray[4];
                //getCoilValues(testArray);
                DV_info("Finished getting coil values");       
                //for (uint8_t i = 0; i < 4; i++) {
                    //DV_info("coil[%i]: %lu\r\n", i, testArray[i]);
                //}
                break;
                                                               
            default:
                break;                
        }
    }


	///* Initialize shutdown external interrupt */
	//init_shdn_ext_irq();
	//
	//DV_info("--- Initializing hardware peripherals ---");
	///* Initialize timer for LT power supply sync signal */
	//init_ps_sync();
//
	///* Initialize I2C interface */
	//init_i2c();
	//
	///* Initialize ADC interface */
	//init_adc();
//
	///* Initialize SPI0 for POEM and configure interrupts */
	//init_spi_slave();

	/* Enable LT power supply sync signal */
	//start_ps_sync();						// Optional

	///* Pull UID and PLC firmware version from flash */
	//DV_info("--- Retrieving contents stored in flash ---");
	//uint32_t deviceId = read_uid();
    //DV_info("Device ID: %x", (unsigned int)deviceId);
    //
    //DV_info("--- Programming flash writer sector ---");
    //progFlashWriter();
//
	///* initialize random seed */
	//srand(deviceId);
//
	///* Initialize plc */
    //dv_plc_init(role, deviceId);
//
    //// init handler callbacks
    //dvhandler_init(role, deviceId);

	while (1) {
		/* Reset watchdog */
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;

		/* Blink led 0 */
		if (suc_led_swap) {
			suc_led_swap = 0;
            ioport_toggle_pin_level(NGLED_PIN);
		}

		/* Turn off led 1 after receiving PLC message */
		if (sb_ind_led_swap) {
			sb_ind_led_swap = false;
			#ifdef LED1_GPIO
            ioport_set_pin_level(NRLED_PIN, 1);     // turn off LED
			#endif
		}

		if (first_boot) {
			/* Determine whether tool is powered by battery */
			if (get_adc_avg() < 2.5) {
				DV_info("Battery voltage detected, powering on tool...");
				gpio_set_pin_low(NSTART);
			} else {
				DV_info("Waiting for power-on packet...");
			}
			first_boot = false;
		}

        //DV_info("Bytes read: %i", handleSpiRxData());

        // next steps:
        // read bytes from the plc interface
        // call dvhandler_bytesFromPlc();

		//dv_plc_handleRxData();
        //dvhandler_update();
        //dv_plc_handleTxData();
	}
}
