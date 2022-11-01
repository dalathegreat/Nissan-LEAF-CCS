
/*
CCS ADD-ON FIRMWARE
When fitted between the LIM Powertrain CAN and EV-CAN, this firmware allows for CCS charging.
 - Connect LIM to CAN1 (1B-6 CAN-H and 1B-7 CAN-L)
 - Connect EV-CAN to CAN2 (anywhere inside the LEAF, remove 120Ohm resistor?)
*/

/* Optional functionality */
//#define USB_SERIAL //NOTE INCREASES CPU LOAD DRASTICALLY
//#define ENABLE_CAN3 

#include "can-bridge-ccs.h"

//General variables
volatile	uint8_t		can_LIM				= 1;	//Define LIM-CAN-bus channel
volatile	uint8_t		can_EV				= 2;	//Define EV-CAN-bus channel
volatile	uint8_t		can_busy			= 0;	//Tracks whether the can_handler() subroutine is running
volatile	uint16_t	sec_timer			= 1;	//Counts down from 1000
volatile	uint8_t		ms_10_timer			= 0;	//Increments on every TCC0 overflow (every ms)
volatile	uint16_t	ms_100_timer		= 0;	//Increments on every TCC0 overflow (every ms)
volatile	uint16_t	ms_200_timer		= 0;	//Increments on every TCC0 overflow (every ms)
volatile	uint16_t	ms_600_timer		= 0;	//Increments on every TCC0 overflow (every ms)
volatile	uint16_t	ms_1000_timer		= 0;	//Increments on every TCC0 overflow (every ms)
volatile	uint8_t		message_10			= 0;	//time to send 100ms messages
volatile	uint8_t		message_100			= 0;	//time to send 100ms messages
volatile	uint8_t		message_200			= 0;	//time to send 200ms messages
volatile	uint8_t		message_600			= 0;	//time to send 600ms messages
volatile	uint8_t		message_1000		= 0;	//time to send 1000ms messages

volatile	uint32_t	LBC_voltage		= 0;
volatile	int16_t		LBC_current		= 0;
volatile	uint16_t	stateOfCharge	= 0;
volatile	uint16_t	Voltspnt		= 435;
volatile	uint8_t		shifter_state	= SHIFT_N;
volatile	uint16_t	LB_Full_Capacity_for_QC = 23000; //Default 24kWh LEAF, if we are on a 2013+ we autodetect capacity from 59E

//Data variables from LIM
volatile	uint16_t		Pilot_AC_Current			= 0;
volatile	uint16_t		Cable_Current				= 0;
volatile	uint8_t			PP_Status					= 0;
volatile	uint8_t			Pilot_Status				= 0;
volatile	uint8_t			Hook_Pos					= 0;
volatile	uint8_t			Hook_Lock					= 0;
volatile	uint16_t		FC_Contactor_State			= 0;
volatile	uint8_t			FC_Contactor_Test			= 0;
volatile	uint8_t			ChargeFlap_Status			= 0;
volatile	uint8_t			CCS_Protocol				= 0;
volatile	uint16_t		CHG_Volts					= 0;
volatile	uint16_t		CHG_Amps					= 0;
volatile	uint16_t		Min_V_Avail					= 0;
volatile	uint16_t		Min_I_Avail					= 0;
volatile	uint8_t			Energy_Transmitted			= 0;


//Data variables for LIM control
enum LIMstates
{
	Standby = 0x0,
	Initialisation = 0x1,
	Subpoena = 0x2,
	EnergyTransfer = 0x3,
	Shutdown = 0x4,
	CableTest = 0x9,
	Reserved = 0xE,
	InvalidSignal = 0xF,
	NotRdy=0x0,
	Init=0x1,
	Rdy=0x2,
	EndCharge = 0x0,
	Charge = 0x1,
	ChargeNotRdy = 0x0,
	ChargeRdy = 0x1
};


volatile	uint8_t			CP_Mode=0;
volatile	uint8_t			Chg_Phase=Standby;
volatile	uint8_t			lim_state=0;
volatile	uint8_t			lim_stateCnt=0;
volatile	uint8_t			ctr_1second=0;
volatile	uint8_t			ctr_5second=0;
volatile	uint8_t			ctr_20ms=0;
volatile	uint8_t			vin_ctr=0;
volatile	uint8_t			Timer_1Sec=0;
volatile	uint8_t			Timer_60Sec=0;
volatile	uint8_t			ChargeType=0;
volatile	uint8_t			CCS_Plim=0;//ccs power limit flag. 0=no,1=yes,3=invalid.
volatile	uint8_t			CCS_Ilim=0;//ccs current limit flag. 0=no,1=yes,3=invalid.
volatile	uint8_t			CCS_Vlim=0;//ccs voltage limit flag. 0=no,1=yes,3=invalid.
volatile	uint8_t			CCS_Stat=0;//ccs charging status. 0=standby,1=charging,3=invalid.
volatile	uint8_t			CCS_Malf=0;//ccs malfunction status. 0=normal,1=fail,3=invalid.
volatile	uint8_t			CCS_Bmalf=0;//ccs battery malfunction status. 0=no,1=yes,3=invalid.
volatile	uint8_t			CCS_Stop=0;//ccs chargeing stop status. 0=tracking,1=supression,3=invalid.
volatile	uint8_t			CCS_Iso=0;//ccs isolation status. 0=invalid,1=valid,2=error,3=invalid signal.
volatile	uint8_t			CCS_IntStat=0;//ccs charger internal status. 0=not ready,1=ready,2=switch off charger,3=interruption,4=pre charge,5=insulation monitor,6=estop,7=malfunction,0x13=reserved,0x14=reserved,0x15=invlaid signal.
volatile	uint32_t		sec_328=0;
volatile	uint16_t		Cont_Volts=0;
volatile	uint16_t		Bulk_SOCt=0;//Time to bulk soc target.
volatile	uint16_t		Full_SOCt=0;//Time to full SOC target.
volatile	uint32_t		CHG_Pwr=0; //calculated charge power. 12 bit value scale x25. Values based on 50kw DC fc and 1kw and 3kw ac logs. From bms???
volatile	int16_t			FC_Cur=0; //10 bit signed int with the ccs dc current command.scale of 1.
volatile	uint8_t			EOC_Time=0x00; //end of charge time in minutes.
volatile	uint8_t			CHG_Status=NotRdy;  //observed values 0 when not charging , 1 and transition to 2 when commanded to charge. only 4 bits used.
												//seems to control led colour.
volatile	uint8_t			CHG_Req=EndCharge;  //observed values 0 when not charging , 1 when requested to charge. only 1 bit used in logs so far.
volatile	uint8_t			CHG_Ready=ChargeNotRdy;  //indicator to the LIM that we are ready to charge. observed values 0 when not charging , 1 when commanded to charge. only 2 bits used.
volatile	uint8_t			CONT_Ctrl=0;  //4 bits with DC ccs contactor command.
volatile	uint8_t			CCSI_Spnt=0;
volatile	uint16_t		CCS_V_Avail = 0;
volatile	uint16_t		CCS_I_Avail = 0;

//CAN messages for LIM
volatile	can_frame_t		BMS_112_message	= {.can_id = 0x112, .can_dlc = 8, .data = {0xF9,0x1F,0x8B,0x0E,0xA6,0x71,0x65,0x5D}};
volatile	can_frame_t		VCU_2FC_message	= {.can_id = 0x2FC, .can_dlc = 8, .data = {0x81,0x00,0x04,0xFF,0xFF,0xFF,0xFF,0xFF}};
static		can_frame_t		VCU_1A1_message = {.can_id = 0x1A1, .can_dlc = 5, .data = {0x7C,0xCB,0x00,0x00,0x8A}}; //Vehicle speed msg
volatile	can_frame_t		VCU_3E9_message	= {.can_id = 0x3E9, .can_dlc = 8, .data = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}};
volatile	can_frame_t		VCU_2F1_message	= {.can_id = 0x2F1, .can_dlc = 8, .data = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xA0}};
volatile	can_frame_t		VCU_2FA_message	= {.can_id = 0x2FA, .can_dlc = 8, .data = {0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00}};
volatile	can_frame_t		VCU_432_message	= {.can_id = 0x432, .can_dlc = 8, .data = {0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00}};
volatile	can_frame_t		VCU_51A_message	= {.can_id = 0x51A, .can_dlc = 8, .data = {0x00,0x00,0x00,0x00,0x50,0x00,0x00,0x1A}};
volatile	can_frame_t		VCU_540_message	= {.can_id = 0x540, .can_dlc = 8, .data = {0x00,0x00,0x00,0x00,0xFD,0x3C,0xFF,0x40}};
volatile	can_frame_t		VCU_510_message	= {.can_id = 0x510, .can_dlc = 8, .data = {0x40,0x10,0x20,0x00,0x00,0x00,0x00,0x00}};
volatile	can_frame_t		VCU_328_message	= {.can_id = 0x328, .can_dlc = 6, .data = {0x00,0x00,0x00,0x00,0x87,0x1e}};
volatile	can_frame_t		VCU_3E8_message	= {.can_id = 0x3E8, .can_dlc = 2, .data = {0xF1,0xFF}};
volatile	can_frame_t		VCU_3F9_message	= {.can_id = 0x3F9, .can_dlc = 8, .data = {0xC0,0xF9,0x80,0xE0,0x43,0x3C,0xC3,0xFF}};	
volatile	can_frame_t		VCU_03C_message	= {.can_id = 0x03C, .can_dlc = 8, .data = {0xFF,0x5F,0x00,0x00,0x00,0x00,0xFF,0xFF}};
volatile	can_frame_t		VCU_431_message	= {.can_id = 0x431, .can_dlc = 8, .data = {0xca,0xff,0x0b,0x02,0x69,0x26,0xF3,0x4B}};	
volatile	can_frame_t		VCU_12f_message	= {.can_id = 0x12f, .can_dlc = 8, .data = {0xf5,0x28,0x8a,0x1d,0xf1,0x35,0x30,0x80}};	






//Because the MCP25625 transmit buffers seem to be able to corrupt messages (see errata), we're implementing
//our own buffering. This is an array of frames-to-be-sent, FIFO. Messages are appended to buffer_end++ as they
//come in and handled according to buffer_pos until buffer_pos == buffer_end, at which point both pointers reset
//the buffer size should be well in excess of what this device will ever see
can_frame_t tx0_buffer[TXBUFFER_SIZE];
uint8_t		tx0_buffer_pos		= 0;
uint8_t		tx0_buffer_end		= 0;

can_frame_t tx2_buffer[TXBUFFER_SIZE];
uint8_t		tx2_buffer_pos		= 0;
uint8_t		tx2_buffer_end		= 0;

can_frame_t tx3_buffer[5]; 
uint8_t		tx3_buffer_pos		= 0;
uint8_t		tx3_buffer_end		= 0;

#ifdef USB_SERIAL
#include "usb-hub-sensor.h"
uint8_t ReadCalibrationByte( uint8_t index );
void ProcessCDCCommand(void);

uint8_t		configSuccess				= false;		//tracks whether device successfully enumerated
static FILE USBSerialStream;							//fwrite target for CDC
uint8_t		signature[11];								//signature bytes
//print variables
volatile	uint8_t		print_char_limit		= 0;
#endif

void hw_init(void){
	uint8_t caninit;

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, 48000000);		
	
	//turn off everything we don' t use
	PR.PRGEN		= PR_AES_bm | PR_RTC_bm | PR_DMA_bm;
	PR.PRPA			= PR_ADC_bm | PR_AC_bm;
	PR.PRPC			= PR_TWI_bm | PR_USART0_bm | PR_HIRES_bm;
	PR.PRPD			= PR_TWI_bm | PR_USART0_bm | PR_TC0_bm | PR_TC1_bm;
	PR.PRPE			= PR_TWI_bm | PR_USART0_bm;
	
	//blink output
	PORTB.DIRSET	= 3;
	
	//start 16MHz crystal and PLL it up to 48MHz
	OSC.XOSCCTRL	= OSC_FRQRANGE_12TO16_gc |		//16MHz crystal
	OSC_XOSCSEL_XTAL_16KCLK_gc;						//16kclk startup
	OSC.CTRL	   |= OSC_XOSCEN_bm;				//enable crystal
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));			//wait until ready
	OSC.PLLCTRL		= OSC_PLLSRC_XOSC_gc | 2;		//XTAL->PLL, 2x multiplier (32MHz)
	OSC.CTRL	   |= OSC_PLLEN_bm;					//start PLL
	while (!(OSC.STATUS & OSC_PLLRDY_bm));			//wait until ready
	CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
	CLK.CTRL		= CLK_SCLKSEL_PLL_gc;			//use PLL output as system clock	
	
	//output 16MHz clock to MCP25625 chips (PE0)
	//next iteration: put this on some other port, pin  4 or 7, so we can use the event system
	TCE0.CTRLA		= TC0_CLKSEL_DIV1_gc;						//clkper/1
	TCE0.CTRLB		= TC0_CCAEN_bm | TC0_WGMODE_SINGLESLOPE_bm;	//enable CCA, single-slope PWM
	TCE0.CCA		= 1;										//compare value
	TCE0.PER		= 1;										//period of 1, generates 24MHz output
	
	PORTE.DIRSET	= PIN0_bm;									//set CLKOUT pin to output
	
	//setup CAN pin interrupts
	PORTC.INTCTRL	= PORT_INT0LVL_HI_gc;
	PORTD.INTCTRL	= PORT_INT0LVL_HI_gc | PORT_INT1LVL_HI_gc;	
	
	PORTD.INT0MASK	= PIN0_bm;						//PORTD0 has can1 interrupt
	PORTD.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	PORTD.INT1MASK	= PIN5_bm;						//PORTD5 has can2 interrupt
	PORTD.PIN5CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	#ifdef ENABLE_CAN3
	PORTC.INT0MASK	= PIN2_bm;						//PORTC2 has can3 interrupt
	PORTC.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	#endif
	
	//buffer checking interrupt
	TCC1.CTRLA		= TC0_CLKSEL_DIV1_gc;			//32M/1/3200 ~ 100usec
	TCC1.PER		= 3200;
	TCC1.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//same priority as can interrupts
	
	//we want to optimize performance, so we're going to time stuff
	//48MHz/48=1us timer, which we just freerun and reset whenever we want to start timing something
	//frame time timer
	TCC0.CTRLA		= TC0_CLKSEL_DIV1_gc;
	TCC0.PER		= 32000;						//32MHz/32000=1ms
	TCC0.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//interrupt on overflow
	
	PORTB.OUTCLR	= (1 << 0);
	
	can_system_init:
			
	//Init SPI and CAN interface:
	if(RST.STATUS & RST_WDRF_bm){ //if we come from a watchdog reset, we don't need to setup CAN
		caninit = can_init(MCP_OPMOD_NORMAL, 1); //on second thought, we do
	} else {
		caninit = can_init(MCP_OPMOD_NORMAL, 1);
	}
	
	if(caninit){		
		//PORTB.OUTSET |= (1 << 0);					//green LED, uncommented to save power
	} else {		
		//PORTB.OUTSET |= (1 << 1);					//red LED, uncommented to save power
		_delay_ms(10);
		goto can_system_init;
	}
	
	//Set and enable interrupts with round-robin
	XMEGACLK_CCP_Write((void * ) &PMIC.CTRL, PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);//PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm| PMIC_HILVLEN_bm;
	
	USB_Init(USB_OPT_RC32MCLKSRC | USB_OPT_BUSEVENT_PRILOW);
	#ifdef USB_SERIAL
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
	#endif
	
	wdt_enable(WDTO_15MS);
	
	sei();
}

int main(void){
	#ifdef USB_SERIAL
	char * str = "";
	#endif
	
	hw_init();

	while(1){
		//Setup complete, wait for CAN messages to trigger interrupts OR check if it is time to send CAN-messages
		if(message_10){ //Send 10ms CAN-messages
			int16_t V_Batt = LBC_voltage * 10;
			uint8_t V_Batt2= LBC_voltage / 4;
			int32_t I_Batt= (LBC_current+819)*10; //dala todo, did the conversion go right from LBC -> LIM?
			uint16_t SOC_Local=stateOfCharge*10;
			BMS_112_message.data[0] = I_Batt & 0xFF; //Battery current LSB. Scale 0.1 offset 819.2. 16 bit unsigned int
			BMS_112_message.data[1] = I_Batt >> 8;	 //Battery current MSB. Scale 0.1 offset 819.2.  16 bit unsigned int
			BMS_112_message.data[2] = V_Batt & 0xFF; //Battery voltage LSB. Scale 0.1. 16 bit unsigned int.
			BMS_112_message.data[3] = V_Batt >> 8;   //Battery voltage MSB. Scale 0.1. 16 bit unsigned int.
			BMS_112_message.data[4] = SOC_Local & 0xFF; //Battery SOC LSB. 12 bit unsigned int. Scale 0.1. 0-100%
			BMS_112_message.data[5] = SOC_Local >> 8; //Battery SOC MSB. 12 bit unsigned int. Scale 0.1. 0-100%
			BMS_112_message.data[6] = 0x65;			 //Low nibble battery status. Seem to need to be 0x5.
			BMS_112_message.data[7] = V_Batt2;		 //DC-link voltage. (set to Battery voltage). Scale 4. 8 bit unsigned int.
			send_can(can_LIM, BMS_112_message);
			message_10 = 0;
			
			
			ctr_20ms++;
			if(ctr_20ms==2)
			{
				ctr_20ms=0;
				send_can(can_LIM, VCU_1A1_message); //Vehicle speed msg should be sent every 20ms
			}
			
		}
		if(message_100){
			send_can(can_LIM, VCU_03C_message);

			uint16_t Wh_Local = LB_Full_Capacity_for_QC;
			CHG_Pwr=(CHG_Pwr & 0xFFF);
			VCU_3E9_message.data[0] = Wh_Local & 0xFF;  //Battery Wh lowbyte
			VCU_3E9_message.data[1] = Wh_Local >> 8;  //BAttery Wh high byte
			VCU_3E9_message.data[2] = (((uint8_t)CHG_Status<<4)|((uint8_t)CHG_Req));  //charge status in bits 4-7.goes to 1 then 2.8 secs later to 2. Plug locking???. Charge request in lower nibble. 1 when charging. 0 when not charging.
			VCU_3E9_message.data[3] = (((CHG_Pwr)<<4)|(uint8_t)CHG_Ready);  //charge readiness in bits 0 and 1. 1 = ready to charge.upper nibble is LSB of charge power.Charge power forecast not actual power!
			VCU_3E9_message.data[4] = CHG_Pwr>>4;   //MSB of charge power.in this case 0x28 = 40x25 = 1000W. Probably net DC power into the Batt.
			VCU_3E9_message.data[5] = FC_Cur & 0xff;   //LSB of the DC ccs current command
			VCU_3E9_message.data[6] = ((CONT_Ctrl<<4)|(FC_Cur>>12));   //bits 0 and 1 MSB of the DC ccs current command.Upper nibble is DC ccs contactor control. Observed in DC fc logs only.
			//transitions from 0 to 2 and start of charge but 2 to 1 to 0 at end. Status and Ready operate the same as in AC logs.
			VCU_3E9_message.data[7] = EOC_Time;    // end of charge timer.
			send_can(can_LIM, VCU_3E9_message); //average 128ms? well its gonna be 100ms now

			//LIM needs to see this but doesnt control anything...
			send_can(can_LIM, VCU_431_message); // average 197ms but as low as 49ms.

			//if(Param::GetInt(Param::opmode)==MOD_RUN) bytes[2] = 0x8a;//ignition on
			//if(Param::GetInt(Param::opmode)!=MOD_RUN) bytes[2] = 0x86;//ignition off 86
			//We are always in ignition ON on this device (for future revisions, consider turning off the LIM if its gonna be on much)
			send_can(can_LIM, VCU_12f_message); // average 100ms

			//Lim command 2. Used in DC mode
			uint16_t V_limit=0;
			//if(lim_state==6) V_limit=401*10;//set to 400v in energy transfer state
			//if(lim_state!=6) V_limit=Param::GetInt(Param::udc)*10;
			if(lim_state==4) V_limit=LBC_voltage*10;// drop vlim only during precharge
			else V_limit=415*10;//set to 415v in all other states
			uint8_t I_limit=125;//125A limit. may not work
			VCU_2F1_message.data[0] = V_limit & 0xFF;  //Charge voltage limit LSB. 14 bit signed int.scale 0.1 0xfa2=4002*.1=400.2Volts
			VCU_2F1_message.data[1] = V_limit >> 8;  //Charge voltage limit MSB. 14 bit signed int.scale 0.1
			VCU_2F1_message.data[2] = I_limit;  //Fast charge current limit. Not used in logs from 2014-15 vehicle so far. 8 bit unsigned int. scale 1.so max 254amps in theory...
			VCU_2F1_message.data[3] = Full_SOCt & 0xFF;  //time remaining in seconds to hit soc target from byte 7 in AC mode. LSB. 16 bit unsigned int. scale 10.Full SOC.
			VCU_2F1_message.data[4] = Full_SOCt >> 8;  //time remaining in seconds to hit soc target from byte 7 in AC mode. MSB. 16 bit unsigned int. scale 10.Full SOC.
			VCU_2F1_message.data[5] = Bulk_SOCt & 0xFF;  //time remaining in seconds to hit soc target from byte 7 in ccs mode. LSB. 16 bit unsigned int. scale 10.Bulk SOC.
			VCU_2F1_message.data[6] = Bulk_SOCt >> 8;  //time remaining in seconds to hit soc target from byte 7 in ccs mode. MSB. 16 bit unsigned int. scale 10.Bulk SOC.
			VCU_2F1_message.data[7] = 0xA0;  //Fast charge SOC target. 8 bit unsigned int. scale 0.5. 0xA0=160*0.5=80%
			send_can(can_LIM, VCU_2F1_message); // average 100ms

			message_100 = 0;
		}
		if(message_200){
			//send_can(can_LIM, VCU_3E9_message);
			send_can(can_LIM, VCU_431_message);
			ControlCharge(); //Control the LIM
			
			if(CP_Mode==0x4||CP_Mode==0x5)
			{
				VCU_2FA_message.data[0] = 0xFC; //change to FC on 5% pilot. Change back to FD during energy transfer
			}			    
			else
			{
				VCU_2FA_message.data[0] = 0xFD; //FD at standby
			}

			VCU_2FA_message.data[1] = 0xFF;//these bytes are used as a timer during energy transfer but not at setup
			VCU_2FA_message.data[2] = (uint8_t)Chg_Phase<<4;  //upper nibble seems to be a mode command to the ccs station. 0 when off, 9 when in constant current phase of cycle.
			//more investigation needed here...
			//Lower nibble seems to be intended for two end charge commands each of 2 bits.
			VCU_2FA_message.data[4] = 0xff;
			VCU_2FA_message.data[5] = 0xff;
			VCU_2FA_message.data[6] = 0xff;
			VCU_2FA_message.data[7] = 0xff;
			send_can(can_LIM, VCU_2FA_message); //Lim command 3. Used in DC mode.
			// this msg varies from 82ms to 1s intervals. Sending it at 200ms as compromise, seems to work on Damiens VCU.
			
			uint16_t SOC_Local2=stateOfCharge*2;
			VCU_432_message.data[0] = 0x2c;//BMS SOC msg. May need to be dynamic
			VCU_432_message.data[1] = 0xe2;
			VCU_432_message.data[2] = 0x10;
			VCU_432_message.data[3] = 0xa3;
			VCU_432_message.data[4] = SOC_Local2;    //display soc. scale 0.5.
			VCU_432_message.data[5] = 0xff;
			VCU_432_message.data[6] = 0x02;
			VCU_432_message.data[7] = 0xff;
			send_can(can_LIM, VCU_432_message); 
			
			message_200 = 0;
		}
		if(message_600){
			send_can(can_LIM, VCU_51A_message); //network management
			send_can(can_LIM, VCU_540_message); //average 640ms
			send_can(can_LIM, VCU_510_message); //average 640ms
			message_600 = 0;
		}
		if(message_1000){
			sec_328++; //increment seconds counter. We don't care about overruns, since we reset the hardware after each charge
			VCU_328_message.data[0] = sec_328;//rtc msg. needs to be every 1 sec. first 32 bits are 1 second wrap counter
			VCU_328_message.data[1] = sec_328<<8;
			VCU_328_message.data[2] = sec_328<<16;
			VCU_328_message.data[3] = sec_328<<24;
			send_can(can_LIM, VCU_328_message);
			
			send_can(can_LIM, VCU_3E8_message);
			
			send_can(can_LIM, VCU_3F9_message);
			
			message_1000 = 0;
		}
		#ifdef USB_SERIAL
		//when USB is essentially unused, we output general status info
		if(!output_can_to_serial){
			if(sec_interrupt){
				sec_interrupt = 0;
			
				/*//current shifter state
				str = "Shift: 00\n";
				int_to_hex((char *) (str + 7), shifter_state);
				print(str, 10);*/
			}
		}
		#endif
	}
}

void ControlCharge(void)
{
    if (PP_Status && (CP_Mode==0x1||CP_Mode==0x2))  //if we have an enable and a plug in and a std ac pilot lets go AC charge mode.
    {	
		//Since we are not interested in LIM AC charging, code for this deleted.
    }


    if (PP_Status &&(CP_Mode==0x4||CP_Mode==0x5||CP_Mode==0x6))  //if we have an enable and a plug in and a 5% pilot or a static pilot lets go DC charge mode.
    {
        /*
        0=no pilot
        1=10-96%PWM not charge ready
        2=10-96%PWM charge ready
        3=error
        4=5% not charge ready
        5=5% charge ready
        6=pilot static
        */

        //Param::SetInt(Param::CCS_State,lim_state);//update state machine level on webui
        switch(lim_state)
        {

        case 0:
        {
			Chg_Phase=Standby;
			CONT_Ctrl=0x0; //dc contactor mode control required in DC
			FC_Cur=0;//ccs current request from web ui for now.
			EOC_Time=0x00;//end of charge timer
			CHG_Status=Init;
			CHG_Req=Charge;
			CHG_Ready=ChargeNotRdy;
			CHG_Pwr=0;//0 power
			CCSI_Spnt=0;//No current
			//if(CP_Mode==0x4 && opmode==MOD_CHARGE) lim_state++;
			lim_stateCnt++; //increment state timer counter
			if(lim_stateCnt>20)//2 second delay
			{
				lim_state++; //next state after 2 secs
				lim_stateCnt=0;
			}

        }
        break;

        case 1:
        {
        //uint16_t I_avail_tmp=Param::GetInt(Param::CCS_I_Avail);
        Chg_Phase=Initialisation;
        CONT_Ctrl=0x0; //dc contactor mode control required in DC
        FC_Cur=0;//ccs current request from web ui for now.
        EOC_Time=0x00;//end of charge timer
        CHG_Status=Init;
        CHG_Req=Charge;
        CHG_Ready=ChargeNotRdy;
        CHG_Pwr=0;//0 power
        CCSI_Spnt=0;//No current
        if(CP_Mode==0x6) lim_state=0; //Reset to state 0 if we get a static pilot
        //if(I_avail_tmp>10 && I_avail_tmp<500) lim_stateCnt++;

        if(ChargeType==0x09) lim_stateCnt++;
        if(lim_stateCnt>25)//2 secs efacec critical! 20 works. 50 does not.
        {
            lim_state++; //next state after 4 secs
            lim_stateCnt=0;
        }

        }
        break;

        case 2:
        {
        //
        Chg_Phase=CableTest;
        CONT_Ctrl=0x0; //dc contactor mode control required in DC
        FC_Cur=0;//ccs current request from web ui for now.
        EOC_Time=0x1E;//end of charge timer 30 mins
        Bulk_SOCt=1800; //Set bulk SOC timer to 30 minutes.
        Full_SOCt=2400; //Set full SOC timer to 40 minutes.
        Timer_1Sec=5;   //Load the 1 second loop counter. 5 loops=1sec.
        Timer_60Sec=60;   //Load the 60 second loop counter. 5 loops=1sec.
        CHG_Status=Init;
        CHG_Req=Charge;
        CHG_Ready=ChargeRdy;
        CHG_Pwr=44000/25;//44kw approx power
        CCSI_Spnt=0;//No current
        if(Cont_Volts>0)lim_state++; //we wait for the contactor voltage to rise before hitting next state.

        }
        break;

        case 3:
        {
        //I don't like this state CableTest here. Should it remain in Initialisation ....
        Chg_Phase=CableTest;
        CONT_Ctrl=0x0; //dc contactor mode control required in DC
        FC_Cur=0;//ccs current request from web ui for now.
// EOC_Time=0x1E;//end of charge timer
        CHG_Status=Init;
        CHG_Req=Charge;
        CHG_Ready=ChargeRdy;
        CHG_Pwr=44000/25;//39kw approx power
        CCSI_Spnt=0;//No current

        if(Cont_Volts<=50)lim_stateCnt++; //we wait for the contactor voltage to return to 0 to indicate end of cable test
        if(lim_stateCnt>20)
        {
            if(CCS_Iso==0x1) lim_state++; //next state after 2 secs if we have valid iso test
            lim_stateCnt=0;
        }

        }
        break;

        case 4:
        {
			Chg_Phase = Subpoena; //precharge phase in this state
			CONT_Ctrl = 0x0;                   //dc contactor mode control required in DC
			FC_Cur = 0;                        //ccs current request from web ui for now.
			// EOC_Time=0x1E;//end of charge timer
			CHG_Status = Init;
			CHG_Req = Charge;
			CHG_Ready = ChargeRdy;
			CHG_Pwr = 44000 / 25; //49kw approx power
			CCSI_Spnt = 0;        //No current

			if ((LBC_voltage - Cont_Volts) < 20)
			{
				lim_stateCnt++; //we wait for the contactor voltage to be 20v or less diff to main batt v
			}
			else
			{
				// If the contactor voltage wanders out of range start again
				lim_stateCnt = 0;
			}

			// Wait for contactor voltage to be stable for 2 seconds
			if (lim_stateCnt > 20)
			{
				lim_state++; //next state after 2 secs
				lim_stateCnt = 0;
			}
        }
        break;
        case 5:
        {
			//precharge phase in this state but voltage close enough to close contactors
			Chg_Phase = Subpoena;
			CONT_Ctrl = 0x2;                   //dc contactor closed
			FC_Cur = 0;                        //ccs current request from web ui for now.
			// EOC_Time=0x1E;//end of charge timer
			CHG_Status = Init;
			CHG_Req = Charge;
			CHG_Ready = ChargeRdy;
			CHG_Pwr = 44000 / 25; //49kw approx power
			CCSI_Spnt = 0;        //No current

			// Once the contactors report as closed we're OK to proceed to energy transfer
			if (FC_Contactor_State == 1) 
			{
				lim_state++;
			}
        }
        break;

        case 6:
        {
			Chg_Phase=EnergyTransfer;
			CONT_Ctrl=0x2; //dc contactor to close mode
			//FC_Cur=Param::GetInt(Param::CCS_ICmd);//ccs manual control
			FC_Cur=CCSI_Spnt;//Param::GetInt(Param::CCS_ICmd);//ccs auto ramp
			CCS_Pwr_Con(); //CCS power control subroutine
			Chg_Timers();   //Handle remaining time timers.
		//  EOC_Time=0x1E;//end of charge timer
			CHG_Status=Rdy;
			CHG_Req=Charge;
			CHG_Ready=ChargeRdy;
			CHG_Pwr=44000/25;//49kw approx power
			//we chill out here charging.

			if(CCS_IntStat==0x02)//if we have a request to terminate from the EVSE then move to next state.
			{ //Dala, note, sketchy rewrite of above line
				FC_Cur=0;//set current to 0
				lim_state++; //move to state 7 (shutdown)
			}

        }
        break;

        case 7:    //shutdown state
        {
			Chg_Phase=Shutdown;
			CONT_Ctrl=0x2; //dc contactor to close mode
			FC_Cur=0;//current command to 0
			EOC_Time=0x1E;//end of charge timer
			CHG_Status=Init;
			CHG_Req=Charge;
			CHG_Ready=Rdy;
			CHG_Pwr=44000/25;//49kw approx power
			lim_stateCnt++;
			if(lim_stateCnt>10) //wait 2 seconds
			{
				lim_state++; //next state after 2 secs
				lim_stateCnt=0;
			}
        }
        break;

        case 8:    //shutdown state
        {
			Chg_Phase=Shutdown;
			CONT_Ctrl=0x1; //dc contactor to open with diag mode
			FC_Cur=0;//current command to 0
			EOC_Time=0x1E;//end of charge timer
			CHG_Status=Init;
			CHG_Req=Charge;
			CHG_Ready=ChargeNotRdy;
			CHG_Pwr=44000/25;//49kw approx power
			lim_stateCnt++;
			if(Cont_Volts==0)lim_stateCnt++; //we wait for the contactor voltage to return to 0 to indicate contactors open
			if(lim_stateCnt>10)
			{
				lim_state++; //next state after 2 secs
				lim_stateCnt=0;
			}

        }
        break;

        case 9:    //shutdown state
        {
			Chg_Phase=Standby;
			CONT_Ctrl=0x0; //dc contactor to open mode
			FC_Cur=0;//current command to 0
			EOC_Time=0x1E;//end of charge timer
			CHG_Status=Init;
			CHG_Req=EndCharge;
			CHG_Ready=ChargeNotRdy;
			CHG_Pwr=0;//0 power
        }
        break;

        }
	}



    if (!PP_Status | !(shifter_state == SHIFT_P))  //if we remove plug OR vehicle is no longer in park , shut down
    {
        lim_state=0;//return to state 0
        //Param::SetInt(Param::CCS_State,lim_state);
        Chg_Phase=Standby;
        CONT_Ctrl=0x0; //dc contactor mode 0 in off
        FC_Cur=0;//ccs current request zero
        EOC_Time=0x00;
        CHG_Status=NotRdy;
        CHG_Req=EndCharge;
        CHG_Ready=ChargeNotRdy;
        CHG_Pwr=0;
    }
}

void CCS_Pwr_Con(void)    //here we control CCS charging during state 6.
{
	uint16_t Tmp_Vbatt = LBC_voltage;//Actual measured battery voltage by LBC
	uint16_t Tmp_Vbatt_Spnt = Voltspnt; //Target voltage
	uint16_t Tmp_ICCS_Lim = CCS_Ilim;
	uint16_t Tmp_ICCS_Avail = CCS_I_Avail;
	//int16_t Tmp_Ibatt=Param::GetInt(Param::idc);

	if(CCSI_Spnt>Tmp_ICCS_Lim)CCSI_Spnt=Tmp_ICCS_Lim; //clamp setpoint to current lim paramater.
	if(CCSI_Spnt>150)CCSI_Spnt=150; //never exceed 150amps for now.
	if(CCSI_Spnt>=Tmp_ICCS_Avail)CCSI_Spnt=Tmp_ICCS_Avail; //never exceed available current
	if(CCSI_Spnt>250)CCSI_Spnt=0; //crude way to prevent rollover
	if((Tmp_Vbatt<Tmp_Vbatt_Spnt)&&(CCS_Ilim==0x0)&&(CCS_Plim==0x0))CCSI_Spnt++;//increment if voltage lower than setpoint and power and current limts not set from charger.
	if(Tmp_Vbatt>Tmp_Vbatt_Spnt)CCSI_Spnt--;//decrement if voltage equal to or greater than setpoint.
	if(CCS_Ilim==0x1)CCSI_Spnt--;//decrement if current limit flag is set
	if(CCS_Plim==0x1)CCSI_Spnt--;//decrement if Power limit flag is set

	// force once more that we stay within our maximum bounds
	if(CCSI_Spnt>=Tmp_ICCS_Avail)CCSI_Spnt=Tmp_ICCS_Avail; //never exceed available current
	if(CCSI_Spnt>Tmp_ICCS_Lim)CCSI_Spnt=Tmp_ICCS_Lim; //clamp setpoint to current lim paramater.

	//Param::SetInt(Param::CCS_Ireq,CCSI_Spnt); //uncommented, webui?
}

void Chg_Timers(void)
{
	Timer_1Sec--;   //decrement the loop counter

	if(Timer_1Sec==0)   //1 second has elapsed
	{
		Timer_1Sec=5;
		Bulk_SOCt--;    //Decrement timers. Just on time for now will be current based in final version
		Full_SOCt--;
		Timer_60Sec--;  //decrement the 1 minute counter
		if(Timer_60Sec==0)
		{
			Timer_60Sec=60;
			EOC_Time--;    //decrement end of charge minutes timer
		}
	}
}

#ifdef USB_SERIAL
/* services commands received over the virtual serial port */
void ProcessCDCCommand(void)
{
	uint16_t	ReportStringLength = 0;
	char *		ReportString = "";
	int16_t cmd = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	
	if(cmd > -1){
		switch(cmd){
			case 48: //0
			break;
			
			case 0: //reset
			case 90: //'Z'
			_delay_ms(1000);
			CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
			RST.CTRL		= RST_SWRST_bm;
			break;
			
			case 255: //send identity
			ReportString = "DALA CAN bridge - v2.5 Leaf\n"; ReportStringLength = 28;
			break;
			
			default: //when all else fails
			ReportString = "Unrecognized Command:   \n"; ReportStringLength = 25;
			ReportString[22] = cmd;
			break;
		}
		
		if(ReportStringLength){
			print(ReportString, ReportStringLength);
		}
		
	}
}
#endif
// Event handler for the LUFA library USB Disconnection event.
void EVENT_USB_Device_Disconnect(void){}

void EVENT_USB_Device_Connect(void){}
#ifdef USB_SERIAL
// Event handler for the LUFA library USB Configuration Changed event.
void EVENT_USB_Device_ConfigurationChanged(void){ configSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface); }

// Event handler for the LUFA library USB Control Request reception event.
void EVENT_USB_Device_ControlRequest(void){	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface); }

//appends string to ring buffer and initiates transmission
void print(char * str, uint8_t len){
	if((print_char_limit + len) <= 120){
		fwrite(str, len, 1, &USBSerialStream);
		print_char_limit += len;
	} else {
		fwrite("X\n",2,1,&USBSerialStream);
	}
}
#endif

//fires every 1ms
ISR(TCC0_OVF_vect){	
	wdt_reset(); //Reset the watchdog
	sec_timer--; //Increment the 1000ms timer
	ms_10_timer++;
	ms_100_timer++;
	ms_200_timer++;
	ms_600_timer++;
	ms_1000_timer++;
	#ifdef USB_SERIAL
	if(!can_busy) ProcessCDCCommand();
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
	//handle second print buffer
	if(print_char_limit <= 64) { print_char_limit = 0; }
	else { print_char_limit -= 64; }
	#endif
	
	if(ms_10_timer == 10) 
	{
		message_10 = 1;
		ms_10_timer = 0;
	}
	if(ms_100_timer == 100)
	{
		message_100 = 1;
		ms_100_timer = 0;
	}
	if(ms_200_timer == 200)
	{
		message_200 = 1;
		ms_200_timer = 0;
	}
	if(ms_600_timer == 600)
	{
		message_600 = 1;
		ms_600_timer = 0;
	}
	if(ms_1000_timer == 1000)
	{
		message_1000 = 1;
		ms_1000_timer = 0;
	}
	
	//fires every second (1000ms tasks go here)
	if(sec_timer == 0){
		PORTB.OUTCLR = (1 << 1);
	}
}

//fires approx. every 100us
ISR(TCC1_OVF_vect){
	check_can1();
	check_can2();
	check_can3();
}

//can1 interrupt
ISR(PORTD_INT0_vect){
	can_busy = 1;
	can_handler(1);
}

//can2 interrupt
ISR(PORTD_INT1_vect){
	can_busy = 1;
	can_handler(2);
}

//can3 receive interrupt
ISR(PORTC_INT0_vect){
	can_busy = 1;
	can_handler(3);
}

//CAN handler, manages reading data from received CAN messages 
void can_handler(uint8_t can_bus){
	can_frame_t frame;
	uint16_t temp_amps;
	uint8_t flag = can_read(MCP_REG_CANINTF, can_bus);
		
	if (flag & (MCP_RX0IF | MCP_RX1IF)){

		if(flag & MCP_RX1IF){ //prioritize the rollover buffer
			can_read_rx_buf(MCP_RX_1, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX1IF, 0x00, can_bus);
			} else {
			can_read_rx_buf(MCP_RX_0, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX0IF, 0x00, can_bus);
		}
		
		switch(frame.can_id){
			case 0x59E:
				//We can take battcap from here on 2013+ LEAFs
				LB_Full_Capacity_for_QC = ((frame.data[2] & 0x1F) | (frame.data[3] & 0xF0));

			break;
			case 0x11A:
				shifter_state = (frame.data[0] & 0xF0);
				// What happens when PP is grounded while car is ON? Can we prevent car from moving that way?
				// can we send 0x60# PLUG inserted?
			break;
			case 0x1DB:
				LBC_voltage = (frame.data[2] << 2) | ((frame.data[3] & 0xC0) >> 6);	//extract voltage
				LBC_voltage = (LBC_voltage/2);
				
				temp_amps = ((frame.data[2] & 0x07) << 8) + frame.data[3];		//extract battery current from frame data
				if(temp_amps > 1024)
				{
					LBC_current = (int16_t) -2048 + (int16_t) temp_amps;			//convert from twos complement to native signed
				}		
				else
				{ 
					LBC_current = (int16_t) temp_amps;
				}				
			break;
			case 0x55B:
				stateOfCharge = (frame.data[0] << 2) | ((frame.data[1] & 0xC0) >> 6); //Store SOC%
				stateOfCharge /= 10; //Remove decimals
			break;
			case 0x3B4: //LIM message
				Pilot_AC_Current = frame.data[0]; //PilotLim in CCS code (used for AC?)
				Cable_Current = frame.data[1];	//CalbeLim in CCS code (used for AC?)
				PP_Status = (frame.data[2] & 0x01); //PlugDet in CCS code
				CP_Mode = (frame.data[4] & 0x07); //Pilot_Status in DBC file
				ChargeType = frame.data[6];
				Cont_Volts = (frame.data[7] * 2);
				break;
			case 0x337: //LIM message
				Hook_Pos = (frame.data[0] & 0x03);
				Hook_Lock = (frame.data[0] & 0x0C);
				break;
			case 0x272: //LIM message
				FC_Contactor_Test = (frame.data[2] & 0xF0);
				FC_Contactor_State = ((frame.data[2] & 0x0C) >> 2); //0open, 1closed, 2error, 3invalid
				ChargeFlap_Status = (frame.data[2] & 0x03); //CP_DOOR
				break;
			case 0x29E: //LIM message (only during fastcharging)
				//CCS_V_Avail = ((frame.data[1] << 8) | frame.data[2]); //V_Avail in DBC (not used in code?)
				CCS_I_Avail = ((frame.data[4] << 8) | frame.data[3]); //I_Available in dbc
				CCS_Iso = ((frame.data[0] & 0xC0) >> 6); //Iso_Status in dbc
				CCS_IntStat = ((frame.data[0] & 0x3C) >> 2); //Internal_Charger_Status in dbc
				break;
			case 0x2B2: //LIM message (only during fastcharging)
				CHG_Volts = ((frame.data[0] << 8) | frame.data[1]);
				CHG_Amps = ((frame.data[2] << 8) | frame.data[3]); //TODO, signed! (but unused?)

				CCS_Ilim = (frame.data[5]>>4) & 0x03;	//I_Limit
				CCS_Vlim = (frame.data[5]>>6) & 0x03;	//V_Limit
				CCS_Stat = frame.data[4] & 0x03;		//Charge_Status
				CCS_Malf = (frame.data[4]>>2) & 0x03;	
				CCS_Bmalf = frame.data[5] & 0x03;
				CCS_Stop = (frame.data[5]>>2) & 0x03;	//DC_Chg_Stop
				break;
			case 0x2EF: //LIM message
				Min_V_Avail = (frame.data[0] | frame.data[1]);
				Min_I_Avail = (frame.data[2] | (frame.data[3] & 0x0F)); //Did bits go correct here?
				CCS_Plim = ((frame.data[6] & 0x30) >> 4); //Power_Limit in dbc 
				Energy_Transmitted = frame.data[7];
				break;
			default:
			break;
			}
		}		
			
	
	if(flag & 0xA0){
		uint8_t flag2 = can_read(MCP_REG_EFLG, can_bus);
		if(flag2 & 0xC0){
			can_write(MCP_REG_EFLG, 0, can_bus); //reset all errors
			//ReportString = "CANX RX OVF\n";
			//ReportString[3] = 48 + can_bus;
			//print(ReportString,12);
		}
		if(flag2 > 0){ PORTB.OUTSET = (1 << 1); }
		if(flag & 0xE0){ can_bit_modify(MCP_REG_CANINTF, (flag & 0xE0), 0x00, can_bus);	}
	}
	can_busy = 0;
}


void send_can(uint8_t can_bus, can_frame_t frame){
	if(can_bus == 1) send_can1(frame);
	if(can_bus == 2) send_can2(frame);
	if(can_bus == 3) send_can3(frame);
}

void send_can1(can_frame_t frame){	
	//put in the buffer
	memcpy(&tx0_buffer[tx0_buffer_end++], &frame, sizeof(frame));
	
	if(tx0_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx0_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can1();
}



void check_can1(void){
	uint8_t reg;
	
	if(tx0_buffer_end != tx0_buffer_pos){
		//check if TXB0 is free use
		reg = can1_read(MCP_REG_TXB0CTRL);
	
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can1_load_txbuff(0, (can_frame_t *) &tx0_buffer[tx0_buffer_pos++]);
			can1_rts(0);
			if(tx0_buffer_pos == tx0_buffer_end){ //end of buffer, reset
				tx0_buffer_end = 0;
				tx0_buffer_pos = 0;
			}
		}
	}
}

void send_can2(can_frame_t frame){
	//put in the buffer
	memcpy(&tx2_buffer[tx2_buffer_end++], &frame, sizeof(frame));
	
	if(tx2_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx2_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can2();
}

void check_can2(void){
	uint8_t reg;
	
	if(tx2_buffer_end != tx2_buffer_pos){
		//check if TXB0 is free use
		reg = can2_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can2_load_txbuff(0, (can_frame_t *) &tx2_buffer[tx2_buffer_pos++]);
			can2_rts(0);
			if(tx2_buffer_pos == tx2_buffer_end){ //end of buffer, reset
				tx2_buffer_end = 0;
				tx2_buffer_pos = 0;
			}
		}
	}
}

void send_can3(can_frame_t frame){
	//put in the buffer
	memcpy(&tx3_buffer[tx3_buffer_end++], &frame, sizeof(frame));
	
	if(tx3_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx3_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can3();
}

void check_can3(void){
	uint8_t reg;
	
	if(tx3_buffer_end != tx3_buffer_pos){
		//check if TXB0 is free use
		reg = can3_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can3_load_txbuff(0, (can_frame_t *) &tx3_buffer[tx3_buffer_pos++]);
			can3_rts(0);
			if(tx3_buffer_pos == tx3_buffer_end){ //end of buffer, reset
				tx3_buffer_end = 0;
				tx3_buffer_pos = 0;
			}
		}
	}
}


