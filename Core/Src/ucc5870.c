/**
 * @file UCC5870.c
 * @brief Defines the behaviour of UCC5870 interface functions.
 */

#define  _UCC5870
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "ucc5870.h"
#include "ucc5870_regs.h"

#include "SPI.h"
#include "UART.h"

// =============================================================================
// UCC5870 gate driver chip address definitions
#define  UH_ADDRESS   1U
#define  UL_ADDRESS   2U
#define  VH_ADDRESS   3U
#define  VL_ADDRESS   4U
#define  WH_ADDRESS   5U
#define  WL_ADDRESS   6U
#define  BROADCAST    15U

// =============================================================================
// UCC5870_Vars array indexing definitions
#define  UH  0
#define  UL  1
#define  VH  2
#define  VL  3
#define  WH  4
#define  WL  5

/**
 * @brief Structure containing configuration data for PWM pin alternate functions
 *
 * While assigning addresses to gate drivers PWM pins are used as digital outputs.
 * For that process they have to be reconfigured to digital outputs and back to timer
 * output pins upon completion of the assignment. Data needed to reconfigure the pins
 * back to timer output pins is stored here.
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t alternate;
} PWM_PinConfig;

PWM_PinConfig PWM_pins[DRIVER_NUM] =
{
		[UH] = { PWM_UH_GPIO_Port, PWM_UH_Pin, GPIO_AF5_TIM8  }, // UH -> PB6
		[UL] = { PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_AF4_TIM8  }, // UL -> PC10
		[VH] = { PWM_VH_GPIO_Port, PWM_VH_Pin, GPIO_AF10_TIM8 }, // UH -> PB8
		[VL] = { PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_AF4_TIM8  }, // UL -> PC11
		[WH] = { PWM_WH_GPIO_Port, PWM_WH_Pin, GPIO_AF10_TIM8 }, // UH -> PB9
		[WL] = { PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_AF4_TIM8  }  // UL -> PC12
};

/* --- UCC5870 status fault mask definitions ---------------------------------*/

#define  STATUS1_FAULT_MASK      ((1 * GD_TWN_FAULT_MASK       )  | \
                                  (1 * PWM_COMP_CHK_FAULT_MASK )   )

#define  STATUS2_FAULT_MASK      ((0 * OR_NFLT2_MASK        )     | \
                                  (1 * OR_NFLT1_MASK        )     | \
                                  (0 * TRIM_CRC_FAULT_MASK  )     | \
                                  (0 * CFG_CRC_FAULT_MASK   )     | \
                                  (0 * CLK_MON_FAULT_MASK   )     | \
                                  (0 * BIST_FAULT_MASK      )     | \
                                  (0 * INT_COMM_FAULT_MASK  )     | \
                                  (0 * INT_REG_FAULT_MASK   )     | \
                                  (0 * SPI_FAULT_MASK       )     | \
                                  (0 * STP_FAULT_MASK       )     | \
                                  (0 * OVLO1_FAULT_MASK     )     | \
                                  (0 * UVLO1_FAULT_MASK     )      )

#define  STATUS3_FAULT_MASK      ((1 * DESAT_FAULT_MASK        )  | \
                                  (1 * OC_FAULT_MASK           )  | \
                                  (1 * SC_FAULT_MASK           )  | \
                                  (1 * VREG2_ILIMIT_FAULT_MASK )  | \
                                  (1 * PS_TSD_FAULT_MASK       )  | \
                                  (1 * VCEOV_FAULT_MASK        )  | \
                                  (1 * UVLO2_FAULT_MASK        )  | \
                                  (1 * OVLO2_FAULT_MASK        )  | \
                                  (1 * UVLO3_FAULT_MASK        )  | \
                                  (1 * OVLO3_FAULT_MASK        )  | \
                                  (0 * MCLP_STATE_MASK         )  | \
                                  (1 * INT_COMM_SEC_FAULT_MASK )  | \
                                  (1 * INT_REG_SEC_FAULT_MASK  )  | \
                                  (1 * GM_FAULT_MASK           )   )

#define  STATUS4_FAULT_MASK      ((0 * RIM_CRC_SEC_FAULT_MASK  )  | \
                                  (0 * CFG_CRC_SEC_FAULT_MASK  )  | \
                                  (0 * CLK_MON_SEC_FAULT_MASK  )  | \
                                  (0 * BIST_SEC_FAULT_MASK     )  | \
                                  (0 * OR_NFLT2_SEC_MASK       )  | \
                                  (1 * OR_NFLT1_SEC_MASK       )  | \
                                  (0 * GD_TSD_SEC_FAULT_MASK   )  | \
                                  (0 * GD_TWN_SEC_FAULT_MASK   )   )

#define  STATUS5_FAULT_MASK       (1 * ADC_FAULT_MASK)

/// Local FAIL/PASS enumeration
enum {
	FAIL = 0,
	PASS = 1
};


/* --- Function prototypes ---------------------------------------------------*/

uint16_t diagnose_UCC5870(uint16_t i);
uint16_t readRegUCC5870(uint16_t CA, uint16_t RA);
void writeRegUCC5870(uint16_t CA, uint16_t RA, uint16_t data);
uint16_t writeVerifyRegUCC5870(uint16_t CA, uint16_t RA, uint16_t data);
uint16_t writeVerify_UCC5870(uint16_t CA);

/* --- Local function macros -------------------------------------------------*/

#define  UCC5870_DRV_EN(CA)       sendCmdUCC5870((CA << 12) | 0x009);
#define  UCC5870_DRV_DIS(CA)	  sendCmdUCC5870((CA << 12) | 0x00a);
#define  UCC5870_CFG_IN(CA)       sendCmdUCC5870((CA << 12) | 0x222);
#define  UCC5870_NOP(CA)          sendCmdUCC5870((CA << 12) | 0x542);
#define  UCC5870_SWRESET(CA)      sendCmdUCC5870((CA << 12) | 0x708);

#define  UCC5870_WR_CA(CA)		  sendCmdUCC5870((BROADCAST << 12) | (0x0da0 + CA));
#define  UCC5870_WR_REG(CA, RA)   sendCmdUCC5870((CA << 12) | (0x0c00 + RA));
#define  UCC5870_WR_DH(CA, DH)    sendCmdUCC5870((CA << 12) | (0x0a00 + DH));
#define  UCC5870_WR_DL(CA, DL)    sendCmdUCC5870((CA << 12) | (0x0b00 + DL));
#define  UCC5870_RD_REG(CA, RA)   sendCmdUCC5870((CA << 12) | (0x0100 + RA));

#define  CLEAR_FAULTS(GATE_DRV)   writeRegUCC5870 (GATE_DRV, CONTROL2, 0x8000)

/* --- Global variables ------------------------------------------------------*/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
UCC5870_Vars ucc5870[DRIVER_NUM] = {
		UCC5870_DEFAULTS,
		UCC5870_DEFAULTS,
		UCC5870_DEFAULTS,
		UCC5870_DEFAULTS,
		UCC5870_DEFAULTS,
		UCC5870_DEFAULTS
};
#pragma GCC diagnostic pop

uint16_t tx_data;  // to monitor cmd in debug window - can be commented out

uint16_t GD[DRIVER_NUM] = {
		UH_ADDRESS,
		UL_ADDRESS,
		VH_ADDRESS,
		VL_ADDRESS,
		WH_ADDRESS,
		WL_ADDRESS
};

uint16_t statusRegAdrs[5] = {
		STATUS1,
		STATUS2,
		STATUS3,
		STATUS4,
		STATUS5
};

uint16_t statusFaultMask[5] = {
		STATUS1_FAULT_MASK,
		STATUS2_FAULT_MASK,
		STATUS3_FAULT_MASK,
		STATUS4_FAULT_MASK,
		STATUS5_FAULT_MASK
};

UCC5870_STATUS1_REG *status1[DRIVER_NUM];
UCC5870_STATUS2_REG *status2[DRIVER_NUM];
UCC5870_STATUS3_REG *status3[DRIVER_NUM];
UCC5870_STATUS4_REG *status4[DRIVER_NUM];

uint16_t initStatus[DRIVER_NUM] = { 0, 0, 0, 0, 0, 0 };
uint16_t diagStatus[DRIVER_NUM] = { 0, 0, 0, 0, 0, 0 };
uint16_t initFault, statusFault, priReadyFault, secReadyFault;

/* --- Local function definitions --------------------------------------------*/

/**
 * @brief Send a command to UCC5870 via SPI
 *
 * Data is transmitted and received during a full-duplex SPI transaction.
 *
 * @param[in] tx_data - Value to be transmitted.
 *
 * @return uint16_t - Value received during SPI transaction
 *
 * @note As per UCC5870 datasheet, SPI communication is done @4Mhz max, in 16 bit frames,
 * with MSB first. Clock idles LOW, and the data is latched on the falling edge.
 */
uint16_t sendCmdUCC5870(uint16_t tx_data)
{
	uint16_t rx_data = 0;
	HAL_GPIO_WritePin(SPI1_CS_Driver_GPIO_Port,
						SPI1_CS_Driver_Pin,
						GPIO_PIN_RESET);
	SPI_TransmitReceive_Driver((uint8_t*) &tx_data,
								(uint8_t*) &rx_data,
								1,
								HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI1_CS_Driver_GPIO_Port,
						SPI1_CS_Driver_Pin,
						GPIO_PIN_SET);

	return rx_data;
}

/**
 * @brief Read data from a UCC5870 register
 *
 * Data is read from a register by sending a RD_REG command with the register's address
 * as a parameter. The from the register is received during the following 16 clock cycles.
 * That can be a NOP command, or any other command. In the current implementation data is
 * read by sending a RD_REG command followed by a NOP command.
 *
 * @param[in] chipAddress - Gate driver IC address
 * @param[in] regAddress  - Register address from which data is to be read
 *
 * @return uint16_t Data read from the register
 */
uint16_t readRegUCC5870(uint16_t chipAddress,
						uint16_t regAddress)
{
	uint16_t rx_data = 0;
	UCC5870_RD_REG(chipAddress, regAddress);
	rx_data = UCC5870_NOP(chipAddress);

	return rx_data;
}

/**
 * @brief Write data to a register
 *
 * Data is written to a register by sending a WR_REG command with the register address as a parameter.
 * This is followed by WR_DH command which writes data to the HIGH byte of the register, and a
 * WR_DL command which writes data to the LOW byte of the register.
 *
 * @param[in] chipAddress - Gate driver IC address
 * @param[in] regAddress  - Register address to which the data is to be written
 * @param[in] data        - Value to be written to register
 */
void writeRegUCC5870(uint16_t chipAddress,
					 uint16_t regAddress,
					 uint16_t  data)
{
	 UCC5870_WR_REG(chipAddress,  regAddress);
	 UCC5870_WR_DH (chipAddress, (data / 256  ));
	 UCC5870_WR_DL (chipAddress, (data % 256  ));
}

/* --- Initialize local UCC5870 registers ------------------------------------*/
void Init_UCC5870_Regs(void) {
	uint16_t i, j;
	uint16_t *srcPtr, *dstPtr;
	//
	// CFG1 register settings
	//
	ucc5870[UH].cfg1.bit.GD_TWN_DIS = OT_warning_enable;
	ucc5870[UH].cfg1.bit.IO_DEGLITCH = io_deglitch_210ns;
	ucc5870[UH].cfg1.bit.NFLT2_DOUT_MUX = nFLT2_DOUT_nFLT2;
	ucc5870[UH].cfg1.bit.OV1_DIS = vcc1_ovlo_enable;
	ucc5870[UH].cfg1.bit.UV1_DIS = vcc1_uvlo_enable;
	ucc5870[UH].cfg1.bit.OVLO1_LEVEL = vcc1_3V3;
	ucc5870[UH].cfg1.bit.UVLO1_LEVEL = vcc1_3V3;
	ucc5870[UH].cfg1.bit.TDEAD = Tdead(0);  // in ns

	//
	// CFG2 register settings
	//
	ucc5870[UH].cfg2.bit.PWM_CHK_FAULT = report_fault_on_nFLT1; //  PWM check fault
	ucc5870[UH].cfg2.bit.VREG1_ILIMIT_FAULT = report_fault_on_nFLT1; //  Vreg Ilimit fault
	ucc5870[UH].cfg2.bit.GD_TWN_FAULT = warning_on_nFLT1; //  temp warning fault
	ucc5870[UH].cfg2.bit.BIST_FAULT = report_fault_on_nFLT1; //  Analog BIST fault
	ucc5870[UH].cfg2.bit.TRIM_CRC_FAULT = report_fault_on_nFLT1;   //  CRC fault
	ucc5870[UH].cfg2.bit.INT_REG_FAULT = report_fault_on_nFLT1; //  Internal regulator fault
	ucc5870[UH].cfg2.bit.CFG_CRC_FAULT = report_fault_on_nFLT1; //  reg CRC fault
	ucc5870[UH].cfg2.bit.SPI_FAULT = dont_report_SPI_fault; //report_SPI_fault_on_nFLT1;  //  SPI fault
	ucc5870[UH].cfg2.bit.CLK_MON_FAULT = report_fault_on_nFLT1; //  Clock monitor fault
	ucc5870[UH].cfg2.bit.STP_FAULT = report_fault_on_nFLT1;     //  STP fault
	ucc5870[UH].cfg2.bit.UVLO1_FAULT = report_fault_on_nFLT1;     //  UVLO fault
	ucc5870[UH].cfg2.bit.OVLO1_FAULT = report_fault_on_nFLT1;     //  OVLO fault
	ucc5870[UH].cfg2.bit.INT_COMM_FAULT = report_comm_fault_on_nFLT1; //  Inter-die comm failure

	//
	// CFG3 register settings
	//
	ucc5870[UH].cfg3.bit.AI_IZTC_SEL = AI1_AI3_AI5_bias_currents_off; //
	ucc5870[UH].cfg3.bit.FS_STATE_CFG_CRC_FAULT = OUTx_LOW; //
	ucc5870[UH].cfg3.bit.ITO2_EN = CURRENT_SRC_OUT_AI2_4_6_DISABLED; //
	ucc5870[UH].cfg3.bit.ITO1_EN = CURRENT_SRC_OUT_AI1_3_5_DISABLED; //
	ucc5870[UH].cfg3.bit.FS_STATE_INT_COMM_FAULT = OUTx_LOW; //
	ucc5870[UH].cfg3.bit.FS_STATE_INT_REG_FAULT = OUTx_LOW; //
	ucc5870[UH].cfg3.bit.FS_STATE_SPI_FAULT = FS_SPI_OUTx_LOW; //
	ucc5870[UH].cfg3.bit.FS_STATE_STP_FAULT = FS_STP_OUTx_LOW; //
	ucc5870[UH].cfg3.bit.FS_STATE_PWM_CHK = OUTx_LOW; //
	ucc5870[UH].cfg3.bit.FS_STATE_OVLO1_FAULT = OUTx_LOW; //
	ucc5870[UH].cfg3.bit.FS_STATE_UVLO1_FAULT = OUTx_LOW; //

	//
	// CFG4 register settings
	//
	ucc5870[UH].cfg4.bit.UVOV3_EN = VEE2_uvlo_ovlo_enable; //
	ucc5870[UH].cfg4.bit.PS_TSD_EN = Switch_thermal_SD_disable; //
	ucc5870[UH].cfg4.bit.OCP_DIS = OCP_disable; //
	ucc5870[UH].cfg4.bit.SCP_DIS = SCP_disable; //
	ucc5870[UH].cfg4.bit.DESAT_EN = DESAT_det_enable; //DESAT_det_disable; //
	ucc5870[UH].cfg4.bit.VCECLP_EN = VCE_CLAMP_enable; //
	ucc5870[UH].cfg4.bit.MCLP_DIS = MILLER_CLAMP_enable; //
	ucc5870[UH].cfg4.bit.GM_EN = Gate_Volt_Mon_enable;
	ucc5870[UH].cfg4.bit.GM_BLK = Gate_volt_mon_blank_2500ns; //
	ucc5870[UH].cfg4.bit.MCLP_CFG = MILLER_CLAMP_internal; //
	ucc5870[UH].cfg4.bit.OV2_DIS = VCC2_ovlo_enable; //
	ucc5870[UH].cfg4.bit.DESAT_DEGLITCH = DESAT_deglitch_316ns; //
	ucc5870[UH].cfg4.bit.PS_TSD_DEGLITCH = TSD_1000ns; //
	ucc5870[UH].cfg4.bit.UV2_DIS = VCC2_uvlo_enable; //

	//
	// CFG5 register settings
	//
	ucc5870[UH].cfg5.bit.PWM_MUTE_EN = PWM_MUTE_FOR_SC_OC_OT_FAULTS_DISABLED;
	ucc5870[UH].cfg5.bit.TLTOFF_STO_EN = STO_2LTOFF_DISABLED; //STO_FOR_SC_DESAT_OC;
	ucc5870[UH].cfg5.bit.STO_CURR = SOFT_TURN_OFF_1_2A;
	ucc5870[UH].cfg5.bit.MCLPTH = MILLER_CLAMP_VTH_3V;
	ucc5870[UH].cfg5.bit.DESAT_DCHG_EN = DESAT_DISCHARGE_ENABLE; //DESAT_DISCHARGE_DISABLE;
	ucc5870[UH].cfg5.bit.DESAT_CHG_CURR = BLANK_CAP_CHRG_CUR_800uA;
	ucc5870[UH].cfg5.bit.DESATTH = DESAT_VTH_3p5;
	ucc5870[UH].cfg5.bit.GM_STO2LTO_DIS = GATE_MON_FLT_DURING_STO_TLTOFF_ENABLED;

	//
	// CFG6 register settings
	//
	ucc5870[UH].cfg6.bit.PS_TSDTH = TSD_TH_2p75;
	ucc5870[UH].cfg6.bit.TEMP_CURR = TEMP_CURR_0p1A;
	ucc5870[UH].cfg6.bit.OC_BLK = OCD_BLANK_TIME_0p5us;
	ucc5870[UH].cfg6.bit.SC_BLK = SCD_BLANK_TIME_0p2us;
	ucc5870[UH].cfg6.bit.SCTH = SCD_VTH_1p25V;
	ucc5870[UH].cfg6.bit.OCTH = OCD_VTH_0p5V;

	//
	// CFG7 register settings
	//
	ucc5870[UH].cfg7.bit.FS_STATE_ADC_FAULT = FS_ADC_FLT_OUT_IGNORE;
	ucc5870[UH].cfg7.bit.ADC_FAULT_P = dont_report_adcfault_on_nFLT1;
	ucc5870[UH].cfg7.bit.ADC_SAMP_DLY = ADC_SAMP_DELAY_1120ns;
	ucc5870[UH].cfg7.bit.ADC_SAMP_MODE = ADC_SAMP_CENTER_ALIGN;
	ucc5870[UH].cfg7.bit.ADC_EN = ADC_SAMP_DISABLE;
	ucc5870[UH].cfg7.bit.OVLO3TH = VEE2_OVLO_VTH_M10V;
	ucc5870[UH].cfg7.bit.UVLO3TH = VEE2_UVLO_VTH_M3V;
	ucc5870[UH].cfg7.bit.OVLO2TH = VCC2_OVLO_VTH_P23V;
	ucc5870[UH].cfg7.bit.UVLO2TH = VCC2_UVLO_VTH_P12V;

	//
	// CFG8 register settings
	//
	ucc5870[UH].cfg8.bit.IOUT_SEL = GATE_DRIVE_STRENGTH_FULL;
	ucc5870[UH].cfg8.bit.AI_ASC_MUX = AI_ASC_MUX_AI;
	ucc5870[UH].cfg8.bit.VREF_SEL = VREF_INTERNAL;
	ucc5870[UH].cfg8.bit.GD_2LOFF_STO_EN = GD_2LOFF_STO_ENABLE;
	ucc5870[UH].cfg8.bit.CRC_DIS = CRC_CHECK_ENABLE;
	ucc5870[UH].cfg8.bit.GD_2LOFF_CURR = GD_2LOFF_DISCHARGE_CURR_0p9A;
	ucc5870[UH].cfg8.bit.GD_2LOFF_TIME = GD_2LOFF_PLATEAU_TIME_150ns;
	ucc5870[UH].cfg8.bit.GD_2LOFF_VOLT = GD_2LOFF_PLATEAU_VOLT_6V;

	//
	// CFG9 register settings
	//
	ucc5870[UH].cfg9.bit.CLK_MON_SEC_FAULT = report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.VREG2_ILIMIT_FAULT = report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.BIST_SEC_FAULT = report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.INT_REG_SEC_FAULT = report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.TRIM_CRC_SEC_FAULT = report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.CFG_CRC_SEC_FAULT = report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.INT_COMM_SEC_FAULT = report_comm_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.GD_TSD_FAULT = report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.PS_TSD_FAULT = dont_report_TSD_on_nFLT1;
	ucc5870[UH].cfg9.bit.OVLO23_FAULT = report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.UVLO23_FAULT = report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.GM_FAULT = GM_FAULT_ON_nFLT1;  //GM_FAULT_IGNORE;  //
	ucc5870[UH].cfg9.bit.OC_FAULT = dont_report_fault_on_nFLT1;
	ucc5870[UH].cfg9.bit.SC_FAULT = report_fault_on_nFLT1;

	//
	// CFG10 register settings
	//
	ucc5870[UH].cfg10.bit.FS_STATE_INT_COMM_SEC = PULLED_LOW;
	ucc5870[UH].cfg10.bit.FS_STATE_GM = PULLED_LOW;
	ucc5870[UH].cfg10.bit.FS_STATE_GD_TSD = PULLED_LOW;
	ucc5870[UH].cfg10.bit.FS_STATE_PS_TSD = NO_ACTION;
	ucc5870[UH].cfg10.bit.FS_STATE_OCP = NO_ACTION;
	ucc5870[UH].cfg10.bit.FS_STATE_INT_REG_FAULT = PULLED_LOW;
	ucc5870[UH].cfg10.bit.FS_STATE_DESAT_SCP = PULLED_LOW;
	ucc5870[UH].cfg10.bit.GD_TWN_SEC_EN = GD_TWN_SEC_ENABLED;

	//
	// CFG11 register settings
	//
	ucc5870[UH].cfg11.bit.FS_STATE_CLK_MON_SEC_FAULT = PULLED_LOW;
	ucc5870[UH].cfg11.bit.VCE_CLMP_HLD_TIME = VCE_CLAMP_HOLD_TIME_100ns;
	ucc5870[UH].cfg11.bit.FS_STATE_CFG_CRC_SEC_FAULT = PULLED_LOW;
	ucc5870[UH].cfg11.bit.FS_STATE_TRIM_CRC_SEC_FAULT = PULLED_LOW;
	ucc5870[UH].cfg11.bit.FS_STATE_OVLO3 = PULLED_LOW;
	ucc5870[UH].cfg11.bit.FS_STATE_UVLO3 = PULLED_LOW;
	ucc5870[UH].cfg11.bit.FS_STATE_OVLO2 = PULLED_LOW;
	ucc5870[UH].cfg11.bit.FS_STATE_UVLO2 = PULLED_LOW;

	//
	// ADCCFG register settings
	//
	ucc5870[UH].adccfg.bit.ADC_ON_CH_SEL_7 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_ON_CH_SEL_6 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_ON_CH_SEL_5 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_ON_CH_SEL_4 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_ON_CH_SEL_3 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_ON_CH_SEL_2 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_ON_CH_SEL_1 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_OFF_CH_SEL_7 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_OFF_CH_SEL_6 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_OFF_CH_SEL_5 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_OFF_CH_SEL_4 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_OFF_CH_SEL_3 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_OFF_CH_SEL_2 = DONT_SAMPLE_CH;
	ucc5870[UH].adccfg.bit.ADC_OFF_CH_SEL_1 = DONT_SAMPLE_CH;

	//
	// DOUTCFG register settings
	//
	ucc5870[UH].doutcfg.bit.DOUT_TO_AI1 = DONT_CONNECT_TO_DOUT;
	ucc5870[UH].doutcfg.bit.DOUT_TO_AI3 = DONT_CONNECT_TO_DOUT;
	ucc5870[UH].doutcfg.bit.DOUT_TO_AI5 = DONT_CONNECT_TO_DOUT;
	ucc5870[UH].doutcfg.bit.DOUT_TO_AI2 = DONT_CONNECT_TO_DOUT;
	ucc5870[UH].doutcfg.bit.DOUT_TO_AI4 = DONT_CONNECT_TO_DOUT;
	ucc5870[UH].doutcfg.bit.DOUT_TO_AI6 = DONT_CONNECT_TO_DOUT;
	ucc5870[UH].doutcfg.bit.DOUT_TO_TJ = DONT_CONNECT_TO_DOUT;
	ucc5870[UH].doutcfg.bit.FREQ_DOUT = FREQ_DOUT_111p4KHz;
	ucc5870[UH].doutcfg.bit.AI6OCSC_EN = PROT_DISABLED;
	ucc5870[UH].doutcfg.bit.AI4OCSC_EN = PROT_DISABLED;
	ucc5870[UH].doutcfg.bit.AI2OCSC_EN = PROT_DISABLED;
	ucc5870[UH].doutcfg.bit.AI5OT_EN = PROT_DISABLED;
	ucc5870[UH].doutcfg.bit.AI3OT_EN = PROT_DISABLED;
	ucc5870[UH].doutcfg.bit.AI1OT_EN = PROT_DISABLED;

	//
	// copying the configuration settings for all devices
	//
	srcPtr = (uint16_t*) &ucc5870[0];
	for (j = 1; j < DRIVER_NUM; j++) {
		dstPtr = (uint16_t*) &ucc5870[j];
		for (i = 0; i < sizeof(UCC5870_Vars); i++) {
			dstPtr[i] = srcPtr[i];
		}
	}

	// new
	for (i = 0; i < DRIVER_NUM; i++) {
		status1[i] = &ucc5870[i].status1;
		status2[i] = &ucc5870[i].status2;
		status3[i] = &ucc5870[i].status3;
		status4[i] = &ucc5870[i].status4;
	}

	return;
}

/* --- Initialize gate driver ICs --------------------------------------------*/
UCC5870_Status_e Init_UCC5870(void) {
	UCC5870_Status_e status;

	/* --- Setup PWM pins (UH, UL, VH, VL, WH and WL) as GPIO outputs ------------*/
	for (uint16_t i = 0; i < DRIVER_NUM; i++) {
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		GPIO_InitStruct.Pin = PWM_pins[i].pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(PWM_pins[i].port, &GPIO_InitStruct);
	}

	/* --- Setup Chip Addresses for all UCC5870s - UH, UL, VH, VL, WH and WL -----*/
	for (uint16_t i = 0; i < DRIVER_NUM; i++) {
		HAL_GPIO_WritePin(PWM_pins[i].port, PWM_pins[i].pin, GPIO_PIN_SET);
		UCC5870_WR_CA(GD[i]);
		HAL_GPIO_WritePin(PWM_pins[i].port, PWM_pins[i].pin, GPIO_PIN_RESET);
	}

	/* --- Setup PWM pins (UH, UL, VH, VL, WH and WL) as PWM_timer outputs -------*/
	for (uint16_t i = 0; i < DRIVER_NUM; i++) {
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		GPIO_InitStruct.Pin = PWM_pins[i].pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = PWM_pins[i].alternate;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(PWM_pins[i].port, &GPIO_InitStruct);
	}


	/* --- Reset drivers and put it in CONFIG mode -------------------------------*/
	UCC5870_DRV_DIS(BROADCAST);  // DRV_DIS and SWRESET sequence helps to ..
	UCC5870_SWRESET(BROADCAST);  //  .. reset and restart the drivers
	UCC5870_CFG_IN(BROADCAST);   // put the drivers in config mode
	CLEAR_FAULTS(BROADCAST);     // clear faults at reset before configuring

	/* --- Configure UCC5870 - UH, UL, VH, VL, WH and WL -------------------------*/
	initFault = 0;  // logic HIGH bit positions indicate the driver at fault

	for(uint16_t i = 0; i < DRIVER_NUM; i++){
		initStatus[i] = writeVerify_UCC5870(i);
		initFault += (initStatus[i] == FAIL) << i;
	}

	/* --- Check if any driver's written data is not verified --------------------*/
	if (initFault) {
		return INIT_FAULT;
	}
	else {
		status = inverterDiagnostics();
		if (status == ALL_GOOD) {
			for(uint16_t i = 0; i < DRIVER_NUM; i++){
				UCC5870_DRV_EN(GD[i]);
			}
		}
		return (status);
	}

	return INIT_FAULT;
}

/**
 * @brief Print formatted string and transmit via UART
 *
 * This function takes a formatted string and variable arguments, formats the output
 * into a temporary buffer, and transmits the resulting string over UART.
 * @param[in] fmt Format string containing text to be transmitted
 */
void uart_printf(const char *fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    UART_Transmit((uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
 * @brief Acquire gate driver IC address and store it in the local copy of the register
 *
 * @param[in] i - Index of the UCC5870 in the global UCC5870 array
 *
 * @return uint16_t GD_ADDR register value received from the gate driver IC
 */
uint16_t get_driver_address(uint8_t i)
{
	uint16_t *addressReg;

	addressReg = (uint16_t *) &ucc5870[i].gd_address;
	*addressReg = readRegUCC5870 (GD[i], GD_ADDR);

	return *addressReg;
}

/**
 * @brief Print header for the status information
 *
 * @note This function prints the interpretation of LOCAL values of the GD_ADDR register.
 * For the latest data it has to be run following the get_driver_address function.
 *
 * @param[in] reg_val - Local value of the GD_ADDR register
 */
void print_status_header(uint16_t regVal)
{
    uart_printf("\r\n\r\nStatus information for: ");

    switch (regVal)
    {
        case UH_ADDRESS:
            uart_printf("UH\r\n");
            break;
        case UL_ADDRESS:
            uart_printf("UL\r\n");
            break;
      case VH_ADDRESS:
            uart_printf("VH\r\n");
            break;
        case VL_ADDRESS:
            uart_printf("VL\r\n");
            break;
        case WH_ADDRESS:
            uart_printf("WH\r\n");
            break;
        case WL_ADDRESS:
            uart_printf("WL\r\n");
            break;
        default:
            uart_printf("Unknown (0x%04X)\r\n", regVal);
            break;
    }
}

/**
 * @brief Print interpretation of STATUS1 register values
 *
 * @note This function prints the interpretation of LOCAL values of the STATUS1 register.
 * For the latest data it has to be run following the diagnose_UCC5870 function.
 *
 * @param[in] reg_val - Local value of the STATUS1 register
 */
void print_status1_reg(uint16_t reg_val)
{
    UCC5870_STATUS1_REG_BITS* bits = (UCC5870_STATUS1_REG_BITS*)&reg_val;

    uart_printf( "--------------------------------------\r\n");
    uart_printf("STATUS1 register\r\n");
    uart_printf("--------------------------------------\r\n");

    uart_printf("GD_TWN_FAULT       : %s\r\n", bits->GD_TWN_FAULT ? "FAULT" : "OK");
    uart_printf("PWM_COMP_CHK_FAULT : %s\r\n", bits->PWM_COMP_CHK_FAULT ? "FAULT" : "OK");

    uart_printf("OPM                : ");
    switch(bits->OPM) {
        case 1:
            uart_printf("CONFIGURATION1");
            break;
        case 2:
            uart_printf("CONFIGURATION2");
            break;
        case 3:
            uart_printf("ACTIVE");
            break;
        default:
            uart_printf("ERROR");
            break;
    }
    uart_printf(" (0x%X)\r\n", bits->OPM);

    uart_printf("EN_STATE           : %s\r\n", bits->EN_STATE ? "ASC ENABLED" : "ASC DISABLED");
    uart_printf("INN_STATE          : %s\r\n", bits->INN_STATE ? "HIGH" : "LOW");
    uart_printf("INP_STATE          : %s\r\n", bits->INP_STATE ? "HIGH" : "LOW");
}

/**
 * @brief Print interpretation of STATUS2 register values
 *
 * @note This function prints the interpretation of LOCAL values of the STATUS2 register.
 * For the latest data it has to be run following the diagnose_UCC5870 function.
 *
 * @param[in] reg_val - Local value of the STATUS2 register
 */
void print_status2_reg(uint16_t reg_val) {
    UCC5870_STATUS2_REG_BITS *bits = (UCC5870_STATUS2_REG_BITS *)&reg_val;

    uart_printf("--------------------------------------\r\n");
    uart_printf("STATUS2 Register\r\n");
    uart_printf("--------------------------------------\r\n");

    uart_printf("OR_NFLT2_PRI   : %s\r\n", bits->OR_NFLT2_PRI ? "ACTIVE" : "INACTIVE");
    uart_printf("OR_NFLT1_PRI   : %s\r\n", bits->OR_NFLT1_PRI ? "ACTIVE" : "INACTIVE");
    uart_printf("DRV_EN_RCVD    : %s\r\n", bits->DRV_EN_RCVD ? "RECEIVED" : "NOT RECEIVED");
    uart_printf("TRIM_CRC_FAULT : %s\r\n", bits->TRIM_CRC_FAULT ? "FAULT" : "OK");
    uart_printf("CFG_CRC_FAULT  : %s\r\n", bits->CFG_CRC_FAULT ? "FAULT" : "OK");
    uart_printf("CLK_MON_FAULT  : %s\r\n", bits->CLK_MON_FAULT ? "FAULT" : "OK");
    uart_printf("BIST_FAULT     : %s\r\n", bits->BIST_FAULT ? "FAIL" : "PASS");
    uart_printf("INT_COMM_FAULT : %s\r\n", bits->INT_COMM_FAULT ? "FAULT" : "OK");
    uart_printf("INT_REG_FAULT  : %s\r\n", bits->INT_REG_FAULT ? "FAULT" : "OK");
    uart_printf("SPI_FAULT      : %s\r\n", bits->SPI_FAULT ? "FAULT" : "OK");
    uart_printf("STP_FAULT      : %s\r\n", bits->STP_FAULT ? "FAULT" : "OK");
    uart_printf("OVLO1_FAULT    : %s\r\n", bits->OVLO1_FAULT ? "OVERVOLTAGE" : "OK");
    uart_printf("UVLO1_FAULT    : %s\r\n", bits->UVLO1_FAULT ? "UNDERVOLTAGE" : "OK");
    uart_printf("PRI_RDY        : %s\r\n", bits->PRI_RDY ? "READY" : "NOT READY");
}

/**
 * @brief Print interpretation of STATUS3 register values
 *
 * @note This function prints the interpretation of LOCAL values of the STATUS3 register.
 * For the latest data it has to be run following the diagnose_UCC5870 function.
 *
 * @param[in] reg_val - Local value of the STATUS3 register
 */
void print_status3_reg(uint16_t reg_val) {
    UCC5870_STATUS3_REG_BITS *bits = (UCC5870_STATUS3_REG_BITS *)&reg_val;

    uart_printf("--------------------------------------\r\n");
    uart_printf("STATUS3 Register\r\n");
    uart_printf("--------------------------------------\r\n");

    uart_printf("DESAT_FAULT        : %s\r\n", bits->DESAT_FAULT ? "FAULT" : "OK");
    uart_printf("OC_FAULT           : %s\r\n", bits->OC_FAULT ? "OVER CURRENT" : "OK");
    uart_printf("SC_FAULT           : %s\r\n", bits->SC_FAULT ? "SHORT CIRCUIT" : "OK");
    uart_printf("VREG2_ILIMIT_FAULT : %s\r\n", bits->VREG2_ILIMIT_FAULT ? "CURRENT LIMIT" : "OK");
    uart_printf("PS_TSD_FAULT       : %s\r\n", bits->PS_TSD_FAULT ? "OVERTEMP" : "OK");
    uart_printf("VCEOV_FAULT        : %s\r\n", bits->VCEOV_FAULT ? "OVERVOLTAGE" : "OK");
    uart_printf("UVLO2_FAULT        : %s\r\n", bits->UVLO2_FAULT ? "UNDERVOLTAGE" : "OK");
    uart_printf("OVLO2_FAULT        : %s\r\n", bits->OVLO2_FAULT ? "OVERVOLTAGE" : "OK");
    uart_printf("UVLO3_FAULT        : %s\r\n", bits->UVLO3_FAULT ? "UNDERVOLTAGE" : "OK");
    uart_printf("OVLO3_FAULT        : %s\r\n", bits->OVLO3_FAULT ? "OVERVOLTAGE" : "OK");
    uart_printf("MCLP_STATE         : %s\r\n", bits->MCLP_STATE ? "ACTIVE" : "INACTIVE");
    uart_printf("INT_COMM_SEC_FAULT : %s\r\n", bits->INT_COMM_SEC_FAULT ? "FAULT" : "OK");
    uart_printf("INT_REG_SEC_FAULT  : %s\r\n", bits->INT_REG_SEC_FAULT ? "FAULT" : "OK");
    uart_printf("GM_FAULT           : %s\r\n", bits->GM_FAULT ? "FAULT" : "OK");
    uart_printf("GM_STATE           : %s\r\n", bits->GM_STATE ? "ACTIVE" : "INACTIVE");
}

/**
 * @brief Print interpretation of STATUS4 register values
 *
 * @note This function prints the interpretation of LOCAL values of the STATUS4 register.
 * For the latest data it has to be run following the diagnose_UCC5870 function.
 *
 * @param[in] reg_val - Local value of the STATUS4 register
 */
void print_status4_reg(uint16_t reg_val) {
    UCC5870_STATUS4_REG_BITS* bits = (UCC5870_STATUS4_REG_BITS*)&reg_val;

    uart_printf("--------------------------------------\r\n");
    uart_printf("STATUS4 Register:\r\n");
    uart_printf("--------------------------------------\r\n");

    uart_printf("SEC_RDY           : %s\r\n", bits->SEC_RDY ? "READY" : "NOT READY");
    uart_printf("RIM_CRC_SEC_FAULT : %s\r\n", bits->RIM_CRC_SEC_FAULT ? "FAULT" : "OK");
    uart_printf("CFG_CRC_SEC_FAULT : %s\r\n", bits->CFG_CRC_SEC_FAULT ? "FAULT" : "OK");
    uart_printf("CLK_MON_SEC_FAULT : %s\r\n", bits->CLK_MON_SEC_FAULT ? "FAULT" : "OK");
    uart_printf("BIST_SEC_FAULT    : %s\r\n", bits->BIST_SEC_FAULT ? "FAIL" : "PASS");
    uart_printf("OR_NFLT2_SEC      : %s\r\n", bits->OR_NFLT2_SEC ? "ACTIVE" : "INACTIVE");
    uart_printf("OR_NFLT1_SEC      : %s\r\n", bits->OR_NFLT1_SEC ? "ACTIVE" : "INACTIVE");
    uart_printf("GD_TSD_SEC_FAULT  : %s\r\n", bits->GD_TSD_SEC_FAULT ? "OVERTEMP" : "OK");
    uart_printf("GD_TWN_SEC_FAULT  : %s\r\n", bits->GD_TWN_SEC_FAULT ? "FAULT" : "OK");
    uart_printf("VCE_STATE         : %s\r\n", bits->VCE_STATE ? "HIGH" : "LOW");
}

/**
 * @brief Print interpretation of STATUS5 register values
 *
 * @note This function prints the interpretation of LOCAL values of the STATUS5 register.
 * For the latest data it has to be run following the diagnose_UCC5870 function.
 *
 * @param[in] reg_val - Local value of the STATUS5 register
 */
void print_status5_reg(uint16_t reg_val) {
    UCC5870_STATUS5_REG_BITS* bits = (UCC5870_STATUS5_REG_BITS*)&reg_val;

    uart_printf("--------------------------------------\r\n");
    uart_printf("STATUS5 Register:\r\n");
    uart_printf("--------------------------------------\r\n");

    uart_printf("ADC Status        : %s\r\n", bits->ADC_FAULT ? "FAULT" : "OK");
}

/* --- Print inverter STATUS register values ---------------------------------*/
void printInverterStatus()
{
	for(uint16_t i = 0; i < DRIVER_NUM; i++)
	{
		diagnose_UCC5870(i);
		get_driver_address(i);

	    print_status_header(ucc5870[i].gd_address.all);
	    print_status1_reg(ucc5870[i].status1.all);
	    print_status2_reg(ucc5870[i].status2.all);
	    print_status3_reg(ucc5870[i].status3.all);
	    print_status4_reg(ucc5870[i].status4.all);
	    print_status5_reg(ucc5870[i].status5.all);
	}
}

/* --- Run diagnostics for all gate driver ICs -------------------------------*/
UCC5870_Status_e inverterDiagnostics()
{
	/* --- Diagnose gate driver status and readiness before enabling -------------*/
	statusFault   = 0;  // logic HIGH bit positions indicate driver at fault
	priReadyFault = 0;  // logic HIGH bit positions indicate driver at fault
	secReadyFault = 0;  // logic HIGH bit positions indicate driver at fault

	for(uint16_t i = 0; i < DRIVER_NUM; i++)
	{
		diagStatus[i] = diagnose_UCC5870(i);
		statusFault   += (diagStatus[i] == FAIL) << UL;
		priReadyFault += (ucc5870[i].status2.bit.PRI_RDY == NOT_READY);
		secReadyFault += (ucc5870[i].status4.bit.SEC_RDY == NOT_READY);
	}

	/* --- Check if any driver's status register indicates fault -----------------*/
	if (statusFault)
	{
		return STATUS_FAULT;
	}

	/* --- Check if any driver's primary ready flag is not set -------------------*/
	if (priReadyFault)
	{
		return PRI_RDY_FAULT;
	}

	/* --- Check if any driver's secondary ready flag is not set -----------------*/
	if (secReadyFault)
	{
		return SEC_RDY_FAULT;
	}

	return ALL_GOOD;
}

/**
 * @brief Check status registers of a UCC5870 to check for faults
 *
 * Status registers are read and ran through a fault mask. Any unmasked faults are reported.
 *
 * @param[in] i - Index of the UCC5870 in the global UCC5870 array
 *
 * @return uint16_t
 *         - PASS if no faults are present on a UCC5870
 *         - FAIL if there is a fault present on a UCC5870
 */
uint16_t diagnose_UCC5870 (uint16_t i)
{
	uint16_t   j,
	faults[5] = {0, 0, 0, 0, 0},
	faultAcc;
	uint16_t * statusReg;

	statusReg = (uint16_t *) &ucc5870[i].status1;
	faultAcc = 0;
	for (j=0; j<4; j++)  // ADC status not checked
	{
		/* --- read STATUSx register / check / accumulate fault flags (HIGH bits) ----*/
		statusReg[j] = readRegUCC5870 (GD[i], statusRegAdrs[j]);
		faults[j]    = statusReg[j] & statusFaultMask[j];
		faultAcc    += faults[j];
	}

	return ( (faultAcc == 0) ? PASS : FAIL );  // faultAcc = 0 if all ok
}

/**
 * @brief Write data to gate driver IC register and read it back to verify
 *
 * @param[in] chipAddress - Gate driver IC address
 * @param[in] regAddress  - Register address to which data should be written to
 * @param[in] data        - Data to be written to register
 *
 * @return uint16_t
 *         - PASS if data read back from IC matches data written to it
 *         - FAIL if data read back from IC does NOT match data written to it
 */
uint16_t writeVerifyRegUCC5870(uint16_t chipAddress,
								uint16_t regAddress,
								uint16_t  data)
{
	uint16_t rx_data;

	writeRegUCC5870(chipAddress, regAddress, data);
	rx_data = readRegUCC5870(chipAddress, regAddress);

	return ( (rx_data == data) ? PASS : FAIL );
}

/**
 * @brief Perform a write-verify cycle for all configuration registers of a gate driver IC
 *
 * @param[in] i - Index of the UCC5870 in the global UCC5870 array
 *
 * @return uint16_t
 *         - PASS if all configuration register data has been verified
 *         - FAIL if there is a fault during verification
 */
uint16_t writeVerify_UCC5870(uint16_t i)
{
	uint16_t   failCnt = 0;
	uint16_t * ptr = (uint16_t *) &ucc5870[i];

	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG2,    ptr[CFG2]   ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG3,    ptr[CFG3]   ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG6,    ptr[CFG6]   ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG7,    ptr[CFG7]   ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG9,    ptr[CFG9]   ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG11,   ptr[CFG11]  ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], ADCCFG,  ptr[ADCCFG] ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], DOUTCFG, ptr[DOUTCFG]) == FAIL);

	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG1,    ptr[CFG1]   ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG4,    ptr[CFG4]   ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG5,    ptr[CFG5]   ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG8,    ptr[CFG8]   ) == FAIL);
	failCnt += (writeVerifyRegUCC5870 (GD[i], CFG10,   ptr[CFG10]  ) == FAIL);

	return ( (failCnt == 0) ? PASS : FAIL );
}

