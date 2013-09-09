/* init: initialize external memory
 *
 * The CONFIG_* settings are board specific, but the method for programming
 * them is board independent.  You should be able to copy and paste this file
 * into your project pretty easily.
 *
 * Each function is independent of the other.  You can take only what you need.
 * For more information on each register setting, consult the HRM.
 */

#include <blackfin.h>
#include <cdefBF534.h>

/* Clock settings for 485mhz CCLK and 121mhz SCLK with 22.118MHz crystal */
#define CONFIG_PLL_LOCKCNT_VAL  0x0300
#define CONFIG_VR_CTL_VAL       0x40fb
#define CONFIG_PLL_CTL_VAL      0x2c00
#define CONFIG_PLL_DIV_VAL      0x0004
/* External memory settings for 32meg chip */
#define CONFIG_EBIU_SDRRC_VAL   0x0817
#define CONFIG_EBIU_SDBCTL_VAL  0x0013
#define CONFIG_EBIU_SDGCTL_VAL  0x0091998d

static inline void init_program_clocks(void)
{
	/* Disable all peripheral wakeups except for the PLL event. */
	*pSIC_IWR = 1;

	*pPLL_LOCKCNT = CONFIG_PLL_LOCKCNT_VAL;

	/* Only reprogram when needed to avoid triggering unnecessary
	 * PLL relock sequences.
	 */
	if (*pVR_CTL != CONFIG_VR_CTL_VAL) {
		*pVR_CTL = CONFIG_VR_CTL_VAL;
		asm("idle;");
	}

	*pPLL_DIV = CONFIG_PLL_DIV_VAL;

	/* Only reprogram when needed to avoid triggering unnecessary
	 * PLL relock sequences.
	 */
	if (*pPLL_CTL != CONFIG_PLL_CTL_VAL) {
		*pPLL_CTL = CONFIG_PLL_CTL_VAL;
		asm("idle;");
	}

	/* Restore all peripheral wakeups. */
	*pSIC_IWR = -1;
}

static inline void init_program_memory_controller(void)
{
	/* Program the external memory controller. */
	*pEBIU_SDRRC  = CONFIG_EBIU_SDRRC_VAL;
	*pEBIU_SDBCTL = CONFIG_EBIU_SDBCTL_VAL;
	*pEBIU_SDGCTL = CONFIG_EBIU_SDGCTL_VAL;
}

__attribute__((saveall))
void initcode(void)
{
	init_program_clocks();
	init_program_memory_controller();
}
