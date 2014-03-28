#ifndef __FPGA_H__
#define __FPGA_H__

#include <linux/types.h>

#define IMX6_EIM_BASE_ADDR 0x021b8000
#define IMX6_EIM_CS0_BASE (0x00)
#define IMX6_EIM_CS1_BASE (0x18)
#define IMX6_EIM_CS2_BASE (0x30)
#define IMX6_EIM_CS3_BASE (0x48)
#define IMX6_EIM_WCR (0x90)
#define IMX6_EIM_WIAR (0x94)

#define IMX6_EIM_CS0_BASE_ADDR 0x08040000
#define IMX6_EIM_CS1_BASE_ADDR 0x0c040000

#define DATA_FIFO_ADDR (IMX6_EIM_CS1_BASE_ADDR + 0xf000)

#define FPGA_W_TEST0       0
#define FPGA_W_TEST1       1

#define FPGA_W_DDR3_P2_CMD  16
#define FPGA_W_DDR3_P2_LADR 17
#define FPGA_W_DDR3_P2_HADR 18
#define FPGA_W_DDR3_P2_WEN  19
#define FPGA_W_DDR3_P2_LDAT 20
#define FPGA_W_DDR3_P2_HDAT 21

#define FPGA_W_DDR3_P3_CMD  24
#define FPGA_W_DDR3_P3_LADR 25
#define FPGA_W_DDR3_P3_HADR 26
#define FPGA_W_DDR3_P3_REN  27

#define FPGA_W_ADC_CTL       128
#define FPGA_W_ADC_SAMPLEN_L 129
#define FPGA_W_ADC_SAMPLEN_H 130

#define FPGA_W_RBK_CTL       136
#define  RBK_CTL_CLEAR_ERROR 0x4
#define  RBK_CTL_INIT        0x2
#define  RBK_CTL_ENABLE      0x1
#define FPGA_W_RBK_PAGE_L    137
#define FPGA_W_RBK_PAGE_H    138
#define FPGA_W_BURSTLEN      139

#define FPGA_R_TEST0        2048
#define FPGA_R_TEST1        2049
#define FPGA_R_DDR3_CAL     2050

#define FPGA_R_DDR3_P2_STAT 2064
#define FPGA_R_DDR3_P3_STAT 2072
#define FPGA_R_DDR3_P3_LDAT 2073
#define FPGA_R_DDR3_P3_HDAT 2074

#define FPGA_R_ADC_STAT     2176
#define FPGA_R_RBK_STAT     2177
#define FPGA_R_MEAS_BURST   2178

#define FPGA_R_DDR3_V_MINOR 4094
#define FPGA_R_DDR3_V_MAJOR 4095

struct spi_device;
struct clk;
struct pinctrl;
struct pinctrl_state;
struct regmap;
struct i2c_device;

struct kosagi_fpga {
        struct spi_device *spi;
        struct clk *lvds_clk, *lvds_switch_clk, *lvds_parent_clk, *eim_slow_clk;
        struct pinctrl *pinctrl;
        struct pinctrl_state *pins_eim;
        struct pinctrl_state *pins_gpio;
        int power_gpio;
        int reset_gpio;

        void __iomem *eim_area;
        void __iomem *byte_area;
        u16  __iomem *fpga_ctrl;
        struct regmap *iomuxc_gpr;

        unsigned int byte_size;
        u8 dma_buffer[4096];
};

#endif /* __FPGA_H__ */
