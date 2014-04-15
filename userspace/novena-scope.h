struct reg_info {
  char *name;
  int offset;
  int size;
  char *description;
};

#define FPGA_REG_OFFSET    0x08040000
#define FPGA_CS1_REG_OFFSET    0x0C040000

#define FPGA_MAP(x)         ( (x - FPGA_REG_OFFSET) >> 1 )
#define F(x)                ( (x - FPGA_REG_OFFSET) >> 1 )
#define F1(x)                ( (x - FPGA_CS1_REG_OFFSET) >> 3 )

#define FPGA_W_TEST0       0x08040000
#define FPGA_W_TEST1       0x08040002

#define FPGA_W_DDR3_P2_CMD  0x08040020
#define FPGA_W_DDR3_P2_LADR 0x08040022
#define FPGA_W_DDR3_P2_HADR 0x08040024
#define FPGA_W_DDR3_P2_WEN  0x08040026
#define FPGA_W_DDR3_P2_LDAT 0x08040028
#define FPGA_W_DDR3_P2_HDAT 0x0804002A

#define FPGA_W_DDR3_P3_CMD  0x08040030
#define FPGA_W_DDR3_P3_LADR 0x08040032
#define FPGA_W_DDR3_P3_HADR 0x08040034
#define FPGA_W_DDR3_P3_REN  0x08040036

#define FPGA_W_ADC_CTL       0x08040100
#define FPGA_W_ADC_SAMPLEN_L 0x08040102
#define FPGA_W_ADC_SAMPLEN_H 0x08040104

#define FPGA_W_RBK_CTL       0x08040110
#define  RBK_CTL_CLEAR_ERROR 0x4
#define  RBK_CTL_INIT        0x2
#define  RBK_CTL_ENABLE      0x1
#define FPGA_W_RBK_PAGE_L    0x08040112
#define FPGA_W_RBK_PAGE_H    0x08040114
#define FPGA_W_BURSTLEN      0x08040116


#define FPGA_R_TEST0        0x08041000
#define FPGA_R_TEST1        0x08041002
#define FPGA_R_DDR3_CAL     0x08041004

#define FPGA_R_DDR3_P2_STAT 0x08041020
#define FPGA_R_DDR3_P3_STAT 0x08041030
#define FPGA_R_DDR3_P3_LDAT 0x08041032
#define FPGA_R_DDR3_P3_HDAT 0x08041034

#define FPGA_R_ADC_STAT     0x08041100
#define FPGA_R_RBK_STAT     0x08041102
#define FPGA_R_MEAS_BURST   0x08041104

#define FPGA_R_DDR3_V_MINOR 0x08041FFC
#define FPGA_R_DDR3_V_MAJOR 0x08041FFE

// burst access registers (in CS1 bank -- only 64-bit access allowed)
#define FPGA_WB_LOOP0       0x0C040000
#define FPGA_WB_LOOP1       0x0C040008

#define FPGA_RBK_REGION     0x0C04F000 // any read to this page incs FIFO and returns next data

int read_kernel_memory(long offset, int virtualized, int size);
int write_kernel_memory(long offset, long value, int virtualized, int size);
