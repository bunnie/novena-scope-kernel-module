void default_adc08d1020();
int adc08d1020_calrun_stat();
void cal_adc08d1020();
void testpattern_adc08d1020();

// CPU-FPGA I2C API mapping
#define FPGA_ADC_WDATA_LSB  0x4
#define FPGA_ADC_WDATA_MSB  0x5
#define FPGA_ADC_COMMIT_ADR 0x6
#define FPGA_ADC_STAT       0x42

// address locations of various commands for ADC
#define ADC08D1020_CAL           0x0
#define ADC08D1020_CONFIG        0x1
#define ADC08D1020_I_OFFSET      0x2
#define ADC08D1020_I_FSADJ       0x3
#define ADC08D1020_EXTCONFIG     0x9
#define ADC08D1020_Q_OFFSET      0xA
#define ADC08D1020_Q_FSADJ       0xB
#define ADC08D1020_FINE_PHASE    0xE
#define ADC08D1020_COARSE_PHASE  0xF


