void afe_setgain(unsigned int auxpwr, unsigned int filter, unsigned int preamp, int attenuation );

#define LMH6518_AUXPWR_BIT  10
#define LMH6518_AUXPWR_ON   0
#define LMH6518_AUXPWR_OFF  1

#define LMH6518_FILT_BIT   6
#define LMH6518_FILT_FULL  0
#define LMH6518_FILT_20MHZ 1
#define LMH6518_FILT_100MHZ 2
#define LMH6518_FILT_200MHZ 3
#define LMH6518_FILT_350MHZ 4
#define LMH6518_FILT_650MHZ 5
#define LMH6518_FILT_750MHZ 6

#define LMH6518_PREAMP_BIT  4
#define LMH6518_PREAMP_LG   0
#define LMH6518_PREAMP_HG   1

#define LMH6518_ATTEN_BIT  0
// 0 dB attenuation (big signal passed through)
#define LMH6518_ATTEN_0DB  0    
#define LMH6518_ATTEN_2DB  1
#define LMH6518_ATTEN_4DB  2
#define LMH6518_ATTEN_6DB  3
#define LMH6518_ATTEN_8DB  4
#define LMH6518_ATTEN_10DB  5
#define LMH6518_ATTEN_12DB  6
#define LMH6518_ATTEN_14DB  7
#define LMH6518_ATTEN_16DB  8
#define LMH6518_ATTEN_18DB  9
// -20 dB attenuation (big signal turned into small signal)
#define LMH6518_ATTEN_20DB  10

