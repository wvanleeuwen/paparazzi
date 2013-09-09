extern void i2cwrite(unsigned char i2c_device, unsigned char *i2c_data, unsigned int pair_count, int sccb_flag);
extern void i2cwritex(unsigned char i2c_device, unsigned char *i2c_data, unsigned int count, int sccb_flag);
extern void i2cread(unsigned char i2c_device, unsigned char *i2c_data, unsigned int pair_count, int sccb_flag);
extern void i2cslowread(unsigned char i2c_device, unsigned char *i2c_data, unsigned int pair_count, int sccb_flag);

