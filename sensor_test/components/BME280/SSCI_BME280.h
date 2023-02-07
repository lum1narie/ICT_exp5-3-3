// refer to:
// https://github.com/SWITCHSCIENCE/samplecodes/tree/31a24e5447cbe298bfb7a99ae700deb55b4f857e/BME280

#include <driver/i2c.h>
#include <esp_log.h>

// I2C Address
#define BME280_ADDRESS 0x76

// BME280 Registers
#define BME280_REG_calib00 0x88
#define BME280_REG_calib25 0xa1
#define BME280_REG_ID 0xd0
#define BME280_REG_reset 0xe0
#define BME280_REG_calib26 0xe1
#define BME280_REG_ctrl_hum 0xf2
#define BME280_REG_status 0xf3
#define BME280_REG_ctrl_meas 0xf4
#define BME280_REG_config 0xf5
#define BME280_REG_press_msb 0xf7
#define BME280_REG_press_lsb 0xf8
#define BME280_REG_press_xlsb 0xf9
#define BME280_REG_temp_msb 0xfa
#define BME280_REG_temp_lsb 0xfb
#define BME280_REG_temp_xlsb 0xfc
#define BME280_REG_hum_msb 0xfd
#define BME280_REG_hum_lsb 0xfe

// Caribration data storage
typedef struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  int8_t dig_H1;
  int16_t dig_H2;
  int8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
} BME280_calib_data;

class SSCI_BME280 {
 public:
  SSCI_BME280();
  esp_err_t setMode(uint8_t i2c_num,     // I2C port
                    uint8_t i2c_addr,    // slave I2C address
                    uint8_t osrs_t,      // Temperature oversampling
                    uint8_t osrs_p,      // Pressure oversampling
                    uint8_t osrs_h,      // Humidity oversampling
                    uint8_t bme280mode,  // Mode Sleep/Forced/Normal
                    uint8_t t_sb,        // Tstandby
                    uint8_t filter,      // Filter off
                    uint8_t spi3w_en     // 3-wire SPI Enable/Disable
  );
  esp_err_t readTrim();
  esp_err_t readData(double *temp_act, double *press_act, double *hum_act);
  void debugGetAllAddress();
  void debugPrintAllCalib();

 private:
  int32_t calibration_T(int32_t adc_T);
  uint32_t calibration_P(int32_t adc_P);
  uint32_t calibration_H(int32_t adc_H);
  esp_err_t writeReg(uint8_t reg_address, uint8_t data);
  esp_err_t readReg(uint8_t reg_address, uint8_t *data);

  signed long int t_fine;
  int _i2c_num;
  int _i2c_addr;
  uint8_t _i2c_write_code;
  uint8_t _i2c_read_code;
  BME280_calib_data calibData;
};
