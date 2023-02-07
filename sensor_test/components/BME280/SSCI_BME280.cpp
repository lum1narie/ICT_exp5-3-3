// refer to:
// https://github.com/SWITCHSCIENCE/samplecodes/tree/31a24e5447cbe298bfb7a99ae700deb55b4f857e/BME280
//
// modified to use with ESP32

#include "SSCI_BME280.h"

esp_err_t SSCI_BME280::writeReg(uint8_t reg_address, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2c_write_code, true);
  i2c_master_write_byte(cmd, reg_address, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t SSCI_BME280::readReg(uint8_t reg_address, uint8_t *data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2c_write_code, true);
  i2c_master_write_byte(cmd, reg_address, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2c_read_code, true);
  i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

SSCI_BME280::SSCI_BME280() {}

esp_err_t SSCI_BME280::setMode(uint8_t i2c_num, uint8_t i2c_addr,
                               uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h,
                               uint8_t bme280mode, uint8_t t_sb, uint8_t filter,
                               uint8_t spi3w_en) {
  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | bme280mode;
  uint8_t config_reg = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg = osrs_h;
  esp_err_t err;

  _i2c_num = i2c_num;
  _i2c_addr = i2c_addr;
  _i2c_write_code = i2c_addr << 1;
  _i2c_read_code = _i2c_write_code | 0x01;

  err = writeReg(BME280_REG_ctrl_hum, ctrl_hum_reg);
  if (err != ESP_OK) {
    ESP_LOGW("SSCI_BME280", "error on write ctrl_hum_reg");
    return err;
  }

  err = writeReg(BME280_REG_ctrl_meas, ctrl_meas_reg);
  if (err != ESP_OK) {
    ESP_LOGW("SSCI_BME280", "error on write ctrl_meas_reg");
    return err;
  }

  err = writeReg(BME280_REG_config, config_reg);
  if (err != ESP_OK) {
    ESP_LOGW("SSCI_BME280", "error on write config_reg");
    return err;
  }

  return ESP_OK;
}

esp_err_t SSCI_BME280::readTrim() {
  uint8_t data[33];
  ptrdiff_t i = 0;
  esp_err_t err;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2c_write_code, true);
  i2c_master_write_byte(cmd, BME280_REG_calib00, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2c_read_code, true);
  i2c_master_read(cmd, data + i, 24, I2C_MASTER_LAST_NACK);
  i += 24;
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (err != ESP_OK) {
    ESP_LOGW("SSCI_BME280", "error on read data[:25]");
    return err;
  }

  err = readReg(BME280_REG_calib25, data + i++);
  if (err != ESP_OK) {
    ESP_LOGW("SSCI_BME280", "error on read data[25]");
    return err;
  }

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2c_write_code, true);
  i2c_master_write_byte(cmd, BME280_REG_calib26, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2c_read_code, true);
  i2c_master_read(cmd, data + i, 8, I2C_MASTER_LAST_NACK);
  // i += 8;
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (err != ESP_OK) {
    ESP_LOGW("SSCI_BME280", "error on read data[26:]");
    return err;
  }

  calibData.dig_T1 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
  calibData.dig_T2 = ((uint16_t)data[3] << 8) | (uint16_t)data[2];
  calibData.dig_T3 = ((uint16_t)data[5] << 8) | (uint16_t)data[4];
  calibData.dig_P1 = ((uint16_t)data[7] << 8) | (uint16_t)data[6];
  calibData.dig_P2 = ((uint16_t)data[9] << 8) | (uint16_t)data[8];
  calibData.dig_P3 = ((uint16_t)data[11] << 8) | (uint16_t)data[10];
  calibData.dig_P4 = ((uint16_t)data[13] << 8) | (uint16_t)data[12];
  calibData.dig_P5 = ((uint16_t)data[15] << 8) | (uint16_t)data[14];
  calibData.dig_P6 = ((uint16_t)data[17] << 8) | (uint16_t)data[16];
  calibData.dig_P7 = ((uint16_t)data[19] << 8) | (uint16_t)data[18];
  calibData.dig_P8 = ((uint16_t)data[21] << 8) | (uint16_t)data[20];
  calibData.dig_P9 = ((uint16_t)data[23] << 8) | (uint16_t)data[22];
  calibData.dig_H1 = data[24];
  calibData.dig_H2 = ((uint16_t)data[26] << 8) | (uint16_t)data[25];
  calibData.dig_H3 = data[27];
  calibData.dig_H4 = ((uint16_t)data[28] << 4) | (0x0F & (uint16_t)data[29]);
  calibData.dig_H5 =
      ((uint16_t)data[30] << 4) | (((uint16_t)data[29] >> 4) & 0x0F);
  calibData.dig_H6 = data[31];

  return ESP_OK;
}

esp_err_t SSCI_BME280::readData(double *temp_act, double *press_act,
                                double *hum_act) {
  uint8_t data[8];
  uint32_t hum_raw, press_raw, temp_raw;
  esp_err_t err;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2c_write_code, true);
  i2c_master_write_byte(cmd, BME280_REG_press_msb, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2c_read_code, true);
  i2c_master_read(cmd, data, 8, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (err != ESP_OK) {
    return err;
  }

  press_raw = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) |
              ((uint32_t)data[2] >> 4);
  temp_raw = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) |
             ((uint32_t)data[5] >> 4);
  hum_raw = ((uint32_t)data[6] << 8) | (uint32_t)data[7];
  *temp_act = (double)calibration_T(temp_raw) / 100.0;
  *press_act = (double)calibration_P(press_raw) / 100.0;
  *hum_act = (double)calibration_H(hum_raw) / 1024.0;

  return ESP_OK;
}

int32_t SSCI_BME280::calibration_T(int32_t adc_T) {
  int32_t var1, var2, T;

  var1 = ((((adc_T >> 3) - ((int32_t)calibData.dig_T1 << 1))) *
          ((int32_t)calibData.dig_T2)) >>
         11;
  var2 = (((((adc_T >> 4) - ((int32_t)calibData.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)calibData.dig_T1))) >>
           12) *
          ((int32_t)calibData.dig_T3)) >>
         14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}
uint32_t SSCI_BME280::calibration_P(int32_t adc_P) {
  int32_t var1, var2;
  uint32_t P;

  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calibData.dig_P6);
  var2 = var2 + ((var1 * ((int32_t)calibData.dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)calibData.dig_P4) << 16);
  var1 = (((calibData.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
          ((((int32_t)calibData.dig_P2) * var1) >> 1)) >>
         18;
  var1 = ((((32768 + var1)) * ((int32_t)calibData.dig_P1)) >> 15);
  if (var1 == 0) {
    return 0;
  }
  P = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (P < 0x80000000) {
    P = (P << 1) / ((uint32_t)var1);
  } else {
    P = (P / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)calibData.dig_P9) *
          ((int32_t)(((P >> 3) * (P >> 3)) >> 13))) >>
         12;
  var2 = (((int32_t)(P >> 2)) * ((int32_t)calibData.dig_P8)) >> 13;
  P = (uint32_t)((int32_t)P + ((var1 + var2 + calibData.dig_P7) >> 4));
  return P;
}

uint32_t SSCI_BME280::calibration_H(int32_t adc_H) {
  int32_t v_x1;

  v_x1 = (t_fine - ((int32_t)76800));
  v_x1 = (((((adc_H << 14) - (((int32_t)calibData.dig_H4) << 20) -
             (((int32_t)calibData.dig_H5) * v_x1)) +
            ((int32_t)16384)) >>
           15) *
          (((((((v_x1 * ((int32_t)calibData.dig_H6)) >> 10) *
               (((v_x1 * ((int32_t)calibData.dig_H3)) >> 11) +
                ((int32_t)32768))) >>
              10) +
             ((int32_t)2097152)) *
                ((int32_t)calibData.dig_H2) +
            8192) >>
           14));
  v_x1 =
      (v_x1 -
       (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)calibData.dig_H1)) >>
        4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (uint32_t)(v_x1 >> 12);
}

void SSCI_BME280::debugGetAllAddress() {
  uint8_t addr = 0;
  uint8_t data = 0;
  esp_err_t err;

  for (addr = BME280_REG_calib00; addr <= BME280_REG_calib25; ++addr) {
    err = SSCI_BME280::readReg(addr, &data);
    ESP_ERROR_CHECK(err);
    printf("0x%02x: %d\n", addr, data);
  }

  addr = BME280_REG_ID;
  err = SSCI_BME280::readReg(addr, &data);
  ESP_ERROR_CHECK(err);
  printf("0x%02x: %d\n", addr, data);

  for (addr = BME280_REG_calib26; addr <= BME280_REG_hum_lsb; ++addr) {
    err = SSCI_BME280::readReg(addr, &data);
    ESP_ERROR_CHECK(err);
    printf("0x%02x: %d\n", addr, data);
  }

}

void SSCI_BME280::debugPrintAllCalib() {
  printf(
      "T1: %d\n"
      "T2: %d\n"
      "T3: %d\n"
      "P1: %d\n"
      "P2: %d\n"
      "P3: %d\n"
      "P4: %d\n"
      "P5: %d\n"
      "P6: %d\n"
      "P7: %d\n"
      "P8: %d\n"
      "P9: %d\n"
      "H1: %d\n"
      "H2: %d\n"
      "H3: %d\n"
      "H4: %d\n"
      "H5: %d\n"
      "H6: %d\n",
      calibData.dig_T1, calibData.dig_T2, calibData.dig_T3, calibData.dig_P1,
      calibData.dig_P2, calibData.dig_P3, calibData.dig_P4, calibData.dig_P5,
      calibData.dig_P6, calibData.dig_P7, calibData.dig_P8, calibData.dig_P9,
      calibData.dig_H1, calibData.dig_H2, calibData.dig_H3, calibData.dig_H4,
      calibData.dig_H5, calibData.dig_H6);

}
