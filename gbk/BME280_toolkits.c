#include "BME280_register_map.h"
//<Addr>
#define BME_ADDR        0x76
#define BME_ADDR_W      (0x76<<1)
#define BME_ADDR_R      (0x76<<1)|1
//<Моды>
#define NORMAL_MODE	    0x03
#define FORCED_MODE	    0x02
#define STANDBY_500US   0x00
#define STANDBY_10MS    0x06
#define STANDBY_20MS    0x07
#define STANDBY_6250US  0x01
#define STANDBY_125MS   0x02
#define STANDBY_250MS   0x03
#define STANDBY_500MS   0x04
#define STANDBY_1000MS  0x05
#define MODULE_DISABLE	0x00
#define OVERSAMPLING_1  0x01
#define OVERSAMPLING_2  0x02
#define OVERSAMPLING_4  0x03
#define OVERSAMPLING_8  0x04
#define OVERSAMPLING_16 0x05
#define FILTER_DISABLE  0x00
#define FILTER_COEF_2   0x01
#define FILTER_COEF_4   0x02
#define FILTER_COEF_8   0x03
#define FILTER_COEF_16  0x04

bool BME_INI(void);
void GET_TEMPERATURE(float*); // BME280_compensate_T_int32
void GET_PRESSURE(float*);    // BME280_compensate_P_int64
void GET_HUMIDITY(float*);    // BME280_Compensate_H_int32
void GET_ALTITUDE_VERTSPEED(void);

struct {												// Structure to store all calibration values
    uint16_t _T1;
    int16_t _T2;
    int16_t _T3;
    uint16_t _P1;
    int16_t _P2;
    int16_t _P3;
    int16_t _P4;
    int16_t _P5;
    int16_t _P6;
    int16_t _P7;
    int16_t _P8;
    int16_t _P9;
    uint8_t _H1;
    int16_t _H2;
    uint8_t _H3;
    int16_t _H4;
    int16_t _H5;
     int8_t _H6;
} CalibrationData;
int32_t t_fine;


bool BME_INI() {
    // Настроечные константы:
    uint8_t _operating_mode = NORMAL_MODE;		// Sensor operation mode
    uint8_t _standby_time = STANDBY_10MS;		// Time between measurements in NORMAL_MODE
    uint8_t _filter_coef = FILTER_COEF_4;      // Filter ratio IIR
    uint8_t _temp_oversampl = OVERSAMPLING_4;   // Temperature module oversampling parameter
    uint8_t _hum_oversampl = OVERSAMPLING_1;	// Humidity module oversampling parameter
    uint8_t _press_oversampl = OVERSAMPLING_4; // Pressure module oversampling parameter
    // <BME280 software reset & ack check>
    if (!writeRegister(BME_ADDR, BME_RESET, 0xB6)) return false;
    _delay_ms(10);
    // <Чтение ID>
    uint8_t ID;
    if (!readRegister(BME_ADDR, BME_ID, &ID)) return false;
    if (ID != 0x60 && ID != 0x58) return false;	    // Check chip ID (bme/bmp280)
    // <Загрузка настроек в BME>
    if (!writeRegister(BME_ADDR, BME_CONFIG, ((_standby_time << BME_T_SB0) |(_filter_coef << BME_FILTER0)))) return false;
    // write temp & press oversampling value , normal mode
    if (!writeRegister(BME_ADDR, BME_CTRL_MEAS, ((_temp_oversampl << BME_OSRS_T0) |
        (_press_oversampl << BME_OSRS_P0) | _operating_mode))) return false;
    // write hum oversampling value
    if (!writeRegister(BME_ADDR, BME_CTRL_HUM, _hum_oversampl)) return false;
    // rewrite hum oversampling register
    uint8_t data1, data2;
    if (!readRegister(BME_ADDR, BME_CTRL_HUM, &data1)) return false;
    if (!writeRegister(BME_ADDR, BME_CTRL_HUM, data1)) return false;
    // Чтение калибровочных констант
    if (!readRegister(BME_ADDR, BME_CALIB00, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB01, &data2)) return false;
    CalibrationData._T1 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB02, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB03, &data2)) return false;
    CalibrationData._T2 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB04, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB05, &data2)) return false;
    CalibrationData._T3 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB06, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB07, &data2)) return false;
    CalibrationData._P1 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB08, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB09, &data2)) return false;
    CalibrationData._P2 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB10, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB11, &data2)) return false;
    CalibrationData._P3 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB12, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB13, &data2)) return false;
    CalibrationData._P4 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB14, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB15, &data2)) return false;
    CalibrationData._P5 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB16, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB17, &data2)) return false;
    CalibrationData._P6 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB18, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB19, &data2)) return false;
    CalibrationData._P7 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB20, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB21, &data2)) return false;
    CalibrationData._P8 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB22, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB23, &data2)) return false;
    CalibrationData._P9 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB25, &data1)) return false;
    CalibrationData._H1 = data1;
    if (!readRegister(BME_ADDR, BME_CALIB26, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB27, &data2)) return false;
    CalibrationData._H2 = data1 | (data2 << 8);
    if (!readRegister(BME_ADDR, BME_CALIB28, &data1)) return false;
    CalibrationData._H3 = data1;
    if (!readRegister(BME_ADDR, BME_CALIB29, &data1)) return false;
    if (!readRegister(BME_ADDR, BME_CALIB30, &data2)) return false;
    CalibrationData._H4 = (data1 << 4) | (data2 & 0x0F);
    if (!readRegister(BME_ADDR, BME_CALIB31, &data1)) return false;
    CalibrationData._H5 = (data1 << 4) | ((data2 >> 4) & 0x0F);
    if (!readRegister(BME_ADDR, BME_CALIB32, &data1)) return false;
    CalibrationData._H5 = data1;
    return true;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23
//DegC.
// t_fine carries fine temperature as global value
void GET_TEMPERATURE(float *temperature_out) {
    // Другое название - BME280_compensate_T_int32
    int32_t adc_T = ((uint32_t)raw2_temperature[0] << 12)|
                    ((uint32_t)raw2_temperature[1] << 4)|
                    ((uint32_t)raw2_temperature[2] >> 4);
    adc_T &= 0x000FFFFF;
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)CalibrationData._T1 << 1)))
		* ((int32_t)CalibrationData._T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)CalibrationData._T1))
		* ((adc_T >> 4) - ((int32_t)CalibrationData._T1))) >> 12)
		* ((int32_t)CalibrationData._T3)) >> 14;
	t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    *temperature_out = T/100.0;
}
void GET_PRESSURE(float *pressure_out) {
    // Другое название - BME280_compensate_P_int64
    int32_t adc_P = ((uint32_t)raw2_pressure[0] << 12)|
                    ((uint32_t)raw2_pressure[1] << 4)|
                    ((uint32_t)raw2_pressure[2] >> 4);
    adc_P &= 0x000FFFFF;
    int64_t var1, var2, p;
    //adc_P >>= 4;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)CalibrationData._P6;
    var2 = var2 + ((var1*(int64_t)CalibrationData._P5)<<17);
    var2 = var2 + (((int64_t)CalibrationData._P4)<<35);
    var1 = ((var1 * var1 * (int64_t)CalibrationData._P3)>>8) +
        ((var1 * (int64_t)CalibrationData._P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)CalibrationData._P1)>>33;
    if (var1 == 0) {
        *pressure_out = 0; // avoid exception caused by division by zero
        return;
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)CalibrationData._P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)CalibrationData._P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)CalibrationData._P7)<<4);
    *pressure_out = (float)p/256.0;
}
void GET_HUMIDITY(float *humidity_out) {
    // Другое название - bme280_compensate_H_int32
    int32_t adc_H = (((uint32_t)raw2_humidity[0] << 8) | (uint32_t)raw2_humidity[1]);
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)CalibrationData._H4) << 20) -
        (((int32_t)CalibrationData._H5) * v_x1_u32r)) +
        ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
        ((int32_t)CalibrationData._H6)) >> 10) *
        (((v_x1_u32r * ((int32_t)CalibrationData._H3)) >> 11) +
        ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
        ((int32_t)CalibrationData._H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
        ((int32_t)CalibrationData._H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    *humidity_out = (float)(v_x1_u32r>>12)/1024.0;
}
void GET_ALTITUDE_VERTSPEED() {
    Altitude = 44330.0 * (1.0 - pow(Pressure / 102599, 0.1903));
    VertSpeed = (Altitude - pre_Altitude)/deltat;
}

