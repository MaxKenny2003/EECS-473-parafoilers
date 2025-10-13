/*
 * @file BNO085_SPI_Library.c
 * @brief SPI Interface and service routine functions for BNO085 via SHTP
 * protocol used by the manufacturer CEVA.
 */

// --- Includes -------------------------------------------------------
// Private includes
#include "BNO085_SPI_Library.h"

// --- Macros and data ------------------------------------------------
// Channels for communication via SHTP (cf. [1], p. 22)
typedef uint8_t byte;
const byte CHANNEL_COMMAND = 0;
const byte CHANNEL_EXECUTABLE = 1;
const byte CHANNEL_CONTROL = 2;
const byte CHANNEL_REPORTS = 3;
// wake and gyro reports not used, only for completeness
const byte CHANNEL_WAKE_REPORTS = 4;
const byte CHANNEL_GYRO = 5;

// Report ID Convention (cf. [2], p. 28 f., figure 32)
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_GET_FEATURE_RESPONSE 0xFC
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

// Reset Report ID (cf. [1], p. 23; [2], p. 55)
#define SHTP_RESET_COMMAND_RESPONSE 1
#define SHTP_RESET_COMMAND_REQUEST 1

// Commands we want use (cf. [2], p. 44 f.; p. 47 ff.)
#define SENSOR_COMMAND_INITIALIZE_RESPONSE 0x04
#define SENSOR_COMMAND_INITIALIZE_RESPONSE_UNSOLICITED 0x84

// Feature reports we want use and can get reports from (cf. [2], p. 38 f., p.
// 71 f., p. 84 f.)
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_ARVR_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_ARVR_GAME_ROTATION_VECTOR 0x29
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13

// Reset of the executable channel, reset complete packet (cf. [1], p.23, figure
// 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

#define MAX_PACKET_SIZE 128
#define HEADER_PACKET_SIZE 4
#define CHANNEL_MAX_NUMBER 6
#define RESET_DELAY_US 100

// Q values for quaternion calculation (cf. [1], p. 25, figure 1-32 and [2], p.
// 71 f.)
int16_t rotationVector_Q1 = 14;
int16_t rotationVectorAccuracy_Q1 = 12;
int16_t accelerometer_Q1 = 8;

bool debug_print = false;

// --- SPI interface --------------------------------------------------

void SPI_Transmit(uint8_t tx_data) {
  uint8_t rx_data;
  HAL_SPI_TransmitReceive(hspi_ptr, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
}

uint8_t SPI_TransmitReceive_Return_Byte(uint8_t tx_data) {
  uint8_t rx_data;
  HAL_SPI_TransmitReceive(hspi_ptr, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
  return rx_data;
}

// --- Private methods ------------------------------------------------

static void rstn(ports_pins ports_pins_config, bool state) {
  HAL_GPIO_WritePin(ports_pins_config.RSTN_Port, ports_pins_config.RSTN_Pin,
                    state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void csn(ports_pins ports_pins_config, bool state) {
  HAL_GPIO_WritePin(ports_pins_config.CSN_Port, ports_pins_config.CSN_Pin,
                    state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void reset_Hardware(ports_pins ports_pins_config) {
  rstn(ports_pins_config, false);
  delay_Us(RESET_DELAY_US);
  rstn(ports_pins_config, true);
}

static uint8_t wait_for_INTN(sensor_meta *sensor) {
  uint32_t timeout_counter = 0;
  const uint32_t TIMEOUT_MS = 500;

  while (HAL_GPIO_ReadPin(sensor->ports_pins.INTN_Port, sensor->ports_pins.INTN_Pin) != GPIO_PIN_RESET) {
    if (timeout_counter >= TIMEOUT_MS) {
      return D_ERR;
    }
    timeout_counter++;
    delay_Us(1000); // 1 ms delay
  }
  sensor->INTN_ready = 1;
  return N_ERR;
}

static void check_INTN(sensor_meta *sensor) {
  if (HAL_GPIO_ReadPin(sensor->ports_pins.INTN_Port,
                        sensor->ports_pins.INTN_Pin) == GPIO_PIN_RESET) {
      sensor->INTN_ready = 1;
  } else {
      sensor->INTN_ready = 0;
  }
}

static float fixpoint_to_float(int16_t fixpoint_Value, uint8_t q_Point) {
  float fixpoint_Value_temp = fixpoint_Value;
  fixpoint_Value_temp *= powf(2, q_Point * -1);
  return (fixpoint_Value_temp);
}

static uint8_t receive_Data(sensor_meta *sensor) {
  if (HAL_GPIO_ReadPin(sensor->ports_pins.INTN_Port, sensor->ports_pins.INTN_Pin) != GPIO_PIN_RESET) {
      return D_NOT;
  }

  sensor->INTN_ready = 0;

  csn(sensor->ports_pins, false);

  uint8_t tx_dummy_data = 0;
  sensor->shtp_package.shtp_Header[0] = SPI_TransmitReceive_Return_Byte(tx_dummy_data);
  sensor->shtp_package.shtp_Header[1] = SPI_TransmitReceive_Return_Byte(tx_dummy_data);
  sensor->shtp_package.shtp_Header[2] = SPI_TransmitReceive_Return_Byte(tx_dummy_data);
  sensor->shtp_package.shtp_Header[3] = SPI_TransmitReceive_Return_Byte(tx_dummy_data);

  uint16_t data_Length = (((uint16_t)sensor->shtp_package.shtp_Header[1]) << 8) | ((uint16_t)sensor->shtp_package.shtp_Header[0]);
  data_Length &= 0x7FFF;

  if (data_Length == 0) {
    csn(sensor->ports_pins, true);
    return D_NOT;
  }

  if (data_Length < 4) {
      csn(sensor->ports_pins, true);
      return D_ERR;
  }

  data_Length -= 4;

  for (uint16_t i = 0; i < data_Length; i++) {
    if (i < MAX_PACKET_SIZE) {
      sensor->shtp_package.shtp_Data[i] = SPI_TransmitReceive_Return_Byte(tx_dummy_data);
    } else {
      SPI_TransmitReceive_Return_Byte(tx_dummy_data); // Read and discard extra bytes
    }
  }

  csn(sensor->ports_pins, true);

  if (sensor->shtp_package.shtp_Header[2] == CHANNEL_EXECUTABLE &&
      sensor->shtp_package.shtp_Data[0] == EXECUTABLE_RESET_COMPLETE) {
    sensor->has_reset = true;
  }

  return N_ERR;
}

static uint8_t send_Data(sensor_meta *sensor, uint8_t channel_Number,
                         uint16_t data_Length) {
  uint16_t packet_Length = data_Length + 4;

  csn(sensor->ports_pins, false);

  SPI_Transmit(packet_Length & 0xFF);
  SPI_Transmit(packet_Length >> 8);
  SPI_Transmit(channel_Number);
  SPI_Transmit(sensor->shtp_package.sequence_Number[channel_Number]++);

  for (uint16_t i = 0; i < data_Length; i++) {
    SPI_Transmit(sensor->shtp_package.shtp_Data[i]);
  }

  csn(sensor->ports_pins, true);

  return N_ERR;
}

static uint8_t set_FeatureCommand(sensor_meta *sensor, uint8_t report_ID,
                                  uint32_t time_between_reports_us,
                                  uint32_t specific_Config) {

  sensor->shtp_package.shtp_Data[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
  sensor->shtp_package.shtp_Data[1] = report_ID;
  sensor->shtp_package.shtp_Data[2] = 0;
  sensor->shtp_package.shtp_Data[3] = 0;
  sensor->shtp_package.shtp_Data[4] = 0;
  sensor->shtp_package.shtp_Data[5] = (time_between_reports_us >> 0) & 0xFF;
  sensor->shtp_package.shtp_Data[6] = (time_between_reports_us >> 8) & 0xFF;
  sensor->shtp_package.shtp_Data[7] = (time_between_reports_us >> 16) & 0xFF;
  sensor->shtp_package.shtp_Data[8] = (time_between_reports_us >> 24) & 0xFF;
  for(int i = 9; i <= 12; i++) sensor->shtp_package.shtp_Data[i] = 0;
  sensor->shtp_package.shtp_Data[13] = (specific_Config >> 0) & 0xFF;
  sensor->shtp_package.shtp_Data[14] = (specific_Config >> 8) & 0xFF;
  sensor->shtp_package.shtp_Data[15] = (specific_Config >> 16) & 0xFF;
  sensor->shtp_package.shtp_Data[16] = (specific_Config >> 24) & 0xFF;

  return send_Data(sensor, CHANNEL_CONTROL, 17);
}

// *** MODIFICATION ***
// This is the new robust parser. It iterates through the packet to find the correct report.
// Add to your parse_InputReport function - modify the existing one
uint16_t parse_InputReport(sensor_meta *sensor) {
    if (sensor == NULL) return 0;

    uint16_t data_len = (sensor->shtp_package.shtp_Header[1] << 8) |
                        sensor->shtp_package.shtp_Header[0];
    data_len &= 0x7FFF;
    if(data_len < 4) return 0;
    data_len -= 4;

    uint16_t cursor = 0;
    uint16_t last_report_found = 0;

    static uint32_t debug_counter = 0;
    bool debug_this_packet = (debug_counter++ % 50 == 0); // Debug every 50th packet

    // Skip timestamp if present
    if (cursor < data_len &&
        sensor->shtp_package.shtp_Data[cursor] == SHTP_REPORT_BASE_TIMESTAMP) {
        cursor += 5;
    }

    // Look for reports
    while (cursor < data_len) {
        uint8_t reportID = sensor->shtp_package.shtp_Data[cursor];
        uint8_t* report = &sensor->shtp_package.shtp_Data[cursor];

        if (debug_this_packet) {
            printf("  [Parser] Found reportID: 0x%02X at cursor=%d\r\n", reportID, cursor);
        }

        // Game Rotation Vector
        if (reportID == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
            if ((cursor + 12) <= data_len) {
                sensor->quaternions.quat_Accuracy = report[2] & 0x03;
                sensor->quaternions.raw_Quat_I = (int16_t)((report[5] << 8) | report[4]);
                sensor->quaternions.raw_Quat_J = (int16_t)((report[7] << 8) | report[6]);
                sensor->quaternions.raw_Quat_K = (int16_t)((report[9] << 8) | report[8]);
                sensor->quaternions.raw_Quat_Real = (int16_t)((report[11] << 8) | report[10]);
                last_report_found = reportID;
                cursor += 12;
                if (debug_this_packet) {
                    printf("  [Parser] Parsed Game Rotation Vector\r\n");
                }
            } else {
                if (debug_this_packet) printf("  [Parser] Not enough data for Game Rot\r\n");
                break;
            }
        }
        // Linear Acceleration
        else if (reportID == SENSOR_REPORTID_LINEAR_ACCELERATION) {
            if ((cursor + 10) <= data_len) {
                sensor->linear_acceleration_data.accelerometer_Accuracy = report[2] & 0x03;
                sensor->linear_acceleration_data.raw_Accel_X = (int16_t)((report[5] << 8) | report[4]);
                sensor->linear_acceleration_data.raw_Accel_Y = (int16_t)((report[7] << 8) | report[6]);
                sensor->linear_acceleration_data.raw_Accel_Z = (int16_t)((report[9] << 8) | report[8]);
                last_report_found = reportID;
                cursor += 10;
                if (debug_this_packet) {
                    printf("  [Parser] Parsed Linear Accel: X=%d Y=%d Z=%d\r\n",
                           sensor->linear_acceleration_data.raw_Accel_X,
                           sensor->linear_acceleration_data.raw_Accel_Y,
                           sensor->linear_acceleration_data.raw_Accel_Z);
                }
            } else {
                if (debug_this_packet) printf("  [Parser] Not enough data for Lin Accel\r\n");
                break;
            }
        }
        else {
            if (debug_this_packet) {
                printf("  [Parser] Unknown reportID: 0x%02X, stopping parse\r\n", reportID);
            }
            break;
        }
    }

    return last_report_found;
}


static uint16_t get_Readings(sensor_meta *sensor) {
  if (receive_Data(sensor) == N_ERR) {
    uint8_t channel = sensor->shtp_package.shtp_Header[2];
    if (channel == CHANNEL_REPORTS) {
        return parse_InputReport(sensor);
    } else if (channel == CHANNEL_CONTROL) {
        if(sensor->shtp_package.shtp_Data[0] == SHTP_REPORT_COMMAND_RESPONSE || sensor->shtp_package.shtp_Data[0] == SHTP_REPORT_GET_FEATURE_RESPONSE) {
            return sensor->shtp_package.shtp_Data[0];
        }
    }
  }
  return 0; // No data or error
}

// --- Public methods -------------------------------------------------

void register_Sensor(sensor_meta *sensor, uint8_t sensor_number,
                     uint16_t sensor_CSN_Pin, GPIO_TypeDef *sensor_CSN_Port,
                     uint16_t sensor_INTN_Pin, GPIO_TypeDef *sensor_INTN_Port,
                     uint16_t sensor_RSTN_Pin, GPIO_TypeDef *sensor_RSTN_Port) {
  memset(sensor, 0, sizeof(sensor_meta));
  sensor->ports_pins.CSN_Pin = sensor_CSN_Pin;
  sensor->ports_pins.CSN_Port = sensor_CSN_Port;
  sensor->ports_pins.INTN_Pin = sensor_INTN_Pin;
  sensor->ports_pins.INTN_Port = sensor_INTN_Port;
  sensor->ports_pins.RSTN_Pin = sensor_RSTN_Pin;
  sensor->ports_pins.RSTN_Port = sensor_RSTN_Port;
  sensor->number = sensor_number;
}

uint8_t clear_init_Message_IMU(sensor_meta *sensor) {
  csn(sensor->ports_pins, true);
  if (wait_for_INTN(sensor) != N_ERR) {
      return D_ERR;
  }
  return receive_Data(sensor);
}

void hardreset_IMU(sensor_meta *sensor) {
  reset_Hardware(sensor->ports_pins);
}

bool get_and_clear_Reset_Status(sensor_meta *sensor) {
  if (sensor->has_reset) {
    sensor->has_reset = false;
    return true;
  }
  return false;
}

uint8_t enable_GameRotationVector(sensor_meta *sensor, uint16_t time_between_reports_ms) {
    sensor->rotation_vector_mode = SENSOR_REPORTID_GAME_ROTATION_VECTOR;
    sensor->rotation_vector_report_frequency = time_between_reports_ms;
    return set_FeatureCommand(sensor, SENSOR_REPORTID_GAME_ROTATION_VECTOR, (uint32_t)time_between_reports_ms * 1000, 0);
}
uint8_t enable_LinearAcceleration(sensor_meta *sensor, uint16_t time_between_reports_ms) {
    return set_FeatureCommand(sensor, SENSOR_REPORTID_LINEAR_ACCELERATION, (uint32_t)time_between_reports_ms * 1000, 0);
}

bool data_available(sensor_meta *sensor) {
  check_INTN(sensor);
  if(sensor->INTN_ready){
      if(get_Readings(sensor) != 0) {
          return true;
      }
  }
  return false;
}

float get_LinearAcceleration_X(sensor_meta *sensor) {
    return fixpoint_to_float(sensor->linear_acceleration_data.raw_Accel_X, accelerometer_Q1);
}

float get_LinearAcceleration_Y(sensor_meta *sensor) {
    return fixpoint_to_float(sensor->linear_acceleration_data.raw_Accel_Y, accelerometer_Q1);
}

float get_LinearAcceleration_Z(sensor_meta *sensor) {
    return fixpoint_to_float(sensor->linear_acceleration_data.raw_Accel_Z, accelerometer_Q1);
}

float get_Quat_I(sensor_meta *sensor) {
  return fixpoint_to_float(sensor->quaternions.raw_Quat_I, rotationVector_Q1);
}

float get_Quat_J(sensor_meta *sensor) {
  return fixpoint_to_float(sensor->quaternions.raw_Quat_J, rotationVector_Q1);
}

float get_Quat_K(sensor_meta *sensor) {
  return fixpoint_to_float(sensor->quaternions.raw_Quat_K, rotationVector_Q1);
}

float get_Quat_Real(sensor_meta *sensor) {
  return fixpoint_to_float(sensor->quaternions.raw_Quat_Real, rotationVector_Q1);
}

uint8_t get_Quat_Accuracy(sensor_meta *sensor) {
  return sensor->quaternions.quat_Accuracy;
}


uint8_t check_Connection_IMU(sensor_meta *sensor) {
  sensor->shtp_package.shtp_Data[0] = SHTP_REPORT_PRODUCT_ID_REQUEST;
  sensor->shtp_package.shtp_Data[1] = 0;

  if (send_Data(sensor, CHANNEL_CONTROL, 2) != N_ERR) return D_ERR;

  if(wait_for_INTN(sensor) != N_ERR) return D_ERR;
  if(receive_Data(sensor) != N_ERR) return D_ERR;

  if (sensor->shtp_package.shtp_Header[2] == CHANNEL_CONTROL &&
      sensor->shtp_package.shtp_Data[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
    return N_ERR;
  }

  return D_ERR;
}

uint8_t check_Command_Success(sensor_meta *sensor, uint8_t status_command) {
  if (!status_command) return D_ERR;

  if(wait_for_INTN(sensor) != N_ERR) return D_ERR;
  if(receive_Data(sensor) != N_ERR) return D_ERR;

  if (sensor->shtp_package.shtp_Header[2] == CHANNEL_CONTROL) {
      return N_ERR;
  }

  return D_ERR;
}

