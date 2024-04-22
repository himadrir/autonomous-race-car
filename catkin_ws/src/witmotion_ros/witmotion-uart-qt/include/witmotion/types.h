/*!
    \file types.h
    \brief Abstract types collection for Witmotion sensor library
    \author Andrey Vukolov andrey.vukolov@elettra.eu

    This header file contains all abstract types and hardware-defined constants to operate Witmotion sensor device.
*/

#ifndef WITMOTION
#define WITMOTION
#include <cmath>
#include <set>
#include <inttypes.h>

#include <QtCore>
#include <QSerialPort>

#include "witmotion/version.h"


/*!
  \brief Main namespace of Witmotion UART connection library

Upper level namespace containing all the declared constants, parameters, classes, functions.

\note It is strictly NOT RECOMMENDED to use this namespace implicitly through `using namespace` directive.
*/
namespace witmotion
{

static const uint8_t WITMOTION_HEADER_BYTE = 0x55; ///< Packet header byte value (vendor protocol-specific)
static const uint8_t WITMOTION_CONFIG_HEADER = 0xFF; ///< Configuration header byte value (vendor protocol-specific)
static const uint8_t WITMOTION_CONFIG_KEY = 0xAA; ///< Configuration marker key byte value (vendor protocol-specific)
static const float DEG2RAD = M_PI / 180.f; ///< \private

/*!
  \brief Packet type IDs from the vendor-defined protocol

  If one of the packet type IDs defined here is registered after \ref WITMOTION_HEADER_BYTE in the data flow received from the sensor, the packet header is considered found and the remaining bytes are considered as body of the packet. See \ref util.h for decoder function reference.
*/
enum witmotion_packet_id
{
    pidRTC = 0x50, ///< Real-Time-Clock: Year from 2000, Month, Day, Hour, Minute, Second (8-bit unsigned integers) + Millisecond (16-bit unsigned integer), representing time passed since last time set up in the \ref ridTimeYearMonth, \ref ridTimeDayHour, \ref ridTimeMinuteSecond and \ref ridTimeMilliseconds registers
    pidAcceleration = 0x51, ///< Linear accelerations + temperature/reserved field [X-Y-Z] (16-bit binary normalized quasi-floats)
    pidAngularVelocity = 0x52, ///< Angular velocities + temperature/reserved field [Roll-Pitch-Yaw] (16-bit binary normalized quasi-floats)
    pidAngles = 0x53, ///< Euler angles + temperature/reserved field [Roll-Pitch-Yaw] (16-bit binary normalized quasi-floats)
    pidMagnetometer = 0x54, ///< Magnetic field tensity + temperature/reserved field [world X-Y-Z] (16-bit binary normalized quasi-floats)
    pidDataPortStatus = 0x55, ///< Data port status packet, vendor-defined value
    pidAltimeter = 0x56, ///< Altimeter + Barometer output (32-bit binary normalized quasi-floats)
    pidGPSCoordinates = 0x57, ///< GPS: longitude + latitude, if supported by hardware (32-bit binary normalized quasi-floats)
    pidGPSGroundSpeed = 0x58, ///< GPS: ground speed (32-bit binary normalized quasi-float) + altitude + angular velocity around vertical axis (16-bit binary normalized quasi-floats), if supported by hardware
    pidOrientation = 0x59, ///< Orientation defined as quaternion [X-Y-Z-W], when available from the sensor firmware (16-bit binary normalized quasi-floats)
    pidGPSAccuracy = 0x5A ///< GPS: visible satellites + variance vector [East-North-Up] (16-bit binary normalized quasi-floats)
};

/*!
  \brief Packet ID set to retrieve descriptions via \ref witmotion_packet_descriptions.

  Contains values referenced in \ref witmotion_packet_id enumeration to explicitly determine a set of currently supported packet IDs. The packet IDs not referenced here sould not be considered supported.
*/
static const std::set<size_t> witmotion_registered_ids = {
    0x50,
    0x51,
    0x52,
    0x53,
    0x54,
    0x55,
    0x56,
    0x57,
    0x58,
    0x59,
    0x5A
};

/*!
  \brief Packet ID string set to store built-in descriptions for \ref message-enumerator.

  Contains values referenced in \ref witmotion_packet_id enumeration with corresponding description strings used by \ref message-enumerator application.
*/
static const std::map<uint8_t, std::string> witmotion_packet_descriptions = {
    {0x50, "Real Time Clock"},
    {0x51, "Accelerations"},
    {0x52, "Angular velocities"},
    {0x53, "Spatial orientation (Euler angles)"},
    {0x54, "Magnetometer/Magnetic orientation"},
    {0x55, "Data ports (D0-D3) status"},
    {0x56, "Barometry/Altimeter"},
    {0x57, "GPS Coordinates"},
    {0x58, "GPS Ground Speed"},
    {0x59, "Spatial orientation (Quaternion)"},
    {0x5A, "GPS accuracy estimation"}
};

/*!
 * \brief Generic structure respresenting the standard 11-byte datapacket defined in Witmotion protocol.
*/
struct witmotion_datapacket
{
    uint8_t header_byte; ///< Header byte, set constantly to \ref WITMOTION_HEADER_BYTE
    uint8_t id_byte; ///< Packet type ID, referring to \ref witmotion_packet_id, otherwise the packet is considered of unknown type.
    union
    {
        int8_t raw_signed[8];
        uint8_t raw[8];
        int16_t raw_cells[4];
        int32_t raw_large[2];
    }datastore; ///< 8-byte internal data storage array represented as C-style memory union. The stored data represented as `int8_t*`, `uint8_t*`, `int16_t*` or `int32_t*` array head pointer.
    uint8_t crc; ///< Validation CRC for the packet. Calculated as an equivalent to the following operation: \f$ crc = \sum_{i=0}^{i < 10}\times\f$`reinterpret_cast<uint8_t*>(this) + i`
};

/*!
 * \brief List of configuration slots (registers) available for the library.
 *
 * List of configuration slots (registers) available for the library. The actual availability depends from the actual sensor and installation circuit.
 * Please refer to the official documentation for detailed explanation.
*/
enum witmotion_config_register_id
{
    ridSaveSettings = 0x00, ///< Saves the settings uploaded in the current bringup session, or resets it to default (if supported). To make factory reset of the sensor, set `raw[0] = 0x01` in \ref witmotion_config_packet instance used.
    /*!
      Sets the sensor to calibration mode. The value stored in \ref witmotion_config_packet.setting.`raw[0]` determines device selection:
      - `0x00` - End calibration
      - `0x01` - Accelerometer calibration
      - `0x03` - Altitude reset (only for barometric altimeter)
      - `0x04` - Yaw [Z] Euler angle origin point reset
      - `0x07` - Magnetometer calibration
      - `0x08` - Angle reference reset
    */
    ridCalibrate = 0x01,
    /*!
      Regulates sensor output. The value stored in \ref witmotion_config_packet.setting.`raw` determines packet ID selection to output from low to high bits by offset. `0` means disabling of the selected data packet output.

      |`raw[0]` offset|Packet type|`raw[1]` offset| Packet type|
      |:-------------:|----------:|:-------------:|-----------:|
      |0|\ref pidRTC|0|\ref pidGPSGroundSpeed|
      |1|\ref pidAcceleration|1|\ref pidOrientation|
      |2|\ref pidAngularVelocity|2|\ref pidGPSAccuracy|
      |3|\ref pidAngles|3|Reserved|
      |4|\ref pidMagnetometer|4|Reserved|
      |5|\ref pidDataPortStatus|5|Reserved|
      |6|\ref pidAltimeter|6|Reserved|
      |7|\ref pidGPSCoordinates|7|Reserved|
    */
    ridOutputValueSet = 0x02,
    /*!
      Regulates output frequency. **NOTE**: the maximum available frequency is determined internally by the available bandwidth obtained from \ref ridPortBaudRate.
      The actual value stored in \ref witmotion_config_packet.setting.`raw[0]` can be determined from the following table. \ref witmotion_config_packet.setting.`raw[1]` is set to `0x00`. Also the table contains argument value for \ref witmotion_output_frequency helper function which is used by the controller applications.

      |Frequency, Hz|Value |Argument|
      |:------------|:----:|:------:|
      |0 (shutdown) |`0x0D`| 0      |
      |0 (single measurement)|`0x0C`| -1 |
      |0.1          |`0x01`| -10    |
      |0.5          |`0x02`| -2     |
      | 1           |`0x03`| 1      |
      | 2           |`0x04`| 2      |
      | 5           |`0x05`| 5      |
      |10 (default) |`0x06`| 10     |
      |20           |`0x07`| 20     |
      |50           |`0x08`| 50     |
      |100          |`0x09`| 100    |
      |125          |`0x0A`| 125    |
      |200          |`0x0B`| 200    |
      |Maximal available by hardware|`0x0C`|Not supported|
    */
    ridOutputFrequency = 0x03,
    /*!
      Regulates port baud rate. **NOTE**: the sensor has no possibility of hardware flow control and it cannot report to the system what baud rate should be explicitly used!
      The actual value stored in \ref witmotion_config_packet.setting.`raw[0]` can be determined from the following table. \ref witmotion_config_packet.setting.`raw[1]` is set to `0x00`.
      The \ref witmotion_baud_rate helper function argument is accepted as `QSerialPort::BaudRate` enumeration member, so only the speed inticated in that enumeration are explicitly supported.
      |**Rate, baud**|1200/1400|4800  |9600  |19200 |38400 |57600 |115200|
      |:-------------|:-------:|:----:|:----:|:----:|:----:|:----:|:----:|
      |**Value**     |`0x00`   |`0x01`|`0x02`|`0x03`|`0x04`|`0x05`|`0x06`|

      This parameter also implicitly sets \ref ridOutputFrequency to the maximal feasible value for the available bandwidth.
    */
    ridPortBaudRate = 0x04,
    ridAccelerationBiasX = 0x05, ///< Sets acceleration zero point bias for X axis, refer to \ref acceleration-bias page for explanation.
    ridAccelerationBiasY = 0x06, ///< Sets acceleration zero point bias for Y axis, refer to \ref acceleration-bias page for explanation.
    ridAccelerationBiasZ = 0x07, ///< Sets acceleration zero point bias for Z axis, refer to \ref acceleration-bias page for explanation.
    ridAngularVelocityBiasX = 0x08, ///< Sets angular velocity zero point bias for X axis. NOT YET PROVEN AS WORKING
    ridAngularVelocityBiasY = 0x09, ///< Sets angular velocity zero point bias for Y axis. NOT YET PROVEN AS WORKING
    ridAngularVelocityBiasZ = 0x0A, ///< Sets angular velocity zero point bias for Z axis. NOT YET PROVEN AS WORKING
    ridMagnetometerBiasX = 0x0B, ///< Sets magnetometer zero point bias for X axis. **MAY BLOCK THE MEASUREMENTS**
    ridMagnetometerBiasY = 0x0C, ///< Sets magnetometer zero point bias for Y axis. **MAY BLOCK THE MEASUREMENTS**
    ridMagnetometerBiasZ = 0x0D, ///< Sets magnetometer zero point bias for Z axis. **MAY BLOCK THE MEASUREMENTS**
    /*!
      Digital port D0 mode. The values are set only via \ref witmotion_config_packet.setting.`raw[0]` whilst \ref witmotion_config_packet.setting.`raw[1]` is set to 0. Please refer to the following table to determine the exact value needed.

      |**Description**|Analog input (default)|Digital input|Digital output (high)|Digital output (low)|PWM output|
      |:--------------|:--------------------:|:-----------:|:-------------------:|:------------------:|:--------:|
      |**Value**      |`0x00`                |`0x01`       |`0x02`               |`0x03`              |`0x04`    |
    */
    ridPortModeD0 = 0x0E,
    /*!
      Digital port D1 mode. The values are set only via \ref witmotion_config_packet.setting.`raw[0]` whilst \ref witmotion_config_packet.setting.`raw[1]` is set to 0. Please refer to the following table to determine the exact value needed.

      |**Description**|Analog input (default)|Digital input|Digital output (high)|Digital output (low)|PWM output|
      |:--------------|:--------------------:|:-----------:|:-------------------:|:------------------:|:--------:|
      |**Value**      |`0x00`                |`0x01`       |`0x02`               |`0x03`              |`0x04`    |

      \note If the external GPS receiver is used to obtain world time, and it is compatible with Witmotion serial protocol, the port D1 should be connected to its **TX** pin and turned into GPS receiver port by the special value `0x05` set for this register. The baud rate on which GPS receiver communicates with the sensor, is set via \ref ridGPSBaudRate register.
    */
    ridPortModeD1 = 0x0F,
    /*!
      Digital port D2 mode. The values are set only via \ref witmotion_config_packet.setting.`raw[0]` whilst \ref witmotion_config_packet.setting.`raw[1]` is set to 0. Please refer to the following table to determine the exact value needed.

      |**Description**|Analog input (default)|Digital input|Digital output (high)|Digital output (low)|PWM output|
      |:--------------|:--------------------:|:-----------:|:-------------------:|:------------------:|:--------:|
      |**Value**      |`0x00`                |`0x01`       |`0x02`               |`0x03`              |`0x04`    |
    */
    ridPortModeD2 = 0x10,
    /*!
      Digital port D3 mode. The values are set only via \ref witmotion_config_packet.setting.`raw[0]` whilst \ref witmotion_config_packet.setting.`raw[1]` is set to 0. Please refer to the following table to determine the exact value needed.

      |**Description**|Analog input (default)|Digital input|Digital output (high)|Digital output (low)|PWM output|
      |:--------------|:--------------------:|:-----------:|:-------------------:|:------------------:|:--------:|
      |**Value**      |`0x00`                |`0x01`       |`0x02`               |`0x03`              |`0x04`    |
    */
    ridPortModeD3 = 0x11,
    ridPortPWMLevelD0 = 0x12, ///< Digital port D0 PWM high level pulse width, microseconds, 16-bit unsigned integer.
    ridPortPWMLevelD1 = 0x13, ///< Digital port D1 PWM high level pulse width, microseconds, 16-bit unsigned integer.
    ridPortPWMLevelD2 = 0x14, ///< Digital port D2 PWM high level pulse width, microseconds, 16-bit unsigned integer.
    ridPortPWMLevelD3 = 0x15, ///< Digital port D3 PWM high level pulse width, microseconds, 16-bit unsigned integer.
    ridPortPWMPeriodD0 = 0x16, ///< Digital port D0 PWM period length, microseconds, 16-bit unsigned integer.
    ridPortPWMPeriodD1 = 0x17, ///< Digital port D1 PWM period length, microseconds, 16-bit unsigned integer.
    ridPortPWMPeriodD2 = 0x18, ///< Digital port D2 PWM period length, microseconds, 16-bit unsigned integer.
    ridPortPWMPeriodD3 = 0x19, ///< Digital port D3 PWM period length, microseconds, 16-bit unsigned integer.
    ridIICAddress = 0x1A, ///< Sets up I2C address of the sensor. Default value is `0x50`, 7-bit unsigned integer in  \ref witmotion_config_packet.setting.`raw[0]` whilst \ref witmotion_config_packet.setting.`raw[1]` is set to 0.
    ridLED = 0x1B, ///< Toggles on/off LED indication (for enclosed sensors only).
    /*!
      Regulates GPS receiver baud rate on port D1 (see \ref ridPortModeD1). The following table contains value set for \ref witmotion_config_packet.setting.`raw[0]` representing the different baud rates. \ref witmotion_config_packet.setting.`raw[1]` should be set to 0.
      |**Rate, baud**|1200/1400|4800  |9600  |19200 |38400 |57600 |115200|230400|460800|921600|
      |:-------------|:-------:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|
      |**Value**     |`0x00`   |`0x01`|`0x02`|`0x03`|`0x04`|`0x05`|`0x06`|`0x07`|`0x08`|`0x09`|
      \note Baud rates over 256000 baud should not be considered standard.
    */
    ridGPSBaudRate = 0x1C,
    /*!
       Regulates internal filter bandwidth according to \ref witmotion_config_packet.setting.`raw[0]` value. Please refer to the following table for details.
       |  Value         |Bandwidth, Hz|
       |:--------------:|------------:|
       |`0x00`          | 256 |
       |`0x01`          | 184 |
       |`0x02`          | 94 |
       |`0x03`          | 44 |
       |`0x04`          | 21 |
       |`0x05`          | 10 |
       |`0x06`          | 5  |

       \ref witmotion_config_packet.setting.`raw[1]` should be set to 0. NOT YET PROVEN AS WORKING
    */
    ridFilterBandwidth = 0x1F,

    /*!
       Regulates gyroscope value range according to \ref witmotion_config_packet.setting.`raw[0]` value. Please refer to the following table for details.
       |  Value         |Range, \f$ deg/s \f$|
       |:--------------:|--------:|
       |`0x00`          |250      |
       |`0x01`          |500      |
       |`0x02`          |1000     |
       |`0x03`          |2000     |

       \ref witmotion_config_packet.setting.`raw[1]` should be set to 0. NOT YET PROVEN AS WORKING
    */
    ridGyroscopeRange = 0x20,
    /*!
       Regulates accelerometer value range according to \ref witmotion_config_packet.setting.`raw[0]` value. Please refer to the following table for details.
       |  Value         |Range, \f$ m/s^2 \f$|
       |:--------------:|--------:|
       |`0x00`          |\f$ 2 \cdot g\f$|
       |`0x01`          |\f$ 4 \cdot g\f$|
       |`0x02`          |\f$ 8 \cdot g\f$|
       |`0x03`          |\f$ 16 \cdot g\f$|

       Here \f$ g = 9.81 m/s^2 \f$. \ref witmotion_config_packet.setting.`raw[1]` should be set to 0. NOT YET PROVEN AS WORKING
    */
    ridAccelerometerRange = 0x21,
    ridStandbyMode = 0x22, ///< Toggles dormant mode. \ref witmotion_config_packet.setting.`raw[0]` should be set to `0x01`, \ref witmotion_config_packet.setting.`raw[1]` to 0.
    ridInstallationDirection = 0x23, ///< Toggles on/off internal rotation transform for vertical installation.  \ref witmotion_config_packet.setting.`raw[1]` should be set to 0, \ref witmotion_config_packet.setting.`raw[0]` being to `0x01` allows vertical installation, to `0x00` - horizontal installation.
    ridTransitionAlgorithm = 0x24, ///< Regulates whether 9-axis (`0x01` in \ref witmotion_config_packet.setting.`raw[0]`) or 6-axis (`0x00`) transition algorithm should be used. \ref witmotion_config_packet.setting.`raw[1]` should be set to 0.
    ridInstructionStart = 0x2D, ///< Instruction mode. `0x00` in \ref witmotion_config_packet.setting.`raw[0]` means starting instruction mode, `0x01` toggles it off whilst \ref witmotion_config_packet.setting.`raw[1]` is set explicitly to 0.

    ridTimeYearMonth = 0x30, ///< Sets RTC to the given year (\ref witmotion_config_packet.setting.`raw[0]`) and month (\ref witmotion_config_packet.setting.`raw[1]`). Year is a signed 8-bit integer with zero origin point set to 2000 year Gregorian calendar. Month is digitized to 1-12, unsigned 8-bit integer.
    ridTimeDayHour = 0x31, ///< Sets RTC to the given day of the month (\ref witmotion_config_packet.setting.`raw[0]`) and hour (\ref witmotion_config_packet.setting.`raw[1]`) in 24H system.
    ridTimeMinuteSecond = 0x32, ///< Sets RTC to the given minute (\ref witmotion_config_packet.setting.`raw[0]`) and second (\ref witmotion_config_packet.setting.`raw[1]`) in 24H system.
    ridTimeMilliseconds = 0x33, ///< Sets RTC to the given milliseconds exposed as 16-bit unsigned integer.
    ridSetAccelerationX = 0x34, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for acceleration on X axis. NOT YET PROVEN AS WORKING
    ridSetAccelerationY = 0x35, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for acceleration on Y axis. NOT YET PROVEN AS WORKING
    ridSetAccelerationZ = 0x36, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for acceleration on Z axis. NOT YET PROVEN AS WORKING
    ridSetAngularVelocityX = 0x37, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for angular velocity on X axis. NOT YET PROVEN AS WORKING
    ridSetAngularVelocityY = 0x38, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for angular velocity on Y axis. NOT YET PROVEN AS WORKING
    ridSetAngularVelocityZ = 0x39, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for angular velocity on Z axis. NOT YET PROVEN AS WORKING
    ridSetMagnetometerX = 0x3A, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for magnetometer on X axis. **MAY BLOCK THE MEASUREMENTS**
    ridSetMagnetometerY = 0x3B, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for magnetometer on Y axis. **MAY BLOCK THE MEASUREMENTS**
    ridSetMagnetometerZ = 0x3C, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for magnetometer on Z axis. **MAY BLOCK THE MEASUREMENTS**
    ridSetAngleRoll = 0x3D, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for Euler angle (roll) over X axis. NOT YET PROVEN AS WORKING
    ridSetAnglePitch = 0x3E, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for Euler angle (pitch) over Y axis. NOT YET PROVEN AS WORKING
    ridSetAngleYaw = 0x3F, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for Euler angle (yaw) over Z axis. NOT YET PROVEN AS WORKING
    ridSetTemperature = 0x40, ///< Sets up origin point or impostor value (needed when the corresponding spatial measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for temperature. NOT YET PROVEN AS WORKING
    ridSetPortStatusD0 = 0x41, ///< Action unknown, not yet documented by Witmotion
    ridSetPortStatusD1 = 0x42, ///< Action unknown, not yet documented by Witmotion
    ridSetPortStatusD2 = 0x43, ///< Action unknown, not yet documented by Witmotion
    ridSetPortStatusD3 = 0x44, ///< Action unknown, not yet documented by Witmotion
    ridSetPressureLow = 0x45, ///< Sets up low part of initial value for 32-bit pressure measurement register. NOT YET PROVEN AS WORKING
    ridSetPressureHigh = 0x46, ///< Sets up high part of initial value for 32-bit pressure measurement register. NOT YET PROVEN AS WORKING
    ridSetAltitudeLow = 0x47, ///< Sets up low part of initial value for 32-bit altitude measurement register. NOT YET PROVEN AS WORKING
    ridSetAltitudeHigh = 0x48, ///< Sets up high part of initial value for 32-bit altitude measurement register. NOT YET PROVEN AS WORKING
    ridSetLongitudeLow = 0x49, ///< Sets up low part of initial value for 32-bit longitude measurement register. NOT YET PROVEN AS WORKING
    ridSetLongitudeHigh = 0x4A, ///< Sets up high part of initial value for 32-bit longitude measurement register. NOT YET PROVEN AS WORKING
    ridSetLatitudeLow = 0x4B, ///< Sets up low part of initial value for 32-bit latitude measurement register. NOT YET PROVEN AS WORKING
    ridSetLatitudeHigh = 0x4C, ///< Sets up high part of initial value for 32-bit latitude measurement register. NOT YET PROVEN AS WORKING
    ridSetGPSAltitude = 0x4D, ///< Sets up initial or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for GPS altitude measurement. NOT YET PROVEN AS WORKING
    ridSetGPSYaw = 0x4E, ///< Sets up initial or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for GPS orientation angle measurement. NOT YET PROVEN AS WORKING
    ridSetGPSGroundSpeedLow = 0x4F, ///< Sets up low part of initial value for 32-bit GPS ground speed measurement register. NOT YET PROVEN AS WORKING
    ridSetGPSGroundSpeedHigh = 0x50, ///< Sets up high part of initial value for 32-bit GPS ground speed measurement register. NOT YET PROVEN AS WORKING
    ridSetOrientationX = 0x51, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for orientation quaternion, X component. NOT YET PROVEN AS WORKING
    ridSetOrientationY = 0x52, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for orientation quaternion, Y component. NOT YET PROVEN AS WORKING
    ridSetOrientationZ = 0x53, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for orientation quaternion, Z component. NOT YET PROVEN AS WORKING
    ridSetOrientationW = 0x54, ///< Sets up origin point or impostor value (needed when the measurement is forced for output by \ref ridOutputValueSet but not actually supported by the sensor) for orientation quaternion, W component. NOT YET PROVEN AS WORKING

    ridGyroscopeAutoCalibrate = 0x63, ///< Toggles on/off automatic precalibration of the gyroscope. \ref witmotion_config_packet.setting.`raw[1]` should be set to 0. `0x01` in \ref witmotion_config_packet.setting.`raw[0]` turns gyroscope automatic precalibration **OFF**. To turn it **ON** the value should be `0x00`.

    ridUnlockConfiguration = 0x69 ///< "Magic" vendor-defined value for configuration unlock packet `0xFF 0xAA 0x69 0x88 0xB5`.
};

/*!
 * \brief Generic structure respresenting the standard 5-byte configuration command defined in Witmotion protocol.
*/
struct witmotion_config_packet
{
    uint8_t header_byte; ///< Header byte, set constantly to \ref WITMOTION_CONFIG_HEADER
    uint8_t key_byte; ///< Packet type, constantly set to \ref WITMOTION_CONFIG_KEY
    uint8_t address_byte; ///< Configuration slot address, refers to the registered values in \ref witmotion_config_register_id
    union
    {
        uint8_t raw[2];
        uint16_t bin;
    }setting; ///< 2-byte internal data storage array represented as C-style memory union. The values should be formulated byte-by-byte referring to the actual sensor's documentation.
};

/*!
  \brief Abstract base class to program convenience classes for the sensors.

  This class allows the developer to write handler functions for multithreaded polling timer control, sensor configuration request and data decoding event.
  It also provides predefined signals to allow the end user to react on actual data acquisition and error occurence events.
  The common use case for this class is to provide a base interface for the family of sensors supporting the same protocol. Please refer to \ref QBaseSerialWitmotionSensorReader class for actual implementation of the currently supported UART-based protocol.
*/
class QAbstractWitmotionSensorReader: public QObject
{   Q_OBJECT
protected slots:
    virtual void ReadData() = 0; ///< Protected abstract slot to be implemented in the derived class. The common usage is as a callback for the polling timer thread.
public slots:
    virtual void SendConfig(const witmotion_config_packet& packet) = 0; ///< Public abstract slot to be implemented in the derived class. \param packet accepts the \ref witmotion_config_packet object for being sent to the sensor configuration registers.
    virtual void RunPoll() = 0; ///< Public abstract slot to be implemented in the derived class. The common use is to start the polling timer thread for the sensors after Qt event loop is started.
signals:
    void Acquired(const witmotion_datapacket& packet); ///< Signal function to be emitted when the data packet is acquired by the polling thread or process. Can only be redefined, not overridden in the class hierarchy.
    void Error(const QString& description); ///< Signal function to be emitted when the internal error reported in the polling thread or process. Can only be redefined, not overridden in the class hierarchy.
};

}

Q_DECLARE_METATYPE(witmotion::witmotion_datapacket); ///< \private
Q_DECLARE_METATYPE(witmotion::witmotion_config_packet); ///< \private

#endif
