/*
  This is a library written for the ST VL53L5CX Time-of-flight sensor
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/18642

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, October 22nd, 2021
  This file implements the VL53L5CX I2C driver class.

  This library uses ST's VL53L5CX driver and parts of Simon Levy's VL53L5CX
  Arduino library available at https://github.com/simondlevy/VL53L5/tree/main/src/st

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_VL53L5CX_IO.h"
#include "SparkFun_VL53L5CX_Library_Constants.h"

bool SparkFun_VL53L5CX_IO::begin(byte address, TwoWire &wirePort) {
  _address = address;
  _i2cPort = &wirePort;
  return isConnected();
}
bool SparkFun_VL53L5CX_IO::isConnected() {
  if (first_error) return false;
  _i2cPort->beginTransmission(_address);
  return record_error(_i2cPort->endTransmission()) == 0;
}
void SparkFun_VL53L5CX_IO::setAddress(uint8_t address) { _address = address; }
uint8_t SparkFun_VL53L5CX_IO::writeMultipleBytes(uint16_t reg,
    uint8_t *buffer, uint16_t size) {
  if (first_error) return first_error;
  for (uint32_t offset = 0; offset < size;) {
    uint16_t len = min(static_cast<uint16_t>(size - offset),
                       static_cast<uint16_t>(wireMaxPacketSize - 2));
    _i2cPort->beginTransmission(_address);
    bool written = _i2cPort->write(highByte(reg)) == 1;
    written &= _i2cPort->write(lowByte(reg)) == 1;
    for (uint16_t i = 0; i < len; ++i)
      written &= _i2cPort->write(buffer[offset + i]) == 1;
    uint8_t status = _i2cPort->endTransmission();
    if (record_error(written ? status : 4)) return first_error;
    offset += len;
    reg += len;
    // Firmware upload contains thousands of chunks; leave time for IDLE0.
    delay(1);
  }
  return 0;
}
uint8_t SparkFun_VL53L5CX_IO::readMultipleBytes(uint16_t reg,
    uint8_t *buffer, uint16_t size) {
  if (first_error) return first_error;
  for (uint32_t offset = 0; offset < size;) {
    uint16_t len = min(static_cast<uint16_t>(size - offset),
                       static_cast<uint16_t>(wireMaxPacketSize));
    _i2cPort->beginTransmission(_address);
    bool written = _i2cPort->write(highByte(reg)) == 1;
    written &= _i2cPort->write(lowByte(reg)) == 1;
    uint8_t status = _i2cPort->endTransmission(false);
    if (record_error(written ? status : 4)) return first_error;
    size_t received = _i2cPort->requestFrom(_address, static_cast<uint8_t>(len));
    if (received != len || _i2cPort->available() < len) {
      while (_i2cPort->available()) _i2cPort->read();
      return record_error(6); // Distinct short-read error.
    }
    for (uint16_t i = 0; i < len; ++i) {
      int value = _i2cPort->read();
      if (value < 0) return record_error(6);
      buffer[offset + i] = static_cast<uint8_t>(value);
    }
    offset += len;
    reg += len;
  }
  return 0;
}
uint8_t SparkFun_VL53L5CX_IO::readSingleByte(uint16_t reg) {
  uint8_t value = 0;
  readMultipleBytes(reg, &value, 1);
  return value;
}
uint8_t SparkFun_VL53L5CX_IO::writeSingleByte(uint16_t reg, uint8_t value) {
  return writeMultipleBytes(reg, &value, 1);
}
