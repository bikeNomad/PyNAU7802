"""
Micropython port by Ned Konz of Bruno-Pier (BrunoB81HK)'s Python NAU7802 library,
(https://github.com/BrunoB81HK/PyNAU7802)
which was itself a port of the SparkFun Qwiic Scale NAU7802 Arduino library.
Translated to snake_case format for a more Pythonic feel.
"""

from time import sleep_ms, ticks_ms, ticks_diff
from struct import unpack
from micropython import const
from machine import I2C

from .constants import (
    NAU7802_LDO_3V0,
    NAU7802_GAIN_128,
    NAU7802_SPS_80,
    NAU7802_ADC,
    NAU7802_PGA_PWR_PGA_CAP_EN,
    NAU7802_PGA_PWR,
    NAU7802_PU_CTRL_CR,
    NAU7802_PU_CTRL,
    NAU7802_ADCO_B2,
    NAU7802_CTRL1,
    NAU7802_CTRL2,
    NAU7802_CHANNEL_1,
    NAU7802_CTRL2_CHS,
    NAU7802_CTRL2_CALS,
    NAU7802_CTRL2_CAL_ERROR,
    NAU7802_PU_CTRL_RR,
    NAU7802_PU_CTRL_PUD,
    NAU7802_PU_CTRL_PUA,
    NAU7802_PU_CTRL_PUR,
    NAU7802_PU_CTRL_AVDDS,
    NAU7802_CTRL1_CRP,
    NAU7802_CAL_IN_PROGRESS,
    NAU7802_CAL_FAILURE,
    NAU7802_CAL_SUCCESS,
    NAU7802_DEVICE_REV,
)

_DEVICE_ADDRESS = const(0x2A)


###########################################
# Classes
###########################################
class nau7802:
    """Class to communicate with the NAU7802"""

    def __init__(self, wire_port: I2C):
        self._i2c_port = wire_port
        self._zero_offset = 0
        self._calibration_factor = 1.0
        # for reading/writing single byte registers
        self._register_buffer = bytearray(1)
        # for unpacking 32-bit values (last byte stays 0)
        self._value_buffer = bytearray(4)
        # for reading 24 bit values into 32-bit buffer
        self._value_view = memoryview(self._value_buffer)

    def initialize(self) -> bool:
        """Check communication and initialize sensor"""
        # Check if the device ACKs over I2C
        try:
            if not (self.reset() and self.power_up() and self.is_connected()):
                return False
        except OSError:
            # There are rare times when the sensor is occupied and doesn't ACK. A 2nd try resolves this.
            try:
                if not (self.reset() and self.power_up() and self.is_connected()):
                    return False
            except OSError:
                return False

        result = True  # Accumulate a result as we do the setup

        try:
            # result &= self.reset()  # Reset all registers
            # Power on analog and digital sections of the scale
            # result &= self.power_up()
            result &= self.set_ldo(NAU7802_LDO_3V0)  # Set LDO to 3.3V
            result &= self.set_gain(NAU7802_GAIN_128)  # Set gain to 128
            # Set samples per second to 10
            result &= self.set_sample_rate(NAU7802_SPS_80)
            # Turn off CLK_CHP. From 9.1 power on sequencing.
            result &= self.set_register(NAU7802_ADC, 0x30)
            # Enable 330pF decoupling cap on ch. 2.
            # From 9.14 application circuit note.
            result &= self.set_bit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR)
            self.get_average(10)  # Clear old values
            # Re-cal analog frontend when we change gain, sample rate, or channel
            result &= self.calibrate_afe()

        except OSError:
            result = False

        return result

    def is_connected(self) -> bool:
        """Returns true if device ACKs at the I2C address"""
        return self.get_revision_code() == 0x0F

    def available(self) -> bool:
        """Returns true if Cycle Ready bit is set (conversion is complete)"""
        return self.get_bit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL)

    def get_reading(self) -> int:
        """Returns 24 bit reading. Assumes CR Cycle Ready bit
        (ADC conversion complete) has been checked by .available()"""
        self._i2c_port.readfrom_mem_into(
            _DEVICE_ADDRESS, NAU7802_ADCO_B2, self._value_view[1:]
        )
        value = unpack(">l", self._value_buffer)[0]  # big-endian signed long
        return value

    def get_average(self, num_averaged: int) -> int:
        """Return the average of a given number of readings"""
        total = 0
        samples_acquired = 0

        start_time = ticks_ms()

        while samples_acquired < num_averaged:
            if self.available():
                total += self.get_reading()
                samples_acquired += 1

            if ticks_diff(ticks_ms(), start_time) > 1000:
                return 0  # Timeout - Bail with error

            sleep_ms(1)

        total //= num_averaged

        return total

    @property
    def zero_offset(self) -> int:
        """Ask library for this value.Useful for storing value into NVM."""
        return self._zero_offset

    @zero_offset.setter
    def zero_offset(self, new_zero_offset: int) -> None:
        """Sets the internal variable. Useful for users who are loading values from NVM."""
        self._zero_offset = new_zero_offset

    def calculate_zero_offset(self, num_averaged: int = 8) -> None:
        """Also called taring. Call this with nothing on the scale"""
        self.zero_offset = self.get_average(num_averaged)

    @property
    def calibration_factor(self) -> float:
        """Ask library for this value.Useful for storing value into NVM."""
        return self._calibration_factor

    @calibration_factor.setter
    def calibration_factor(self, new_cal_factor: float) -> None:
        """Pass a known calibration factor into library.Helpful if users is loading settings from NVM."""
        self._calibration_factor = new_cal_factor

    def calculate_calibration_factor(
        self, weight_on_scale: float, average_amount: int = 8
    ) -> None:
        """Call this with the value of the thing on the scale.
        Sets the calibration factor based on the weight on scale and zero offset."""
        on_scale = self.get_average(average_amount)
        new_cal_factor = (on_scale - self._zero_offset) / weight_on_scale
        self.calibration_factor = new_cal_factor

    def get_weight(
        self, allow_negative_weights: bool = True, samples_to_take: int = 8
    ) -> float:
        """Once you 've set zero offset and cal factor, you can ask the library to do the calculations for you."""
        on_scale = self.get_average(samples_to_take)

        # Prevent the current reading from being less than zero offset. This happens when the scale
        # is zero'd, unloaded, and the load cell reports a value slightly less than zero value
        # causing the weight to be negative or jump to millions of pounds

        if not allow_negative_weights:
            if on_scale < self._zero_offset:
                on_scale = self._zero_offset  # Force reading to zero

        weight = (on_scale - self._zero_offset) / self._calibration_factor
        return weight

    def set_gain(self, gain_value: int) -> bool:
        """Set the gain.x1, 2, 4, 8, 16, 32, 64, 128 are available"""
        if gain_value > 0b111:
            gain_value = 0b111  # Error check

        value = self.get_register(NAU7802_CTRL1)
        value &= 0b11111000  # Clear gain bits
        value |= gain_value  # Mask in new bits

        return self.set_register(NAU7802_CTRL1, value)

    def set_ldo(self, ldo_value: int) -> bool:
        """Set the on board Low - Drop - Out voltage regulator to a given value.
        2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5 V are available"""
        if ldo_value > 0b111:
            ldo_value = 0b111  # Error check

        # Set the value of the LDO
        value = self.get_register(NAU7802_CTRL1)
        value &= 0b11000111  # Clear LDO bits
        value |= ldo_value << 3  # Mask in new LDO bits
        self.set_register(NAU7802_CTRL1, value)

        return self.set_bit(
            NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL
        )  # Enable the internal LDO

    def set_sample_rate(self, rate: int) -> bool:
        """Set the readings per second. 10, 20, 40, 80, and 320 samples per second is available"""
        if rate > 0b111:
            rate = 0b111  # Error check

        value = self.get_register(NAU7802_CTRL2)
        value &= 0b10001111  # Clear CRS bits
        value |= rate << 4  # Mask in new CRS bits

        return self.set_register(NAU7802_CTRL2, value)

    def set_channel(self, channel_number: int) -> bool:
        """Select between 1 and 2"""
        if channel_number == NAU7802_CHANNEL_1:
            return self.clear_bit(
                NAU7802_CTRL2_CHS, NAU7802_CTRL2
            )  # Channel 1 (default)
        else:
            return self.set_bit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)  # Channel 2

    def calibrate_afe(self) -> bool:
        """Synchronous calibration of the analog front end of the NAU7802.
        Returns true if CAL_ERR bit is 0 (no error)"""
        self.begin_calibrate_afe()
        return self.wait_for_calibrate_afe(1000)

    def begin_calibrate_afe(self) -> None:
        """Begin asynchronous calibration of the analog front end of the NAU7802.
        Poll for completion with cal_afe_status() or wait with wait_for_calibrate_afe()."""
        self.set_bit(NAU7802_CTRL2_CALS, NAU7802_CTRL2)

    def wait_for_calibrate_afe(self, timeout_ms: int = 0) -> bool:
        """Wait for asynchronous AFE calibration to complete with optional timeout."""
        begin = ticks_ms()
        cal_ready = self.cal_afe_status()

        while cal_ready == NAU7802_CAL_IN_PROGRESS:
            if (timeout_ms > 0) and (ticks_diff(ticks_ms(), begin) > timeout_ms):
                break
            sleep_ms(1)
            cal_ready = self.cal_afe_status()

        if cal_ready == NAU7802_CAL_SUCCESS:
            return True
        else:
            return False

    def cal_afe_status(self) -> int:
        """Check calibration status."""
        if self.get_bit(NAU7802_CTRL2_CALS, NAU7802_CTRL2):
            return NAU7802_CAL_IN_PROGRESS

        if self.get_bit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2):
            return NAU7802_CAL_FAILURE

        # Calibration passed
        return NAU7802_CAL_SUCCESS

    def reset(self) -> bool:
        """Resets all registers to Power Off Defaults"""
        self.set_bit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)  # Set RR
        sleep_ms(1)
        return self.clear_bit(
            NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL
        )  # Clear RR to leave reset state

    def power_up(self) -> bool:
        """Power up digital and analog sections of scale, ~2 mA"""
        self.set_bit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL)
        self.set_bit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL)

        # Wait for Power Up bit to be set - takes approximately 200us
        counter = 0
        while not self.get_bit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL):
            sleep_ms(1)
            if counter > 100:
                return False  # Error
            counter += 1

        return True

    def power_down(self) -> bool:
        """Puts scale into low - power 200 nA mode"""
        self.clear_bit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL)
        return self.clear_bit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL)

    def set_int_polarity_high(self) -> bool:
        """Set Int pin to be high when data is ready(default)"""
        return self.clear_bit(
            NAU7802_CTRL1_CRP, NAU7802_CTRL1
        )  # 0 = CRDY pin is high active (ready when 1)

    def set_int_polarity_low(self) -> bool:
        """Set Int pin to be low when data is ready"""
        return self.set_bit(
            NAU7802_CTRL1_CRP, NAU7802_CTRL1
        )  # 1 = CRDY pin is low active (ready when 0)

    def get_revision_code(self) -> int:
        """Get the revision code of this IC.Always 0x0F."""
        revision_code = self.get_register(NAU7802_DEVICE_REV)
        return revision_code & 0x0F

    def set_bit(self, bit_number: int, register_address: int) -> bool:
        """Mask & set a given bit within a register"""
        value = self.get_register(register_address)
        value |= 1 << bit_number  # Set this bit
        return self.set_register(register_address, value)

    def clear_bit(self, bit_number: int, register_address: int) -> bool:
        """Mask & clear a given bit within a register"""
        value = self.get_register(register_address)
        value &= ~(1 << bit_number)  # Set this bit
        return self.set_register(register_address, value)

    def get_bit(self, bit_number: int, register_address: int) -> bool:
        """Return a given bit within a register"""
        value = self.get_register(register_address)
        value &= 1 << bit_number  # Clear all but this bit
        return bool(value)

    def get_register(self, register_address: int) -> int:
        """Get contents of a register"""
        try:
            data = self._i2c_port.readfrom_mem(
                _DEVICE_ADDRESS, register_address, 1)
            return data[0]

        except OSError:
            return -1  # Sensor did not ACK

    def set_register(self, register_address: int, value: int) -> bool:
        """Send a given value to be written to given address.Return true if successful"""
        try:
            self._register_buffer[0] = value & 0xFF
            self._i2c_port.writeto_mem(
                _DEVICE_ADDRESS, register_address, self._register_buffer
            )
            return True

        except OSError:
            return False
