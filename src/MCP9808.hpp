/**
 * @file MCP9808.hpp
 * @author your name (corey.maxwell18@proton.me)
 * @brief MCP9808 Device Driver
 * @version 0.1
 * @date 2024-09-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef MCP9808_HPP
#define MCP9808_HPP

#include <Arduino.h>
#include <FixedPoints.h>
#include <Wire.h>
#include <memory>
#include <variant>

#define DATA_LENGTH 2
/**
 * @brief Functions, Classes and Important data for communicating with the MCP9808 over i2c
 *
 */
namespace MCP9808
{
/**
 * @brief The default Slave Address
 *
 * From the datasheet the default slave adress is 0x18, however the address can be altered by pulling the AD0, AD1 or
 * AD2 pins up Thus the Slave Address is {5b00011, AD0, AD1, AD2} where AD0-2 are eithe r1 or 0.
 */
#define DEFAULT_SLAVE_ADDRESS 0x18
/**
 * @brief Enum encapsulating all Registers on the MCP9808
 *
 * The enum contains all the internat registers inside the MCP9808 chip as well as the corresponding register address
 * (in hex).
 */
enum Register
{
    CONFIG = 0x01,
    T_UPPER = 0x02,
    T_LOWER = 0x03,
    T_CRITICAL = 0x04,
    T_AMBIENT = 0x05,
    MFER_ID = 0x06,
    DEV_ID_REV = 0x07,
    RESOLUTION = 0x08
};

/**
 * @brief Union Dataclass for Short to 2 Bytes
 * Splits a short into 2 Bytes
 */
union ShortToByte {
    short s;
    struct Byte
    {
        char c1, c2;
    } byte;
    ShortToByte();
    ShortToByte(char _c1, char _c2);
    ShortToByte(short _s);
};

/**
 * @brief Enum encapsulating all possible resolution for the MCP9808
 *
 * The enum contains the possible resolutions well as a 2 bit code that is written to register 0x8 in order to change
 * the resolution of the measurement
 */
enum Resolution
{
    RES_0_5 = 0b00,
    RES_0_25 = 0b01,
    RES_0_125 = 0b10,
    RES_0_0625 = 0b11,
};

class Sensor
{
  public:
    /**
     * @brief Constructor For a Sensor Object
     * Creates an object that encapsulates all functionality for interacting with the MCP9808 High temperature sensor.
     * The default Constructor sets the interal slave address to 0x18 which is the default address for the MCP9808 with
     * no pull down resistors.
     */
    Sensor();
    /**
     * @brief Alternative Contructor for a different sensor address
     * @param _slave_address The 7 bit i2c slave address for MCP9808
     */
    Sensor(byte _slave_address);
    /**
     * @brief Destructor for sensor Object
     * Not used.
     */
    ~Sensor();
    /**
     * @brief Gets the temperature from the MCP9808
     * @return A temperature reading
     * Returns the value in the 0x5 register if the MCP9808 and returns a value to a given resolution, (+/- 0.5, 0.25,
     * 0.125, 0.0625)
     */
    float get_temperature();
    /**
     * @brief Sets teh resolution of the temperature sensor.
     * @param res Resolution enum.
     * Sets the internal resolution register (0x08) on the MCP9808 to a given resolution (see Resolution Enum).
     */
    void set_resolution(Resolution res);
    /**
     * @brief Sets the Hysterises flag of the Config (0x01) Register
     * Hystersis can be used to mitigate output chatter  and introduces a boundary (shown below). This only appliesto
     * decreasing temperature only.
     * Hysteresis Bit | Temperature Hysteresis
     * ---------------|-----------------------
     * 0b00 | 0.0degC
     * 0b01 | +1.5degC
     * 0b10 | +3.0degC
     * 0b11 | +6.0degC
     *
     * @param u_flag The upper hysteresis bit
     * @param l_flag The lower hysteresis bit
     */
    void temp_hysteresis_flag(bool u_flag, bool l_flag);
    /**
     * @brief  Sets the shutdown flag in the config (0x01) register.
     * This disables all power consuming activities while still allowing for serial communication until the. Note the
     * shutdown bit cannot be set while the LOCK bits (CRIT_LOCK, WIN_LOCK) are set, however SHDN can still be cleared.
     * @param flag SHDN Flag
     */
    void shutdown_flag(bool flag);
    /**
     * @brief Sets the Critical lock flag in the config register.
     * Locks the T_CRIT (0x04) register so that the critical temperature cannot be changed.
     * @param flag  CRIT_LOCK flag
     */
    void critical_lock_flag(bool flag);
    /**
     * @brief Sets the window lock flag in the config register
     * Locks the T_LOWER (0x03) and T_UPPER (0x02) registers so that the lower and upper temperature limits cannot be
     * changed.
     *
     * @param flag WINDOW_LOCK flag
     */
    void window_lock_flag(bool flag);
    /**
     * @brief Sets the interrupt clear bit in the config register
     * @param flag INTERRUPT CLEAR BIT
     */
    void interrupt_clear_flag(bool flag);
    /**
     * @brief Sets the alert Output status bit in the config register
     * If the bit is set to 0 the Alert output is not asserted by the device whereas if the bit is set to 1 the alert
     * output is asserted as a comparator/interrupt of critical temperature output.
     * @param flag 1 or 0
     */
    void alert_output_status_flag(bool flag);
    /**
     * @brief Enables/Disables the Alert pin on the MCP9808
     * 0 = Disabled (Power Up Default)
     * 1 = Enabled
     * @param flag 1 or 0
     */
    void alert_output_control_flag(bool flag);
    /**
     * @brief Chooses what criterea causes the alert pin to be asserted.
     * 0 = Alert output for T_UPPER, T_LOWER and T_CRITICAL
     * 1 = Alert output for T_AMBIENT > T_CRITICAL Only
     * @param flag 1 or 0
     */
    void alert_output_select_flag(bool flag);
    /**
     * @brief Sets the polarity of the Alert pin
     * 0 = Active-low (requires Pull up resistor)
     * 1 = Active-high
     * @param flag 1 or 0
     */
    void alert_output_polarity_flag(bool flag);
    /**
     * @brief Sets the alert output mode bit
     * 0 = Comparator Output (power-up default)
     * 1 = Interrupt output
     * @param flag 1 or 0
     */
    void alert_output_mode_flag(bool flag);
    /**
     * @brief Sets the config register on the MCP9808 given the internal flags.
     */
    void config();
    /**
     * @brief Set the upper tempertature limit register
     *
     * @param temp
     */
    void set_upper_limit(int temp);
    /**
     * @brief Set the lower temperature limit register
     *
     * @param temp
     */
    void set_lower_limit(int temp);
    /**
     * @brief Set the critical temperathre limit register
     *
     * @param temp
     */
    void set_critical_temp(int temp);

    /**
     * @brief Sets the SCL clock speed and intialises the Wire object.
     */
    void begin();

	/**
	 * @brief Sets an internal register on the MCP9808
	 * 
	 * @param w_reg Register Address
	 * @param data Data to set the register with.
	 * @return int 
	 */
    int set_register(Register w_reg, short data);

    /**
     * @brief Read the data stored within a given register. 
     * @param w_reg The register address.
     * @return The data read from the register
     */
    short read_register(Register w_reg);

  private:
    /**
     * @brief Encoded the config flags into a 16 bit value to be leaded into the config register
     * @return 
     */
    short config_encode();
    /**
     * @brief The I2C addressof the MCP9808
     */
    byte slave_address;
	/**
	 * @brief The resolution of the MCP9808 
	 */
	Resolution res;

    // Config Flags
    byte T_HYST;
    byte SHDN;
    byte CRIT_LOCK;
    byte WIN_LOCK;
    byte INT_CLEAR;
    byte ALERT_STAT;
    byte ALERT_CNT;
    byte ALERT_SEL;
    byte ALERT_POL;
    byte ALERT_MOD;
};

} // namespace MCP9808
#endif // MCP9808_HPP