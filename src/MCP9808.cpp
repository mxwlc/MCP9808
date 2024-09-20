#include "MCP9808.hpp"

namespace MCP9808
{

ShortToByte::ShortToByte()
{
}

ShortToByte::ShortToByte(char _c1, char _c2)
{
    byte.c1 = _c1;
    byte.c2 = _c2;
}

ShortToByte::ShortToByte(short _s)
{
    s = _s;
}

short Sensor::config_encode()
{
    return (T_HYST << 9) + (SHDN << 8) + (CRIT_LOCK << 7) + (WIN_LOCK << 6) + (INT_CLEAR << 5) + (ALERT_STAT << 4) +
           (ALERT_CNT << 3) + (ALERT_SEL << 2) + (ALERT_POL << 1) + ALERT_MOD;
}

Sensor::Sensor()
{
    slave_address = DEFAULT_SLAVE_ADDRESS;

    T_HYST = byte(0);
    SHDN = byte(0);
    CRIT_LOCK = byte(0);
    WIN_LOCK = byte(0);
    INT_CLEAR = byte(0);
    ALERT_STAT = byte(0);
    ALERT_CNT = byte(0);
    ALERT_SEL = byte(0);
    ALERT_POL = byte(0);
    ALERT_MOD = byte(0);
}

Sensor::Sensor(byte _slave_address)
{
    slave_address = _slave_address;
    res = Resolution::RES_0_0625;
    T_HYST = byte(0);
    SHDN = byte(0);
    CRIT_LOCK = byte(0);
    WIN_LOCK = byte(0);
    INT_CLEAR = byte(0);
    ALERT_STAT = byte(0);
    ALERT_CNT = byte(0);
    ALERT_SEL = byte(0);
    ALERT_POL = byte(0);
    ALERT_MOD = byte(0);
}

void Sensor::begin()
{
    Wire.begin();
    Wire.setClock(10000);
    config();
}

Sensor::~Sensor()
{
}

void Sensor::set_resolution(Resolution _res)
{
    res = _res;
	set_register(Register::RESOLUTION, res);
}

void Sensor::temp_hysteresis_flag(bool u_flag, bool l_flag)
{
    T_HYST = (u_flag << 1) + l_flag;
}
void Sensor::shutdown_flag(bool flag)
{
    SHDN = flag;
}
void Sensor::critical_lock_flag(bool flag)
{
    CRIT_LOCK = flag;
}
void Sensor::window_lock_flag(bool flag)
{
    WIN_LOCK = flag;
}
void Sensor::interrupt_clear_flag(bool flag)
{
    INT_CLEAR = flag;
}
void Sensor::alert_output_status_flag(bool flag)
{
    ALERT_STAT = flag;
}
void Sensor::alert_output_control_flag(bool flag)
{
    ALERT_CNT = flag;
}
void Sensor::alert_output_select_flag(bool flag)
{
    ALERT_SEL = flag;
}
void Sensor::alert_output_polarity_flag(bool flag)
{
    ALERT_POL = flag;
}
void Sensor::alert_output_mode_flag(bool flag)
{
    ALERT_MOD = flag;
}

int Sensor::set_register(Register w_reg, short data)
{
    Wire.beginTransmission(byte(slave_address));
    Wire.write(w_reg);
    ShortToByte stb;
    stb.s = data;
    Wire.write(stb.byte.c1);
    Wire.write(stb.byte.c2);
    return Wire.endTransmission();
}
short Sensor::read_register(Register w_reg)
{
    int ret;
    byte lsb, msb;
    short out;
    Wire.beginTransmission(slave_address);
    Wire.write(byte(w_reg));
    ret = Wire.endTransmission();
    if (ret != 0)
    {
    }

    ret = Wire.requestFrom((uint16_t)slave_address, (size_t)DATA_LENGTH, (bool)0);
    if (Wire.available() <= 2)
    {
        msb = Wire.read();
        lsb = Wire.read();
    }
    ret = Wire.endTransmission();
    ShortToByte stb(msb, lsb);
    if (ret != 0)
    {
    }
    return stb.s;
}

float Sensor::get_temperature()
{
    ShortToByte stb;
    stb.s = read_register(Register::T_AMBIENT);
    byte c1 = stb.byte.c1;
    byte c2 = stb.byte.c2;
    c1 = c1 & 0x1F;
    float out = 0;
    switch (res)
    {
    case Resolution::RES_0_0625: {
        SFixed<8, 4> t_0_0625;
        short integer = (c1 << 4) + ((c2 & 0xF0) >> 4);
        u_char fraction = stb.byte.c2 & 0x0F;
        t_0_0625 = SFixed<8, 4>(integer, fraction);
        out = static_cast<float>(t_0_0625);
        break;
    }
    case Resolution::RES_0_125: {
        SFixed<9, 3> t_0_125;
        short integer = (c1 << 5) + ((c2 & 0xF8) >> 3);
        u_char fraction = c2 & 0x07;
        t_0_125 = SFixed<9, 3>(integer, fraction);
        out = static_cast<float>(t_0_125);
        break;
    }
    case Resolution::RES_0_25: {
        SFixed<10, 2> t_0_25;
        short integer = (c1 << 6) + ((c2 & 0xFC) >> 2);
        u_char fraction = c2 & 0x03;
        t_0_25 = SFixed<10, 2>(integer, fraction);
        out = static_cast<float>(t_0_25);
        break;
    }
    case Resolution::RES_0_5: {
        SFixed<11, 1> t_0_5;
        short integer = (c1 << 7) + ((c2 & 0xFC) >> 1);
        u_char fraction = c2 & 0x1;
        t_0_5 = SFixed<11, 1>(integer, fraction);
        out = static_cast<float>(t_0_5);
        break;
    }
    default:
        out = 0.0f;
        break;
    }
    return out;
}

void Sensor::config()
{
    set_register(Register::CONFIG, config_encode());
}

void Sensor::set_upper_limit(int temp)
{
    set_register(Register::T_UPPER, temp);
}
void Sensor::set_lower_limit(int temp)
{
    set_register(Register::T_LOWER, temp);
}
void Sensor::set_critical_temp(int temp)
{
    set_register(Register::T_CRITICAL, temp);
}
}; // namespace MCP9808