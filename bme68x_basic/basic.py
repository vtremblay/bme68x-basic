import ctypes
import time

from smbus2 import SMBus

klipper_bm68x = ctypes.CDLL('/tmp/tmp.4bR0oZpBAF/cmake-build-debug---rpi-builder/libklipper_bme68x.so')
klipper_bm68x.sensor_init.argtypes = (ctypes.c_int, ctypes.POINTER(ctypes.c_int))


BME68X_INTF_RET_TYPE = ctypes.c_int8


class BME68xCalibData(ctypes.Structure):
    _fields_ = [
        ("par_h1", ctypes.c_uint16),
        ("par_h2", ctypes.c_uint16),
        ("par_h3", ctypes.c_int8),
        ("par_h4", ctypes.c_int8),
        ("par_h5", ctypes.c_int8),
        ("par_h6", ctypes.c_uint8),
        ("par_h7", ctypes.c_int8),
        ("par_gh1", ctypes.c_int8),
        ("par_gh2", ctypes.c_int16),
        ("par_gh3", ctypes.c_int8),
        ("par_t1", ctypes.c_uint16),
        ("par_t2", ctypes.c_int16),
        ("par_t3", ctypes.c_int8),
        ("par_p1", ctypes.c_uint16),
        ("par_p2", ctypes.c_int16),
        ("par_p3", ctypes.c_int8),
        ("par_p4", ctypes.c_int16),
        ("par_p5", ctypes.c_int16),
        ("par_p6", ctypes.c_int8),
        ("par_p7", ctypes.c_int8),
        ("par_p8", ctypes.c_int16),
        ("par_p9", ctypes.c_int16),
        ("par_p10", ctypes.c_uint8),
        ("t_fine", ctypes.c_float),  # Assuming BME68X_USE_FPU is always defined
        ("res_heat_range", ctypes.c_uint8),
        ("res_heat_val", ctypes.c_int8),
        ("range_sw_err", ctypes.c_int8),
    ]


class BME68xIntf(ctypes.c_int):
    BME68X_INTF_SPI = 0
    BME68X_INTF_I2C = 1


class BME68xDev(ctypes.Structure):
    _fields_ = [
        ("chip_id", ctypes.c_uint8),
        ("intf_ptr", ctypes.c_void_p),
        ("variant_id", ctypes.c_uint32),
        ("intf", BME68xIntf),
        ("mem_page", ctypes.c_uint8),
        ("amb_temp", ctypes.c_int8),
        ("calib", BME68xCalibData),  # Assuming bme68x_calib_data is defined as a structure
        ("read", ctypes.CFUNCTYPE(BME68X_INTF_RET_TYPE, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.c_void_p)),
        ("write", ctypes.CFUNCTYPE(BME68X_INTF_RET_TYPE, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.c_void_p)),
        ("delay_us", ctypes.CFUNCTYPE(None, ctypes.c_uint32, ctypes.c_void_p)),  # Replace the None with the appropriate return type
        ("intf_rslt", BME68X_INTF_RET_TYPE),
        ("info_msg", ctypes.c_uint8),
    ]


class I2CConfig(ctypes.Structure):
    _fields_ = [("bus", ctypes.c_uint8),
                ("address", ctypes.c_uint8)]


class SensorConfig(ctypes.Structure):
    _fields_ = [("os_hum", ctypes.c_uint8),
                ("os_temp", ctypes.c_uint8),
                ("os_pres", ctypes.c_uint8),
                ("filter", ctypes.c_uint8),
                ("odr", ctypes.c_uint8)]


class HeaterConfig(ctypes.Structure):
    _fields_ = [("enable", ctypes.c_uint8),
                ("heatr_temp", ctypes.c_uint16),
                ("heatr_dur", ctypes.c_uint16),
                ("heatr_temp_prof", ctypes.POINTER(ctypes.c_uint16)),
                ("heatr_dur_prof", ctypes.POINTER(ctypes.c_uint16)),
                ("profile_len", ctypes.c_uint8),
                ("shared_heatr_dur", ctypes.c_uint16)]


class Sensor(ctypes.Structure):
    _fields_ = [("sensor", SensorConfig),
                ("heater", HeaterConfig),
                ("interface_settings", ctypes.c_void_p),
                ("bme68x_dev", BME68xDev)]


bme68x_read_fptr_t = ctypes.CFUNCTYPE(
    BME68X_INTF_RET_TYPE,
    ctypes.c_uint8,
    ctypes.POINTER(ctypes.c_uint8),
    ctypes.c_uint32,
    ctypes.c_void_p
)
bme68x_write_fptr_t = ctypes.CFUNCTYPE(
    BME68X_INTF_RET_TYPE,
    ctypes.c_uint8,
    ctypes.POINTER(ctypes.c_uint8),
    ctypes.c_uint32,
    ctypes.c_void_p
)
bme68x_delay_us_fptr_t = ctypes.CFUNCTYPE(None, ctypes.c_uint32, ctypes.c_void_p)


@bme68x_read_fptr_t
def i2c_read(reg_addr, reg_data, data_len, intf_ptr):
    i2c_config = ctypes.cast(intf_ptr, ctypes.POINTER(I2CConfig))

    with SMBus(i2c_config.contents.bus) as bus:
        data_from_i2c = bus.read_i2c_block_data(
            i2c_config.contents.address, reg_addr, data_len
        )

        for i in range(data_len):
            reg_data[i] = data_from_i2c[i]

    return 0


@bme68x_write_fptr_t
def i2c_write(reg_addr, reg_data, data_len, intf_ptr):
    i2c_config = ctypes.cast(intf_ptr, ctypes.POINTER(I2CConfig))

    with SMBus(i2c_config.contents.bus) as bus:
        bus.write_i2c_block_data(i2c_config.contents.address, reg_addr, reg_data)

    return 0


@bme68x_delay_us_fptr_t
def sleep_us(microseconds, intf_ptr):
    seconds = microseconds / 1e6  # Convert microseconds to seconds
    time.sleep(seconds)


def main():
    sensor_config = SensorConfig(
        os_hum=1,
        os_pres=1,
        os_temp=1,
        odr=8,
        filter=0
    )
    heater_config = HeaterConfig(
        enable=0x01,
        heatr_temp=300,
        heatr_dur=100
    )
    i2c_config = I2CConfig(bus=1, address=0x77)
    sensor = Sensor(
        sensor=sensor_config,
        heater=heater_config,
        interface_settings=ctypes.cast(ctypes.byref(i2c_config), ctypes.c_void_p),
        bme68x_dev=BME68xDev()
    )

    klipper_bm68x.sensor_init(bme68x_write_fptr_t(i2c_write), i2c_read, sleep_us, ctypes.byref(sensor))


if __name__ == "__main__":
    main()
