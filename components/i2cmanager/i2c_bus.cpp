#include "i2c_bus.hpp"
#include "esp_log.h"

using namespace std::chrono;

static const char* TAG = "I2C_NEW";

I2CBus::I2CBus(i2c_port_num_t port)
    : m_port(port)
{
}

I2CBus::I2CBus(I2CBus&& other) noexcept
{
    m_port = other.m_port;
    m_bus = other.m_bus;
    other.m_bus = nullptr;
}

I2CBus& I2CBus::operator=(I2CBus&& other) noexcept
{
    if (this != &other)
    {
        if (m_bus)
        {
            i2c_del_master_bus(m_bus);
        }
        m_port = other.m_port;
        m_bus = other.m_bus;
        other.m_bus = nullptr;
    }
    return *this;
}

I2CBus::~I2CBus()
{
    if (m_bus)
    {
        i2c_del_master_bus(m_bus);
        m_bus = nullptr;
    }
}

esp_err_t I2CBus::init(gpio_num_t sda,
                       gpio_num_t scl,
                       uint32_t clk_source_hz,
                       bool pullup)
{
    if (m_bus)
    {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = m_port;
    bus_cfg.sda_io_num = sda;
    bus_cfg.scl_io_num = scl;
    bus_cfg.clk_source = clk_source_hz ? I2C_CLK_SRC_DEFAULT : I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = pullup;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &m_bus);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C bus created on port %d", m_port);
    return ESP_OK;
}

esp_err_t I2CBus::add_device(uint8_t addr_7bit,
                             I2CDevice& out_dev,
                             uint32_t scl_speed_hz)
{
    if (!m_bus)
    {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.device_address = addr_7bit;
    dev_cfg.scl_speed_hz = scl_speed_hz;

    i2c_master_dev_handle_t dev_handle = nullptr;
    esp_err_t err = i2c_master_bus_add_device(m_bus, &dev_cfg, &dev_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    out_dev.m_bus = this;
    out_dev.m_dev = dev_handle;
    return ESP_OK;
}

// -------- I2CDevice --------

I2CDevice::I2CDevice(I2CBus& bus, uint8_t addr_7bit, uint32_t scl_speed_hz)
{
    bus.add_device(addr_7bit, *this, scl_speed_hz);
}

I2CDevice::I2CDevice(I2CDevice&& other) noexcept
{
    m_bus = other.m_bus;
    m_dev = other.m_dev;
    other.m_bus = nullptr;
    other.m_dev = nullptr;
}

I2CDevice& I2CDevice::operator=(I2CDevice&& other) noexcept
{
    if (this != &other)
    {
        if (m_bus && m_dev)
        {
            i2c_master_bus_rm_device(m_dev);
        }
        m_bus = other.m_bus;
        m_dev = other.m_dev;
        other.m_bus = nullptr;
        other.m_dev = nullptr;
    }
    return *this;
}

I2CDevice::~I2CDevice()
{
    if (m_bus && m_dev)
    {
        i2c_master_bus_rm_device(m_dev);
        m_dev = nullptr;
    }
}

esp_err_t I2CDevice::write(const uint8_t* data,
                           size_t len,
                           milliseconds timeout)
{
    if (!m_dev || !data || len == 0)
    {
        return ESP_ERR_INVALID_STATE;
    }

    std::unique_lock<std::mutex> lk(m_mutex, std::defer_lock);
    return i2c_master_transmit(m_dev, data, len,
                               static_cast<int>(timeout.count()));
}

esp_err_t I2CDevice::read(uint8_t* data,
                          size_t len,
                          milliseconds timeout)
{
    if (!m_dev || !data || len == 0)
    {
        return ESP_ERR_INVALID_STATE;
    }

    std::unique_lock<std::mutex> lk(m_mutex, std::defer_lock);
    return i2c_master_receive(m_dev, data, len,
                              static_cast<int>(timeout.count()));
}

esp_err_t I2CDevice::write_read(const uint8_t* wdata,
                                size_t wlen,
                                uint8_t* rdata,
                                size_t rlen,
                                milliseconds timeout)
{
    if (!m_dev || !wdata || !rdata || wlen == 0 || rlen == 0)
    {
        return ESP_ERR_INVALID_STATE;
    }

    std::unique_lock<std::mutex> lk(m_mutex, std::defer_lock);

    esp_err_t err = i2c_master_transmit(m_dev, wdata, wlen,
                                        static_cast<int>(timeout.count()));
    if (err != ESP_OK)
    {
        return err;
    }

    return i2c_master_receive(m_dev, rdata, rlen,
                              static_cast<int>(timeout.count()));
}

esp_err_t I2CDevice::writeRegister(uint8_t reg,
                                   uint8_t value,
                                   milliseconds timeout)
{
    uint8_t buf[2] = {reg, value};

    std::unique_lock<std::mutex> lk(m_mutex, std::defer_lock);
    return i2c_master_transmit(
        m_dev,
        buf,
        sizeof(buf),
        static_cast<int>(timeout.count())
    );
}

esp_err_t I2CDevice::readRegister(uint8_t reg,
                                  uint8_t& out,
                                  milliseconds timeout)
{
    std::unique_lock<std::mutex> lk(m_mutex, std::defer_lock);

    // Write register address
    esp_err_t err = i2c_master_transmit(
        m_dev,
        &reg,
        1,
        static_cast<int>(timeout.count())
    );
    if (err != ESP_OK)
        return err;

    // Read 1 byte
    return i2c_master_receive(
        m_dev,
        &out,
        1,
        static_cast<int>(timeout.count())
    );
}

esp_err_t I2CDevice::readRegister16(uint8_t reg,
                                    uint16_t& out,
                                    milliseconds timeout)
{
    uint8_t buf[2];

    std::unique_lock<std::mutex> lk(m_mutex, std::defer_lock);

    // Write register address
    esp_err_t err = i2c_master_transmit(
        m_dev,
        &reg,
        1,
        static_cast<int>(timeout.count())
    );
    if (err != ESP_OK)
        return err;

    // Read 2 bytes
    err = i2c_master_receive(
        m_dev,
        buf,
        2,
        static_cast<int>(timeout.count())
    );
    if (err != ESP_OK)
        return err;

    out = (buf[0] << 8) | buf[1];
    return ESP_OK;
}
