/**
 * @file WireSlave.cpp
 * @author Gutierrez PS <https://github.com/gutierrezps>
 * @brief TWI/I2C slave library for ESP32 based on ESP-IDF slave API
 * @date 2021-02-21
 *
 */
#ifdef ARDUINO_ARCH_ESP32
#include <Arduino.h>
#include <driver/i2c.h>
#include <soc/i2c_reg.h>
#include <esp_err.h>

#include "WireSlave.h"

TwoWireSlave::TwoWireSlave(uint8_t bus_num)
    :num(bus_num & 1)
    ,portNum(i2c_port_t(bus_num & 1))
    ,sda(-1)
    ,scl(-1)
//    ,isr_handle_(nullptr)
    ,rxIndex(0)
    ,rxLength(0)
    ,rxQueued(0)
    ,txIndex(0)
    ,txLength(0)
    ,txAddress(0)
    ,txQueued(0)
    ,packer_()
    ,unpacker_()
{

}

TwoWireSlave::~TwoWireSlave()
{
    flush();
    i2c_driver_delete(portNum);
    //i2c_isr_free(isr_handle_);
}


bool TwoWireSlave::begin(int sda, int scl, int address)
{
    i2c_config_t config;
    config.sda_io_num = gpio_num_t(sda);
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_io_num = gpio_num_t(scl);
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.mode = I2C_MODE_SLAVE;
    config.slave.addr_10bit_en = 0;
    config.slave.slave_addr = address & 0x7F;

    esp_err_t res = i2c_param_config(portNum, &config);

    if (res != ESP_OK) {
        log_e("invalid I2C parameters");
        return false;
    }


    // int intr_alloc_flags = 0;//I2C_SLAVE_TRAN_COMP_INT_RAW;//I2C_END_DETECT_INT_RAW;
    // res = i2c_isr_register(portNum, &TwoWireSlave::i2c_isr_callback, this, intr_alloc_flags, &isr_handle_);
    // if (res != ESP_OK) {
    //     log_e("failed to register I2C isr");
    //     Serial.print("failed to register I2C isr: ");
    //     Serial.print(esp_err_to_name(res));
    //     Serial.print(" ");
    //     Serial.println(res);
    // }

    res = i2c_driver_install(
            portNum,
            config.mode,
            2 * I2C_BUFFER_LENGTH,  // rx buffer length
            2 * I2C_BUFFER_LENGTH,  // tx buffer length
            0);

    if (res != ESP_OK) {
        log_e("failed to install I2C driver");
    }

    return res == ESP_OK;
}

// void TwoWireSlave::i2c_isr_callback(void* arg)
// {
//     Serial.print("i2c_isr_callback: ");
//     TwoWireSlave* me = reinterpret_cast<TwoWireSlave*>(arg);
//     int interruptNo = esp_intr_get_intno(me->isr_handle_);
//     Serial.println(interruptNo);
//     Serial.print(", R ");
//     Serial.print(*((volatile uint32_t *) (I2C_INT_RAW_REG(0))));
//     Serial.print(" ");
//     Serial.println(*((volatile uint32_t *) (I2C_INT_RAW_REG(1))));
//     Serial.print(", E ");
//     Serial.print(*((volatile uint32_t *) (I2C_INT_ENA_REG(0))));
//     Serial.print(" ");
//     Serial.println(*((volatile uint32_t *) (I2C_INT_ENA_REG(1))));
//     Serial.print(", S ");
//     Serial.print(*((volatile uint32_t *) (I2C_INT_STATUS_REG(0))));
//     Serial.print(" ");
//     Serial.println(*((volatile uint32_t *) (I2C_INT_STATUS_REG(1))));

        

//     *((volatile uint32_t *) (I2C_INT_CLR_REG(0))) |= interruptNo;
//     *((volatile uint32_t *) (I2C_INT_CLR_REG(0))) |= 0xff;
//     *((volatile uint32_t *) (I2C_INT_CLR_REG(1))) |= 0xff;

//     esp_intr_disable(me->isr_handle_);
//     esp_intr_enable(me->isr_handle_);

//     uint8_t inputBuffer[I2C_BUFFER_LENGTH] = {0};
//     int16_t inputLen = 0;

//     //inputLen = i2c_slave_read_buffer(me->portNum, inputBuffer, I2C_BUFFER_LENGTH, 1);
//     // Serial.print("read: ");
//     // Serial.println(inputLen);

    
// 	// if(i2c_isr_free(me->isr_handle_) == ESP_OK)
//     // {
// 	// 	me->isr_handle_ = nullptr;
// 	// 	ets_printf("Free-ed interrupt handler\n");
// 	// }
//     // else
//     // {
// 	// 	ets_printf("Failed to free interrupt handler\n");
// 	// }
// }


void TwoWireSlave::enqueueSend(int size, const uint8_t* buffer)
{
    //Serial.println("sending");
    i2c_reset_tx_fifo(portNum);
    i2c_slave_write_buffer(portNum, const_cast<uint8_t*>(buffer), size, 0);
}

void TwoWireSlave::update()
{
    uint8_t inputBuffer[I2C_BUFFER_LENGTH] = {0};
    int16_t inputLen = 0;

    //Serial.print("TwoWireSlave::update(), ");
    inputLen = i2c_slave_read_buffer(portNum, inputBuffer, I2C_BUFFER_LENGTH, 1);

    if (inputLen <= 0) {
        // nothing received or error
        //Serial.println("nothing received");
        return;
    }

    if(user_onRawReceive)
    {
        //Serial.println("received");
        user_onRawReceive(inputLen, inputBuffer);
        return;
    }

    if (!unpacker_.isPacketOpen()) {
        // start unpacking
        unpacker_.reset();
    }

    unpacker_.write(inputBuffer, size_t(inputLen));

    if (unpacker_.isPacketOpen() || unpacker_.totalLength() == 0) {
        // still waiting bytes,
        // or received bytes that are not inside a packet
        //Serial.println("waiting bytes");
        return;
    }

    if (unpacker_.hasError()) {
        //Serial.println("unpacker_.hasError");
        return;
    }

    if (unpacker_.available()) {
        rxIndex = 0;
        rxLength = unpacker_.available();

        // transfer bytes from packet to rxBuffer
        while (unpacker_.available()) {
            rxBuffer[rxIndex] = unpacker_.read();
            rxIndex++;
        }
        rxIndex = 0;

        // call user callback
        if (user_onReceive) {
            user_onReceive(rxLength);
        }
    }
    else if (user_onRequest) {
        txIndex = 0;
        txLength = 0;
        packer_.reset();
        user_onRequest();
        packer_.end();

        while (packer_.available()) {
            txBuffer[txIndex] = packer_.read();
            ++txIndex;
        }
        txLength = txIndex;

        i2c_reset_tx_fifo(portNum);
        i2c_slave_write_buffer(portNum, txBuffer, txLength, 0);
    }
}

size_t TwoWireSlave::write(uint8_t data)
{
    if (packer_.packetLength() >= I2C_BUFFER_LENGTH) {
        return 0;
    }

    return packer_.write(data);
}

size_t TwoWireSlave::write(const uint8_t *data, size_t quantity)
{
    for (size_t i = 0; i < quantity; ++i) {
        if (!write(data[i])) {
            return i;
        }
    }
    return quantity;
}

int TwoWireSlave::available(void)
{
    return rxLength - rxIndex;
}

int TwoWireSlave::read(void)
{
    int value = -1;
    if(rxIndex < rxLength) {
        value = rxBuffer[rxIndex];
        ++rxIndex;
    }
    return value;
}

int TwoWireSlave::peek(void)
{
    int value = -1;
    if(rxIndex < rxLength) {
        value = rxBuffer[rxIndex];
    }
    return value;
}

void TwoWireSlave::flush(void)
{
    rxIndex = 0;
    rxLength = 0;
    txIndex = 0;
    txLength = 0;
    rxQueued = 0;
    txQueued = 0;
    i2c_reset_rx_fifo(portNum);
    i2c_reset_tx_fifo(portNum);
}


TwoWireSlave WireSlave = TwoWireSlave(0);
TwoWireSlave WireSlave1 = TwoWireSlave(1);

#endif      // ifdef ARDUINO_ARCH_ESP32
