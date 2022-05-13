#pragma once

#include <string>
#include <unordered_map>

class LumiledsI2C{
    public:
        LumiledsI2C(): i2c_addr_{0x79}, i2c_filename_{"/dev/i2c-1"}, i2c_data_len_{4}, rows_{3}, cols_{3}, regs_{2}
        {
            i2c_conn_ = setupI2C();
            buffer_ = new unsigned char[i2c_data_len_];

        };
        int setupI2C();
        int makeFlash();
        int makeTorch();
        int clearDacs();
        int setTorchToDac();
        int setFlashToDac();
        int clearErrors();
        int softReset();
        int directWrite(uint8_t cmd, bool read_back); 
        int writeBlock(uint8_t reg_addr, uint8_t data, bool read_back);
        int readBlock(uint8_t reg_addr, unsigned char* return_buffer);
        int write16BitData(const uint8_t reg_adr [], uint16_t data, bool read_back);
        static uint8_t* split16Bit(uint16_t);

        ~LumiledsI2C();

    private:
        int i2c_addr_;
        std::string i2c_filename_;
        int i2c_data_len_;
        int rows_, cols_, regs_;
        int i2c_conn_;
        unsigned char* buffer_;
        static std::unordered_map<std::string, int> commands;
};


