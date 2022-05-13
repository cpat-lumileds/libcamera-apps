#include <unistd.h>        //Needed for I2C port
#include <fcntl.h>          //Needed for I2C port
#include <sys/ioctl.h>      //Needed for I2C port
#include <linux/i2c-dev.h>  //Needed for I2C port
#include <linux/i2c.h>      //Needed for I2C port

#include <iostream>
#include <iomanip>
#include <cerrno>
#include <cstdint>
#include <cstring>

#include <string>
#include <unordered_map>

#include "lumileds_i2c.hpp"

std::unordered_map<std::string, int> LumiledsI2C::commands {
    {"torch", 0x0C},
    {"flash", 0x0B},
    {"clearDacs", 0x08},
    {"flashToDac", 0x01},
    {"torchToDac", 0x09},
    {"clearErrors", 0x03},
    {"softReset", 0x05},
};

LumiledsI2C::~LumiledsI2C(){
    delete [] buffer_;
}

int LumiledsI2C::makeFlash(){
    return directWrite(commands["flash"], true);
}

int LumiledsI2C::makeTorch(){
    return directWrite(commands["torch"], true);
}

int LumiledsI2C::clearDacs(){
    return directWrite(commands["clearDacs"], true);
}

int LumiledsI2C::setTorchToDac(){
    return directWrite(commands["torchToDac"], true);
}

int LumiledsI2C::setFlashToDac(){
    return directWrite(commands["flashToDac"], true);
}

int LumiledsI2C::clearErrors(){
    return directWrite(commands["clearErrors"], true);
}

int LumiledsI2C::softReset(){
    return directWrite(commands["softReset"], true);
}

int LumiledsI2C::setupI2C()
{
    //----- OPEN THE I2C BUS -----

    int fd;
    int rv;

    if ((fd = open(i2c_filename_.c_str(), O_RDWR)) < 0)
    {
        //ERROR HANDLING: you can check errno to see what went wrong
        std::cerr << "Failed to open the i2c bus. Error code: " << fd << std::endl;
        return fd;
    }

    if ((rv = ioctl(fd, I2C_SLAVE, i2c_addr_)) < 0)
    {
        std::cerr << "Failed to acquire bus access and/or talk to slave. Error code: " << rv << std::endl;
        //ERROR HANDLING; you can check errno to see what went wrong
        return rv;
    }

    return fd;
}

int LumiledsI2C::readBlock(uint8_t reg_addr, unsigned char* return_buffer){
    unsigned char values[] {0x01, reg_addr, 0x00, 0x00};

    if ((write(i2c_conn_, values, i2c_data_len_)) != i2c_data_len_) {
		std::cerr << "Error writing to i2c slave..." << std::endl;
        return -1;
	}

    if (read(i2c_conn_, return_buffer, i2c_data_len_) != i2c_data_len_){
        std::cerr << "Unable to read from i2c slave ..." << std::endl;
        return -1;
    }

    return 0;

}

int LumiledsI2C::writeBlock(uint8_t reg_addr, uint8_t data, bool read_back){
    unsigned char values[] {0x00, reg_addr, data, 0x00};

    if ((write(i2c_conn_, values, i2c_data_len_)) != i2c_data_len_) {
		std::cerr << "Error writing to i2c slave..." << std::endl;
        return -1;
	}

    if (read_back){
        if (read(i2c_conn_, buffer_, i2c_data_len_) != i2c_data_len_){
		    std::cerr << "Unable to read from i2c slave ..." << std::endl;
		    return -1;
	    }
    }

    return 0;
}


int LumiledsI2C::directWrite(uint8_t command, bool read_back){
    unsigned char values[] {0x04, command, 0x00, 0x00};

    if ((write(i2c_conn_, values, i2c_data_len_)) != i2c_data_len_) {
		std::cerr << "Error writing to i2c slave..." << std::endl;
        return -1;
	}

    if (read_back){
        if (read(i2c_conn_, buffer_, i2c_data_len_) != i2c_data_len_){
		    std::cerr << "Unable to read from i2c slave ..." << std::endl;
		    return -1;
	    }
    }

    return 0;
}

uint8_t* LumiledsI2C::split16Bit(uint16_t x){
    static uint8_t r[2];
    r[0] = x & 0xff;
    r[1] = x >> 8;
    return r;
}


int LumiledsI2C::write16BitData(const uint8_t reg_adr [], uint16_t data, bool read_back){  
    int rv {0};
  
    uint8_t* d = split16Bit(data);
    for (int i = 0; i < 2; i++){
        rv = writeBlock(reg_adr[i], *(d + i), read_back);
    }
    return rv;
}