// I2Cdev library collection - HMC5883L I2C device class header file
// Based on Honeywell HMC5883L datasheet, 10/2010 (Form #900405 Rev B)
// 6/12/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-12 - fixed swapped Y/Z axes
//     2011-08-22 - small Doxygen comment fixes
//     2011-07-31 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HMC5883L_H_
#define _HMC5883L_H_

#include "I2Cdev.h"

#include "HMC5883Lx2_inc.h"


class HMC5883L
{
    public:
        HMC5883L();
        HMC5883L(uint8_t address);
        
        void init( const uint8_t &, const uint8_t & );
        void initialize();
        bool testConnection();

        // CONFIG_A register
        uint8_t getSampleAveraging();
        void setSampleAveraging(uint8_t averaging);
        uint8_t getDataRate();
        void setDataRate(uint8_t rate);
        uint8_t getMeasurementBias();
        void setMeasurementBias(uint8_t bias);

        // CONFIG_B register
        uint8_t getGain();
        void setGain(uint8_t gain);

        // MODE register
        uint8_t getMode();
        void setMode(uint8_t mode);

        // DATA* registers
        void getHeading(int16_t *x, int16_t *y, int16_t *z);
        int16_t getHeadingX();
        int16_t getHeadingY();
        int16_t getHeadingZ();

        // STATUS register
        bool getLockStatus();
        bool getReadyStatus();

        // ID_* registers
        uint8_t getIDA();
        uint8_t getIDB();
        uint8_t getIDC();

    private:
        uint8_t devAddr;
        uint8_t buffer[6];
        uint8_t mode;
};

#endif /* _HMC5883L_H_ */
