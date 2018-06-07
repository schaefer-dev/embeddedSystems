//
// Created by rafael on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SPIMASTER_H
#define EMBEDDEDSYSTEMS18_SPIMASTER_H


#define SPI_EDGE_LEADING  0
#define SPI_EDGE_TRAILING (1<<CPHA)

#define SPI_MSB_FIRST     0
#define SPI_LSB_FIRST     (1<<DORD)

#define SPI_SCK_IDLE_LOW    0
#define SPI_SCK_IDLE_HIGH   (1<<CPOL)

#define SPI_SPEED_DIVIDER_2   0b100
#define SPI_SPEED_DIVIDER_4   0b000
#define SPI_SPEED_DIVIDER_8   0b101
#define SPI_SPEED_DIVIDER_16  0b001
#define SPI_SPEED_DIVIDER_32  0b110
#define SPI_SPEED_DIVIDER_64  0b010
#define SPI_SPEED_DIVIDER_128 0b011

#define DESELECT 0
#define SELECT_ADC 1
#define SELECT_RF 2


class SPIMaster
{
public:

    static void SPIMasterInit(unsigned char speed_divider, unsigned char options);

    static void slaveSelect(unsigned char slave);

    static unsigned char transmitByte(unsigned char data);

    static void setTimer(int duration);

};


#endif
