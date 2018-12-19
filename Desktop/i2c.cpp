//
//  i2c.cpp
//  scdtr
//
//  Created by David Teles on 08/11/2018.
//

#include <stdio.h>
#include <pigpio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <memory.h>
#define DESTINATION_ADDR 0x00
#define SLAVE_ADDR 0x00




int close_slave(bsc_xfer_t & xfer);
int init_slave(bsc_xfer_t &xfer, int addr);

int main(int argc, char *argv[]) {
    int status, j;
    int key = 0;
    int handle;
    int length = 12; //11 chars + \0
    char *message;

    if (gpioInitialise() < 0) {
        printf("Erro 1\n");
        return 1;
    }

    bsc_xfer_t xfer;
    status = init_slave(xfer, SLAVE_ADDR);
    handle = i2cOpen(1, DESTINATION_ADDR, 0); /* Initialize */
    printf("Press q to quit. Any other key to continue.\n");

    while(1) {
        xfer.txCnt = 0;
        status = bscXfer(&xfer);
        if (xfer.rxCnt > 0) {
	    printf("Received %d bytes\n", xfer.rxCnt);
	    for (j = 0; j < xfer.rxCnt; j++) {
                printf("%d",xfer.rxBuf[j]);
	    }
            printf("\n");
	    memset(xfer.rxBuf,0,strlen(xfer.rxBuf));
	    xfer.rxBuf[0] = '\0';
        }
    }

    i2cClose(handle); /* close master */
    status = close_slave(xfer);
    gpioTerminate();
}

int init_slave(bsc_xfer_t &xfer, int addr) {
    gpioSetMode(18, PI_ALT3);
    gpioSetMode(19, PI_ALT3);
    xfer.control = (addr<<16) | /* Slave address */
    (0x00<<13) | /* invert transmit status flags */
    (0x00<<12) | /* enable host control */
    (0x00<<11) | /* enable test fifo */
    (0x00<<10) | /* invert receive status flags */
    (0x01<<9) | /* enable receive */
    (0x01<<8) | /* enable transmit */
    (0x00<<7) | /* abort and clear FIFOs */
    (0x00<<6) | /* send control reg as 1st I2C byte */
    (0x00<<5) | /* send status regr as 1st I2C byte */
    (0x00<<4) | /* set SPI polarity high */
    (0x00<<3) | /* set SPI phase high */
    (0x01<<2) | /* enable I2C mode */
    (0x00<<1) | /* enable SPI mode */
    0x01 ; /* enable BSC peripheral */
    return bscXfer(&xfer);
}

int close_slave(bsc_xfer_t & xfer) {
    xfer.control = 0;
    return bscXfer(&xfer);
}

