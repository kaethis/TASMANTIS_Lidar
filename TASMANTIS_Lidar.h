#ifndef _TASMANTIS_LIDAR_H_

#define _TASMANTIS_LIDAR_H_


#include <Arduino.h>

#include "RPLidar.h"

#include "TASMANTIS_Lidar.h"


#define RPLIDAR_SERIAL Serial2


class TASMANTIS_Lidar { /* -----------------------------------------------------------------------------------------------------
* This CLASS ...
* --------------------------------------------------------------------------------------------------------------------------- */

  public:

    struct scan {

        int dists[360] = { 0 };
    };


    const int SPEED_ON = 255;

    const int SPEED_OFF = 0;

    const int STBIT_N_HIGH = 100;

    const int STBIT_N_MED = 50;

    const int STBIT_N_LOW = 10;

    const unsigned long WAIT_MS = 1000;

    const byte QUALITY_OK = 0x0F;


    TASMANTIS_Lidar(int, int);


    bool start(void);

    void stop(void);

    bool capture(struct scan*, int, int*);

    bool capture(struct scan *s, int *stbit_i) { return capture(s, STBIT_N_LOW, stbit_i); } 


  private:

    RPLidar lidar;

    int pwr_pin,

        mtr_pin;

    unsigned long dists[360];

    int dists_n[360];

    bool is_start,

         is_ready,

         is_capture,

         is_new;

    unsigned long timer;

    unsigned long sample_n;
};

#endif /* _TASMANTIS_LIDAR_H_ */
