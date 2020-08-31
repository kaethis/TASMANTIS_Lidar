#include "TASMANTIS_Lidar.h"


TASMANTIS_Lidar::TASMANTIS_Lidar(int pwr_pin, int mtr_pin) { /* -----------------------------------------------------------------
* This CONSTRUCTOR ...
* ---------------------------------------------------------------------------------------------------------------------------- */


    this->pwr_pin = pwr_pin;

    this->mtr_pin = mtr_pin;


    pinMode(pwr_pin, OUTPUT);

    pinMode(mtr_pin, OUTPUT);


    is_start = false;

    is_ready = false;

    is_capture = false;

    is_new = false;


    digitalWrite(pwr_pin, HIGH);


    lidar.begin(RPLIDAR_SERIAL);
}


bool TASMANTIS_Lidar::start() { /* ---------------------------------------------------------------------------------------------
* This CONSTRUCTOR ...
* --------------------------------------------------------------------------------------------------------------------------- */

    rplidar_response_device_info_t info;


    if (!is_start) {

        if (IS_OK(lidar.getDeviceInfo(info, 100))) {

            lidar.startScan();
       

            is_start = true;

            is_ready = false;


            analogWrite(mtr_pin, SPEED_ON);


            timer = millis();

        } else { 

            //Serial.println("LIDAR NOT FOUND!"); 
        }

    } else

        if (millis() >= (timer + WAIT_MS))

            is_ready = true;


    return is_ready;
}


void TASMANTIS_Lidar::stop() { /* ----------------------------------------------------------------------------------------------
* This FUNCTION ...
* --------------------------------------------------------------------------------------------------------------------------- */

    is_start = false;

    is_ready = false;

    is_capture = false;

    is_new = false;


    analogWrite(mtr_pin, SPEED_OFF); 
}


bool TASMANTIS_Lidar::capture(struct scan *s, int stbit_n, int *stbit_i) { /* --------------------------------------------------
* This FUNCTION ...
* --------------------------------------------------------------------------------------------------------------------------- */

    float dist,

          angle;

    byte quality;

    bool is_stbit;


    if (is_ready) {

        if (!is_capture) {

            for(int i = 0; i < 360; i++) {
               
                s->dists[i] = 0;


                dists[i] = 0;

                dists_n[i] = 0;
            }


            is_capture = true;

            is_new = false;


            *stbit_i = 0;
            
        } else {

            if (IS_OK(lidar.waitPoint())) {

                dist = (lidar.getCurrentPoint().distance / 10.0f);

                angle = lidar.getCurrentPoint().angle;

                is_stbit = lidar.getCurrentPoint().startBit;

                quality = lidar.getCurrentPoint().quality;


                if (!is_new) {

                    if (is_stbit && (quality == QUALITY_OK)) {

                        dists[359 - int(angle)] = int(dist);

                        dists_n[359 - int(angle)]++;


                        *stbit_i += 1;


                        is_new = true;
                    }

                } else {

                    if (quality == QUALITY_OK) {

                        dists[359 - int(angle)] += int(dist);

                        dists_n[359 - int(angle)]++;


                        if (is_stbit) *stbit_i += 1;
                    }


                    if (*stbit_i == stbit_n) {

                        is_capture = false;

                        
                        for (int i = 0; i < 360; i++)
                            
                            if (dists_n[i] != 0) s->dists[i] = int(dists[i] / dists_n[i]);


                        return true;
                    }
                }
            }
        }
    }


    return false;
}
