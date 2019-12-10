/*
 * Metro.hpp
 *
 *  Created on: 4 déc. 2019
 *      Author: Tic-Tac
 */

#ifndef LL_TIME_METRO_HPP_
#define LL_TIME_METRO_HPP_


#include "UTILITY/timing.h"


class Metro{
public:

    uint32_t current;
    uint32_t last_check;
    uint32_t interval;

    Metro(uint32_t interval_ms){
        this->last_check = millis();
        this->interval = interval_ms;
    }

    bool check(){
        this->current = millis();
        if(this->current-this->last_check>this->interval){
            this->last_check=this->current;
            return true;
        }
        else{
            return false;
        }
    }

    void reset(){
        last_check=millis();
    }
};


#endif /* LL_TIME_METRO_HPP_ */
