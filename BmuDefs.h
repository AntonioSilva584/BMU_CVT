#ifndef BMUDEFS_H
#define BMUDEFS_H

/* Conversion definitions */

#define sample 150
#define CVTsample 50
#define ADCVoltageLimit  3.3  

typedef enum
{
    IDLE_ST,
    Voltage_ST,
    CVTtemperature_ST,
    SystemCurrent_ST  // send data for debug
    
} state_t;

#endif