#include "mbed.h"
#include <cstdint>
#include "CANMsg.h"
//#include "TextLCD.h"
#include "definitions.h"
#include "BmuDefs.h"
#include "MLX90614.h"
#include <math.h>


#define default_addr   (0x00)

//Variables Iniciation
double MeasureVoltage = 0.0;
double MeasureSystemCurrent= 0.0;
float Calculate_VacsI0 = 0.0;

uint8_t MeasureCVTtemperature;

uint8_t SOC;
uint8_t MeasureVoltage100 ;
double a ;
double b ;
double c ;
double d ;

double A, B, C, D;



/*Pins*/
AnalogIn ReadVoltage(PA_1);                 // VARIÁVEL PARA ARMAZENAR A LEITURA DO PINO ANALÓGICO
AnalogIn ReadSystemCurrent(PA_5);    
DigitalOut Led(PC_13);


/* Communication protocols */
I2C i2c(PB_7, PB_6);   //sda,scl
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_9, PA_10, 115200);

//I2C i2c_lcd(PB_7, PB_6); // SDA, SCL
//TextLCD_I2C lcd(&i2c_lcd, 0x4E, TextLCD::LCD16x2); // I2C bus, PCF8574 Slaveaddress, LCD Type, Device Type 


/*General Libraries*/
MLX90614 mlx(&i2c);

/* General functions*/
void setupInterrupts();
double Voltage_moving_average();
double SystemCurrent_moving_average();
double CVT_Temperature();

/* Debug variables */
Timer t;
bool buffer_full = false;
unsigned int t0, t1;

/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
Ticker ticker16mHz;
Ticker ticker33mHz;
Ticker ticker66mHz;
CircularBuffer <state_t, BUFFER_SIZE> state_buffer;
state_t current_state = IDLE_ST;

/* Interrupt services routine */
void ticker16mHzISR();
void ticker33mHzISR();
void ticker66mHzISR();


// main() runs in its own thread in the OS

int main()
{
    /*
    lcd.setMode(TextLCD::DispOn); //DispOff, DispOn
    lcd.setBacklight(TextLCD::LightOff);//LightOff, LightOn
    lcd.setCursor(TextLCD::CurOff_BlkOff);//CurOff_BlkOff, CurOn_BlkOff, CurOff_BlkOn, CurOn_BlkOn

    lcd.cls();
    lcd.printf("%.2f",MeasureVoltage);
    lcd.printf("%.2f",MeasureSystemCurrent);
    lcd.printf("%.2f",Calculate_VacsI0);
    */

    /* Main variables */
    CANMsg txMsg;
    setupInterrupts();  

    while (true) {
                 
       if (state_buffer.full())
        {
            buffer_full = true;
            //Led = 0;
            state_buffer.pop(current_state);
        }
        else
        {
            //Led = 1;
            buffer_full = false;
            if (!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = IDLE_ST;
        }

      switch(current_state) {
        
        case IDLE_ST:
           
            break;

        case Voltage_ST:
     
            MeasureVoltage = Voltage_moving_average();
            
             a = -8E-8*pow(MeasureVoltage,4);
             b = 3E-5*pow(MeasureVoltage,3);
             c = -0.0026*pow(MeasureVoltage,2);
             d = -0.4058*MeasureVoltage;
            SOC = (uint8_t)(a + b + c + d + 100);
             //Led = !Led;
            //SOC = 100;

            MeasureVoltage100 = 100 * (uint8_t ) MeasureVoltage; 
            txMsg.clear(Voltage_ID);
            txMsg << MeasureVoltage100;
            can.write(txMsg);
            /*
            if (can.write(txMsg)){
               Led = !Led;
            }  
          
          */            
            txMsg.clear(SOC_ID);
            txMsg << SOC;
            can.write(txMsg);
            
            break;

        case CVTtemperature_ST:
            
            
            MeasureCVTtemperature = (uint8_t) CVT_Temperature(); 
            //MeasureCVTtemperature = 90;
            txMsg.clear(TempCVT_ID);
            txMsg << MeasureCVTtemperature;
            
            if (can.write(txMsg)){
               Led = !Led;
            }           
            
            break;

        case SystemCurrent_ST:

            uint16_t SignalVacsI0 = ReadSystemCurrent.read_u16();
            Calculate_VacsI0 = (SignalVacsI0 * (ADCVoltageLimit / 65535.0));


            MeasureSystemCurrent = SystemCurrent_moving_average();
            //Led = !Led;
            
             

            txMsg.clear(Current_ID);
            txMsg << MeasureSystemCurrent;
            can.write(txMsg);
            break;          
        
        }
    
     
    
        //print na serial MED 
        
        serial.printf("temp_med Ambiente = ");
        serial.printf("%d",MeasureCVTtemperature);
        serial.printf("  Voltage Battery = ");
        serial.printf("%.2f", MeasureVoltage);
        serial.printf("  State of charge = ");
        serial.printf("%d", SOC);
        serial.printf("  Calculate_VacsI0 = ");
        serial.printf("%.3f", Calculate_VacsI0);
        serial.printf("  MeasureSystemCurrent = ");
        serial.printf("%.3f", MeasureSystemCurrent);
        serial.printf("\n");

        ThisThread::sleep_for(200);

    }
 }

/* General functions */
void setupInterrupts()
{
    ticker16mHz.attach(&ticker16mHzISR, 5);
    ticker33mHz.attach(&ticker33mHzISR, 2);
    ticker66mHz.attach(&ticker66mHzISR, 6);
}



void ticker16mHzISR()
{
    state_buffer.push(Voltage_ST);
    
}


void ticker33mHzISR()
{
    state_buffer.push(SystemCurrent_ST);
}


void ticker66mHzISR()
{
    state_buffer.push(CVTtemperature_ST);
}


double Voltage_moving_average(){   
    int i,j;
    double value, aux, ADCvoltage ,InputVoltage, AverageVoltage = 0.0 ;
    uint16_t SignalVoltage = 0.0;
    float Calibration_Factor = 0.987;
    float R1_Value = 30000.0;               // VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO
    float R2_Value = 7500.0;                // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO


    for(j = 0; j < (sample); j++){ 

        for(i = 0; i < sample ; i++){

            SignalVoltage = ReadVoltage.read_u16();
            InputVoltage = (SignalVoltage * ADCVoltageLimit) / 65535.0;
            ADCvoltage = Calibration_Factor*(InputVoltage / (R2_Value/(R1_Value + R2_Value)));
            aux += ADCvoltage;
        }
        
            value = aux / (double)sample;
        
        
            if(value > AverageVoltage){
                AverageVoltage = value;
            }
        
    }
    //return value;
    return AverageVoltage/(double)sample; // I don't know why we need divide by sample, but works.
}


double SystemCurrent_moving_average(){   
    int i,j;
    double value,aux, InputSystemCurrent, AverageSystemCurrent, ADCSystemCurrent = 0.0 ;
    uint16_t SignalCurrent = 0.0;
    float VacsI0 = 2.256397;
    float Current_Calibration_Factor = 1.575;
    

    for(j = 0; j < (sample); j++){ 

        for(i = 0; i < (sample)*2 ; i++){

            SignalCurrent = ReadSystemCurrent.read_u16();
            InputSystemCurrent = (SignalCurrent * (ADCVoltageLimit / 65535.0));
            ADCSystemCurrent = (VacsI0 - InputSystemCurrent) / 0.185;
            aux += ADCSystemCurrent;
        }       
            
            value = (aux * Current_Calibration_Factor)  / ((double)sample*2);
            /*
            if(value > AverageSystemCurrent){
                AverageSystemCurrent = value;
            }
           */
    }
    //return value;
    return value/(double)sample; // I don't know why we need divide by sample, but works.
}

double CVT_Temperature(){
int i,j;
    double AverageObjectTemp, AverageEnviromentTemp = 0.0;
    double temp_amb,med_amb,x_amb;
    double temp_obj,med_obj,x_obj; 

//teste comunicação i2c
    char ucdata_write[2];

 if (!i2c.write((default_addr|0x00), ucdata_write, 1, 0)){// Check for ACK from i2c Device

    for(j = 0; j < (CVTsample); j++){ 

        for(i = 0; i < (CVTsample) ; i++){
                      
            // temp_amb = mlx.read_temp(0);
           

                temp_obj = mlx.read_temp(1);
                x_obj += temp_obj;
                med_obj = x_obj/ (double)CVTsample;
    
                if(med_obj > AverageObjectTemp){
                AverageObjectTemp = med_obj;
                 }
            }
        }
    }

    else{
        AverageObjectTemp = 0;
    }

    //return value;
    return AverageObjectTemp/(double)CVTsample; // I don't know why we need divide by sample, but works.
}

