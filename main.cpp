/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include "driver_mbed_TH02.h"
#include "CayenneLPP.h"
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"
#include "Servo.h"

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

#define MAX_SIZE 200 // depends on spreading factor and frequency used
#define Num_Table 0

CayenneLPP Payload(MAX_SIZE);

// Dummy values
float celsius = -4.1;
float accel[] = {1.234, -1.234, 0};
float rh = 30;
float hpa = 1014.1;
//float latitude = 42.3519;
//float longitude = -87.9094;
//float altitude=10;


int PRESSION[3][14]={640,751,752,763,774,785,856,947,958,969,1010,1111,1212,1313,
             130,211,222,233,244,255,356,617,638,629,1110,1211,1312,1413,
             10,21,32,43,54,65,76,87,98,99,110,111,112,113 };

int SISMIQUE[3][14] ={6400,7501,7502,7603,7704,7805,8506,9407,9508,9609,10010,11011,12012,13013,
             1300,2101,2202,2303,204,2505,3506,6107,6308,6209,10010,12011,13012,14013,
             100,201,302,403,504,605,706,807,908,909,1010,1011,1212,1313 }    ;
/*
float latitude[2][13]={48.866667,43.7009358,43.2961743,43.6112422,43.6044622,44.841225,43.4832523,46.1591126,46.5802596,48.1113387,48.3905283,50.6365654,49.2577886,
48.2971626,47.7961287,47.3215806,47.2380222,45.7578137,45.4401467,45.1875602,44.9332277,43.9492493,43.8374249,44.5612032,43.1257311,38.1458689};  
float longitude[2][13]={2.333333,7.2683912,5.3699525,3.8767337,1.4442469,0.5800364,1.5592776,1.1520434,0.340196,1.6800198,4.4860088,3.0635282,4.031926,
4.0746257,3.570579,5.0414701,6.0243622,4.8320114,4.3873058,5.7357819,4.8920811,4.8059012,4.3600687,6.0820639,5.9304919,-7.3668309}   ;
*/
// for malaysie

float latitude[2][14]={3.1516964,48.866667,3.1516964,2.9140567,43.6112422,43.6044622,44.841225,43.4832523,46.1591126,46.5802596,48.1113387,48.3905283,50.6365654,49.2577886,
48.2971626,2.9140567,3.1516964,47.2380222,45.7578137,45.4401467,45.1875602,44.9332277,43.9492493,43.8374249,44.5612032,43.1257311,38.1458689,38.1458689};  
float longitude[2][14]={101.6942371,2.333333,101.6942371,101.6838531,3.8767337,1.4442469,0.5800364,1.5592776,1.1520434,0.340196,1.6800198,4.4860088,3.0635282,4.031926,
4.0746257,101.6838531,101.6942371,6.0243622,4.8320114,4.3873058,5.7357819,4.8920811,4.8059012,4.3600687,6.0820639,5.9304919,-7.3668309,-7.3668309}   ;//


//int Num_Table=0;     
int Temps=0;



char MY_NFC[4]={0x04,0xAB,0x65,0x6C};
char MY_KIFFY[4]={0x01,0x02,0x03,0x04};
int size = 0;


//DigitalOut Alarme (PC_13);// alarme LED output
//DigitalOut Alarme (LED1);// alarme LED output for disco//

//Servo Myservo(PA_7); //servomotor output for disco 

Servo Myservo(PA_7); //servomotor output for mdot
//TH02 MyTH02 (I2C_SDA,I2C_SCL,TH02_I2C_ADDR<<1);// connect hsensor on RX2 TX2
//GroveGPS MyGPS(PA_2,PA_3);for mdot

DigitalOut Alarme (PA_0);    // alarme LED output for mdot
//DigitalOut Alarme (LED2);       // alarme LED output for disco
//Servo Myservo(PA_7);            // servomotor output
//TH02 MyTH02 (I2C_SDA,I2C_SCL,TH02_I2C_ADDR<<1);// connect hsensor on RX2 TX2

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        5000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Dummy pin for dummy sensor
 */
#define PC_9                            0


/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

void servo(uint8_t uAngle)
{

}

/**
 * Entry point for application
 */
int main(void)
{
    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

/*************************************************************************************************************
 * Sends a message to the Network Server
 *************************************************************************************************************/
static void send_message()
{
    int iTime;
    uint16_t packet_len;
    int16_t retcode;
    float fTemp,fHumid;
    // Read Sensor temp and humidity values
   /* fTemp = myTH02.ReadTemperature();
    printf("Temp=%.2f\t",fTemp);
    fHumid = myTH02.ReadHumidity();
    printf("Humidity=%.2f\n",fHumid);
    // Payload is in Cayenne format*/


//printf ("\r\n Temps=%d, table=%d, Pression=%d, Sismique=%d", Temps,Num_Table+1,PRESSION[Temps][Num_Table],SISMIQUE[Temps][Num_Table] );
//printf ("\r\n Temps=%d, table=%d, Pression=%d, latitude=%f longitude=%f ", Temps,Num_Table,PRESSION[Temps][Num_Table],latitude[Temps%2][Num_Table],longitude[Temps%2][Num_Table] );
   printf ("\r\n Temps=%d, table=%d, Pression=%f, latitude=%f longitude=%f ", Temps,Num_Table,PRESSION[Temps][Num_Table],latitude[Temps%2][Num_Table],longitude[Temps%2][Num_Table] );
    Payload.reset();
    size = Payload.addDigitalInput(1,(int8_t) Num_Table);
    size =size+ Payload.addPression (2,PRESSION[Temps][Num_Table]);    // Add Temp in payload
    size =size+ Payload.addGPS(3,latitude[Temps%2][Num_Table],longitude[Temps%2][Num_Table],(float)45.2);
   //size =size+ Payload.addSismique(4,SISMIQUE[Temps][Num_Table]);    // Add Temp in payload

      if (Temps==2)
      Temps=0;
      else Temps++;

  

    

    // Send complete message with cayenne format
    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, Payload.getBuffer(), Payload.getSize(),
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(5000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    int num_port, iPosition=0,iIndex,iEtatAlarme;
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x", rx_buffer[i]);
    }

    // printf("\n test value=%d", port);
    // *****************************code todo here   ********************************************
    switch (port) {
        case 3: // control led
            printf("\n led=%x", (int)rx_buffer[0]);
            if ((rx_buffer[0]-0x30)==0)// ascii command for led 
          //  if (rx_buffer[0]==0)
                iEtatAlarme=0;
            else 
                iEtatAlarme=1;
            Alarme.write(iEtatAlarme);

            printf("\n alarme=%d",iEtatAlarme);
            break;
        case 4: // control servomotor
            for (iIndex=0; iIndex<retcode; iIndex++) {
                iPosition = iPosition*10 + (rx_buffer[iIndex]-0x30);   // convert receive string to angular position
            }

            printf("\n Servo position =%d",iPosition);
            Myservo.position ( iPosition-45 ); // set servo motor position from 0 to 180
            break;
        default:
            printf("\n port inconnu =%d",(int)port);
            break;
    }

//  ***************************** end code todo here   *****************************************
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    printf("\r\nEventCode = %d \r\n", event);
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
            printf("\r\n timeout in reception - Code = %d \r\n", event);
            break;
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
