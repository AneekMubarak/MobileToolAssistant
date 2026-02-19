#include "dw1000.h"

DWM_Module dwm1 = { D8, D4 }; // CS=D8, RESET=D4

void setup() {
    Serial.begin(115200);
    pinMode(dwm1.cs_pin, OUTPUT);
    pinMode(dwm1.reset_pin, OUTPUT);
    digitalWrite(dwm1.cs_pin, HIGH);

    // to allow time to load the serial monitor
    int countdown=10;
    for (int i = countdown; i >= 0 ; i--){
        Serial.println(i);
        delay(1000);
    }
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);

    dwm_reset(&dwm1);
    
    delay(1000);

    dwm_configure(&dwm1);

}

int test_counter = 0;
uint8_t device_id[4] = {0};
uint8_t device_id_low[2] = {0};
int txfrs_error_count = 0;
uint8_t sys_cfg[4] = {0};

uint8_t clear_tx[5] = {0};

void loop(){

    test_counter++;
	dwm_read_reg(&dwm1, 0x00, device_id, 4);
	dwm_read_reg_sub(&dwm1, 0x40, 0x02, device_id_low, 2);
	// dwm_basic_transmit(&dwm1);
	// delay(10);

    uint8_t sys_event_status_reg[5] = {0};
    uint8_t rx_frame_info_reg[4] = {0};
    uint8_t rx_buffer[6] = {0}; // tflen = 6 but the top2 crc bits can be ignored here


    //Txn
	// dwm_read_reg(&dwm1, 0x0f, sys_event_status_reg, 5);

    // if(!(sys_event_status_reg[0]&0x80)){
    // 	txfrs_error_count++;
    // }

    // clear_tx[0] = 0xF0;
    // dwm_write_reg(&dwm1, 0x0F, clear_tx, 5);
    
    
    //Rxn
	dwm_read_reg(&dwm1, 0x0f, sys_event_status_reg, 5);

    uint8_t sys_ctrl[4] = {0};
    dwm_read_reg(&dwm1, 0x0D, sys_ctrl, 4);

    printRegHex(sys_event_status_reg,5,"System Even Status Reg Before Transmit");

    delay(5000);

    sys_ctrl[1] |= (1 << 0); // RXENAB
    dwm_write_reg(&dwm1, 0x0D, sys_ctrl, 4);


    for (int i = 0; i < 20 ;i++){
        dwm_read_reg(&dwm1, 0x0F, sys_event_status_reg, 5);
        printRegHex(sys_event_status_reg,5,"System Even Status Reg waiting....");
	    delay(100);

        if(sys_event_status_reg[1]&0x20){ // RXDFR is high
                dwm_read_reg(&dwm1, 0x10,rx_frame_info_reg,4);
                printRegHex(rx_frame_info_reg,4,"Rx Frame Info");
                dwm_read_reg(&dwm1, 0x11,rx_buffer,6);
                printRegHex(rx_buffer,4,"Rx Data"); //ignore CRC
        }

    }   
	// delay(500);


    //clear tx flags
    // dwm_write_reg(&dwm1, 0x0F, clear_tx, 5);
	// dwm_read_reg(&dwm1, 0x0f, sys_event_status_reg, 5);

    // Serial.print("Count: ");
    // Serial.println(test_counter);
    printRegHex(device_id,4,"Device ID");
    printRegHex(device_id_low,2,"Device ID Low");
    // Serial.print("Tx Error Count: ");
    // Serial.println(txfrs_error_count);
    // printRegHex(sys_event_status_reg,5,"System Even Status Reg");






}
