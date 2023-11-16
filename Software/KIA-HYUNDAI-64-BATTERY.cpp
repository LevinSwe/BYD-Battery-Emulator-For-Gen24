#include "KIA-HYUNDAI-64-BATTERY.h"
#include "ESP32CAN.h"
#include "CAN_config.h"

/* Do not change code below unless you are sure what you are doing */
static unsigned long previousMillis10 = 0; // will store last time a 10ms CAN Message was send
static unsigned long previousMillis100 = 0; // will store last time a 100ms CAN Message was send
static const int interval10 = 10; // interval (ms) at which send CAN Messages
static const int interval100 = 100; // interval (ms) at which send CAN Messages
static uint8_t CANstillAlive = 12; //counter for checking if CAN is still alive

CAN_frame_t KIA64_7E4_id1 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7E4,.data = {0x03, 0x22, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00}}; //Poll PID 03 22 01 01
CAN_frame_t KIA64_7E4_id2 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7E4,.data = {0x03, 0x22, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00}}; //Poll PID 03 22 01 02
CAN_frame_t KIA64_7E4_id3 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7E4,.data = {0x03, 0x22, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00}}; //Poll PID 03 22 01 03
CAN_frame_t KIA64_7E4_id4 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7E4,.data = {0x03, 0x22, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00}}; //Poll PID 03 22 01 04
CAN_frame_t KIA64_7E4_id5 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7E4,.data = {0x03, 0x22, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00}}; //Poll PID 03 22 01 05
CAN_frame_t KIA64_7E4_id6 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7E4,.data = {0x03, 0x22, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00}}; //Poll PID 03 22 01 06

CAN_frame_t KIA64_7E4_ack = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7E4,.data = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; //Ack frame, correct PID is returned

//folowing frames is for test to fool battery is in a running car.. bot not comlete yet.. 
//elco
CAN_frame_t KIA64_7E4_con1o = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7E4,.data = {0x04, 0x2F, 0xF0, 0x38, 0x00, 0x00, 0x00, 0x00}}; //debug contactors
CAN_frame_t KIA64_7E4_con1c = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7E4,.data = {0x04, 0x2F, 0xF0, 0x38, 0x03, 0x00, 0x00, 0x00}}; //debug contactors

CAN_frame_t KIA64_7E4_7ea = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x7EA,.data = {0x10, 0x18, 0x61, 0x01, 0xff, 0xf8, 0x00, 0x00}}; //7ea 10 18 61 01 ff f8 00 00

CAN_frame_t KIA64_200_1 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x200,.data = {0x17, 0x00, 0x5B, 0x10, 0x00, 0xFB, 0xD0, 0x00}}; //17	00	5B	10	00	FB	D0	00
CAN_frame_t KIA64_200_2 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x200,.data = {0x17, 0x00, 0x5B, 0x10, 0x00, 0x3B, 0xD0, 0x00}}; //17	00	5B	10	00	3B	D0	00
CAN_frame_t KIA64_200_3 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x200,.data = {0x17, 0x00, 0x5B, 0x10, 0x00, 0x7B, 0xD0, 0x00}}; //17	00	5B	10	00	7B	D0	00
CAN_frame_t KIA64_200_4 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x200,.data = {0x17, 0x00, 0x5B, 0x10, 0x00, 0xBB, 0xD0, 0x00}}; //17	00	5B	10	00	BB	D0	00

//No differens if included or not
CAN_frame_t KIA64_201_1 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x201,.data = {0xD1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD1}}; 
CAN_frame_t KIA64_201_2 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x201,.data = {0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11}}; 
CAN_frame_t KIA64_201_3 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x201,.data = {0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x51}}; 
CAN_frame_t KIA64_201_4 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x201,.data = {0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x91}}; 

CAN_frame_t KIA64_291_1 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x291,.data = {0x00, 0x00, 0x00, 0x00, 0xE1, 0xFC, 0xFC, 0x0A}}; //00	00	00	00	FF	FC	FC	30    00	00	00	00	BE	00	00	0A

CAN_frame_t KIA64_2A2_1 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x2A2,.data = {0x15, 0x00, 0x00, 0x08, 0x06, 0x00, 0x00, 0x3E}}; //15	00	00	08	06	00	00	3E
CAN_frame_t KIA64_2A2_2 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x2A2,.data = {0x14, 0x00, 0x00, 0x0A, 0x06, 0x00, 0x00, 0x3E}}; //14	00	00	0A	06	00	00	3E

//2B0 clears U0110 Comm error drive motor control module sent in 10ms
CAN_frame_t KIA64_2B0_1 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x0F}}; //90	01	00	07	0F
CAN_frame_t KIA64_2B0_2 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0xF0}}; //90	01	00	07	F0
CAN_frame_t KIA64_2B0_3 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0xE1}}; //90	01	00	07	E1
CAN_frame_t KIA64_2B0_4 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0xD2}}; //90	01	00	07	D2
CAN_frame_t KIA64_2B0_5 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0xC3}}; //90	01	00	07	C3
CAN_frame_t KIA64_2B0_6 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0xB4}}; //90	01	00	07	B4
CAN_frame_t KIA64_2B0_7 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0xA5}}; //90	01	00	07	A5
CAN_frame_t KIA64_2B0_8 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x96}}; //90	01	00	07	96
CAN_frame_t KIA64_2B0_9 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x87}}; //90	01	00	07	87
CAN_frame_t KIA64_2B0_10 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x78}}; //90	01	00	07	78
CAN_frame_t KIA64_2B0_11 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x69}}; //90	01	00	07	69
CAN_frame_t KIA64_2B0_12 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x5A}}; //90	01	00	07	5A
CAN_frame_t KIA64_2B0_13 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x4B}}; //90	01	00	07	4B
CAN_frame_t KIA64_2B0_14 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x3C}}; //90	01	00	07	3C
CAN_frame_t KIA64_2B0_15 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x2D}}; //90	01	00	07	2D
CAN_frame_t KIA64_2B0_16 = {.FIR = {.B = {.DLC = 5,.FF = CAN_frame_std,}},.MsgID = 0x2B0,.data = {0x90, 0x01, 0x00, 0x07, 0x1E}}; //90	01	00	07	1E

CAN_frame_t KIA64_523_0 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x523,.data = {0x00, 0x38, 0x28, 0x28, 0x28, 0x28, 0x00, 0x01}}; //00	38	28	28	28	28	00	01 //Sets BMSigniton to OFF
CAN_frame_t KIA64_523_1 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x523,.data = {0x0C, 0x38, 0x4C, 0x4C, 0x45, 0x49, 0xA5, 0x00}}; //0C	38	4C	4C	45	49	(A5)	00 //Sets BMSigniton to ON

CAN_frame_t KIA64_524_1 = {.FIR = {.B = {.DLC = 8,.FF = CAN_frame_std,}},.MsgID = 0x524,.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; //vcu volts byte 0&1


#define LB_MAX_SOC 1000  //BMS never goes over this value. We use this info to rescale SOC% sent to Inverter
#define LB_MIN_SOC 0   //BMS never goes below this value. We use this info to rescale SOC% sent to Inverter

static int16_t SOC_BMS = 0;
static int16_t SOC_Display = 0;
static int16_t battSOH = 0;
static int16_t leakSens = 0;
static int16_t aux12volt = 0;
static int16_t CellVoltMax = 0;
static int16_t CellVmaxNo = 0;
static int16_t CellVoltMin = 0;
static int16_t CellVminNo = 0;
static int16_t allowedDischargePower = 0;
static int16_t allowedChargePower = 0;
static int16_t battVolts = 0;
static int16_t battAmps = 0;
static int8_t tempMax = 0;
static int8_t tempMin = 0;
static int8_t temp1 = 0;
static int8_t temp2 = 0;
static int8_t temp3 = 0;
static int8_t temp4 = 0;
static int8_t tempinlet = 0;
static uint8_t battMagntMode = 0;
static uint8_t BMS_ign = 0;
static int16_t poll_data_pid = 0;
static int16_t pollA2A = 0;
static int16_t poll200 = 0;
static int16_t poll201 = 0;
static int16_t poll2B0 = 0;
static int16_t poll523 = 0;
static int8_t heatertemp = 0;
static int airbag = 0;
static int battrly = 0;
static int8_t PRAtemp = 0;
static int16_t Inv_VoltsTmp = 0;
static int16_t Inv_Volts = 0;
static uint8_t mode491 = 0;
static int8_t radiatorTemp = 0;
static int8_t chargeTemp = 0;
static int16_t frame524volts = 0;
static int8_t fanMode = 0;
static int8_t fanSpeed = 0;

void update_values_kiaHyundai_64_battery()
{ //This function maps all the values fetched via CAN to the correct parameters used for modbus
    bms_status = ACTIVE; //Startout in active mode

    SOC = (SOC_Display * 10); //Increase decimals from 50.0% -> 50.00%
    
    StateOfHealth = (battSOH * 10); //Increase decimals from 100.0% -> 100.00%

    battery_voltage = (uint16_t)battVolts; //value is *10

    battery_current = (int16_t)battAmps; //value is *10 //Todo, the signed part breaks something here for sure

    capacity_Wh = BATTERY_WH_MAX;

    remaining_capacity_Wh = ((SOC/10000)*BATTERY_WH_MAX);

    max_target_discharge_power = (uint16_t)allowedDischargePower * 10; //From kW*100 to Watts

    max_target_charge_power = (uint16_t)allowedChargePower * 10; //From kW*100 to Watts

    stat_batt_power = ((uint16_t)battVolts * (int16_t)battAmps) / 100; //Power in watts, Negative = charging batt. //Todo, the signed part will break something here

    temperature_min = ((int8_t)tempMin * 10); //Increase decimals, 17C -> 17.0C
    
    temperature_max = ((int8_t)tempMax * 10); //Increase decimals, 18C -> 18.0C
    
    cell_max_voltage = CellVoltMax; //in millivolt
    
    cell_min_voltage = CellVoltMin; //in millivolt
	
	/* Check if the BMS is still sending CAN messages. If we go 60s without messages we raise an error*/
	if(!CANstillAlive)
	{
	bms_status = FAULT;
	Serial.println("No CAN communication detected for 60s. Shutting down battery control.");
	}
	else
	{
	CANstillAlive--;
	}

  if(printValues)  
  {  //values heading towards the inverter
    Serial.println(); //sepatator
    Serial.println("Values from battery: ");
    Serial.print("SOC BMS: ");
    Serial.print((int16_t)SOC_BMS / 10.0, 1);
    Serial.print("%  |  SOC Display: ");
    Serial.print((int16_t)SOC_Display / 10.0, 1);
    Serial.print("%  |  SOH ");
    Serial.print((int16_t)battSOH / 10.0, 1);
    Serial.println("%");
    Serial.print((int16_t)battAmps / 10.0, 1);
    Serial.print(" Amps  |  ");
    Serial.print((int16_t)battVolts / 10.0, 1);
    Serial.print(" Volts  |  ");
    Serial.print((int16_t)stat_batt_power);
    Serial.println(" Watts");
    Serial.print("Allowed Charge ");
    Serial.print((uint16_t)allowedChargePower * 10);
    Serial.print(" W  |  Allowed Discharge ");
    Serial.print((uint16_t)allowedDischargePower * 10);
    Serial.println(" W");
    Serial.print("MaxCellVolt ");
    Serial.print(CellVoltMax);
    Serial.print(" mV  No  ");
    Serial.print(CellVmaxNo);
    Serial.print("  |  MinCellVolt ");
    Serial.print(CellVoltMin);
    Serial.print(" mV  No  ");
    Serial.println(CellVminNo);
    Serial.print("BattHi ");
    Serial.print((int8_t)tempMax);
    Serial.print("°C  BattLo ");
    Serial.print((int8_t)tempMin);
    Serial.print("°C  Temp1 ");
    Serial.print((int8_t)temp1);
    Serial.print("°C  Temp2 ");
    Serial.print((int8_t)temp2);
    Serial.print("°C  Temp3 ");
    Serial.print((int8_t)temp3);
    Serial.print("°C  Temp4 ");
    Serial.print((int8_t)temp4);
    Serial.print("°C  WaterInlet ");
    Serial.print((int8_t)tempinlet);
    Serial.print("°C  PowerRelay ");
    Serial.print((int8_t)PRAtemp * 2);
    Serial.println("°C");
    Serial.print("Aux12volt: ");
    Serial.print((int16_t)aux12volt / 10.0, 1);
    Serial.print("V  |  ");
    Serial.print("LeakSensor: ");
    Serial.println(leakSens);
    Serial.print("BmsMagementMode ");
    Serial.print((uint8_t)battMagntMode, BIN);
    Serial.print("  |  IGN ");
    Serial.print((uint8_t)BMS_ign, BIN); //bit 2=IGN
    if (bitRead((uint8_t)BMS_ign, 2) == 1){
   Serial.print(" BmsIgitionON");
  }
   else {
     Serial.print(" IGN 0");
   }
    Serial.print("  |  mode491 ");
    Serial.println((uint8_t)mode491, HEX);
     Serial.print("HeaterTemp ");
    Serial.print((int8_t)heatertemp);
    Serial.print("  |  RadTemp ");
    Serial.print((int8_t)radiatorTemp);
    Serial.print("  |  ChargeTemp ");
    Serial.print((int8_t)chargeTemp);
    Serial.print("  |  airbagDut% ");
    Serial.print((uint8_t)airbag);
    Serial.print("  |  RLY ");
    Serial.print((uint8_t)battrly, BIN); //Bit0 - BMS relay ,Bit 5 - normal charge, Bit 6 rapid charge, Bit 7 charging
     
  if (bitRead((uint8_t)battrly, 0) == 1){
   Serial.println(" RLY bit0 Är ETT");
  }
   else {
     Serial.print(" RLY bit0 är noll");
   }

    Serial.print("  |  Inverter ");
    Serial.print((int16_t)Inv_Volts);
    Serial.println(" Volts");
  }
}

void receive_can_kiaHyundai_64_battery(CAN_frame_t rx_frame)
{
	CANstillAlive = 12;
	switch (rx_frame.MsgID) //3F6 -> 5D8 - Data sent from battery every 100ms, littleEndian
	{
	  case 0x3F6:
	break;
  	case 0x491:
    mode491 = rx_frame.data.u8[0];
	break;
  	case 0x493:
	break;
  	case 0x497:
	break;
  	case 0x498:
	break;
  	case 0x4DD:
	break;
  	case 0x4DE:
    radiatorTemp = rx_frame.data.u8[4]; //-15
	break;
  	case 0x4E2:
	break;
  	case 0x542:
    SOC_Display = rx_frame.data.u8[0] * 5; //100% = 200 ( 200 * 5 = 1000 )
	break;
  	case 0x594:
    SOC_BMS = rx_frame.data.u8[5] * 5; //100% = 200 ( 200 * 5 = 1000 )
	break;
  	case 0x595:
    battVolts = (rx_frame.data.u8[7] << 8) + rx_frame.data.u8[6];
    battAmps = (rx_frame.data.u8[5] << 8) + rx_frame.data.u8[4];
    frame524volts = (int16_t)battVolts / 10;
    KIA64_524_1.data = {(uint8_t)frame524volts, (uint8_t)(frame524volts >> 8), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //vcu volts to bms
	break;
  	case 0x596:
    aux12volt = rx_frame.data.u8[1]; //12v Battery Volts
    tempMin = rx_frame.data.u8[6]; //Lowest temp in battery
    tempMax = rx_frame.data.u8[7]; //Highest temp in battery
	break;
  	case 0x597:
	break;
  	case 0x598:
    chargeTemp = rx_frame.data.u8[7]; //ChargeportTemperature
	break;
  	case 0x599:
	break;
  	case 0x59C:
	break;
  	case 0x59E:
	break;
  	case 0x5A3:
	break;
  	case 0x5D5:
    leakSens = rx_frame.data.u8[3];  //Water sensor inside pack, value 164 is no water --> 0 is short
    PRAtemp = rx_frame.data.u8[7]; //PowerRelayTemp
	break;
  	case 0x5D6:
	break;
  	case 0x5D7:
	break;
  	case 0x5D8:
    //PRAtemp = rx_frame.data.u8[2]; //PowerRelayTemp same as in 5D5

    //ESP32Can.CANWriteFrame(&KIA64_524_1);
    //ESP32Can.CANWriteFrame(&KIA64_291_1); //motor speed to 32767 if 100ms sent

    if (poll_data_pid >= 10){
      poll_data_pid = 0;
      }
    poll_data_pid++;
    if (poll_data_pid == 1){
      ESP32Can.CANWriteFrame(&KIA64_7E4_id1);
      }
    else if (poll_data_pid == 2){
      ESP32Can.CANWriteFrame(&KIA64_7E4_id2);
      }
    else if (poll_data_pid == 5){
      ESP32Can.CANWriteFrame(&KIA64_7E4_id5);
      }
    else if (poll_data_pid == 6){
      ESP32Can.CANWriteFrame(&KIA64_7E4_id6);
      }
    else if (poll_data_pid == 8){
      ESP32Can.CANWriteFrame(&KIA64_200_1);
      }
      else if (poll_data_pid == 9){
      }
    
	break;
  	case 0x7EC:  //Data From polled PID group, BigEndian
      switch (rx_frame.data.u8[0])
      {
        case 0x10:  //"PID Header"
         if (rx_frame.data.u8[4] == poll_data_pid){
           ESP32Can.CANWriteFrame(&KIA64_7E4_ack);  //Send ack to BMS if the same frame is sent as polled
         }
      break;
        case 0x21:  //First frame in PID group
         if (poll_data_pid == 1){
           allowedChargePower = ((rx_frame.data.u8[3] << 8) + rx_frame.data.u8[4]);
           allowedDischargePower = ((rx_frame.data.u8[5] << 8) + rx_frame.data.u8[6]);
           battrly = rx_frame.data.u8[7];
         }
      break;
        case 0x22:  //Second datarow in PID group
         if (poll_data_pid == 1){
           //battAmps = (rx_frame.data.u8[1] << 8) + rx_frame.data.u8[2]; //moved to passive data
           //battVolts = (rx_frame.data.u8[3] << 8) + rx_frame.data.u8[4]; //moved to passive data
           //tempMax = rx_frame.data.u8[5]; //moved to passive data
           //tempMin = rx_frame.data.u8[6]; //moved to passive data
           temp1 = rx_frame.data.u8[7];
         }
         if (poll_data_pid == 6){
           battMagntMode = rx_frame.data.u8[5];
         }
      break;
        case 0x23:  //Third datarow in PID group
         if (poll_data_pid == 1){
           temp2 = rx_frame.data.u8[1];
           temp3 = rx_frame.data.u8[2];
           temp4 = rx_frame.data.u8[3];
           tempinlet = rx_frame.data.u8[6];
           CellVoltMax = (rx_frame.data.u8[7] * 20);  //(volts *50) *20 =mV
         }
         if (poll_data_pid == 5){
           airbag = rx_frame.data.u8[6];
           heatertemp = rx_frame.data.u8[7];
         }
      break;
        case 0x24:  //Fourth datarow in PID group
         if (poll_data_pid == 1){
           CellVmaxNo = rx_frame.data.u8[1];
           CellVminNo = rx_frame.data.u8[3];
           CellVoltMin = (rx_frame.data.u8[2] * 20);  //(volts *50) *20 =mV
           fanMode = rx_frame.data.u8[4];
           fanSpeed = rx_frame.data.u8[5];
         }
         else if (poll_data_pid == 5){
           battSOH = ((rx_frame.data.u8[2] << 8) + rx_frame.data.u8[3]);
         }
      break;
        case 0x25:  //Fifth datarow in PID group
      break;
        case 0x26:  //Sixth datarow in PID group
      break;
        case 0x27:  //Seventh datarow in PID group
         if (poll_data_pid == 1){
         BMS_ign = rx_frame.data.u8[6];
         Inv_VoltsTmp = rx_frame.data.u8[7];
         }
      break;
        case 0x28:  //Eighth datarow in PID group
         if (poll_data_pid == 1){
         Inv_Volts = (Inv_VoltsTmp << 8) + rx_frame.data.u8[1];
         }
      break;
      }
  default:
	break;
  }    
}
void send_can_kiaHyundai_64_battery()
{
  unsigned long currentMillis = millis();
	// Send 100ms CAN Message
	if (currentMillis - previousMillis100 >= interval100)
	{
		previousMillis100 = currentMillis;
    //every 100ms update frame with inverterVolts from battery volts readout
    //ESP32Can.CANWriteFrame(&KIA64_523_1);
	}
  //Send 10ms message
	if (currentMillis - previousMillis10 >= interval10)
	{ 
		previousMillis10 = currentMillis;
    ESP32Can.CANWriteFrame(&KIA64_523_1);
    ESP32Can.CANWriteFrame(&KIA64_524_1);
    ESP32Can.CANWriteFrame(&KIA64_291_1); // needs to be sent in 10ms?

        //CAN FRAME 200
    poll200++;
    if (poll200 >= 5) { poll200 = 1; }
    if (poll200 == 1){
      ESP32Can.CANWriteFrame(&KIA64_200_1);
    }
    else if (poll200 == 2 ){
      ESP32Can.CANWriteFrame(&KIA64_200_2);
    }
    else if (poll200 == 3 ){
      ESP32Can.CANWriteFrame(&KIA64_200_3);
    }
    else if (poll200 == 4 ){
      ESP32Can.CANWriteFrame(&KIA64_200_4);
    }

    //CAN FRAME 201
    poll201++;
    if (poll201 >= 5) { poll201 = 1; }
    if (poll201 == 1){
      ESP32Can.CANWriteFrame(&KIA64_201_1);
    }
    else if (poll201 == 2 ){
      ESP32Can.CANWriteFrame(&KIA64_201_2);
    }
    else if (poll201 == 3 ){
      ESP32Can.CANWriteFrame(&KIA64_201_3);
    }
    else if (poll201 == 4 ){
      ESP32Can.CANWriteFrame(&KIA64_201_4);
    }

    //CAN FRAME 2A2
    if (pollA2A >= 3) { pollA2A = 1; }
    if (pollA2A == 1){
      ESP32Can.CANWriteFrame(&KIA64_2A2_1);
    }
    else if (pollA2A == 2 ){
      ESP32Can.CANWriteFrame(&KIA64_2A2_2);
    }



        //CAN FRAME 2B0
    poll2B0++;
    if (poll2B0 >= 17) { poll2B0 = 1; }
    if (poll2B0 == 1){
      ESP32Can.CANWriteFrame(&KIA64_2B0_1);
    }
    else if (poll2B0 == 2 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_2);
    }
    else if (poll2B0 == 3 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_3);
    }
    else if (poll2B0 == 4 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_4);
    }
    else if (poll2B0 == 5 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_5);
    }
    else if (poll2B0 == 6 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_6);
    }
    else if (poll2B0 == 7 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_7);
    }
    else if (poll2B0 == 8 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_8);
    }
    else if (poll2B0 == 9 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_9);
    }
    else if (poll2B0 == 10 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_10);
    }
    else if (poll2B0 == 11 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_11);
    }
    else if (poll2B0 == 12 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_12);
    }
    else if (poll2B0 == 13 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_13);
    }
    else if (poll2B0 == 14 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_14);
    }
    else if (poll2B0 == 15 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_15);
    }
    else if (poll2B0 == 16 ){
      ESP32Can.CANWriteFrame(&KIA64_2B0_16);
    }
	}
}
