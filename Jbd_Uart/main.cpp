HardwareSerial Sw(1);  // Use UART1

float *temp;

void setup() {
  Serial.begin(115200);
  Sw.begin(9600, SERIAL_8N1, 16, 17);
  Sw.flush();
}

void loop() {
  uint8_t command1[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
  uint8_t command2[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
  uint8_t command3[] = {0xDD, 0xA5, 0x05, 0x00, 0xFF, 0xFB, 0x77};
  int ResponseLength1 = 100;
  int ResponseLength2 = 100;
  int ResponseLength3 = 100;
  
  byte rxbuf1[ResponseLength1];
  byte rxbuf2[ResponseLength2];
  byte rxbuf3[ResponseLength3];

  sendCommand(command1, sizeof(command1));
  if (readResponse(ResponseLength1, rxbuf1) == 0) return;                   // Return if no valid response
  float chargePercentage = rxbuf1[23];  //Charging Calculation
  Serial.print("Charging Percentage   :  ");
  Serial.print(chargePercentage);
  Serial.println(" %");

  float voltage = float((rxbuf1[4] << 8) | rxbuf1[5]) / 100;  //Voltage Calculation
  Serial.print("Battery Voltage       :  ");
  Serial.print(voltage);
  Serial.println(" V");

  float current;
  uint16_t value = ((rxbuf1[6] << 8) | rxbuf1[7]);
  if (value & 0b1000000000000000)  // Test for positive / negative 1st bit is.
  {
   current = float(65536 - value)/(-100);       //divide by -100 for discharge current value negative
  }else {
    current = float(value)/100;
  }

  Serial.print("Current               :  ");
  Serial.print(current);
  Serial.println(" A");

  float power = voltage * current;    //Power Calculation
  Serial.print("Power                 :  ");
  Serial.print(power);
  Serial.println(" W");

  float cpcty = float(((rxbuf1[10] << 8) | rxbuf1[11]) * 10)/1000;  //Battery Capacity Calculation (Multiply by 10 for mAH)
  Serial.print("Total Battery Capacity:  ");
  Serial.print(cpcty);
  Serial.println(" AH");

  float rem_cpcty = float(((rxbuf1[8] << 8) | rxbuf1[9]) * 10)/1000;  //Battery Remaining Capacity Calculation (Multiply by 10 for mAH)
  Serial.print("Remaining Capacity    :  ");
  Serial.print(rem_cpcty);
  Serial.println(" AH");

  uint8_t probe = rxbuf1[26];  //Temperature probe no. Calculation
  Serial.print("Temperature probe No. :  ");
  Serial.println(probe);

  int NumOfTempSensor = probe;
  getTempSensorData(probe,rxbuf1);

   for(int i = 0; i<NumOfTempSensor; i++){
     Serial.print("Temperature Sensor ");
     Serial.print(i+1);
     Serial.print("  :  ");
     Serial.print(*(temp+i));
     Serial.println(" ° Celsius");
  }
  
  uint8_t date = ((rxbuf1[14] << 8) | rxbuf1[15]) & (0b00011111);          //Date Calculation
  uint8_t month = (((rxbuf1[14] << 8) | rxbuf1[15]) >> 5) & (0b00001111);  //Month Calculation
  uint8_t year = (((rxbuf1[14] << 8) | rxbuf1[15]) >> 9);                  //Year Calculation
  Serial.print("BMS Manufacturing Date:  ");
  Serial.print(date);
  Serial.print("-");
  Serial.print(month);
  Serial.print("-");
  Serial.println(2000 + year, DEC);

  float version = (rxbuf1[22] >> 4) + ((rxbuf1[22] & 0x0f) * 0.1f);   //BMS Version Calculation
  Serial.print("BMS Software Version  :  ");
  Serial.println(version);

  int cycle = ((rxbuf1[12] << 8) | rxbuf1[13]);                     //Cycle Calculation
  Serial.print("Cycle-Index           :  ");
  Serial.println(cycle);

  uint8_t MOS_State = rxbuf1[24];                                     //MOS state Calculation
  Serial.print("MOS State             :  ");
  Serial.println(MOS_State, BIN);

  uint8_t mos_array[2];
  // Extract each bit and store it in the array
    for (int i = 0; i < 2; i++) {
        mos_array[i] = (MOS_State >> i) & 1;     //Storing single bit of MOS State in array
    }

  if(mos_array[0] == 1){
    Serial.println("Charging MOS ON.");
  }else{
    Serial.println("Charging MOS OFF.");
  }
  
  if(mos_array[1] == 1){
    Serial.println("Discharging MOS ON.");
  }else{
    Serial.println("Discharging MOS OFF.");
  }

  uint16_t protectionState = ((rxbuf1[20] << 8) | rxbuf1[21]);        //Protection state Calculation
  Serial.print("Protection State      :  ");
  Serial.println(protectionState, BIN);

  uint8_t bit_array[13];
  // Extract each bit and store it in the array
    for (int i = 0; i < 13; i++) {
        bit_array[i] = (protectionState >> i) & 1;     //Storing single bit of Protection State in array
    }
  
  protectionStateLogs(bit_array);
  delay(100);

  sendCommand(command2, sizeof(command2));
  if (readResponse(ResponseLength2, rxbuf2) == 0) return;        // Return if no valid response
  int NumOfCells = rxbuf2[3] / 2;  //Total cells in battery
  Serial.print("Num of Cells          :  ");
  Serial.print(NumOfCells);
  Serial.println(" Cells");

  double CellSum = 0;       //Storing sum of all cell voltages
  double Celllow = 5000;    //5v
  double Cellhigh = 0;      //Storing High cell voltage
  double CellVoltage[50];  //Storing each cell voltage
  byte offset = 4;
  for (int i = 0; i < NumOfCells; i++) {
    CellVoltage[i] = double((rxbuf2[i * 2 + offset] << 8) | rxbuf2[i * 2 + 1 + offset]) / 1000;  //Each cell voltage
  
    CellSum += CellVoltage[i];
    Serial.print("Voltage of Cell " + (String)(i + 1) + "    :  ");
    Serial.print(CellVoltage[i]);
    Serial.println(" V");

    if (CellVoltage[i] > Cellhigh)               //High cell voltage
    {
      Cellhigh = CellVoltage[i];
    }
    if (CellVoltage[i] < Celllow)                 //Low cell voltage
    {
      Celllow = CellVoltage[i];
    }
  }

  double CellHigh = Cellhigh;
  Serial.print("High Cell Voltage     :  ");
  Serial.print(CellHigh);
  Serial.println(" V");
  
  double CellLow = Celllow;
  Serial.print("Low Cell Voltage      :  ");
  Serial.print(CellLow);
  Serial.println(" V");

  double CellDiff = CellHigh - CellLow;                //Cell voltage difference
  Serial.print("Cell Voltage Diff.    :  ");
  Serial.print(CellDiff);
  Serial.println(" V");

  double CellAvg = CellSum / NumOfCells;               //Average cell voltage
  Serial.print("Average Cell Voltage  :  ");
  Serial.print(CellAvg);
  Serial.println(" V");
  delay(100);

  sendCommand(command3, sizeof(command3));
  int index3 = readResponse(ResponseLength3, rxbuf3); // Capture the response length
  if (index3 == 0) {
    return; // Return if no valid response
  }
  // Perform further operations with the index value
  String message;
  Serial.print("Device Model          :  ");                // Extracting BMS device model by converting hex value to ascii value
  for (int i = 4; i < (index3 - 3); i++) {
    message += (char)rxbuf3[i];                             // Printing ASCII values
  }
  Serial.println(message);

  delay(3000);
}

void sendCommand(uint8_t* command, size_t length) {
  Sw.write(command, length);
  Serial.print("Command sent by PC    :  ");
  for (size_t i = 0; i < length; i++) {
    Serial.print(command[i], HEX);
    Serial.print("  ");
  }
  Sw.flush();
  Serial.println();
}

int readResponse(int responseLength, byte* rxbuf) {
  int index = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < 1000) { // Timeout of 1 second
    if (Sw.available()) {
      rxbuf[index++] = Sw.read();
      if (index >= responseLength) break;
    }
  }

  // Check if the response starts with 0xDD and ends with 0x77 and status byte is zero.
  if (index > 1 && rxbuf[0] == 0xDD && rxbuf[index - 1] == 0x77 && rxbuf[2] == 0x00) {
    Serial.print("Valid BMS response    :  ");
    for (int i = 0; i < index; i++) {
      Serial.print(rxbuf[i], HEX);
      Serial.print("  ");
    }
    Serial.println();
    return index; // Return the length of the response
  } else {
    Serial.println("No valid response.");
    return 0; // Return 0 if no valid response
  }
}

void getTempSensorData(int no_of_sensor, byte* rxbuf1){

  int i = 0;
  int startIndex = 27;
  //float temp1[no_of_sensor] ; 
  if(NULL != temp){
    free(temp);
  }
  temp = (float*)malloc(no_of_sensor * sizeof(float));
  for(i=0; i<no_of_sensor;i++){
     
     temp[i] = float(((rxbuf1[startIndex] << 8) | rxbuf1[startIndex+1]) - 2731) / 10;
     startIndex = startIndex+2;
  }
}


void protectionStateLogs(uint8_t* bit_array){
  if(bit_array[0] == 1){
    Serial.println("single cell overvoltage protection ON.");
  }

  if(bit_array[1] == 1){       
    Serial.println("single cell undervoltage protection ON.");
  }

  if(bit_array[2] == 1){
    Serial.println("whole pack overvoltage protection ON.");
  }

  if(bit_array[3] == 1){
    Serial.println("Whole pack undervoltage protection ON.");
  }

  if(bit_array[4] == 1){
    Serial.println("charging over-temperature protection ON.");
  }

  if(bit_array[5] == 1){
    Serial.println("charging low temperature protection ON.");
  }

  if(bit_array[6] == 1){
    Serial.println("Discharge over temperature protection ON.");
  }

  if(bit_array[7] == 1){
    Serial.println("discharge low temperature protection ON.");
  }

  if(bit_array[8] == 1){
    Serial.println("charging overcurrent protection ON.");
  }

  if(bit_array[9] == 1){
    Serial.println("Discharge overcurrent protection ON.");
  }

  if(bit_array[10] == 1){
    Serial.println("short circuit protection ON.");
  }

  if(bit_array[11] == 1){
    Serial.println("Front-end IC error detected.");
  }

  if(bit_array[12] == 1){
    Serial.println("software locked MOS.");
  }

}
