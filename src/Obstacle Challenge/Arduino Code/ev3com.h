// ---------------------- EV3 <-> Arduino (I2C Slave on Wire1) ----------------------
#define SLAVE_ADDRESS 0x04
#define EV3_SDA 12   // EV3 bus SDA -> D12
#define EV3_SCL 11   // EV3 bus SCL -> D11
TwoWire EV3Wire = TwoWire(1); // second I2C controller for EV3 link (slave)



// ---------------------- EV3 command state ----------------------
volatile int lastCmd = 0;
volatile bool cmdReady = false;



// ======================================================================
//                      EV3 I2C slave callbacks (Wire1)
// ======================================================================
void receiveData(int byteCount) {
  while (EV3Wire.available() > 0) {
    lastCmd = EV3Wire.read();
    cmdReady = true;
  }
  // Commands: 0x01 -> re-zero yaw
}

void sendData() {
  int16_t centi = (int16_t)roundf(accumYaw * 100.0f);  // deg * 100, signed
  uint8_t buf[5];
  // Little-endian: LSB first, then MSB
  buf[0] = (uint8_t)(centi & 0xFF);
  buf[1] = (uint8_t)((centi >> 8) & 0xFF);
  buf[2] = sig;
  //buf[3] = (uint8_t)(x_CO & 0xFF);
  //buf[4] = (uint8_t)((x_CO >> 8) & 0xFF);
  EV3Wire.write(buf, sizeof(buf));
}

