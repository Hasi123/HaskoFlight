#include <Arduino.h>
#include "MPU6050.h"
#include "inv_dmp_uncompress.h"

MPU6050 mpu;

//constructor
MPU6050::MPU6050(uint8_t Addr) {
  m_mpuAddr = Addr;
}

//Write to the DMP memory
int8_t MPU6050::m_mpu_write_mem(uint16_t mem_addr, uint16_t length, uint8_t *data) {

  if (!intTW.writeWord(m_mpuAddr, MPU6050_RA_BANK_SEL, mem_addr))
    return -1;
  if (!intTW.writeBytes(m_mpuAddr, MPU6050_RA_MEM_R_W, length, data))
    return -2;
  return 0;
}

//Read from the DMP memory
int8_t MPU6050::m_mpu_read_mem(uint16_t mem_addr, uint16_t length, uint8_t *data) {

  if (!intTW.writeWord(m_mpuAddr, MPU6050_RA_BANK_SEL, mem_addr))
    return -1;
  if (!intTW.readBytes(m_mpuAddr, MPU6050_RA_MEM_R_W, length, data))
    return -1;
  return 0;
}

//Load and verify DMP image
int8_t MPU6050::m_load_dmp() {  //using compressed DMP firmware
  //#define VERIFY_DMP //should we verify loading the DMP?

  uint16_t ii, this_write;
  uint8_t progBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
#ifdef VERIFY_DMP
  uint8_t cur[MPU6050_DMP_MEMORY_CHUNK_SIZE];
#endif

  /* start loading */
  inv_dmp_uncompress_reset();

  for (ii = 0; ii < UNCOMPRESSED_DMP_CODE_SIZE; ii += this_write) {
    this_write = min(MPU6050_DMP_MEMORY_CHUNK_SIZE, UNCOMPRESSED_DMP_CODE_SIZE - ii);

    /* decompress chunk */
    for (uint16_t progIndex = 0; progIndex < this_write; progIndex++)
      progBuffer[progIndex] = inv_dmp_uncompress();

    //write
    if (m_mpu_write_mem(ii, this_write, progBuffer))
      return -1;
#ifdef VERIFY_DMP  //check
    if (m_mpu_read_mem(ii, this_write, cur))
      return -1;
    if (memcmp(progBuffer, cur, this_write))
      return -2;
#endif
  }

  /* Set program start address. */
  if (!intTW.writeWord(m_mpuAddr, MPU6050_RA_DMP_CFG_1, MPU6050_DMP_START_ADDRESS))
    return -1;

  return 0;
}

//Init the sensor, load calibration data and dmp
void MPU6050::init(void) {
  intTW.begin();
  intTW.writeByte(m_mpuAddr, MPU6050_RA_PWR_MGMT_1, _BV(MPU6050_PWR1_DEVICE_RESET_BIT));  //reset
  delay(100);
  intTW.writeByte(m_mpuAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);         //wake up and set clock to gyro X (recomended by datasheet)
  intTW.writeByte(m_mpuAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000 << 3);      //Gyro range
  intTW.writeByte(m_mpuAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS << 3);         //Accel range
  intTW.writeByte(m_mpuAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);                  //DLPF
  intTW.writeByte(m_mpuAddr, MPU6050_RA_SMPLRT_DIV, 1000 / MPU6050_SAMPLE_RATE - 1);  //sample rate divider
  //writeByte(m_mpuAddr, MPU6050_RA_INT_ENABLE, 0); //disable interrupts, already 0 by default
  intTW.writeByte(m_mpuAddr, MPU6050_RA_INT_PIN_CFG, _BV(MPU6050_INTCFG_LATCH_INT_EN_BIT) | _BV(MPU6050_INTCFG_INT_RD_CLEAR_BIT));  //setup interrupt pin

  m_load_dmp();

  //writeByte(m_mpuAddr, MPU6050_RA_FIFO_EN, 0); //disable FIFO, already 0 by default
  //intTW.writeByte(m_mpuAddr, MPU6050_RA_USER_CTRL, _BV(MPU6050_USERCTRL_DMP_RESET_BIT) | _BV(MPU6050_USERCTRL_FIFO_RESET_BIT));  //reset FIFO and DMP
  //delay(50);
  intTW.writeByte(m_mpuAddr, MPU6050_RA_USER_CTRL, _BV(MPU6050_USERCTRL_DMP_EN_BIT) | _BV(MPU6050_USERCTRL_FIFO_EN_BIT));  //enable FIFO and DMP
  attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), MPU6050::intHandler, RISING);
  intTW.writeByte(m_mpuAddr, MPU6050_RA_INT_ENABLE, _BV(MPU6050_INTERRUPT_DMP_INT_BIT));  //enable hardware DMP interrupt
}

bool MPU6050::newData(void) {
  if (m_newData) {
    m_newData = false;
    return true;
  }
  return false;
}

int32_t MPU6050::getQuat(const uint8_t index) {
  //return ((int32_t)m_fifo_data[index * 4] << 24) | ((int32_t)m_fifo_data[index * 4 + 1] << 16) | ((int32_t)m_fifo_data[index * 4 + 2] << 8) | m_fifo_data[index * 4 + 3];
  int32_t tmp = *(int32_t *)&m_fifo_data[index * 4];
  return __builtin_bswap32(tmp);
}

int16_t MPU6050::getAccel(const uint8_t index) {
  //return ((int16_t)m_fifo_data[index * 2 + 16] << 8) | m_fifo_data[index * 2 + 17];
  int16_t tmp = *(int16_t *)&m_fifo_data[index * 2 + 16];
  return __builtin_bswap16(tmp);
}

int16_t MPU6050::getGyro(const uint8_t index) {
  //return ((int16_t)m_fifo_data[index * 2 + 22] << 8) | m_fifo_data[index * 2 + 23];
  int16_t tmp = *(int16_t *)&m_fifo_data[index * 2 + 22];
  return __builtin_bswap16(tmp);
}

void MPU6050::intHandler(void) {
  //mpu.mpuGetFIFOcount();
  mpu.m_mpuGetFIFO();
}

/*
void MPU6050::mpuGetFIFOcount(void) {
  intTW.setRxBuffer((uint8_t*)fifoCount);
  intTW.start((uint8_t*)mpuReadFifoCount, sizeof(mpuReadFifoCount), INTTW_USE_PROGMEM | INTTW_KEEP_BUS, mpuCheckFIFOcount);
}

void MPU6050::mpuCheckFIFOcount(void) {
  uint16_t fifo_count = ((uint16_t)fifoCount[0] << 8) | (uint16_t)fifoCount[1];
  if (fifo_count != MPU6050_FIFO_LENGTH) {  //reset FIFO if not exactly 1 packet
    intTW.stop();
    newData = -1;
  } else {
    m_mpuGetFIFO();
  }
}
*/

void MPU6050::m_mpuGetFIFO(void) {
  intTW.setRxBuffer(m_fifo_data);
  intTW.start((uint8_t *)mpuReadFifo, sizeof(mpuReadFifo), INTTW_USE_PROGMEM, dataReady);
}

void MPU6050::dataReady(void) {
  mpu.m_newData = 1;
}
