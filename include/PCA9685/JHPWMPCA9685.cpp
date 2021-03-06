#include <JHPWMPCA9685.h>
#include <math.h>

PCA9685::PCA9685(int address) {
    kI2CBus = 1 ;           // Default I2C bus for Jetson TK1
    kI2CAddress = address ; // Defaults to 0x40 for PCA9685 ; jumper settable
    error = 0 ;
}

PCA9685::~PCA9685() {
    closePCA9685() ;
}

/**
 * PCA9685 보드와 i2c 통신을 위하여 파일들을 오픈하는 함수
 * Return value :
 *      파일이 열리지 않는 경우 ( 대부분 sudo 문제) False
 **/
        
bool PCA9685::openPCA9685()
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
    if (kI2CFileDescriptor < 0) {
        // Could not open the file
       error = errno ;
       return false ;
    }
    if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kI2CAddress) < 0) {
        // Could not open the device on the bus
        error = errno ;
        return false ;
    }
    return true ;
}

/**
 * PCA9685 포인터 닫는 함수. 
 * 설명에는 단순히 닫는 것보다는 Errno 분석 후 닫으라 써져 있음. 
 */
void PCA9685::closePCA9685()
{
    if (kI2CFileDescriptor > 0) {
        close(kI2CFileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        kI2CFileDescriptor = -1 ;
    }
}

/**
 * 현재는 해석 불가
 */
void PCA9685::reset () {
    writeByte(PCA9685_MODE1, PCA9685_ALLCALL );
    writeByte(PCA9685_MODE2, PCA9685_OUTDRV) ;
    // Wait for oscillator to stabilize
    usleep(5000) ;
}

/**
 * PWM의 frequency 설정, Frequency 는 40~1000 범위를 가짐
 * @Parms :
 *      frequency : 사용자 임의의 frequency
 */
void PCA9685::setPWMFrequency ( float frequency ) {
    printf("Setting PCA9685 PWM frequency to %f Hz\n",frequency) ;
    float rangedFrequency = fmin(fmax(frequency,40),1000) ;
    rangedFrequency *= 0.9 ;   // Correct for overshoot on PCA9685; The method
                               // described in the datasheet overshoots by 1/0.9
    int prescale = (int)(25000000.0f / (4096 * rangedFrequency) - 0.5f) ;
    // For debugging
    // printf("PCA9685 Prescale: 0x%02X\n",prescale) ;
    int oldMode = readByte(PCA9685_MODE1) ;
    int newMode = ( oldMode & 0x7F ) | PCA9685_SLEEP ;
    writeByte(PCA9685_MODE1, newMode) ;
    writeByte(PCA9685_PRE_SCALE, prescale) ;
    writeByte(PCA9685_MODE1, oldMode) ;
    // Wait for oscillator to stabilize
    usleep(5000) ;
    writeByte(PCA9685_MODE1, oldMode | PCA9685_RESTART) ;
}

/**
 * PCA9685 PWM 설정하는 함수
 * 채널은 4Byte 로 구성되어있고, 16개의 채널(0~15)을 가지고 있다.
 * @params
 *      channel : 채널 번호
 *      onValue : 
 *      offValue :
 */
void PCA9685::setPWM ( int channel, int onValue, int offValue) {
    writeByte(PCA9685_LED0_ON_L+4*channel, onValue & 0xFF) ;
    writeByte(PCA9685_LED0_ON_H+4*channel, onValue >> 8) ;
    writeByte(PCA9685_LED0_OFF_L+4*channel, offValue & 0xFF) ;
    writeByte(PCA9685_LED0_OFF_H+4*channel, offValue >> 8) ;
}

/**
 * PCA9685 전체 PWM 설정하는 함수
 * @params
 *      onValue :
 *      offValue :
 */
void PCA9685::setAllPWM (int onValue, int offValue) {
    writeByte(PCA9685_ALL_LED_ON_L, onValue & 0xFF) ;
    writeByte(PCA9685_ALL_LED_ON_H, onValue >> 8) ;
    writeByte(PCA9685_ALL_LED_OFF_L, offValue & 0xFF) ;
    writeByte(PCA9685_ALL_LED_OFF_H, offValue >> 8) ;
}


/**
 * i2c 통신에서 값을 읽어옴
 * @params
 *      readRegister : 레지스터 주소값
 */
int PCA9685::readByte(int readRegister)
{
    int toReturn = i2c_smbus_read_byte_data(kI2CFileDescriptor, readRegister);
    if (toReturn < 0) {
        printf("PCA9685 Read Byte error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    // For debugging
    // printf("Device 0x%02X returned 0x%02X from register 0x%02X\n", kI2CAddress, toReturn, readRegister);
    return toReturn ;
}

/**
 * i2c 통신으로 레지스터에 값을 씀
 * @params
 *      writeRegister : 값을 작성할 레지스터
 *      writeValue : 
 */
int PCA9685::writeByte(int writeRegister, int writeValue)
{   // For debugging:
    // printf("Wrote: 0x%02X to register 0x%02X \n",writeValue, writeRegister) ;
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, writeRegister, writeValue);
    if (toReturn < 0) {
        printf("PCA9685 Write Byte error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}
