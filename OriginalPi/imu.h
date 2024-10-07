#ifndef _IMU_CLASS
#define _IMU_CLASS

#include <unistd.h> // Needed for closing the I2C connection
#include <wiringPiI2C.h>

// Define the I2C address of the IMU
static constexpr int IMU_address = 0b1101000;

// IMU register addresses
static constexpr int GYRO_CONFIG_REGISTER_ADDRESS = 0x1B;
static constexpr int ACCEL_CONFIG_REGISTER_ADDRESS = 0x1C;
static constexpr int PWR_MGMT_1_REGISTER_ADDRESS = 0x6B;

static constexpr int ACCEL_XOUT_H_REGISTER_ADDRESS = 0x3B;
static constexpr int ACCEL_XOUT_L_REGISTER_ADDRESS = 0x3C;

static constexpr int ACCEL_YOUT_H_REGISTER_ADDRESS = 0x3D;
static constexpr int ACCEL_YOUT_L_REGISTER_ADDRESS = 0x3E;

static constexpr int ACCEL_ZOUT_H_REGISTER_ADDRESS = 0x3F;
static constexpr int ACCEL_ZOUT_L_REGISTER_ADDRESS = 0x40;

static constexpr int GYRO_XOUT_H_REGISTER_ADDRESS = 0x43;
static constexpr int GYRO_XOUT_L_REGISTER_ADDRESS = 0x44;

static constexpr int GYRO_YOUT_H_REGISTER_ADDRESS = 0x45;
static constexpr int GYRO_YOUT_L_REGISTER_ADDRESS = 0x46;

static constexpr int GYRO_ZOUT_H_REGISTER_ADDRESS = 0x47;
static constexpr int GYRO_ZOUT_L_REGISTER_ADDRESS = 0x48;

namespace Quad
{
	namespace IMU
	{

		enum class gyroConfigEnum
		{
			GYRO_FS_NOT_SET,
			FS_250_DPS,
			FS_500_DPS,
			FS_1000_DPS,
			FS_2000_DPS
		};

		enum class accelConfigEnum
		{
			AFS_NOT_SET,
			AFS_2_G,
			AFS_4_G,
			AFS_8_G,
			AFS_16_G,
		};

		class intertialMeasurementUnit {
		public:
			intertialMeasurementUnit(const gyroConfigEnum& gyroFS, const accelConfigEnum& accelFS);
			~intertialMeasurementUnit();

			void initialiseIMU();

			void readIMU_Data(short& gyroPitch, short& gyroRoll, short& gyroYaw,
				short& accelX, short& accelY, short& accelZ);

		private:
			void calibrateIMU();

			void setIMU_MaxRates(const gyroConfigEnum& gyroCfg, const accelConfigEnum& accelCfg);

			int imuHandle;

			float GYRO_LSB_VALUE; // See p29/46 of register map document
			float ACCEL_LSB_VALUE; // See p31/46 of register map document

			gyroConfigEnum gyroFullScale = gyroConfigEnum::GYRO_FS_NOT_SET;
			accelConfigEnum accelFullScale = accelConfigEnum::AFS_NOT_SET;

			float offsetGyroPitch;
			float offsetGyroRoll;
			float offsetGyroYaw;
			float offsetAccelX;
			float offsetAccelY;
			float offsetAccelZ;

			// IMU data masks
			int gyroConfigMask; // See register 27 on p14/46 of register map document
			int accelConfigMask; // See register 28 on p15/46 of register map document
		};

	} // namespace IMU
} // namespace Quad

#endif // _IMU_CLASS
