#include "mpu6050.h"
//#include "sys.h"
//#include "delay.h"
//#include "usart.h"   
#include "i2c_eeprom.h"
#include "soft_delay.h"
//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init(void)
{
  i2c_eeprom_init(0x69, I2C_FAST, I2C_7BIT_ADDR, I2C_1BYTE_ADDR);
	
	i2c_eeprom_write_byte(0x6b,0x80);
	delay_ms(100);
	i2c_eeprom_write_byte(0x6b,0x00);

	i2c_eeprom_write_byte(0x1b,0x18);

	i2c_eeprom_write_byte(0x1c,0x18);

	i2c_eeprom_write_byte(0x1a,0x03);

	i2c_eeprom_write_byte(0x19,0x13);

#if 0
	u8 res; 
//	MPU_IIC_Init();//��ʼ��IIC����
	i2c_eeprom_init(0x69,I2C_FAST, I2C_7BIT_ADDR, I2C_1BYTE_ADDR);
	i2c_eeprom_write_byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
//    delay_ms(100);
//	i2c_eeprom_write_byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
//	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
//	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
//	MPU_Set_Rate(50);						//���ò�����50Hz
//	i2c_eeprom_write_byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
//	i2c_eeprom_write_byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
//	i2c_eeprom_write_byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
//	i2c_eeprom_write_byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
//	res=i2c_eeprom_read_byte(MPU_DEVICE_ID_REG); 
//	if(res==MPU_ADDR)//����ID��ȷ
//	{
//		i2c_eeprom_write_byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
//		i2c_eeprom_write_byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
//		MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
// 	}else return 1;
#endif
	return 0;
}


void get_mpu_id(unsigned char *data)
{
  *data = i2c_eeprom_read_byte(0x75);
}

void get_mpu_accel(short int data_read[6])
{
	unsigned char data[6] = {0},data_g[6] = {0};
  short int accel_x,accel_y,accel_z,g_x,g_y,g_z;
  i2c_eeprom_read_data(data,0x3b,6);
  accel_x = (data[0] << 8) + data[1];
  accel_y = (data[2] << 8) + data[3];
  accel_z = (data[4] << 8) + data[5];
  data_read[0] = accel_x/204;
  data_read[1] = accel_y/204;
  data_read[2] = accel_z/204;
  i2c_eeprom_read_data(data_g,0x43,6);
  g_x = (data_g[0] << 8) + data_g[1];
  g_y = (data_g[2] << 8) + data_g[3];
  g_z = (data_g[4] << 8) + data_g[5];
  data_read[3] = g_x/16;
  data_read[4] = g_y/16;
  data_read[5] = g_z/16;
}


//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
void MPU_Set_Gyro_Fsr(u8 fsr)
{
	 i2c_eeprom_write_byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
void MPU_Set_Accel_Fsr(u8 fsr)
{
	 i2c_eeprom_write_byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
void MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	i2c_eeprom_write_byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
void MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	i2c_eeprom_write_byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	 MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	//MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
	i2c_eeprom_read_data(buf,MPU_TEMP_OUTH_REG,2);
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
//	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	i2c_eeprom_read_data(buf,MPU_GYRO_XOUTH_REG,6);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
//	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	i2c_eeprom_read_data(buf,MPU_ACCEL_XOUTH_REG,6);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}


