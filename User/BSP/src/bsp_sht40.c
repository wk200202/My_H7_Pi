#include "bsp.h"

/*************************************************************************
*		                          函数声明	
**************************************************************************
*/
static uint8_t SHT40_Crc8(uint8_t *data, int len);
static int sht40_send_byte(uint8_t addr, uint8_t *data, uint8_t len);
static int sht40_read_byte(uint8_t addr, uint8_t *data, uint8_t len);

/*************************************************************************
*	@brief 		读取sht40温湿度传感器的值
*	@param 		temp 指向存储温度值的指针
*	@param 		humi 指向存储湿度值的指针
*	@retval		成功返回0，失败返回-1
*	@note  		None
*	@author		HarryWang
*	@date  		2024/11/10
**************************************************************************
*/
int read_sht40_data(float *temp, float *humi)
{
    uint8_t writeData[1] = {0xFD};
    uint8_t readData[6] = {0};
    uint8_t retryCount = 0;
    do {
        if (sht40_send_byte(SHT40_ADDR, writeData, 1) == -1) {
            return -1;
        }
        HAL_Delay(20);
        if (sht40_read_byte(SHT40_ADDR, readData, 6) == -1) {
            return -1;
        }
        retryCount++;
        if (retryCount > 10) {
            return -1;
        }
    } while (SHT40_Crc8(&readData[0], 2) != readData[2] || SHT40_Crc8(&readData[3], 2) != readData[5]);

    *temp = (175.0 * ((readData[0] << 8) + readData[1])) / 65535.0 - 45;
    *humi = (125.0 * ((readData[3] << 8) + readData[4])) / 65535.0 - 6.0;

    return 0;
}

/*************************************************************************
*	@brief :	SHT40 crc8校验函数
*	@param :	data 要校验的数据
*	@param :	len 要校验的数据的字节数
*	@retval:	校验结果
*	@note  :	该校验适合SHT40温湿度传感器的数据校验
*	@author:	HarryWang
*	@date  :	2024/11/10
**************************************************************************
*/
static uint8_t SHT40_Crc8(uint8_t *data, int len)
{
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;
    int i, j;

    for (i = 0; i < len; ++i) {
        crc ^= *data++;

        for (j = 0; j < 8; ++j) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }

    return crc;
}

/*************************************************************************
*	@brief  	SHT40发送字节函数
*	@param  	addr 从机地址
*	@param  	write_buff 要写入的数据
*	@param  	buff_size 要写入的数据的长度
*	@retval 	成功返回0，失败返回-1
*	@note   	None
*	@author 	HarryWang
*	@date   	2024/11/10
**************************************************************************
*/
int sht40_send_byte(uint8_t addr, uint8_t *write_buff, uint8_t buff_size)
{
    uint16_t i, j;

    // 1. 发送一个停止信号
    iic_stop();

    /* 通过检查器件应答的方式，判断内部写操作是否完成, 一般小于 10ms
    CLK频率为200KHz时，查询次数为30次左右
    */
    for (i = 0; i < SHT40_INQUIRE_CNT; i++) {
        // 2. 发起I2C总线启动信号
        iic_start();

        // 3. 发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读
        iic_send_byte(addr << 1 | SHT40_I2C_WRITE);

        // 4. 发送一个时钟，判断器件是否正确应答
        if (iic_wait_ack() == 0) {
            break;
        }
    }
    if (i == SHT40_INQUIRE_CNT) {
        iic_stop();   // 发送写超时
        return -1;
    }

    for (i = 0; i < buff_size; ++i) {
        for (j = 0; j < SHT40_INQUIRE_CNT; j++) {
            // 5. 发送数据
            iic_send_byte(write_buff[i]);

            // 6. 等待ACK
            if (iic_wait_ack() == 0) {
                break;
            }
        }

        if (j == SHT40_INQUIRE_CNT) {
            iic_stop();   // 发送写超时
            return -1;
        }
    }

    // 7. 发送停止信号
    iic_stop();

    return 0;
}

/*************************************************************************
*	@brief 		SHT40读取字节函数
*	@param 		None
*	@retval		成功返回0，失败返回-1
*	@note  		None
*	@author		HarryWang
*	@date  		2024/11/10
**************************************************************************
*/
int sht40_read_byte(uint8_t addr, uint8_t *read_buff, uint8_t buff_size)
{
    uint16_t i;

    for (i = 0; i < SHT40_INQUIRE_CNT; i++) {
        /* 1. 发起I2C总线启动信号 */
        iic_start();

        /* 2. 读的话，先写入从机地址 */
        iic_send_byte(addr << 1 | SHT40_I2C_WRITE);

        /* 3. 发送一个时钟，判断器件是否正确应答 */
        if (iic_wait_ack() == 0) {
            break;
        }
    }
    if (i == SHT40_INQUIRE_CNT) {
        iic_stop();   // 发送写超时
        return -1;
    }

    for (i = 0; i < SHT40_INQUIRE_CNT; i++) {
        /* 4. 重新启动I2C总线。前面的代码的目的是传送地址，下面开始读取数据 */
        iic_start();

        /* 5. 发送读控制 */
        iic_send_byte(addr << 1 | SHT40_I2C_READ);

        /* 6. 发送一个时钟，判断器件是否正确应答 */
        if (iic_wait_ack() == 0) {
            break;
        }
    }
    if (i == SHT40_INQUIRE_CNT) {
        iic_stop();   // 发送写超时
        return -1;
    }

    /* 7. 循环读取数据 */
    for (i = 0; i < buff_size; i++) {
        read_buff[i] = iic_read_byte(2);   // 读数据，发送NACK

        /* 每读完1个字节后，需要发送Ack， 最后一个字节不需要Ack，发Nack */
        if (i != buff_size - 1) {
            iic_ack();   // 中间字节读完后，CPU产生ACK信号(驱动SDA = 0)
        } else {
            iic_nack();   // 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1)
        }
    }

    /* 8. 发送I2C总线停止信号 */
    iic_stop();

    return 0; /* 执行成功 */
}