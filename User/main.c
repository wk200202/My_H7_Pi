#include "bsp.h"

#define TEST_ADDR 0    /* 读写测试地址 */
#define TEST_SIZE 1024 /* 读写测试数据大小 */
ALIGN_32BYTES(uint8_t buf[TEST_SIZE]);
ALIGN_32BYTES(uint8_t SpeedTestbuf[16 * 1024]); /* 仅用于读速度测试目的 */

/*************************************************************************
*	@brief 		主函数
*	@param 		None
*	@retval		None
*	@note  		None
*	@author		HarryWang
*	@date  		2024/11/10
**************************************************************************
*/
int main(void)
{
    bsp_Init();
    bsp_InitQSPI_W25Q256();

    uint32_t id;
    uint32_t uiAddr;

    bsp_StartAutoTimer(0, 2000); /* 启动定时器 */

    while (1) {
        bsp_RGB_Change_Color(10, 0, 0);
        bsp_DelayMS(100);
        bsp_RGB_Change_Color(0, 10, 0);
        bsp_DelayMS(100);
        bsp_RGB_Change_Color(0, 0, 10);
        bsp_DelayMS(100);
        if (bsp_CheckTimer(0) == 1) {
            /* 检测串行Flash OK */
            id = QSPI_ReadID();
            printf("检测到串行Flash, ID = %08X, 型号: W25Q256JV\r\n", id);
            printf(" 容量 : 32M字节, 扇区大小 : 4096字节, 页大小：256字节\r\n");

            uint32_t i;

            for (i = 0; i < TEST_SIZE; i++) {
                buf[i] = i;
            }

            QSPI_EraseSector(TEST_ADDR);

            for (i = 0; i < TEST_SIZE; i += QSPI_PAGE_SIZE) {
                if (QSPI_WriteBuffer(buf, TEST_ADDR + i, QSPI_PAGE_SIZE) == 0) {
                    printf("写串行Flash出错！\r\n");
                }
            }
            printf("写串行Flash成功！\r\n");

            /* 清缓冲区为0x55 */
            memset(buf, 0x55, TEST_SIZE);
            QSPI_ReadBuffer(buf, 0, 1);
            printf("读取第1个字节，数据如下,若为0x55则读取失败\r\n");
            printf(" %02X \r\n", buf[0]);

            /* 清空缓冲区为AA */
            memset(buf, 0xAA, TEST_SIZE);
            QSPI_ReadBuffer(buf, 0, 10);
            printf("读取前10个字节，数据如下,若为0xAA则读取失败\r\n");
            for (i = 0; i < 10; i++) {
                printf(" %02X", buf[i]);
            }
            printf("\r\n");

            /* 清空缓冲区 */
            memset(buf, 0, TEST_SIZE);
            QSPI_ReadBuffer(buf, 0, TEST_SIZE);
            printf("读串行Flash成功，数据如下，若为0则读取失败\r\n");

            /* 打印数据 */
            for (i = 0; i < TEST_SIZE; i++) {
                printf(" %02X", buf[i]);

                if ((i & 31) == 31) {
                    printf("\r\n"); /* 每行显示16字节数据 */
                } else if ((i & 31) == 15) {
                    printf(" - ");
                }
            }
        }
    }
}