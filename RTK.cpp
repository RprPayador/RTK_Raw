// RTK.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <iomanip>
#include "RTK_Structs.h"

// 声明外部函数，如果已经有头文件包含请忽略
extern void CommonTimeToMJDTime(const COMMONTIME* CT, MJDTIME* MJDT);
extern void MJDTimeToGPSTime(const MJDTIME* MJDT, GPSTIME* GT);
extern void GPSTimeToMJDTime(const GPSTIME* GT, MJDTIME* MJDT);
extern void MJDTimeToCommonTime(const MJDTIME* MJDT, COMMONTIME* CT);

int main()
{
    // 测试代码保留原有的时间/坐标转换测试（可选，若需要可移至单独函数）
    // 此处主要实现 NovAtel OEM7 二进制文件解码主逻辑

    const char* fileName = "OEM7_Data.bin"; // 需确保目录下有此测试文件
    FILE* fp = fopen(fileName, "rb");
    if (!fp) {
        printf("错误: 无法打开数据文件 %s\n", fileName);
        return -1;
    }

    unsigned char buff[MAXRAWLEN];
    int len = 0; // 当前 buff 中有效字节数
    
    // 初始化数据存储结构
    EPOCHOBS obs;
    GPSEPHREC geph[MAXGPSNUM], beph[MAXBDSNUM];

    printf("开始读取并解码二进制文件: %s...\n", fileName);

    while (!feof(fp)) {
        // 1. 读取数据到缓冲区 (从上次剩余的位置之后开始追加)
        int nRead = fread(buff + len, 1, MAXRAWLEN - len, fp);
        if (nRead <= 0) break;
        len += nRead;

        // 2. 调用解码主调度函数
        // 在文件模式下，DecodeNovOem7Dat 解码到一条观测值后会返回 1
        while (true) {
            int ret = DecodeNovOem7Dat(buff, len, &obs, geph, beph);

            if (ret == 1) { // 解码到历元观测值
                printf(">>> 解码到一个历元: GPS Week=%d, SecOfWeek=%.3f, SatNum=%d\n",
                        obs.Time.Week, obs.Time.SecOfWeek, obs.SatNum);
                
                // 此处可以衔接 SPP(观测值, 星历) 或 RTK 计算
                // SPP(&obs, ...);
            }
            else if (ret == 2) { // 解码到星历
                // 星历已更新在 geph/beph 中，继续循环解码 buff
                continue; 
            }
            else {
                // 当前 buff 已无完整消息，退出内层循环去 fread 读更多数据
                break;
            }
        }
    }

    fclose(fp);
    printf("文件读取结束。\n");

    return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
// 3. 使用输出窗口查看生成输出和其他消息
// 4. 使用错误列表窗口查看错误
// 5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
// 6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
