// RTK.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <iomanip>
#include "RTK_Structs.h"

// 声明外部函数，如果已经有头文件包含请忽略
extern void CommonTimeToMJDTime(const COMMONTIME* CT, MJDTIME* MJDT);
extern void MJDTimeToGPSTime(const MJDTIME* MJDT, GPSTIME* GT);
void PrintEph(FILE* fp, GPSEPHREC* eph) {
    if (!fp || !eph) return;
    const char* tag = (eph->Sys == GPS) ? "GPSEPH" : "BDSEPH";
    fprintf(fp, "%s PRN:%02d Week:%d TOC:%.3f a0: %22.15E a1: %22.15E a2: %22.15E TOE:%.3f dn: %22.15E M0: %22.15E SqrtA: %22.15E e: %22.15E OMEGA0: %22.15E i0: %22.15E omega: %22.15E idot: %22.15E OMGDot: %22.15E Crs: %22.15E Crc: %22.15E Cis: %22.15E Cic: %22.15E Cus: %22.15E Cuc: %22.15E Health: %d TGD: %22.15E\n",
            tag, eph->PRN, eph->TOE.Week, eph->TOC.SecOfWeek,
            eph->ClkBias, eph->ClkDrift, eph->ClkDriftRate,
            eph->TOE.SecOfWeek, eph->DeltaN, eph->M0, eph->SqrtA, eph->e,
            eph->OMEGA, eph->i0, eph->omega, eph->iDot, eph->OMEGADot,
            eph->Crs, eph->Crc, eph->Cis, eph->Cic, eph->Cus, eph->Cuc,
            eph->SVHealth, eph->TGD1);
}

int main(int argc, char* argv[])
{
    const wchar_t* fileName = L"OEM719原始数据/oem719-202603111200.bin"; 
    FILE* fp = _wfopen(fileName, L"rb");
    if (!fp) { wprintf(L"错误: 无法打开数据文件\n"); return -1; }

    FILE* fout = fopen("result.txt", "w");
    if (!fout) { printf("错误: 无法创建输出文件\n"); return -1; }

    unsigned char buff[MAXRAWLEN];
    int len = 0;
    EPOCHOBS obs;
    GPSEPHREC geph[MAXGPSNUM], beph[MAXBDSNUM];

    printf("开始解析并生成统一日志 result.txt...\n");

    while (!feof(fp)) {
        int nRead = fread(buff + len, 1, MAXRAWLEN - len, fp);
        if (nRead <= 0) break;
        len += nRead;

        while (true) {
            int res = DecodeNovOem7Dat(buff, len, &obs, geph, beph);
            if (res == 0) break;

            int msgID = res & 0xFFFF;
            int prn   = (res >> 16) & 0xFF;

            if (msgID == 43) { // RANGEB
                fprintf(fout, "RANGE Week:%d Sec:%.3f SatNum:%d\n", obs.Time.Week, obs.Time.SecOfWeek, obs.SatNum);
                for (int i = 0; i < obs.SatNum; i++) {
                    fprintf(fout, "  PRN:%02d Sys:%d c1:%.3f l1:%.3f p2:%.3f l2:%.3f d1:%.3f d2:%.3f\n",
                            obs.SatObs[i].Prn, (int)obs.SatObs[i].System,
                            obs.SatObs[i].c1, obs.SatObs[i].l1, obs.SatObs[i].p2, obs.SatObs[i].l2,
                            obs.SatObs[i].d1, obs.SatObs[i].d2);
                }
            }
            else if (msgID == 7) { // GPS星历
                if (prn >= 1 && prn <= MAXGPSNUM) PrintEph(fout, &geph[prn-1]);
            }
            else if (msgID == 1696) { // BDS星历
                if (prn >= 1 && prn <= MAXBDSNUM) PrintEph(fout, &beph[prn-1]);
            }
        }
    }

    fclose(fp); fclose(fout);
    printf("处理完成！结果已存入 result.txt\n");
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
