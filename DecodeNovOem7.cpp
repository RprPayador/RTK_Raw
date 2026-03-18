#include "RTK_Structs.h"
#include <string.h>

// NovAtel CRC校验程序 (来自老师课件)
#define POLYCRC32 0xEDB88320u

unsigned int crc32(const unsigned char *buff, int len) {
    int i, j;
    unsigned int crc = 0;
    for (i = 0; i < len; i++) {
        crc ^= buff[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}

// 解码调度主函数
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[]) {
    int i = 0;
    int ret = 0;

    // 循环扫描同步字符 0xAA 0x44 0x12
    while (i <= Len - 3) {
        if (Buff[i] == 0xAA && Buff[i+1] == 0x44 && Buff[i+2] == 0x12) {
            // 找到同步头，检查剩余字节是否足够解码 28 字节消息头
            if (i + 28 > Len) break;

            // 解码消息头中的 MsgID 和 MsgLen
            unsigned short msgID = *(unsigned short*)(Buff + i + 4);
            unsigned short msgLen = *(unsigned short*)(Buff + i + 8);

            // 检查整条消息（头 28 + 体 msgLen + 尾 4 字节 CRC）是否完整
            int totalLen = 28 + msgLen + 4;
            if (i + totalLen > Len) break;

            // CRC 校验
            unsigned int crcComputed = crc32(Buff + i, 28 + msgLen);
            unsigned int crcInMsg = *(unsigned int*)(Buff + i + 28 + msgLen);

            if (crcComputed != crcInMsg) {
                // CRC 失败，跳过同步头继续搜索
                i += 3;
                continue;
            }

            // CRC 通过，根据 MsgID 分发
            switch (msgID) {
                case 43:   // RANGEB (二进制观测值)
                    ret = decode_rangeb_oem7(Buff + i, obs);
                    break;
                case 7:    // GPSEPHEM (GPS星历)
                    ret = decode_gpsephem(Buff + i, geph);
                    break;
                case 1696: // BDSEPHEM (BDS星历)
                    ret = decode_bdsephem(Buff + i, beph);
                    break;
                case 47:   // PSRPOS (定位结果)
                    // ret = decode_psrpos(Buff + i, pos);
                    break;
                default:
                    // 其他不需要的消息，忽略
                    break;
            }

            // 处理完一条消息，移动指针到下一条可能的消息起始位置
            i += totalLen;

            // 如果是观测值消息（文件模式），直接返回
            if (ret == 1) break; 
        } else {
            i++;
        }
    }

    // 循环结束后，处理剩余的不完整数据块
    if (i > 0) {
        int remaining = Len - i;
        if (remaining > 0) {
            memmove(Buff, Buff + i, remaining);
        }
        Len = remaining; // 更新 Len 供下次读文件拼接
    }

    return ret;
}

// 解码观测值消息 RANGEB (MsgID: 43)
int decode_rangeb_oem7(unsigned char *buff, EPOCHOBS* obs) {
    if (!buff || !obs) return 0;

    // 1. 从消息头读取时间 (头偏移 14 字节是 GPS 周，16 字节是周内秒ms)
    unsigned short week = *(unsigned short*)(buff + 14);
    unsigned int ms = *(unsigned int*)(buff + 16);
    obs->Time.Week = week;
    obs->Time.SecOfWeek = ms / 1000.0;

    // 2. 解码观测值数量 (消息体从 buff+28 开始，头4字节是观测值总数 N)
    unsigned int nObs = *(unsigned int*)(buff + 28);
    unsigned char *ptr = buff + 28 + 4; // 指向第一个观测值 block (44字节)

    obs->SatNum = 0; // 重置当前历元

    // 3. 循环解析每一个信号通道
    for (unsigned int i = 0; i < nObs; i++) {
        unsigned short prn = *(unsigned short*)(ptr);
        unsigned int status = *(unsigned int*)(ptr + 4);
        double psr = *(double*)(ptr + 8);      // 伪距
        double adr = -*(double*)(ptr + 20);     // 载波相位 (NovAtel ADR符号需取反)
        float dopp = *(float*)(ptr + 32);      // 多普勒
        float cno = *(float*)(ptr + 36);       // 载噪比
        float lockTime = *(float*)(ptr + 40);  // 锁定时间

        // 解析 Tracking Status
        int sysId = (status >> 16) & 0x07;     // Bit 16-18: Satellite System
        int sigType = (status >> 10) & 0x1F;   // Bit 10-14: Signal Type

        GNSSSys sys = UNKS;
        int s = -1; // 频率索引：0=第一频率, 1=第二频率

        if (sysId == 0) { // GPS
            sys = GPS;
            if (sigType == 0) s = 0;       // L1 C/A
            else if (sigType == 16) s = 1; // L2 P(Y)
        } else if (sysId == 4) { // BDS
            sys = BDS;
            if (sigType == 0) s = 0;       // B1I
            else if (sigType == 2) s = 1;  // B3I
        }

        if (s == -1) { ptr += 44; continue; }

        // 查找或添加卫星
        int idx = -1;
        for (int k = 0; k < obs->SatNum; k++) {
            if (obs->SatObs[k].Prn == (short)prn && obs->SatObs[k].System == sys) {
                idx = k; break;
            }
        }
        if (idx == -1 && obs->SatNum < MAXCHANNUM) {
            idx = obs->SatNum++;
            obs->SatObs[idx].Prn = (short)prn;
            obs->SatObs[idx].System = sys;
        }

        if (idx != -1) {
            if (s == 0) {
                obs->SatObs[idx].c1 = psr; obs->SatObs[idx].l1 = adr;
                obs->SatObs[idx].d1 = dopp; obs->SatObs[idx].cn0[0] = (double)cno;
                obs->SatObs[idx].LockTime[0] = (double)lockTime;
            } else {
                obs->SatObs[idx].p2 = psr; obs->SatObs[idx].l2 = adr;
                obs->SatObs[idx].d2 = dopp; obs->SatObs[idx].cn0[1] = (double)cno;
                obs->SatObs[idx].LockTime[1] = (double)lockTime;
            }
            obs->SatObs[idx].Valid = true;
        }
        ptr += 44;
    }
    return 1;
}

// 解码 GPS 星历消息 GPSEPHEM (MsgID: 7)
int decode_gpsephem(unsigned char* buff, GPSEPHREC* eph) {
    if (!buff || !eph) return 0;
    
    unsigned char *ptr = buff + 28; // 消息体开始 (H)

    // 根据手册图表偏移量提取数据
    unsigned int prn = *(unsigned int*)(ptr + 0);    // Field 2: PRN
    if (prn < 1 || prn > MAXGPSNUM) return 0;

    GPSEPHREC *p = &eph[prn-1];
    p->PRN = (unsigned short)prn;
    p->Sys = GPS;

    // 时间与标识
    p->SVHealth     = (short)(*(unsigned int*)(ptr + 12)); // Field 4: health
    p->IODE         = *(unsigned int*)(ptr + 16);          // Field 5: IODE1
    p->TOC.Week     = p->TOE.Week = (short)(*(unsigned int*)(ptr + 24)); // Field 7: week
    p->TOE.SecOfWeek = *(double*)(ptr + 32);               // Field 9: toe (Double)
    
    // 轨道参数
    double A = *(double*)(ptr + 40);                       // Field 10: A (meters)
    p->SqrtA = (A > 0) ? sqrt(A) : 0;                      // 结构体需要 SqrtA
    p->DeltaN       = *(double*)(ptr + 48);                // Field 11
    p->M0           = *(double*)(ptr + 56);                // Field 12
    p->e            = *(double*)(ptr + 64);                // Field 13
    p->omega        = *(double*)(ptr + 72);                // Field 14
    p->Cuc          = *(double*)(ptr + 80);                // Field 15
    p->Cus          = *(double*)(ptr + 88);                // Field 16
    p->Crc          = *(double*)(ptr + 96);                // Field 17
    p->Crs          = *(double*)(ptr + 104);               // Field 18
    p->Cic          = *(double*)(ptr + 112);               // Field 19
    p->Cis          = *(double*)(ptr + 120);               // Field 20
    p->i0           = *(double*)(ptr + 128);               // Field 21
    p->iDot         = *(double*)(ptr + 136);               // Field 22
    p->OMEGA        = *(double*)(ptr + 144);               // Field 23: RA (Omega0)
    p->OMEGADot     = *(double*)(ptr + 152);               // Field 24: Omega_dot

    // 钟差参数
    p->IODC         = *(unsigned int*)(ptr + 160);         // Field 25
    p->TOC.SecOfWeek = *(double*)(ptr + 164);              // Field 26: toc (Double)
    p->TGD1         = *(double*)(ptr + 172);               // Field 27: tgd
    p->ClkBias      = *(double*)(ptr + 180);               // Field 28: af0
    p->ClkDrift     = *(double*)(ptr + 188);               // Field 29: af1
    p->ClkDriftRate = *(double*)(ptr + 196);               // Field 30: af2
    
    p->SVAccuracy   = *(double*)(ptr + 216);               // Field 33: URA (variance)

    return 2;
}

// 解码 BDS 星历消息 BDSEPHEM (MsgID: 1696)
int decode_bdsephem(unsigned char* buff, GPSEPHREC* eph) {
    if (!buff || !eph) return 0;
    
    unsigned char *ptr = buff + 28; // 消息体开始 (H)

    // 根据表格偏移量提取数据
    unsigned int prn  = *(unsigned int*)(ptr + 0);   // Field 2: satellite ID
    unsigned int week = *(unsigned int*)(ptr + 4);   // Field 3: Week
    
    if (prn < 1 || prn > MAXBDSNUM) return 0;

    GPSEPHREC *p = &eph[prn-1];
    p->PRN = (unsigned short)prn;
    p->Sys = BDS;

    // 填充星历参数
    p->SVAccuracy   = *(double*)(ptr + 8);           // Field 4: URA
    p->SVHealth     = (short)(*(unsigned int*)(ptr + 16)); // Field 5: health 1
    p->TGD1         = *(double*)(ptr + 20);          // Field 6: tgd1 (B1)
    p->TGD2         = *(double*)(ptr + 28);          // Field 7: tgd2 (B2)
    p->IODE         = *(unsigned int*)(ptr + 36);    // Field 8: AODC
    p->TOC.SecOfWeek = (double)(*(unsigned int*)(ptr + 40)); // Field 9: toc (Uint)
    p->ClkBias      = *(double*)(ptr + 44);          // Field 10: a0
    p->ClkDrift     = *(double*)(ptr + 52);          // Field 11: a1
    p->ClkDriftRate = *(double*)(ptr + 60);          // Field 12: a2
    p->IODC         = *(unsigned int*)(ptr + 68);    // Field 13: AODE
    p->TOE.SecOfWeek = (double)(*(unsigned int*)(ptr + 72)); // Field 14: toe (Uint)
    p->SqrtA        = *(double*)(ptr + 76);          // Field 15: RootA
    p->e            = *(double*)(ptr + 84);          // Field 16: ecc
    p->omega        = *(double*)(ptr + 92);          // Field 17: omega
    p->DeltaN       = *(double*)(ptr + 100);         // Field 18: Delta N
    p->M0           = *(double*)(ptr + 108);         // Field 19: M0
    p->OMEGA        = *(double*)(ptr + 116);         // Field 20: Omega 0
    p->OMEGADot     = *(double*)(ptr + 124);         // Field 21: Omega_dot
    p->i0           = *(double*)(ptr + 132);         // Field 22: i0
    p->iDot         = *(double*)(ptr + 140);         // Field 23: IDOT
    p->Cuc          = *(double*)(ptr + 148);         // Field 24: Cuc
    p->Cus          = *(double*)(ptr + 156);         // Field 25: Cus
    p->Crc          = *(double*)(ptr + 164);         // Field 26: Crc
    p->Crs          = *(double*)(ptr + 172);         // Field 27: Crs
    p->Cic          = *(double*)(ptr + 180);         // Field 28: Cic
    p->Cis          = *(double*)(ptr + 188);         // Field 29: Cis

    p->TOC.Week = p->TOE.Week = (short)week;         // 北斗周

    return 2; // 返回 2 表示解码到星历
}

// 解码定位结果 PSRPOS (MsgID: 47)
int decode_psrpos(unsigned char* buff, PPRESULT* pos) {
    if (!buff || !pos) return 0;
    unsigned char *ptr = buff + 28;
    pos->IsSuccess = (*(unsigned int*)(ptr) == 0); // 0=SOL_COMPUTED
    pos->Position[0] = *(double*)(ptr + 12); // Lat (deg)
    pos->Position[1] = *(double*)(ptr + 20); // Lon (deg)
    pos->Position[2] = *(double*)(ptr + 28); // Hgt (m)
    return 3;
}
