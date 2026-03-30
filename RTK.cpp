#include <iostream>
#include <iomanip>
#include "RTK_Structs.h"

int main()
{
    FILE* fp, * fout;
    unsigned char Buff[MAXRAWLEN];
    int Len, LenRead, val;
    EPOCHOBS Obs;
    GPSEPHREC GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM];

    if ((fp = fopen("D:\\GNSS Algorithm\\RTK\\RTK\\oem719-202603111200.bin", "rb")) == NULL) { // C4996
        printf("The file 'oem719-202603111200.bin' was not opened\n");
        return 0;
    }

    Len = 0;
    do {
        LenRead = fread(Buff + Len, sizeof(unsigned char), MAXRAWLEN - Len, fp);
        Len = Len + LenRead;

        val = DecodeNovOem7Dat(Buff, Len, &Obs, GpsEph, BdsEph);

    } while (!feof(fp));
    return 1;
}

