#include "RTK_Structs.h"

// 移除暂时未使用的 Eigen 引用

void CommonTimeToMJDTime(const COMMONTIME* CT, MJDTIME* MJDT)
{
    MJDTIME temp;
    int y = CT->Year;
    int m = CT->Month;

    if (m <= 2) {
        y -= 1;
        m += 12;
    }

    // 1720981.5 - 2400000.5 = -679019
    temp.Days = int(y * 365.25) + int((m + 1) * 30.6001) + CT->Day - 679019;

    temp.FracDay = (CT->Hour * 3600.0 + CT->Minute * 60.0 + CT->Second) / 86400.0;
    
    if (MJDT) *MJDT = temp;
}

void MJDTimeToCommonTime(const MJDTIME* MJDT, COMMONTIME* CT)
{
    COMMONTIME temp;
    int a = MJDT->Days + 2400001;
    int b = a + 1537;
    int c = int((b-122.1)/365.25);
    int d = int(c*365.25);
    int e = int((b-d)/30.6001);
    temp.Day = b - d - int(30.6001*e);
    temp.Month = e - 1 - 12*int(e/14);
    temp.Year = c - 4715 -int((7+temp.Month)/10);
    temp.Hour = int(MJDT->FracDay*24);
    temp.Minute = int((MJDT->FracDay*24-temp.Hour)*60);
    temp.Second = (MJDT->FracDay*24*60-temp.Hour*60-temp.Minute)*60;
    
    if (CT) *CT = temp;
}

void MJDTimeToGPSTime(const MJDTIME* MJDT, GPSTIME* GT)
{
    GPSTIME temp;
    int diff_days = MJDT->Days - 44244;
    temp.Week = diff_days / 7;
    int day_of_week = diff_days - temp.Week * 7;
    temp.SecOfWeek = day_of_week * 86400.0 + MJDT->FracDay * 86400.0;
    if (GT) *GT = temp;
}

void GPSTimeToMJDTime(const GPSTIME* GT, MJDTIME* MJDT)
{
    MJDTIME temp;
    temp.Days = 44244 + GT->Week * 7 + int(GT->SecOfWeek / 86400);
    double frac = GT->SecOfWeek / 86400.0;
    temp.FracDay = frac - int(frac);
    if (MJDT) *MJDT = temp;
}

void CommonTimeToGPSTime(const COMMONTIME* CT, GPSTIME* GT)
{
    MJDTIME tempMJDT;
    CommonTimeToMJDTime(CT, &tempMJDT);
    MJDTimeToGPSTime(&tempMJDT, GT);
}

void GPSTimeToCommonTime(const GPSTIME* GT, COMMONTIME* CT)
{
    MJDTIME tempMJDT;
    GPSTimeToMJDTime(GT, &tempMJDT);
    MJDTimeToCommonTime(&tempMJDT, CT);
}
