#include "RTK_Structs.h"
#include <math.h>

double GetDiffTime(const GPSTIME* t2, const GPSTIME* t1) {
    return (double)(t2->Week - t1->Week) * 604800.0 + (t2->SecOfWeek - t1->SecOfWeek);
}

bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATMIDRES* Mid) {
    GPSEPHREC* eph = NULL;

    if (Sys == GPS) {
        eph = &GPSEph[Prn - 1];
    }
    else if (Sys == BDS) {
        eph = &BDSEph[Prn - 1];
    }

    if (eph == NULL || eph->PRN == 0) return false;

    double dt = GetDiffTime(t, &eph->TOC);

    Mid->SatClkOft = eph->ClkBias + eph->ClkDrift * dt + eph->ClkDriftRate * dt * dt;
    Mid->SatClkSft = eph->ClkDrift + 2 * eph->ClkDriftRate * dt;

    Mid->Tgd1 = eph->TGD1;
    if (Sys == GPS) {
        Mid->Tgd2 = (77.0 / 60.0) * (77.0 / 60.0) * eph->TGD1;
    } else {
        Mid->Tgd2 = eph->TGD2;
    }

    return true;
}

/* GPS 卫星轨道计算 */
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* eph, SATMIDRES* Mid) {
    if (eph == NULL || eph->PRN == 0) return false;

    double tk = GetDiffTime(t, &eph->TOE);
    if (tk > 302400.0)  tk -= 604800.0;
    if (tk < -302400.0) tk += 604800.0;

    double n0 = sqrt(GM_Earth) / pow(eph->SqrtA, 3);
    double n = n0 + eph->DeltaN;
    double M = eph->M0 + n * tk;
    double E = M, E_old;
    for (int i = 0; i < 20; i++) {
        E_old = E;
        E = M + eph->e * sin(E);
        if (fabs(E - E_old) < 1.0e-12) break;
    }
    double f = atan2(sqrt(1 - eph->e * eph->e) * sin(E), cos(E) - eph->e);
    double phi = eph->omega + f;
    double du = eph->Cuc * cos(2 * phi) + eph->Cus * sin(2 * phi);
    double dr = eph->Crc * cos(2 * phi) + eph->Crs * sin(2 * phi);
    double di = eph->Cic * cos(2 * phi) + eph->Cis * sin(2 * phi);
    double u = phi + du;
    double r = eph->SqrtA * eph->SqrtA * (1 - eph->e * cos(E)) + dr;
    double i = eph->i0 + eph->iDot * tk + di;

    double x_plane = r * cos(u);
    double y_plane = r * sin(u);

    double L = eph->OMEGA + (eph->OMEGADot - Omega_WGS) * tk - Omega_WGS * eph->TOE.SecOfWeek;

    Mid->SatPos[0] = x_plane * cos(L) - y_plane * cos(i) * sin(L);
    Mid->SatPos[1] = x_plane * sin(L) + y_plane * cos(i) * cos(L);
    Mid->SatPos[2] = y_plane * sin(i);

    Mid->Valid = true;
    return true;
}


bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* eph, SATMIDRES* Mid) {
    if (eph == NULL || eph->PRN == 0) return false;

    GPSTIME t_bds = *t;
    t_bds.SecOfWeek -= 14.0;
    t_bds.Week -= 1356;
    if (t_bds.SecOfWeek < 0) {
        t_bds.SecOfWeek += 604800.0;
        t_bds.Week -= 1;
    }

    double tk = GetDiffTime(&t_bds, &eph->TOE);
    if (tk > 302400.0)  tk -= 604800.0;
    if (tk < -302400.0) tk += 604800.0;

    double n0 = sqrt(GM_BDS) / pow(eph->SqrtA, 3);
    double n = n0 + eph->DeltaN;
    double M = eph->M0 + n * tk;
    double E = M, E_old;
    for (int k = 0; k < 20; k++) {
        E_old = E;
        E = M + eph->e * sin(E);
        if (fabs(E - E_old) < 1.0e-12) break;
    }
    double f = atan2(sqrt(1 - eph->e * eph->e) * sin(E), cos(E) - eph->e);
    double phi = eph->omega + f;
    double du = eph->Cuc * cos(2 * phi) + eph->Cus * sin(2 * phi);
    double dr = eph->Crc * cos(2 * phi) + eph->Crs * sin(2 * phi);
    double di = eph->Cic * cos(2 * phi) + eph->Cis * sin(2 * phi);
    double u = phi + du;
    double r = eph->SqrtA * eph->SqrtA * (1 - eph->e * cos(E)) + dr;
    double i = eph->i0 + eph->iDot * tk + di;

    double x_plane = r * cos(u);
    double y_plane = r * sin(u);

    bool is_geo = (Prn >= 1 && Prn <= 5) || (Prn >= 59 && Prn <= 63);

    if (is_geo) {
        double Omega_k = eph->OMEGA + eph->OMEGADot * tk - Omega_BDS * eph->TOE.SecOfWeek;
        double x_gk = x_plane * cos(Omega_k) - y_plane * cos(i) * sin(Omega_k);
        double y_gk = x_plane * sin(Omega_k) + y_plane * cos(i) * cos(Omega_k);
        double z_gk = y_plane * sin(i);

        double ang_phi = Omega_BDS * tk;
        double Rx = -5.0 * PAI / 180.0;

        double xt = x_gk;
        double yt = y_gk * cos(Rx) + z_gk * sin(Rx);
        double zt = -y_gk * sin(Rx) + z_gk * cos(Rx);

        Mid->SatPos[0] = xt * cos(ang_phi) + yt * sin(ang_phi);
        Mid->SatPos[1] = -xt * sin(ang_phi) + yt * cos(ang_phi);
        Mid->SatPos[2] = zt;
    } else {
        double L = eph->OMEGA + (eph->OMEGADot - Omega_BDS) * tk - Omega_BDS * eph->TOE.SecOfWeek;
        Mid->SatPos[0] = x_plane * cos(L) - y_plane * cos(i) * sin(L);
        Mid->SatPos[1] = x_plane * sin(L) + y_plane * cos(i) * cos(L);
        Mid->SatPos[2] = y_plane * sin(i);
    }

    Mid->Valid = true;
    return true;
}
