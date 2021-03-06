// -*- mode: C++; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4; -*-
// vim: set shiftwidth=4 softtabstop=4 expandtab:
/*
 ********************************************************************
 ** NIDAS: NCAR In-situ Data Acquistion Software
 **
 ** 2005, Copyright University Corporation for Atmospheric Research
 **
 ** This program is free software; you can redistribute it and/or modify
 ** it under the terms of the GNU General Public License as published by
 ** the Free Software Foundation; either version 2 of the License, or
 ** (at your option) any later version.
 **
 ** This program is distributed in the hope that it will be useful,
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ** GNU General Public License for more details.
 **
 ** The LICENSE.txt file accompanying this software contains
 ** a copy of the GNU General Public License. If it is not found,
 ** write to the Free Software Foundation, Inc.,
 ** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 **
 ********************************************************************
*/
#include "IRS_HW_HG2001GD.h"

using namespace nidas::dynld::raf;

NIDAS_CREATOR_FUNCTION_NS(raf,IRS_HW_HG2001GD);

double IRS_HW_HG2001GD::processLabel(const int data,sampleType* stype)
{
    int sign = 1;
    float carry = 0.0;

    // Default to single precision. If some label needs to be
    // DOUBLE_ST, change it in the appropriate case.
    *stype = FLOAT_ST;

    //err("%4o 0x%08lx", (int)(data & 0xff), (data & (unsigned int)0xffffff00) );

    switch (data & 0xff) {

    case 0007:  // BCD - time to nav          (min)
        if (((data & SSM) == NCD) || ((data & SSM) == TST)) break;
        return (
                ((data & (0xf<<22)) >> 22) * 1.0 +
                ((data & (0xf<<18)) >> 18) * 0.1
               ); // no sign

    case 0011:  // BCD - pos longitude        (deg)
        carry = ((data & (0x1<<28)) >> 28) *  100.0;
    case 0010:  // BCD - pos latitude         (deg)
        if (((data & SSM) == NCD) || ((data & SSM) == TST)) break;
        if ((data & SSM) == SSM) sign = -1;
        return (
                carry +
                ((data & (0xf<<24)) >> 24) *  10.0 +
                ((data & (0xf<<20)) >> 20) *  1.0 +
                (
                 ((data & (0xf<<16)) >> 16) * 10.0 +
                 ((data & (0xf<<12)) >> 12) * 1.0 +
                 ((data & (0xf<< 8)) >>  8) * 0.1
                ) / 60.0
               ) * sign;

    case 0012:  // BCD - ground speed         (knot)
        if (((data & SSM) == NCD) || ((data & SSM) == TST)) break;
        return (
                ((data & (0x7<<26)) >> 26) * 1000.0 +
                ((data & (0xf<<22)) >> 22) * 100.0 +
                ((data & (0xf<<18)) >> 18) * 10.0 +
                ((data & (0xf<<14)) >> 14) * 1.0
               ) * KTS_MS; // no sign

    case 0013:  // BCD - trk angle true       (deg)
    case 0014:  // BCD - mag heading          (deg)
    case 0044:  // BCD - true heading         (deg)
        if (((data & SSM) == NCD) || ((data & SSM) == TST)) break;
        if ((data & SSM) == SSM) {sign = -1; carry = 360.0;}
        return (
                ((data & (0x3<<26)) >> 26) * 100.0 +
                ((data & (0xf<<22)) >> 22) * 10.0 +
                ((data & (0xf<<18)) >> 18) * 1.0 +
                ((data & (0xf<<14)) >> 14) * 0.1
               ) * sign + carry;

    case 0015:  // BCD - wind speed           (knot)
        if (((data & SSM) == NCD) || ((data & SSM) == TST)) break;
        return (
                ((data & (0x3<<26)) >> 26) * 100.0 +
                ((data & (0xf<<22)) >> 22) * 10.0 +
                ((data & (0xf<<18)) >> 18) * 1.0
               ) * KTS_MS; // no sign

    case 0016:  // BCD - wind dir true        (deg)
        if (((data & SSM) == NCD) || ((data & SSM) == TST)) break;
        if ((data & SSM) == SSM) {sign = -1; carry = 360.0;}
        return (
                ((data & (0x3<<26)) >> 26) * 100.0 +
                ((data & (0xf<<22)) >> 22) * 10.0 +
                ((data & (0xf<<18)) >> 18) * 1.0
               ) * sign + carry;

    case 0126:  // BNR - Time in Nav          (min)
        if ((data & SSM) != SSM) break;
        return (data<<4>>17) * 1.0; // no sign

    case 0132:  // BNR - 15 sig bits - hybrid true_heading  (deg)
    case 0137:  // BNR - 15 sig bits - hybrid track_angle_true (deg)
        if ((data & SSM) != SSM) break;
        if (data & 0x10000000) carry += 360.0;
        return (data<<3>>16) * 0.0055 + carry;

    case 0135:  // BNR - 18 sig bits - hybrid Vertical FOM  (feet)
        if ((data & SSM) != SSM) break;
        return (data<<4>>13) * FT_MTR / (1<<3); // no sign

    case 0175:  // BNR - 15 sig bits - hybrid ground_speed  (knot)
        if ((data & SSM) != SSM) break;
        return (data<<4>>17) * 0.125 * KTS_MS; // no sign

    case 0300:  // BNR - delta theta x        (radian)
    case 0301:  // BNR - delta theta y        (radian)
    case 0302:  // BNR - delta theta z        (radian)
        if ((data & SSM) != SSM) break;
        return (data<<1>>9) * 3.7252902984619140625e-9 * RAD_DEG;

    case 0303:  // BNR - delta theta v x      (ft/s)
    case 0304:  // BNR - delta theta v y      (ft/s)
    case 0305:  // BNR - delta theta v z      (ft/s)
        if ((data & SSM) != SSM) break;
        return (data<<1>>9) * 4.76837158203125e-7 * FT_MTR;

    case 0254:  // BNR - 20 sig bits - hybrid latitude      (deg)
        if ((data & SSM) != SSM) {
            _lat = doubleNAN;
            break;
        }
        if (data & (1<<28)) _lat_sign = -1;
        else                _lat_sign = 1;
        return (_lat = (data<<3>>11) * 1.71661376953125e-4); // 180.0/(1<<20)

    case 0255:  // BNR - 20 sig bits - hybrid longitude     (deg)
        if ((data & SSM) != SSM) {
            _lon = doubleNAN;
            break;
        }
        if (data & (1<<28)) _lon_sign = -1;
        else                _lon_sign = 1;
        return (_lon = (data<<3>>11) * 1.71661376953125e-4); // 180.0/(1<<20)

    case 0256:  // BNR - 11 sig bits - hybrid latitude fine (deg)
        if ((data & SSM) != SSM) break;
        return _lat + (data<<4>>21) * 8.381903171539306640625e-8 * _lat_sign; // 180.0/(1<<31)

    case 0257:  // BNR - 11 sig bits - hybrid longitude fine(deg)
        if ((data & SSM) != SSM) break;
        return _lon + (data<<4>>21) * 8.381903171539306640625e-8 * _lon_sign; // 180.0/(1<<31)

    case 0310:  // BNR - 20 sig bits - pos_latitude         (deg)
    case 0311:  // BNR - 20 sig bits - pos_longitude        (deg)
        if ((data & SSM) != SSM) break;
        return (data<<3>>11) * 1.71661376953125e-4; // 180.0/(1<<20)

    case 0312:  // BNR - ground_speed         (knot)
        if ((data & SSM) != SSM) break;
        return (data<<4>>14) * 0.015625 * KTS_MS; // no sign

    case 0324:  // BNR - pitch_angle          (deg)
        carry = _irs_ptch_corr;
        goto corr;
    case 0325:  // BNR - roll_angle           (deg)
        carry = _irs_roll_corr;
        goto corr;
    case 0314:  // BNR - true_heading         (deg)
        carry = _irs_thdg_corr;
    case 0313:  // BNR - track_angle_true     (deg)
    case 0316:  // BNR - wind_dir_true        (deg)
    case 0317:  // BNR - trk angle mag        (deg)
    case 0320:  // BNR - mag heading          (deg)
    case 0334:  // BNR - platform_hdg         (deg)
        if (data & 0x10000000) carry += 360.0;
        goto corr;
    case 0321:  // BNR - drift_angle          (deg)
    case 0322:  // BNR - flt pth angle        (deg)
corr:
        if ((data & SSM) != SSM) break;
        return (data<<3>>13) * 6.866455078125e-4 + carry; // 180.0/(1<<18)

    case 0315:  // BNR - wind_speed           (knot)
        if ((data & SSM) != SSM) break;
        return (data<<3>>13) * 9.765625e-4 * KTS_MS; // no sign

    case 0323:  // BNR - flt pth accel        (G)
    case 0331:  // BNR - long_accel           (G)
    case 0332:  // BNR - lat_accel            (G)
    case 0333:  // BNR - normal_accel         (G)
    case 0362:  // BNR - along trk accel      (G)
    case 0363:  // BNR - cross trk accel      (G)
    case 0364:  // BNR - vertical_accel       (G)
    case 0375:  // BNR - along hdg accel      (G)
    case 0376:  // BNR - cross hdg accel      (G)
        if ((data & SSM) != SSM) break;
        return (data<<3>>13) * 1.52587890625e-5 * G_MPS2;

    case 0370:  // BNR - norm_accel           (G)
        if ((data & SSM) != SSM) break;
        return (data<<3>>13) * 3.0517578125e-5 * G_MPS2;

    case 0326:  // BNR - pitch_rate           (deg/s)
    case 0327:  // BNR - roll_rate            (deg/s)
    case 0330:  // BNR - yaw_rate             (deg/s)
    case 0336:  // BNR - pitch_att_rate       (deg/s)
    case 0337:  // BNR - roll_att_rate        (deg/s)
        if ((data & SSM) != SSM) break;
        return (data<<3>>13) * 4.8828125e-4;

    case 0335:  // BNR - track_ang_rate       (deg/s)
        if ((data & SSM) != SSM) break;
        if (data & 0x10000000) carry += 360.0;
        return (data<<3>>13) * 1.220703125e-4 + carry;

    case 0351:  // BCD - time_to_nav_ready    (min)
        if (((data & SSM) == NCD) || ((data & SSM) == TST)) break;
        return (
                ((data & (0xf<<18)) >> 18) * 1.0 +
                ((data & (0xf<<14)) >> 14) * 0.1
               ); // no sign

    case 0354:  // BNR - total time           (count)
        if ((data & SSM) != SSM) break;
        return (data<<4>>14) * 1.0; // no sign

    case 0245:  // BNR - hyb vert_speed       (ft/min)
    case 0360:  // BNR - pot_vert_speed       (ft/min)
    case 0365:  // BNR - vert_speed           (ft/min)
        if ((data & SSM) != SSM) break;
        return (data<<3>>13) * 0.125 * FPM_MPS;

    case 0261:  // BNR - 20 sig bits - hybrid inertial_alt  (ft)
    case 0361:  // BNR - 20 sig bits - inertial_alt         (ft)
        if ((data & SSM) != SSM) break;
        return (data<<3>>11) * 0.125 * FT_MTR;

    case 0264:  // BNR - 18 sig bits - hybrid Horiz FOM     (NM)
        if ((data & SSM) != SSM) break;
        return (data<<4>>14) * NM_MTR / (1<<14); // no sign

    case 0266:  // BNR - 15 sig bits - hybrid velocity_ns   (knot)
    case 0267:  // BNR - 15 sig bits - hybrid velocity_ew   (knot)
        if ((data & SSM) != SSM) break;
        return (data<<3>>16) * 0.125 * KTS_MS;

    case 0366:  // BNR - velocity_ns          (knot)
    case 0367:  // BNR - velocity_ew          (knot)
        if ((data & SSM) != SSM) break;
        return (data<<3>>13) * 0.015625 * KTS_MS;

    case 0226:  // DIS - Data Loader SAL      ()
    case 0270:  // DIS - irs_discretes        ()
    case 0277:  // DIS - Test Word            ()
    case 0350:  // DIS - irs_maint_discretes  ()
    case 0371:  // DIS - equipment_id         ()
    default:
        // unrecognized label type, return raw data
        *stype = UINT32_ST;
        return (data<<3>>13);
        break;
    }
    return doubleNAN;
}
