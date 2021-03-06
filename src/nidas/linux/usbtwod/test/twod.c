/* -*- mode: C; indent-tabs-mode: nil; c-basic-offset: 8; tab-width: 8; -*- */
/* vim: set shiftwidth=8 softtabstop=8 expandtab: */
/*
 ********************************************************************
 ** NIDAS: NCAR In-situ Data Acquistion Software
 **
 ** 2007, Copyright University Corporation for Atmospheric Research
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
/* Test program to read 4k data records from the driver, add P2d_rec header
 * from header.h and then write the data to a disk file, viewable with xpms2d.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "../usbtwod.h"

#define P2D_DATA        4096            /* PMS 2D image buffer array size */

struct P2d_rec {
  short id;                             /* 'P1','C1','P2','C2', H1, H2 */
  short hour;
  short minute;
  short second;
  short spare1;
  short spare2;
  short spare3;
  short tas;                            /* true air speed */
  short msec;                           /* msec of this record */
  short overld;                         /* overload time, msec */
  unsigned char data[P2D_DATA];         /* image buffer */
};
typedef struct P2d_rec P2d_rec;


int fd;

int sendTAS(float tas)
{
  Tap2D tx_tas;
  float resolution = 25e-6;

  TASToTap2D(&tx_tas, tas, resolution);
  return ioctl(fd, USB2D_SET_TAS, (void*)&tx_tas);
}

int main()
{
  int i, n;
  FILE * fp_o = fopen("data.2d", "w+");
  P2d_rec rec;
  char buffer[5000];

  rec.id = htons(0x4331);

  while ((fd = open("/dev/usbtwod0", O_RDWR)) < 0)
  {
    usleep(10000);
//    printf("open failure\n");
//    exit(1);
  }

  printf("/dev/usbtwod opened\n");

  // Test sending tas.
  printf("send tas ioctl = %d\n", sendTAS(125.0));

  for (i = 0; i < 1000; )
  {
    n = read(fd, buffer, 4096+8);

    if (n == 4096+8)
    {
      unsigned long t = ((long *)buffer)[0];
      memcpy(rec.data, &buffer[sizeof(dsm_sample_t)], 4096);
//      printf("read n = %d\n", n);

      rec.msec = htons(t % 1000);
      t /= 1000;
      rec.hour = htons(t / 3600);
      t -= (t/3600) * 3600;
      rec.minute = htons(t / 60);
      t -= (t/60) * 60;
      rec.second = htons(t);

      fwrite(&rec, sizeof(rec), 1, fp_o);

      ++i;
//      usleep(10000);
    }
  }

  close(fd);
  fclose(fp_o);
  return 0;
}

