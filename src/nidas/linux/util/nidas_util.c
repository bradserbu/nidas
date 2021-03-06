/* -*- mode: C++; indent-tabs-mode: nil; c-basic-offset: 8; tab-width: 8; -*- */
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
/*

Module containing utility functions for NIDAS linux device drivers.

Original author:	Gordon Maclean

*/

#include <nidas/linux/util.h>
// #define DEBUG
#include <nidas/linux/klog.h>
#include <nidas/linux/Revision.h>    // REPO_REVISION

#include <linux/module.h>
#include <linux/init.h>

#include <linux/sched.h>
#include <linux/slab.h>		/* kmalloc, kfree */
#include <linux/uaccess.h>

#ifndef REPO_REVISION
#define REPO_REVISION "unknown"
#endif

MODULE_AUTHOR("Gordon Maclean <maclean@ucar.edu>");
MODULE_DESCRIPTION("NCAR nidas utilities");
MODULE_LICENSE("GPL");
MODULE_VERSION(REPO_REVISION);

/*
 * Allocate a circular buffer of dsm_samples.  If the size of one
 * sample is less than PAGE_SIZE, they are allocated in blocks of
 * size up to PAGE_SIZE.
 * dlen: length in bytes of the data portion of each sample.
 * blen: number of samples in the circular buffer.
 */
int alloc_dsm_circ_buf(struct dsm_sample_circ_buf* c,size_t dlen,int blen)
{
        int isamp = 0;
        int samps_per_page;
        int j,n;
        char *sp;

        /* count number of bits set, which should be one for a
         * power of 2.  Or check if first bit set
         * is the same as the last bit set: ffs(blen) == fls(blen)
         */
        if (blen == 0 || ffs(blen) != fls(blen)) {
                KLOG_ERR("circular buffer size=%d is not a power of 2\n",blen);
                return -EINVAL;
        }

        c->head = c->tail = c->size = c->npages = 0;
        c->pages = 0;

        KLOG_DEBUG("kmalloc %u bytes\n",blen * sizeof(void*));
        if (!(c->buf = kmalloc(blen * sizeof(void*),GFP_KERNEL))) return -ENOMEM;
        memset(c->buf,0,blen * sizeof(void*));

        /* Total size of a sample. Make it a multiple of sizeof(int)
         * so that samples are aligned to an int */
        dlen += SIZEOF_DSM_SAMPLE_HEADER;
        n = dlen % sizeof(int);
        if (n) dlen += sizeof(int) - n;

        samps_per_page = PAGE_SIZE / dlen;
        if (samps_per_page < 1) samps_per_page = 1;

        /* number of pages to allocate */
        n = (blen - 1) / samps_per_page + 1;
        if (!(c->pages = kmalloc(n * sizeof(void*),GFP_KERNEL))) {
                kfree(c->buf);
                c->buf = 0;
                return -ENOMEM;
        }
        memset(c->pages,0,n * sizeof(void*));
        c->npages = n;

        KLOG_INFO("sample len=%zu, buf len=%d, samps_per_page=%d, npages=%d\n",
                        dlen,blen,samps_per_page,n);

        for (n = 0; n < c->npages; n++) {
                j = blen - isamp;       /* left to allocate */
                if (j > samps_per_page) j = samps_per_page;
                sp = kmalloc(dlen * j,GFP_KERNEL);
                if (!sp) {
                        for (j = 0; j < n; j++) kfree(c->pages[j]);
                        kfree(c->pages);
                        c->pages = 0;
                        c->npages = 0;
                        kfree(c->buf);
                        c->buf = 0;
                        return -ENOMEM;
                }
                c->pages[n] = sp;

                for (j = 0; j < samps_per_page && isamp < blen; j++) {
                        c->buf[isamp++] = (struct dsm_sample*) sp;
                        sp += dlen;
                }
        }
        c->size = blen;
        smp_mb();
        return 0;
}

void free_dsm_circ_buf(struct dsm_sample_circ_buf* c)
{
        int i;
        if (c->pages) {
                for (i = 0; i < c->npages; i++) kfree(c->pages[i]);
        }
        kfree(c->pages);
        c->pages = 0;
        c->npages = 0;

        kfree(c->buf);
        c->buf = 0;

        c->size = 0;
        smp_mb();
}

int realloc_dsm_circ_buf(struct dsm_sample_circ_buf* c,size_t dlen,int blen)
{
        free_dsm_circ_buf(c);
        return alloc_dsm_circ_buf(c,dlen,blen);
}

void init_dsm_circ_buf(struct dsm_sample_circ_buf* c)
{
        c->head = c->tail = 0;
        smp_mb();
}

ssize_t
nidas_circbuf_read_nowait(struct file *filp, char __user* buf, size_t count,
                struct dsm_sample_circ_buf* cbuf, struct sample_read_state* state)
{
        size_t countreq = count;
        struct dsm_sample* insamp;
        size_t bytesLeft = state->bytesLeft;
        char* samplePtr = state->samplePtr;
        size_t n;

        for ( ; count; ) {
                if ((n = min(bytesLeft,count)) > 0) {
                        if (copy_to_user(buf,samplePtr,n)) return -EFAULT;
                        bytesLeft -= n;
                        count -= n;
                        samplePtr += n;
                        buf += n;
                        if (bytesLeft > 0) break;   // user buffer filled
                        samplePtr = 0;
                        INCREMENT_TAIL(*cbuf,cbuf->size);
                }
                insamp = GET_TAIL(*cbuf,cbuf->size);
                if (!insamp) break;    // no more samples
                samplePtr = (char*)insamp;
                bytesLeft = insamp->length + SIZEOF_DSM_SAMPLE_HEADER;
                KLOG_DEBUG("bytes left=%zd\n",bytesLeft);
        }
        state->samplePtr = samplePtr;
        state->bytesLeft = bytesLeft;
        KLOG_DEBUG("read return = %u\n",countreq - count);
        return countreq - count;
}


ssize_t
nidas_circbuf_read(struct file *filp, char __user* buf, size_t count,
                struct dsm_sample_circ_buf* cbuf, struct sample_read_state* state,
                wait_queue_head_t* readq)
{
        while(state->bytesLeft == 0 && ACCESS_ONCE(cbuf->head) == cbuf->tail) {
                if (filp->f_flags & O_NONBLOCK) return -EAGAIN;
                KLOG_DEBUG("waiting for data,head=%d,tail=%d\n",cbuf->head,cbuf->tail);
                if (wait_event_interruptible(*readq,(ACCESS_ONCE(cbuf->head) != cbuf->tail)))
                        return -ERESTARTSYS;
                KLOG_DEBUG("woken\n");
        }
        return nidas_circbuf_read_nowait(filp,buf,count,cbuf,state);
}

static void __exit nidas_util_cleanup(void)
{
        KLOG_DEBUG("nidas_util done\n");
        return;
}
static int __init nidas_util_init(void)
{	
        KLOG_NOTICE("version: %s\n",REPO_REVISION);
        return 0;
}

void screen_timetag_init(struct screen_timetag_data* td,
        int deltaT_Usec, int adjustUsec)
{
        // How often to make an adjustment to the time tags
        td->nptsCalc = adjustUsec / deltaT_Usec;

        td->dtTmsec = deltaT_Usec / USECS_PER_TMSEC;
        td->dtUsec = (deltaT_Usec % USECS_PER_TMSEC) * USECS_PER_TMSEC;
        td->samplesPerDay = TMSECS_PER_DAY / td->dtTmsec;

        KLOG_DEBUG("dtTmsec=%u, dtUsec=%u, samplesPerDay=%d\n",
                td->dtTmsec, td->dtUsec, td->samplesPerDay);
        td->nDt = -99999;
}

/**
 * Adjust time tags in a series which should have a fixed delta-T,
 * such as from an A2D.  The A2D driver modules assign the original
 * time tag based on the time of the read of the A2D FIFO by the
 * interrupt service routine or polling method. These time tags
 * may be late due to ISR or polling latency.
 *
 * If DO_TIMETAG_ERROR_AVERAGE is defined then this function keeps
 * a running mean of the differences between the original and the
 * expected time tags. This average is used to generate a time series
 * with (approximately) the requested delta-T, but which can slowly
 * drift to account for any clock drift of the A2D, or to correct
 * an error in the initial time tag in the series.
 *
 * If DO_TIMETAG_ERROR_AVERAGE is not defined then this function keeps
 * track of the minimum difference between the original and expected
 * time tags. After npts have been received, this minimum difference
 * is added to the added to the generated time tags. This will do
 * step changes (hopefully small) correct for clock drift between
 * the CPU and the A2D.
 *
 * The time tags are assumed to be in the raw form generated by the NIDAS
 * A2D driver modules, as the number of 1/10 milliseconds since 00:00 UTC.
 */
dsm_sample_time_t screen_timetag(struct screen_timetag_data* td, dsm_sample_time_t tt)
{
        unsigned int toff;
        int tterr;
        dsm_sample_time_t tt_est;
#ifdef DO_TIMETAG_ERROR_AVERAGE
        int sum;
#endif

        /* First point in time series, not adjusted. */
        if (td->nDt == -99999) {
                td->tt0 = tt;
                td->nDt = 1;
#ifdef DO_TIMETAG_ERROR_AVERAGE
                td->nSum = 0;
                td->errAvg = 0;
#else
                td->tdiffmin = TMSECS_PER_SEC;
                td->nmin = 0;
                td->nptsMin = 10;
#endif
                return tt;
        }

        /* offset from tt0, assuming a fixed delta-T */
        toff = (td->nDt * td->dtTmsec) +
                (td->nDt * td->dtUsec) / USECS_PER_TMSEC;

        if (toff > TMSECS_PER_DAY) {
                td->nDt -= td->samplesPerDay;
                toff = (td->nDt * td->dtTmsec) +
                    (td->nDt * td->dtUsec) / USECS_PER_TMSEC;
        }

        /* Expected time, based on a fixed delta-T from tt0 */
        tt_est = td->tt0 + toff;

        /* time tag difference between actual and expected */
        tterr = (tt - tt_est);

        /* Account for midnight rollover */
        if (tterr < -TMSECS_PER_DAY/2) tterr += TMSECS_PER_DAY;
        if (tterr >  TMSECS_PER_DAY/2) tterr -= TMSECS_PER_DAY;

#ifdef DO_TIMETAG_ERROR_AVERAGE
        /* compute average of differences. Convert to usecs to
         * improve precision.
         */
        tterr *= USECS_PER_TMSEC;

        /* running average of differences */
        if (td->nSum < td->nptsCalc) td->nSum++;
        sum = (td->nSum - 1) * td->errAvg + tterr;
        if (sum < 0) sum -= td->nSum / 2;   // round
        else sum += td->nSum / 2;

        td->errAvg = sum / td->nSum;

        /*
         * Add the smoothed difference to tt0, which could 
         * become negative around midnight. That's OK, since
         * dsm_sample_time_t is signed.
         */
        td->tt0 += td->errAvg / USECS_PER_TMSEC;
#else
        td->nmin++;
        td->tdiffmin = min(tterr, td->tdiffmin);

        if (td->nmin == td->nptsMin) {
                /* Add the minimum difference to tt0 */
                td->tt0 += td->tdiffmin;
                td->nmin = 0;
                td->tdiffmin = TMSECS_PER_SEC;
                td->nptsMin = td->nptsCalc;
        }

#endif
        if (td->tt0 >= TMSECS_PER_DAY)
                td->tt0 -= TMSECS_PER_DAY;

        td->nDt++;

#ifdef DO_TIMETAG_ERROR_AVERAGE
        KLOG_DEBUG("tt0=%u, nDt=%d, tterr=%d, errAvg=%d, nSum=%d, toff=%u\n",
            td->tt0, td->nDt, tterr, td->errAvg, td->nSum, toff);
#else
        KLOG_DEBUG("tt0=%u, nDt=%d, tterr=%d, tdiffmin=%d, nmin=%d, toff=%u\n",
            td->tt0, td->nDt, tterr, td->tdiffmin, td->nmin, toff);
#endif

        tt_est = td->tt0 + toff;
        if (tt_est >= TMSECS_PER_DAY)
                tt_est -= TMSECS_PER_DAY;
        return tt_est;
}

EXPORT_SYMBOL(alloc_dsm_circ_buf);
EXPORT_SYMBOL(free_dsm_circ_buf);
EXPORT_SYMBOL(realloc_dsm_circ_buf);
EXPORT_SYMBOL(init_dsm_circ_buf);
EXPORT_SYMBOL(nidas_circbuf_read);
EXPORT_SYMBOL(nidas_circbuf_read_nowait);
EXPORT_SYMBOL(screen_timetag_init);
EXPORT_SYMBOL(screen_timetag);

module_init(nidas_util_init);
module_exit(nidas_util_cleanup);
