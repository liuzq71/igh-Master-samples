/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C)      2011  IgH Andreas Stewering-Bone
 *                     2012  Florian Pose <fp@igh-essen.com>
 *
 *  This file is part of the IgH EtherCAT master
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT master. If not, see <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *build:
   $g++ -o xmc4800_relax xmc4800_relax.cpp -I/opt/etherlab/include -I/opt/rtai-5.1_v4.9.80/include -L/opt/rtai-5.1_v4.9.80/lib -L/opt/etherlab/lib -llxrt -lrtdm -lethercat -lpthread -Wl,--rpath=/opt/rtai-5.1_v4.9.80/lib -Wl,--rpath=/opt/etherlab/lib
 *run:
 * $sudo insmod /opt/rtai-5.1_v4.9.80/modules/rtai_hal.ko
 * $sudo insmod /opt/rtai-5.1_v4.9.80/modules/rtai_sched.ko
 * $sudo insmod /opt/rtai-5.1_v4.9.80/modules/rtai_fifos.ko
 * $sudo insmod /opt/rtai-5.1_v4.9.80/modules/rtai_sem.ko
 * $sudo insmod /opt/rtai-5.1_v4.9.80/modules/rtai_shm.ko
 * $sudo insmod /opt/rtai-5.1_v4.9.80/modules/rtai_rtdm.ko
 * $sudo ./xmc4800_relax
 *****************************************************************************/

#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>

#include <rtai_lxrt.h>
#include <rtdm/rtdm.h>

#include "ecrt.h"

#include <vector>
#include <iostream>
using namespace std;

#define rt_printf(X, Y)

#define NSEC_PER_SEC 1000000000

RT_TASK *task;

static unsigned int cycle_ns = 1000000; /* 1 ms */

static int run = 1;

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static uint8_t *domain1_pd = NULL;

static ec_slave_config_t *sc_dig_out_01 = NULL;

/****************************************************************************/

// EtherCAT distributed clock variables

#define DC_FILTER_CNT          1024
#define SYNC_MASTER_TO_REF        1

static uint64_t dc_start_time_ns = 0LL;
static uint64_t dc_time_ns = 0;
#if SYNC_MASTER_TO_REF
static uint8_t  dc_started = 0;
static int32_t  dc_diff_ns = 0;
static int32_t  prev_dc_diff_ns = 0;
static int64_t  dc_diff_total_ns = 0LL;
static int64_t  dc_delta_total_ns = 0LL;
static int      dc_filter_idx = 0;
static int64_t  dc_adjust_ns;
#endif
static int64_t  system_time_base = 0LL;
static uint64_t wakeup_time = 0LL;
static uint64_t overruns = 0LL;

/****************************************************************************/

// process data

#define BusCoupler01_Pos  0, 0
#define DigOutSlave01_Pos 0, 2
#define Xmc4800_relax_Pos 0, 0

#define SLAVE_POSITION(Alias,Position)  Alias,Position

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
//#define Beckhoff_EL2004 0x00000002, 0x07d43052
#define Beckhoff_EL2008 0x00000002, 0x07d83052

#define XMC4800_RELAX  0x0000034e, 0x00000000
#define SLAVE_VENDORID(VendorId,ProductCode) VendorId,ProductCode

#define INFINEON_ID 0x34E
#define XMC4800_RELAX_PRODUCT_CODE1 0x00

// offsets for PDO entries
static unsigned int off_dig_out0 = 0;

/*x86_64: char-1byte,int-4bytes,long-8bytes*/
struct XMC4800 {
   //RxPdo  0x1600
   unsigned int out_gen_int1; //0x7000,subindex:1,bitlen:16
   unsigned int out_gen_int2; //0x7000,subindex:2,bitlen:16
   unsigned int out_gen_int3; //0x7000,subindex:3,bitlen:16
   unsigned int out_gen_int4; //0x7000,subindex:4,bitlen:16
   unsigned int out_gen_bit1; //0x7000,subindex:5,bitlen:1
   
   //TxPdo  0x1a00
   unsigned int in_gen_int1; //0x6000,subindex:1,bitlen:16
   unsigned int in_gen_int2; //0x6000,subindex:2,bitlen:16
   unsigned int in_gen_int3; //0x6000,subindex:3,bitlen:16
   unsigned int in_gen_int4; //0x6000,subindex:4,bitlen:16
   unsigned int in_gen_bit1; //0x6000,subindex:5,bitlen:1
};

// process data

static struct XMC4800 xmc4800;

#if 0
const static ec_pdo_entry_reg_t domain1_regs[] = {
   //{DigOutSlave01_Pos, Beckhoff_EL2004, 0x7000, 0x01, &off_dig_out0, NULL},
   /*{DigOutSlave01_Pos, Beckhoff_EL2008, 0x7000, 0x01, &off_dig_out0, NULL},*/
//RXPDO,0x1600 - 对于主站来说是Tx,对于本从站来说是Rx
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x01, &xmc4800.out_gen_int1,NULL},
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x02, &xmc4800.out_gen_int2,NULL},
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x03, &xmc4800.out_gen_int3,NULL},
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x04, &xmc4800.out_gen_int4,NULL},
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x05, &xmc4800.out_gen_bit1,NULL},
//TXPDO,0x1A00 - 对于主站来说是Rx,对于本从站来说是Tx   
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x01, &xmc4800.in_gen_int1,NULL},
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x02, &xmc4800.in_gen_int2,NULL},
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x03, &xmc4800.in_gen_int3,NULL},
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x04, &xmc4800.in_gen_int4,NULL},
   {Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x05, &xmc4800.in_gen_bit1,NULL},
   {}
};
#endif 

static ec_pdo_entry_reg_t *domain1_regs;

//重新设置eeprom内的PDO entry,如果使用Fixed PDOs则不需要这3个数组
static ec_pdo_entry_info_t XMC4800_pdo_entries[] = {
//XMC4800 RxPdo 0x1600, output to slaves
   {0x7000, 0x01, 16}, 
   {0x7000, 0x02, 16},
   {0x7000, 0x03, 16},
   {0x7000, 0x04, 16},
   {0x7000, 0x05, 1},
   {0x7000, 0x06, 1},
   {0x7000, 0x07, 1},
   {0x7000, 0x08, 1},
   {0x7000, 0x09, 1},
   {0x7000, 0x0a, 1},
   {0x7000, 0x0b, 1},
   {0x7000, 0x0c, 1},
//XMC4800 TxPdo 0x1a00, input from slaves
   {0x6000, 0x01, 16},
   {0x6000, 0x02, 16},
   {0x6000, 0x03, 16},
   {0x6000, 0x04, 16},
   {0x6000, 0x05, 1},
   {0x6000, 0x06, 1},
   {0x6000, 0x07, 1},
   {0x6000, 0x08, 1},
   {0x6000, 0x09, 1},
   {0x6000, 0x0a, 1},
   {0x6000, 0x0b, 1},
   {0x6000, 0x0c, 1},
};

static ec_pdo_info_t XMC4800_pdos[] = {
   //RxPdo
   {0x1600, 12, XMC4800_pdo_entries + 0},
   //TxPdo
   {0x1a00, 12, XMC4800_pdo_entries + 12}
};

/*
{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
*/

static ec_sync_info_t XMC4800_syncs[] = {
   { 0, EC_DIR_OUTPUT, 0 ,NULL, EC_WD_DISABLE },
   { 1, EC_DIR_INPUT, 0 ,NULL, EC_WD_DISABLE },
   { 2, EC_DIR_OUTPUT, 1 ,XMC4800_pdos + 0 ,EC_WD_ENABLE },//主站是output
   { 3, EC_DIR_INPUT, 1 ,XMC4800_pdos + 1 ,EC_WD_DISABLE },//主站是intput
   {0xFF}
};
/****************************************************************************/

/* Slave 1, "EL2004"
 * Vendor ID:       0x00000002
 * Product code:    0x07d43052
 * Revision number: 0x00100000
 */

/* Slave 1, "EL2008"
 * Vendor ID:       0x00000002
 * Product code:    0x07d83052
 * Revision number: 0x00100000
 */
ec_pdo_entry_info_t slave_1_pdo_entries[] = {
   {0x7000, 0x01, 1}, /* Output */
   {0x7010, 0x01, 1}, /* Output */
   {0x7020, 0x01, 1}, /* Output */
   {0x7030, 0x01, 1}, /* Output */
   {0x7040, 0x01, 1}, /* Output */
   {0x7050, 0x01, 1}, /* Output */
   {0x7060, 0x01, 1}, /* Output */
   {0x7070, 0x01, 1}, /* Output */
};

ec_pdo_info_t slave_1_pdos[] = {
   {0x1600, 1, slave_1_pdo_entries + 0}, /* Channel 1 */
   {0x1601, 1, slave_1_pdo_entries + 1}, /* Channel 2 */
   {0x1602, 1, slave_1_pdo_entries + 2}, /* Channel 3 */
   {0x1603, 1, slave_1_pdo_entries + 3}, /* Channel 4 */
   {0x1604, 1, slave_1_pdo_entries + 4}, /* Channel 5 */
   {0x1605, 1, slave_1_pdo_entries + 5}, /* Channel 6 */
   {0x1606, 1, slave_1_pdo_entries + 6}, /* Channel 7 */
   {0x1607, 1, slave_1_pdo_entries + 7}, /* Channel 8 */
};

ec_sync_info_t slave_1_syncs[] = {
   {0, EC_DIR_OUTPUT, 8, slave_1_pdos + 0, EC_WD_ENABLE},
   {0xff}
};

//ec_pdo_entry_reg_t *aa;

/*****************************************************************************
 * Realtime task
 ****************************************************************************/

/** Get the time in ns for the current cpu, adjusted by system_time_base.
 *
 * \attention Rather than calling rt_get_time_ns() directly, all application
 * time calls should use this method instead.
 *
 * \ret The time in ns.
 */
uint64_t system_time_ns(void)
{
    RTIME time = rt_get_time_ns();

    if (system_time_base > time) {
        rt_printk("%s() error: system_time_base greater than"
                " system time (system_time_base: %lld, time: %llu\n",
                __func__, system_time_base, time);
        return time;
    }
    else {
        return time - system_time_base;
    }
}

/****************************************************************************/

/** Convert system time to RTAI time in counts (via the system_time_base).
 */
RTIME system2count(
        uint64_t time
        )
{
    RTIME ret;

    if ((system_time_base < 0) &&
            ((uint64_t) (-system_time_base) > time)) {
        rt_printk("%s() error: system_time_base less than"
                " system time (system_time_base: %lld, time: %llu\n",
                __func__, system_time_base, time);
        ret = time;
    }
    else {
        ret = time + system_time_base;
    }

    return nano2count(ret);
}

/*****************************************************************************/

/** Synchronise the distributed clocks
 */
void sync_distributed_clocks(void)
{
#if SYNC_MASTER_TO_REF
    uint32_t ref_time = 0;
    uint64_t prev_app_time = dc_time_ns;
#endif

    dc_time_ns = system_time_ns();

    // set master time in nano-seconds
    ecrt_master_application_time(master, dc_time_ns);

#if SYNC_MASTER_TO_REF
    // get reference clock time to synchronize master cycle
    ecrt_master_reference_clock_time(master, &ref_time);
    dc_diff_ns = (uint32_t) prev_app_time - ref_time;
#else
    // sync reference clock to master
    ecrt_master_sync_reference_clock(master);
#endif

    // call to sync slaves to ref slave
    ecrt_master_sync_slave_clocks(master);
}

/*****************************************************************************/

/** Return the sign of a number
 *
 * ie -1 for -ve value, 0 for 0, +1 for +ve value
 *
 * \retval the sign of the value
 */
#define sign(val) \
    ({ typeof (val) _val = (val); \
    ((_val > 0) - (_val < 0)); })

/*****************************************************************************/

/** Update the master time based on ref slaves time diff
 *
 * called after the ethercat frame is sent to avoid time jitter in
 * sync_distributed_clocks()
 */
void update_master_clock(void)
{
#if SYNC_MASTER_TO_REF
    // calc drift (via un-normalised time diff)
    int32_t delta = dc_diff_ns - prev_dc_diff_ns;
    prev_dc_diff_ns = dc_diff_ns;

    // normalise the time diff
    dc_diff_ns =
        ((dc_diff_ns + (cycle_ns / 2)) % cycle_ns) - (cycle_ns / 2);

    // only update if primary master
    if (dc_started) {

        // add to totals
        dc_diff_total_ns += dc_diff_ns;
        dc_delta_total_ns += delta;
        dc_filter_idx++;

        if (dc_filter_idx >= DC_FILTER_CNT) {
            // add rounded delta average
            dc_adjust_ns +=
                ((dc_delta_total_ns + (DC_FILTER_CNT / 2)) / DC_FILTER_CNT);

            // and add adjustment for general diff (to pull in drift)
            dc_adjust_ns += sign(dc_diff_total_ns / DC_FILTER_CNT);

            // limit crazy numbers (0.1% of std cycle time)
            if (dc_adjust_ns < -1000) {
                dc_adjust_ns = -1000;
            }
            if (dc_adjust_ns > 1000) {
                dc_adjust_ns =  1000;
            }

            // reset
            dc_diff_total_ns = 0LL;
            dc_delta_total_ns = 0LL;
            dc_filter_idx = 0;
        }

        // add cycles adjustment to time base (including a spot adjustment)
        system_time_base += dc_adjust_ns + sign(dc_diff_ns);
    }
    else {
        dc_started = (dc_diff_ns != 0);

        if (dc_started) {
            // output first diff
            rt_printk("First master diff: %d.\n", dc_diff_ns);

            // record the time of this initial cycle
            dc_start_time_ns = dc_time_ns;
        }
    }
#endif
}

/****************************************************************************/

void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state) {
        rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void rt_check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

/** Wait for the next period
 */
void wait_period(void)
{
    while (1)
    {
        RTIME wakeup_count = system2count(wakeup_time);
        RTIME current_count = rt_get_time();

        if ((wakeup_count < current_count)
                || (wakeup_count > current_count + (50 * cycle_ns))) {
            rt_printk("%s(): unexpected wake time!\n", __func__);
        }

        switch (rt_sleep_until(wakeup_count)) {
            case RTE_UNBLKD:
                rt_printk("rt_sleep_until(): RTE_UNBLKD\n");
                continue;

            case RTE_TMROVRN:
                rt_printk("rt_sleep_until(): RTE_TMROVRN\n");
                overruns++;

                if (overruns % 100 == 0) {
                    // in case wake time is broken ensure other processes get
                    // some time slice (and error messages can get displayed)
                    rt_sleep(cycle_ns / 100);
                }
                break;

            default:
                break;
        }

        // done if we got to here
        break;
    }

    // calc next wake time (in sys time)
    wakeup_time += cycle_ns;
}

/****************************************************************************/

void my_cyclic(void)
{
    int cycle_counter = 0;
    unsigned int blink = 0;

    // oneshot mode to allow adjustable wake time
    rt_set_oneshot_mode();

    // set first wake time in a few cycles
    wakeup_time = system_time_ns() + 10 * cycle_ns;

    // start the timer
    start_rt_timer(nano2count(cycle_ns));

    rt_make_hard_real_time();

    while (run) {
        // wait for next period (using adjustable system time)
        wait_period();

        cycle_counter++;

        if (!run) {
            break;
        }

        // receive EtherCAT
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        rt_check_domain_state();

        if (!(cycle_counter % 1000)) {
            rt_check_master_state();
        }

        if (!(cycle_counter % 200)) {
            blink = !blink;
        }

        //EC_WRITE_U8(domain1_pd + off_dig_out0, blink ? 0x00 : 0x0FF); //输出到El2008
        EC_WRITE_U32(domain1_pd + xmc4800.out_gen_bit1, blink ? 0x55 : 0xaa); //输出到XMC4800_Relax开发板
        //EC_WRITE_U16(domain1_pd + off_dig_out0,1);
        //fprintf(stdout,"%d\n",EC_READ_U32(domain1_pd+xmc4800.out_gen_bit1));
        // queue process data
        ecrt_domain_queue(domain1);

        // sync distributed clock just before master_send to set
        // most accurate master clock time
        sync_distributed_clocks();

        // send EtherCAT data
        ecrt_master_send(master);

        // update the master clock
        // Note: called after ecrt_master_send() to reduce time
        // jitter in the sync_distributed_clocks() call
        update_master_clock();
    }

    rt_make_soft_real_time();
    stop_rt_timer();
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/****************************************************************************
 * Main function
 ***************************************************************************/

int main(int argc, char *argv[])
{
    ec_slave_config_t *sc_ek1100;
    ec_slave_config_t *sc_xmc4800_relax;
    int ret;

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }

    printf("Creating slave configurations...\n");

/*    // Create configuration for bus coupler
    sc_ek1100 =
        ecrt_master_slave_config(master, BusCoupler01_Pos, Beckhoff_EK1100);
    if (!sc_ek1100) {
        return -1;
    }*/

    /*sc_dig_out_01 =
        ecrt_master_slave_config(master, DigOutSlave01_Pos, Beckhoff_EL2004);
    if (!sc_dig_out_01) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }*/

/*    sc_dig_out_01 =
        ecrt_master_slave_config(master, DigOutSlave01_Pos, Beckhoff_EL2008);
    if (!sc_dig_out_01) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }*/

    /*//设置EL2008　eeprom
    if (ecrt_slave_config_pdos(sc_dig_out_01, EC_END, slave_1_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }*/

/*    //设置PDO映射
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }*/

    //配置xmc4800_relax
    if (!(sc_xmc4800_relax = ecrt_master_slave_config(
                    master, Xmc4800_relax_Pos, XMC4800_RELAX))) {
        fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
    }

    printf("Configuring PDOs...\n");

    if (ecrt_slave_config_pdos(sc_xmc4800_relax, EC_END, XMC4800_syncs)) {
        fprintf(stderr, "Failed to configure xmc4800_relax PDOs.\n");
        return -1;
    }

    /*a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x01, &xmc4800.out_gen_int1,NULL});
    a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x02, &xmc4800.out_gen_int2,NULL});
    a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x03, &xmc4800.out_gen_int3,NULL});
    a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x04, &xmc4800.out_gen_int4,NULL});
    a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x7000, 0x05, &xmc4800.out_gen_bit1,NULL});
   
    a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x01, &xmc4800.in_gen_int1,NULL});
    a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x02, &xmc4800.in_gen_int2,NULL});
    a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x03, &xmc4800.in_gen_int3,NULL});
    a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x04, &xmc4800.in_gen_int4,NULL});
    a.push_back({Xmc4800_relax_Pos, XMC4800_RELAX, 0x6000, 0x05, &xmc4800.in_gen_bit1,NULL});
    a.push_back({});*/

    //动态生成domain1_regs数组
    domain1_regs = new ec_pdo_entry_reg_t[11]();
    
    domain1_regs[0]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x7000, 0x01, &xmc4800.out_gen_int1,NULL};
    domain1_regs[1]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x7000, 0x02, &xmc4800.out_gen_int2,NULL};
    domain1_regs[2]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x7000, 0x03, &xmc4800.out_gen_int3,NULL};
    domain1_regs[3]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x7000, 0x04, &xmc4800.out_gen_int4,NULL};
    domain1_regs[4]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x7000, 0x05, &xmc4800.out_gen_bit1,NULL};
   
    domain1_regs[5]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x6000, 0x01, &xmc4800.in_gen_int1,NULL};
    domain1_regs[6]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x6000, 0x02, &xmc4800.in_gen_int2,NULL};
    domain1_regs[7]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x6000, 0x03, &xmc4800.in_gen_int3,NULL};
    domain1_regs[8]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x6000, 0x04, &xmc4800.in_gen_int4,NULL};
    domain1_regs[9]={SLAVE_POSITION(0,0),SLAVE_VENDORID(INFINEON_ID,XMC4800_RELAX_PRODUCT_CODE1), 0x6000, 0x05, &xmc4800.in_gen_bit1,NULL};
    domain1_regs[10]={};
    
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "xmc4800_relax PDO entry registration failed!\n");
        return -1;
    }

    /*Registers a PDO entry for process data exchange in domain*/
/*    if(ecrt_slave_config_reg_pdo_entry(sc_xmc4800_relax, 0x7000, 0x05, domain1, &xmc4800.out_gen_bit1) < 0) {
       fprintf(stderr,"PDO entry (0x7000:0x05) registration failed!\n");
       return -1;
    }*/
    
    //配置xmc4800_relax结束
    /* Set the initial master time and select a slave to use as the DC
     * reference clock, otherwise pass NULL to auto select the first capable
     * slave. Note: This can be used whether the master or the ref slave will
     * be used as the systems master DC clock.
     */
    dc_start_time_ns = system_time_ns();
    dc_time_ns = dc_start_time_ns;

    /* Attention: The initial application time is also used for phase
     * calculation for the SYNC0/1 interrupts. Please be sure to call it at
     * the correct phase to the realtime cycle.
     */
    ecrt_master_application_time(master, dc_start_time_ns);

    ret = ecrt_master_select_reference_clock(master, sc_xmc4800_relax/*sc_ek1100*/);
    if (ret < 0) {
        fprintf(stderr, "Failed to select reference clock: %s\n",
                strerror(-ret));
        return ret;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }

    /*ecrt_slave_config_sdo8(sc_xmc4800_relax,0x7000,0x0c,1);
      ecrt_slave_config_sdo32(sc_xmc4800_relax,0x7000,1,1);
      ecrt_slave_config_sdo8(sc_xmc4800_relax,0x7000,0x5,1);
      ecrt_slave_config_sdo8(sc_xmc4800_relax,0x7000,0x6,1);*/

    /* Create cyclic RT-thread */
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        puts("ERROR IN SETTING THE SCHEDULER");
        perror("errno");
        return -1;
    }

    task = rt_task_init(nam2num("ec_rtai_rtdm_example"),
            0 /* priority */, 0 /* stack size */, 0 /* msg size */);

    my_cyclic();

    rt_task_delete(task);

    printf("End of Program\n");
    ecrt_release_master(master);
    
    delete[] domain1_regs;

    return 0;
}

/***********************************NO MORE**********************************/

