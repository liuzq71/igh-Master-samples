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
 *  这个例子实时性更好,在preempt rt下,使用ec_generic.ko都跑得不停顿(不发生ER.E08错)
 *  gcc -o is620n_dc_rt_csv is620n_dc_rt_csv.c -I/opt/etherlab/include -L. -L/opt/etherlab/lib -Wl,--rpath=./ -Wl,--rpath=/opt/etherlab/lib -lethercat -lrt -lipipe_64
 *****************************************************************************/

#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
//#include <rtai_lxrt.h>
//#include <rtdm/rtdm.h>

#include <sys/mman.h>

#include "ecrt.h"

#include <time.h>
#include <errno.h>
#include "ipipe_64.h"

//#define rt_printf(X, Y)

#define CLOCK_TO_USE CLOCK_MONOTONIC //CLOCK_REALTIME
#define NSEC_PER_SEC 1000000000

//RT_TASK *task;

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
#define DigOutSlave01_Pos 0, 1

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
//#define Beckhoff_EL2004 0x00000002, 0x07d43052
#define Beckhoff_EL2008 0x00000002, 0x07d83052

// offsets for PDO entries
static unsigned int off_dig_out0 = 0;

// process data
#if 0
const static ec_pdo_entry_reg_t domain1_regs[] = {
   //{DigOutSlave01_Pos, Beckhoff_EL2004, 0x7000, 0x01, &off_dig_out0, NULL},
     {DigOutSlave01_Pos, Beckhoff_EL2008, 0x7000, 0x01, &off_dig_out0, NULL},
   {}
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
};

ec_pdo_info_t slave_1_pdos[] = {
   {0x1600, 1, slave_1_pdo_entries + 0}, /* Channel 1 */
   {0x1601, 1, slave_1_pdo_entries + 1}, /* Channel 2 */
   {0x1602, 1, slave_1_pdo_entries + 2}, /* Channel 3 */
   {0x1603, 1, slave_1_pdo_entries + 3}, /* Channel 4 */
};

ec_sync_info_t slave_1_syncs[] = {
   {0, EC_DIR_OUTPUT, 4, slave_1_pdos + 0, EC_WD_ENABLE},
   {0xff}
};
#endif

#define IS620NSlavePos         0,0           /*EtherCAT address on the bus*/
#define INOVANCE_IS620N        0x00100000, 0x000C0108    /*IS620N Vendor ID, product code*/

/*x86_64: char-1byte,int-4bytes,long-8bytes*/
struct IS620N_offset {
   //RxPDO
   unsigned int control_word_6040_0; //0x6040:控制字,subindex:0,bitlen:16
   unsigned int target_position_607a_0;//0x607A:目标位置,subindex:0,bitlen:32
   unsigned int touch_probe_60b8_0; //0x60B8:探针,subindex:0,bitlen:16
   unsigned int pysical_outputs_60fe_1;//0x60FE:pysical_outputs,subindex:1,bitlen:32
   unsigned int target_velocity_60ff_0;//0x60FF:target_velocity,subindex:0,bitlen:32
   unsigned int target_torque_6071_0;//0x6071:int target_torque,subindex:0,bitlen:16
   unsigned int modes_operation_6060_0;//0x6060:Modes of operation,subindex:0,bitlen:8
   unsigned int max_profile_velocity_607f_0;//0x607F:max_profile_velocity,subindex:0,bitlen:32
   unsigned int positive_torque_limit_value_60e0_0;//0x60E0:positive_torque_limit_value,subindex:0,bitlen:16
   unsigned int negative_torque_limit_value_60e1_0;//0x60E1:negaitive_torque_limit_value,subindex:0,bitlen:16
   unsigned int torque_offset_60b2_0;//0x60B2:torque offset,subindex:0,bitlen:16
   unsigned int max_torque_6072_0;//0x6072:max torque,subindex:0,bitlen:16

   //TxPDo
   unsigned int status_word_6041_0;//0x6041:status_word,subindex:0,bitlen:16
   unsigned int position_actual_value_6064_0;//0x6064:position_actual_value,subindex:0,bitlen:32
   unsigned int touch_probe_status_60b9_0;//0x60B9,subindex:0,bitlen:16
   unsigned int touch_probe_pos1_pos_value_60ba_0;//0x60BA,subindex:0,bitlen:32
   unsigned int touch_probe_pos2_pos_value_60bc_0;//0x60BC ,subindex:0,bitlen:32
   unsigned int error_code_603f_0;//0x603F,subindex:0,bitlen:16
   unsigned int digital_inputs_60fd_0;//0x60FD,subindex:0,bitlen:32
   unsigned int torque_actual_value_6077_0;//0x6077,subindex:0,bitlen:16
   unsigned int following_error_actual_value_60f4_0;//0x60F4,subindex:0,bitlen:32
   unsigned int modes_of_operation_display_6061_0;//0x6061,subindex:0,bitlen:8
   unsigned int velocity_actual_value_606c_0;//0x606C,subindex:0,bitlen:32
};

static struct IS620N_offset offset;

const static ec_pdo_entry_reg_t domain1_regs[] = {
    //RxPDO
    {IS620NSlavePos, INOVANCE_IS620N, 0x6040, 0, &offset.control_word_6040_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x607A, 0, &offset.target_position_607a_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x60FF, 0, &offset.target_velocity_60ff_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x6071, 0, &offset.target_torque_6071_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x6060, 0, &offset.modes_operation_6060_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x60B8, 0, &offset.touch_probe_60b8_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x607F, 0, &offset.max_profile_velocity_607f_0},
    //TxPDO
    {IS620NSlavePos, INOVANCE_IS620N, 0x603F, 0, &offset.error_code_603f_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x6041, 0, &offset.status_word_6041_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x6064, 0, &offset.position_actual_value_6064_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x6077, 0, &offset.torque_actual_value_6077_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x6061, 0, &offset.modes_of_operation_display_6061_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x60B9, 0, &offset.touch_probe_status_60b9_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x60BA, 0, &offset.touch_probe_pos1_pos_value_60ba_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x60BC, 0, &offset.touch_probe_pos2_pos_value_60bc_0},
    {IS620NSlavePos, INOVANCE_IS620N, 0x60FD, 0, &offset.digital_inputs_60fd_0},
    {}
};

/*重新映射PDO才需要定义下面这3个数组,如果使用servo的固定映射则不需要*/

/*Config PDOs*/
static ec_pdo_entry_info_t IS620N_pdo_entries[] = {
    //RxPdo 0x1702
    {0x6040, 0x00, 16},
    {0x607A, 0x00, 32},
    {0x60FF, 0x00, 32}, 
    {0x6071, 0x00, 16},
    {0x6060, 0x00, 8}, 
    {0x60B8, 0x00, 16},
    {0x607F, 0x00, 32},
    //TxPdo 0x1B02
    {0x603F, 0x00, 16},
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x60B9, 0x00, 16},
    {0x60BA, 0x00, 32},
    {0x60BC, 0x00, 32},
    {0x60FD, 0x00, 32},
};

static ec_pdo_info_t IS620N_pdos[] = {
    //RxPdo
    {0x1600, 7, IS620N_pdo_entries + 0 },
    //TxPdo
    {0x1A00, 9, IS620N_pdo_entries + 7 }
};

static ec_sync_info_t IS620N_syncs[] = {
    /*{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
    { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },*/
    { 2, EC_DIR_OUTPUT, 1, IS620N_pdos + 0/*,EC_WD_ENABLE*//*, EC_WD_DISABLE*/ },
    { 3, EC_DIR_INPUT, 1, IS620N_pdos + 1/*,EC_WD_DISABLE*//*, EC_WD_DISABLE*/ },
    { 0xFF}
};

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
        /*rt_printk("%s() error: system_time_base greater than"
                " system time (system_time_base: %lld, time: %llu\n",
                __func__, system_time_base, time);*/
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
        /*rt_printk("%s() error: system_time_base less than"
                " system time (system_time_base: %lld, time: %llu\n",
                __func__, system_time_base, time);*/
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

#if SYNC_MASTER_TO_REF
    // get reference clock time to synchronize master cycle
    ecrt_master_reference_clock_time(master, &ref_time);
    dc_diff_ns = (uint32_t) prev_app_time - ref_time;
#else
    // sync reference clock to master
    ecrt_master_sync_reference_clock_to(master, dc_time_ns);
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
            //rt_printk("First master diff: %d.\n", dc_diff_ns);

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
        //rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state) {
        //rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void rt_check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        //rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        //rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        //rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

/** Wait for the next period
 */

#if 0
void wait_period(void)
{
    while (1)
    {
        RTIME wakeup_count = system2count(wakeup_time);
        RTIME current_count = rt_get_time();

        if ((wakeup_count < current_count)
                || (wakeup_count > current_count + (50 * cycle_ns))) {
            //rt_printk("%s(): unexpected wake time!\n", __func__);
        }

        /*switch (rt_sleep_until(wakeup_count)) {
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
        }*/

        // done if we got to here
        break;
    }

    // set master time in nano-seconds
    ecrt_master_application_time(master, wakeup_time);

    // calc next wake time (in sys time)
    wakeup_time += cycle_ns;
}
#endif


void wait_period(void)
{
    while (1)
    {
        RTIME wakeup_count = system2count(wakeup_time);
        RTIME current_count = rt_get_time();

        if ((wakeup_count < current_count)
                || (wakeup_count > current_count + (50 * cycle_ns))) {
            //rt_printk("%s(): unexpected wake time!\n", __func__);
        }

	struct timespec wakeupTime;
	/*wakeupTime.tv_sec= wakeup_time / NSEC_PER_SEC;
	wakeupTime.tv_nsec= wakeup_time % NSEC_PER_SEC;
	
        int s =clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);*/

	wakeupTime.tv_sec= cycle_ns / NSEC_PER_SEC;
	wakeupTime.tv_nsec= cycle_ns % NSEC_PER_SEC;
	
        int s =clock_nanosleep(CLOCK_TO_USE, 0, &wakeupTime, NULL);

        if (s != 0) { //避免过度睡眠
            if (s == EINTR)
               printf("Interrupted by signal handler\n");
            else
               printf("clock_nanosleep:errno=%d\n",s);
        }

        // done if we got to here
        break;
    }

    // set master time in nano-seconds
    ecrt_master_application_time(master, wakeup_time);

    // calc next wake time (in sys time)
    wakeup_time += cycle_ns;
}
/****************************************************************************/

void my_cyclic(void)
{
    int cycle_counter = 0;
    unsigned int blink = 0;
   
    signed long temp[8]={};

    // oneshot mode to allow adjustable wake time
    //rt_set_oneshot_mode();

    // set first wake time in a few cycles
    wakeup_time = system_time_ns() + 10 * cycle_ns;

    // start the timer
    //start_rt_timer(nano2count(cycle_ns));

    //rt_make_hard_real_time();

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

        //EC_WRITE_U8(domain1_pd + off_dig_out0, blink ? 0x00 : 0x0F);

	//is620n control
	temp[0]=EC_READ_U16(domain1_pd + offset.status_word_6041_0);
        temp[1]=EC_READ_U32(domain1_pd + offset.position_actual_value_6064_0);
	//printf("\r%6f    \t    ",((float)temp[1]/1000) );
      
        // write process data
        /*if(servooff==1){//servo off
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x0006 );
            //deactive++;
        }
        else*/if(temp[0] == 0x218){
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x0080 );
        } 
        else if( (temp[0]&0x04f) == 0x0040  ){  //temp[0]==0x250
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x0006 );
            //printf("1.state = %x\n",temp[0]);
        }
        else if( (temp[0]&0x06f) == 0x0021){ //temp[0]==0x231
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x0007 );
            //printf("2.state = %x\n",temp[0]);
        }
        else if( (temp[0]&0x06f) == 0x023){ //temp[0]==0x233
            EC_WRITE_S8(domain1_pd+offset.modes_operation_6060_0, 9);//csv mode
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x000f);
            //printf("3.state = %x\n",temp[0]);
        }
        else if( (temp[0]&0x06f) == 0x027){//600 800 //temp[0]=0x237
            EC_WRITE_U32(domain1_pd+offset.target_velocity_60ff_0, 0x1000000);
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x001f);
            //printf("4.state = %x\n",temp[0]);
        }
	//is620n control end

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

    //rt_make_soft_real_time();
    //stop_rt_timer();
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
    //ec_slave_config_t *sc_ek1100;
    ec_slave_config_t *sc_is620n;
    int ret;

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }

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

#if 0
    // Create configuration for bus coupler
    sc_ek1100 =
        ecrt_master_slave_config(master, BusCoupler01_Pos, Beckhoff_EK1100);
    if (!sc_ek1100) {
        return -1;
    }

    /*sc_dig_out_01 =
        ecrt_master_slave_config(master, DigOutSlave01_Pos, Beckhoff_EL2004);
    if (!sc_dig_out_01) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }*/

    sc_dig_out_01 =
        ecrt_master_slave_config(master, DigOutSlave01_Pos, Beckhoff_EL2008);
    if (!sc_dig_out_01) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    //设置EL2008　eeprom
    if (ecrt_slave_config_pdos(sc_dig_out_01, EC_END, slave_1_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    //设置PDO映射
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }
#endif

//配置IS620
    if (!(sc_is620n = ecrt_master_slave_config(
                    master, IS620NSlavePos, INOVANCE_IS620N))) {
	fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
    }

    printf("Configuring PDOs...\n");
	
    if (ecrt_slave_config_pdos(sc_is620n, EC_END, IS620N_syncs)) {
        fprintf(stderr, "Failed to configure is620n PDOs.\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "is620n PDO entry registration failed!\n");
        return -1;
    }	
//配置IS620结束

    /* Set the initial master time and select a slave to use as the DC
     * reference clock, otherwise pass NULL to auto select the first capable
     * slave. Note: This can be used whether the master or the ref slave will
     * be used as the systems master DC clock.
     */
    dc_start_time_ns = system_time_ns();
    dc_time_ns = dc_start_time_ns;

#if 0
    ret = ecrt_master_select_reference_clock(master, sc_ek1100);
    if (ret < 0) {
        fprintf(stderr, "Failed to select reference clock: %s\n",
                strerror(-ret));
        return ret;
    }
#endif

    ecrt_slave_config_dc(sc_is620n, 0x0300, 1000000, 1000000/2, 0, 0);//1ms

    ret = ecrt_master_select_reference_clock(master, sc_is620n);
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

    /* Create cyclic RT-thread */
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        puts("ERROR IN SETTING THE SCHEDULER");
        perror("errno");
        return -1;
    }

    //task = rt_task_init(nam2num("ec_rtai_rtdm_example"),
    //        0 /* priority */, 0 /* stack size */, 0 /* msg size */);

    my_cyclic();

    //rt_task_delete(task);

    printf("End of Program\n");
    ecrt_release_master(master);

    return 0;
}

/****************************************************************************/
