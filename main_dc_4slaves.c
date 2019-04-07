/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  $gcc -o main_dc_4slaves main_dc_4slaves.c -I/opt/etherlab/include -L/opt/etherlab/lib -lethercat -lrt
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING

/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
        (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

#define BusCouplerPos    0, 0
#define DigInSlavePos    0, 1
#define DigOutSlavePos   0, 2

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL1008 0x00000002, 0x03f03052
#define Beckhoff_EL2008 0x00000002, 0x07d83052

// offsets for PDO entries
static int off_dig_out;
static int off_dig_in;
//static int off_counter_in;
//static int off_counter_out;

static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

/*****************************************************************************/
static unsigned int servooff=1;
static signed long temp[8]={};
uint32_t target_position = 0xfffff;

#define IS620NSlavePos         0,3           /*EtherCAT address on the bus*/
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

/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/****************************************************************************/

void cyclic_task()
{
    struct timespec wakeupTime, time;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

    while(1) {
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

        // Write application time to master
        //
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        //
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
#endif

        // receive process data
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        // check process data state (optional)
        check_domain1_state();

        if (counter) {
            counter--;
        } else { // do this at 1 Hz
            counter = FREQUENCY;

            // check for master state (optional)
            check_master_state();

#ifdef MEASURE_TIMING
            // output timing stats
            printf("period     %10u ... %10u\n",
                    period_min_ns, period_max_ns);
            printf("exec       %10u ... %10u\n",
                    exec_min_ns, exec_max_ns);
            printf("latency    %10u ... %10u\n",
                    latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
#endif

            // calculate new process data
            blink = !blink;
        }

        // write process data
        EC_WRITE_U8(domain1_pd + off_dig_out, blink ? 0x66 : 0x99);
        //EC_WRITE_U8(domain1_pd + off_counter_out, blink ? 0x00 : 0x02);

        //IS620n control
        temp[0]=EC_READ_U16(domain1_pd + offset.status_word_6041_0);
        temp[1]=EC_READ_U32(domain1_pd + offset.position_actual_value_6064_0);

        //printf("\r%6f    \t    ",((float)temp[1]/1000) );

        //printf("after value = %x\n",temp[0]);
        // write process data
        /*if(servooff==1){//servo off
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x0006 );
            //deactive++;
        }
        else*/ if( (temp[0]&0x004f) == 0x0040  ){
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x0006 );
            //printf("1.state = %x\n",temp[0]);
        }
        else if( (temp[0]&0x006f) == 0x0021){
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x0007 );
            //printf("2.state = %x\n",temp[0]);
        }
        else if( (temp[0]&0x027f) == 0x0233){
            //EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x000f);
            //EC_WRITE_S32(domain1_pd+interpolateddata, 0);
            //EC_WRITE_S32(domain1_pd+tar_velo, 0xfffff);
            //EC_WRITE_S32(domain1_pd+max_torq, 0xf00);
	    
            EC_WRITE_S8(domain1_pd+offset.modes_operation_6060_0, 9);//csv mode
            //EC_WRITE_U16(domain1_pd+offset.target_position_607a_0,target_position);
	    //EC_WRITE_U32(domain1_pd+offset.target_velocity_60ff_0, 0x1000000);
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x000f);
            //printf("3.state = %x\n",temp[0]);
        }
        else if( (temp[0]&0x027f) == 0x0237){//600 800
            //EC_WRITE_S32(domain1_pd+interpolateddata,( value+=1000 ));
            //EC_WRITE_U32(domain1_pd+offset.target_velocity_60ff_0, 0x1000000);
            
            //EC_WRITE_U16(domain1_pd+offset.target_position_607a_0,(target_position+=0xfffff)); ///////
	    EC_WRITE_U32(domain1_pd+offset.target_velocity_60ff_0, 0x1000000);
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x001f);
            //printf("4.state = %x\n",temp[0]);
        }

        //IS620n control end

        if (sync_ref_counter) {
            sync_ref_counter--;
        } else {
            sync_ref_counter = 1; // sync every cycle

            clock_gettime(CLOCK_TO_USE, &time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
        }
        ecrt_master_sync_slave_clocks(master);

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
    }
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }

    master = ecrt_request_master(0);
    if (!master)
        return -1;

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
        return -1;

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100);
    if (!sc)
        return -1;

    if (!(sc = ecrt_master_slave_config(master,
                    DigOutSlavePos, Beckhoff_EL2008))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    off_dig_out = ecrt_slave_config_reg_pdo_entry(sc,
            0x7000, 1, domain1, NULL);
    if (off_dig_out < 0)
        return -1;

    if (!(sc = ecrt_master_slave_config(master,
                    DigInSlavePos, Beckhoff_EL1008))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    off_dig_in = ecrt_slave_config_reg_pdo_entry(sc,
            0x6000, 1, domain1, NULL);
    if (off_dig_in < 0)
        return -1;

    /*off_counter_out = ecrt_slave_config_reg_pdo_entry(sc,
            0x7020, 1, domain1, NULL);
    if (off_counter_out < 0)
        return -1;*/

//配置IS620
    if (!(sc = ecrt_master_slave_config(
                    master, IS620NSlavePos, INOVANCE_IS620N))) {
	fprintf(stderr, "Failed to get slave3 configuration.\n");
        return -1;
    }

    //printf("Configuring PDOs...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, IS620N_syncs)) {
        fprintf(stderr, "Failed to configure 1st PDOs.\n");
        return -1;
    }

   /****************motor1馬達domain註冊到domain_process data *******************/
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "1st motor RX_PDO entry registration failed!\n");
        return -1;
    }	
//配置IS620结束

    // configure SYNC signals for this slave
    //ecrt_slave_config_dc(sc, 0x0700, PERIOD_NS, 4400000, 0, 0);
    ecrt_slave_config_dc(sc,0x0300, 4000000, 125000,0,0);  //added by liuzq

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    printf("Starting cyclic function.\n");
    cyclic_task();

    return 0;
}

/****************************************************************************/
