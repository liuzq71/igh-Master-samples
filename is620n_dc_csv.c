/*****************************************************************************
 *
 *  $Id: is620n_dc.c,v bc2d4bf9cbe5 2012/09/06 18:22:24 fp $
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
 *  适用于汇川IS620N伺服(IS620N只有DC模式没有Free Run模式)
 *  compile: gcc -o is620n_dc_csv is620n_dc_csv.c -Wall -I/opt/etherlab/include -L/opt/etherlab/lib -Wl,--rpath=/opt/etherlab/lib -lethercat -rt
 *  run: $sudo ./is620n_dc_csv
 *  必须使用igh提供的实时网卡驱动，使用ec_generic.ko的话伺服会出现Er.08(过程数据错)
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

/*
#include <stdlib.h>
#include <math.h>  
*/
/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 1000 //500
#define CLOCK_TO_USE CLOCK_REALTIME
#define CONFIGURE_PDOS 1

// Optional features
#define PDO_SETTING1	1

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
static ec_slave_config_t *sc  = NULL;
static ec_slave_config_state_t sc_state = {};
/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

//signal to turn off servo on state
static unsigned int servooff;
static unsigned int deactive;
static signed long temp[8]={};

float value = 0;
static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

/*****************************************************************************/
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

#if PDO_SETTING1
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
#endif

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

void endsignal(int sig)
{
    servooff = 1;
    signal( SIGINT , SIG_DFL );
}

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);

    //struct timespec time_wc1,time_wc2;
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
    uint32_t target_position = 0xfffff;

    struct timespec wakeupTime, time,rx_time_start,rx_time_end,tx_time_start,tx_time_end;
    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

    while(1) {

        /*if(deactive==1){
            break;
        }*/

        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

	ecrt_master_application_time(master,TIMESPEC2NS(wakeupTime)); //added by me
#if 0
clock_gettime(CLOCK_TO_USE, &rx_time_start);
#endif
        ecrt_master_receive(master);
#if 0
clock_gettime(CLOCK_TO_USE, &rx_time_end);
printf("rx:%ldns     ",(rx_time_end.tv_sec-rx_time_start.tv_sec)*NSEC_PER_SEC+rx_time_end.tv_nsec-rx_time_start.tv_nsec);
#endif
        ecrt_domain_process(domain1);

        //printf("\r%6f    \t    ",((float)temp[1]/1000) );

        if(counter) {
            counter--;
        } else { // do this at 1 Hz
            counter = FREQUENCY;
            blink = !blink;
        }

        /*if(servooff==1)
                   value=0;
                else if(value<=6.28)
                   value=value+0.000628;
                else
                   value=0;*/

	temp[0]=EC_READ_U16(domain1_pd + offset.status_word_6041_0);
        temp[1]=EC_READ_U32(domain1_pd + offset.position_actual_value_6064_0);
	//printf("\r%6f    \t    ",((float)temp[1]/1000) );
        //printf("after value = %x\n",temp[0]);
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
            //EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x000f);
            //EC_WRITE_S32(domain1_pd+interpolateddata, 0);
            //EC_WRITE_S32(domain1_pd+tar_velo, 0xfffff);
            //EC_WRITE_S32(domain1_pd+max_torq, 0xf00);
	   
            EC_WRITE_S8(domain1_pd+offset.modes_operation_6060_0, 9);//csv mode
            //EC_WRITE_U16(domain1_pd+offset.target_position_607a_0,temp[1]);
	    //EC_WRITE_U32(domain1_pd+offset.target_velocity_60ff_0, 0x1000000);
            EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x000f);
            //printf("3.state = %x\n",temp[0]);
        }
        else if( (temp[0]&0x06f) == 0x027){//600 800 //temp[0]=0x237
            //EC_WRITE_S32(domain1_pd+interpolateddata,( value+=1000 ));
            EC_WRITE_U32(domain1_pd+offset.target_velocity_60ff_0, 0x1000000);
            
            //EC_WRITE_U16(domain1_pd+offset.target_position_607a_0,temp[1]+0x1000); ///////
	    EC_WRITE_U16(domain1_pd+offset.control_word_6040_0, 0x001f);
            //printf("4.state = %x\n",temp[0]);
        }

        // write application time to master
        //clock_gettime(CLOCK_TO_USE, &time);
        //ecrt_master_application_time(master, TIMESPEC2NS(time));

        if (sync_ref_counter) {
            sync_ref_counter--;
        }
        else {
            sync_ref_counter = 1; // sync every cycle

	    clock_gettime(CLOCK_TO_USE,&time);    //added by me		

            //ecrt_master_sync_reference_clock(master);
	    ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
        }
        ecrt_master_sync_slave_clocks(master);

        //send process data
        ecrt_domain_queue(domain1);
        //ecrt_domain_queue(domain2);
#if 0
clock_gettime(CLOCK_TO_USE, &tx_time_start);
#endif
        ecrt_master_send(master);
#if 0
clock_gettime(CLOCK_TO_USE, &tx_time_end);
printf("tx:%ldns\n",(tx_time_end.tv_sec-tx_time_start.tv_sec)*NSEC_PER_SEC+tx_time_end.tv_nsec-tx_time_start.tv_nsec);
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

//配置IS620
    if (!(sc = ecrt_master_slave_config(
                    master, IS620NSlavePos, INOVANCE_IS620N))) {
		fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
        }

#if CONFIGURE_PDOS
    printf("Configuring PDOs...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, IS620N_syncs)) {
        fprintf(stderr, "Failed to configure 1st PDOs.\n");
        return -1;
    }
#endif
   /****************motor1馬達domain註冊到domain_process data *******************/
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "1st motor RX_PDO entry registration failed!\n");
        return -1;
    }	
//配置IS620结束
	
    //ecrt_slave_config_dc(sc, 0x0300, 4000000, 125000, 0, 0);
    //ecrt_slave_config_dc(sc, 0x0300, 4000000, 4000000/2, 0, 0);  //orig
    //ecrt_slave_config_dc(sc, 0x0300, 2000000, 2000000/2, 0, 0);//不行
    ecrt_slave_config_dc(sc, 0x0300, 3000000, 3000000/2, 0, 0);//行
    //ecrt_slave_config_dc(sc, 0x0300, 1500000, 1500000/2, 0, 0);//不行

    printf("Activating master...\n");
	
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }
    	
    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -20))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));

	signal( SIGINT , endsignal ); //按CTRL+C 利用中斷結束程式			
	printf("Starting cyclic function.\n");
    	cyclic_task();
	ecrt_release_master(master);
	
    return 0;
}
//no more
