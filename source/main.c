/*****************************************************************************
 *
 *  $Id: main.c,v 6a6dec6fc806 2012/09/19 17:46:58 fp $
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
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include	<sys/mman.h>
#include	<time.h>
/****************************************************************************/
#include<fcntl.h>
#include <ecrt.h>
#include<sched.h>
#include"main.h"
/****************************************************************************/

// Application parameters
#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY 2000
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec*NSEC_PER_SEC+(T).tv_nsec)

#define PRIORITY 0
#define CONFIG_DC 1
// Optional features
#define CONFIGURE_PDOS  1
#define SDO_ACCESS      0

#define CLOCK_TO_USE CLOCK_MONOTONIC

/****************************************************************************/

// EtherCAT
int dc_timens=NSEC_PER_SEC/FREQUENCY;
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
//static ec_slave_config_t *armslave=NULL;
static ec_slave_config_state_t sc_ana_in_state = {};
//static ec_slave_config_state_t sc_arm_state = {};
static int16_t aint;

// Timer
static unsigned int sig_alarms = 0;


/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

//#define BusCouplerPos  0, 0
/*
#define DigOutSlavePos 0, 2
#define AnaInSlavePos  0, 3
#define AnaOutSlavePos 0, 4
*/
#define EVBoard_PBPos  0,	0
//#define ARMBoard_Pos 1,0
/*
#define Beckhoff_EK1100 0x00000002, 0x044c2c52

#define Beckhoff_EL2004 0x00000002, 0x07d43052
#define Beckhoff_EL2032 0x00000002, 0x07f03052
#define Beckhoff_EL3152 0x00000002, 0x0c503052
#define Beckhoff_EL3102 0x00000002, 0x0c1e3052
#define Beckhoff_EL4102 0x00000002, 0x10063052
*/
#define EVBoard_PB      0x00000009, 0x26483056     
//#define FSMC				0x00000009,	0x26483052
// offsets for PDO entries
/*
static unsigned int off_ana_in_status;
static unsigned int off_ana_in_value;
static unsigned int off_ana_out;
static unsigned int off_dig_out;
*/
static unsigned int ev_off_dig_in;
static unsigned int ev_off_ana_in;
static unsigned int ev_off_dig_out;
//static unsigned int arm_off_dig_out;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};
static inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2) ;
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
struct timespec timespec_sub(struct timespec time1, struct timespec time2)
{
	struct timespec result;

	if ((time1.tv_nsec - time2.tv_nsec) <0) {
		result.tv_sec = 0;
		result.tv_nsec = 0;
	} else {
		result.tv_sec = time1.tv_sec - time2.tv_sec;
		result.tv_nsec = time1.tv_nsec - time2.tv_nsec;
	}

	return result;
}
const static ec_pdo_entry_reg_t domain1_regs[] = {
    {EVBoard_PBPos,  EVBoard_PB,  0x6020, 0x11, &ev_off_ana_in},
    {EVBoard_PBPos,  EVBoard_PB,  0x6000, 1,  &ev_off_dig_in},
    {EVBoard_PBPos,  EVBoard_PB,  0x7010, 1,  &ev_off_dig_out},
	 //{ARMBoard_Pos,	FSMC,0x7010,1,&arm_off_dig_out},
	 {}
    /*
    {AnaInSlavePos,  Beckhoff_EL3102, 0x3101, 1, &off_ana_in_status},
    {AnaInSlavePos,  Beckhoff_EL3102, 0x3101, 2, &off_ana_in_value},
    {AnaOutSlavePos, Beckhoff_EL4102, 0x3001, 1, &off_ana_out},
    {DigOutSlavePos, Beckhoff_EL2032, 0x3001, 1, &off_dig_out},
    */
};

static unsigned int counter = 0;
static unsigned int blink = 0;

/*****************************************************************************/

#if CONFIGURE_PDOS

// Analog in --------------------------
/*
static ec_pdo_entry_info_t el3102_pdo_entries[] = {
    {0x3101, 1,  8}, // channel 1 status
    {0x3101, 2, 16}, // channel 1 value
    {0x3102, 1,  8}, // channel 2 status
    {0x3102, 2, 16}, // channel 2 value
    {0x6401, 1, 16}, // channel 1 value (alt.)
    {0x6401, 2, 16}  // channel 2 value (alt.)
};
*/
static ec_pdo_entry_info_t evboard_pdo_entries[]={
		    {0x7010, 0x01, 1}, /* L */
		    {0x7010, 0x02, 1}, /* L */
		    {0x7010, 0x03, 1}, /* L */
		    {0x7010, 0x04, 1}, /* L */
		    {0x7010, 0x05, 1}, /* L */
		    {0x7010, 0x06, 1}, /* L */
		    {0x7010, 0x07, 1}, /* L */
		    {0x7010, 0x08, 1}, /* L */
		    {0x0000, 0x00, 8}, /* Gap */
		    {0x6000, 0x01, 1}, /* S */
		    {0x6000, 0x02, 1}, /* S */
		    {0x6000, 0x03, 1}, /* S */
		    {0x6000, 0x04, 1}, /* S */
		    {0x6000, 0x05, 1}, /* S */
		    {0x6000, 0x06, 1}, /* S */
		    {0x6000, 0x07, 1}, /* S */
		    {0x6000, 0x08, 1}, /* S */
		    {0x0000, 0x00, 8}, /* Gap */
		    {0x6020, 0x01, 1}, /* U */
		    {0x6020, 0x02, 1}, /* O */
		    {0x6020, 0x03, 2}, /* L */
		    {0x6020, 0x05, 2}, /* L */
		    {0x0000, 0x00, 8}, /* Gap */
		    {0x1802, 0x07, 1}, /* T */
		    {0x1802, 0x09, 1}, /* T */
		    {0x6020, 0x11, 16} /* A */
};

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x7010, 0x01, 1}, /* LED 1 */
    {0x7010, 0x02, 1}, /* LED 2 */
    {0x7010, 0x03, 1}, /* LED 3 */
    {0x7010, 0x04, 1}, /* LED 4 */
    {0x7010, 0x05, 1}, /* LED 5 */
    {0x7010, 0x06, 1}, /* LED 6 */
    {0x7010, 0x07, 1}, /* LED 7 */
    {0x7010, 0x08, 1}, /* LED 8 */
    {0x0000, 0x00, 8},
    {0x7011, 0x00, 32}, /* Var0x7011 */
    {0x6000, 0x01, 1}, /* Switch 1 */
    {0x6000, 0x02, 1}, /* Switch 2 */
    {0x6000, 0x03, 1}, /* Switch 3 */
    {0x6000, 0x04, 1}, /* Switch 4 */
    {0x6000, 0x05, 1}, /* Switch 5 */
    {0x6000, 0x06, 1}, /* Switch 6 */
    {0x6000, 0x07, 1}, /* Switch 7 */
    {0x6000, 0x08, 1}, /* Switch 8 */
    {0x0000, 0x00, 8},
    {0x6020, 0x01, 1}, /* Underrange */
    {0x6020, 0x02, 1}, /* Overrange */
    {0x6020, 0x03, 2}, /* Limit 1 */
    {0x6020, 0x05, 2}, /* Limit 2 */
    {0x0000, 0x00, 8},
    {0x1802, 0x07, 1}, /* TxPDO State */
    {0x1802, 0x09, 1}, /* TxPDO Toggle */
    {0x6020, 0x0b, 16}, /* Analog input */
};

/*
static ec_pdo_info_t el3102_pdos[] = {
    {0x1A00, 2, el3102_pdo_entries},
    {0x1A01, 2, el3102_pdo_entries + 2}
};
*/

static ec_pdo_info_t evboard_pdos[]={
		 {0x1601, 9, evboard_pdo_entries + 0}, /* D */
		    {0x1a00, 9, evboard_pdo_entries + 9}, /* D */
		    {0x1a02, 8, evboard_pdo_entries + 18} /* A */

};
ec_pdo_info_t slave_0_pdos[] = {
    {0x1601, 10, slave_0_pdo_entries + 0}, /* DO Outputs */
    {0x1a00, 9, slave_0_pdo_entries + 10}, /* DI Inputs */
    {0x1a02, 8, slave_0_pdo_entries + 19}, /* AI Inputs */
};

/*
static ec_sync_info_t el3102_syncs[] = {
    {2, EC_DIR_OUTPUT},
    {3, EC_DIR_INPUT, 2, el3102_pdos},
    {0xff}
};
*/
static ec_sync_info_t evboard_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		    {2, EC_DIR_OUTPUT, 1, evboard_pdos + 0, EC_WD_ENABLE},
		    {3, EC_DIR_INPUT, 2, evboard_pdos + 1, EC_WD_DISABLE},
		    {0xff}

};
ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// Analog out -------------------------
/*
static ec_pdo_entry_info_t el4102_pdo_entries[] = {
    {0x3001, 1, 16}, // channel 1 value
    {0x3002, 1, 16}, // channel 2 value
};

static ec_pdo_info_t el4102_pdos[] = {
    {0x1600, 1, el4102_pdo_entries},
    {0x1601, 1, el4102_pdo_entries + 1}
};

static ec_sync_info_t el4102_syncs[] = {
    {2, EC_DIR_OUTPUT, 2, el4102_pdos},
    {3, EC_DIR_INPUT},
    {0xff}
};

// Digital out ------------------------

static ec_pdo_entry_info_t el2004_channels[] = {
    {0x3001, 1, 1}, // Value 1
    {0x3001, 2, 1}, // Value 2
    {0x3001, 3, 1}, // Value 3
    {0x3001, 4, 1}  // Value 4
};
*/
/*
static ec_pdo_info_t el2004_pdos[] = {
    {0x1600, 1, &el2004_channels[0]},
    {0x1601, 1, &el2004_channels[1]},
    {0x1602, 1, &el2004_channels[2]},
    {0x1603, 1, &el2004_channels[3]}
};
*/
/*
static ec_sync_info_t el2004_syncs[] = {
    {0, EC_DIR_OUTPUT, 4, el2004_pdos},
    {1, EC_DIR_INPUT},
    {0xff}
};
*/
#endif

/*****************************************************************************/

#if SDO_ACCESS
static ec_sdo_request_t *sdo;
#endif

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

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_ana_in, &s);
    if (s.al_state != sc_ana_in_state.al_state)
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    if (s.online != sc_ana_in_state.online)
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_ana_in_state.operational)
        printf("AnaIn: %soperational.\n",
                s.operational ? "" : "Not ");
    sc_ana_in_state = s;
/*
    ecrt_slave_config_state(armslave, &s1);
    if(s1.al_state!=sc_arm_state.al_state)
    	printf("ArmBoard: State 0x%02X.\n",s1.al_state);
    if(s1.online!=sc_arm_state.online)
    	printf("ArmBoard: %s.\n",s1.online?"online":"offline");
    if(s1.operational!=sc_arm_state.operational)
    	printf("ArmBoard: %soperational.\n",s.operational ? "" : "Not ");
    sc_arm_state=s1;
*/
}

/*****************************************************************************/

#if SDO_ACCESS
void read_sdo(void)
{
    switch (ecrt_sdo_request_state(sdo)) {
        case EC_REQUEST_UNUSED: // request was not used yet
            ecrt_sdo_request_read(sdo); // trigger first read
            break;
        case EC_REQUEST_BUSY:
            fprintf(stderr, "Still busy...\n");
            break;
        case EC_REQUEST_SUCCESS:
            fprintf(stderr, "SDO value: 0x%04X\n",
                    EC_READ_U16(ecrt_sdo_request_data(sdo)));
            ecrt_sdo_request_read(sdo); // trigger next read
            break;
        case EC_REQUEST_ERROR:
            fprintf(stderr, "Failed to read SDO!\n");
            ecrt_sdo_request_read(sdo); // retry reading
            break;
    }
}
#endif

/****************************************************************************/

void cyclic_task()
{
     struct timespec wakeupTime, time,startTime,endTime,usetime;
     	 int64_t diff;
     clock_gettime(CLOCK_TO_USE, &wakeupTime);
     while(1)
     {
    	    //cycletime=timespec_sub(endTime,startTime);


	 //wakeupTime=timespec_sub(wakeupTime,usetime);
         // receive process data

         clock_gettime(CLOCK_TO_USE, &startTime);
        //clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);
         nanosleep(&cycletime,NULL);
         ecrt_master_receive(master);
         ecrt_domain_process(domain1);

         // check process data state (optional)


        if (counter) 
        {
            counter--;
        } 
        else 
        { 
        // do this at 1 Hz
        counter = FREQUENCY;
        // calculate new process data
        blink = !blink;
        // check for master state (optional)
        check_master_state();
        check_domain1_state();
        // check for islave configuration state(s) (optional)
        check_slave_config_states();

        #if 1
    // write process data
            EC_WRITE_U8(domain1_pd + ev_off_dig_out, blink ? 0x06 : 0x09);
            //EC_WRITE_U8(domain1_pd+arm_off_dig_out,blink?0x06:0x09);
        #endif
        printf("aint=%d\n ",aint);
        printf("diff=%ld\n",diff);
     }
    aint=EC_READ_S16(domain1_pd + ev_off_ana_in);
    // write application time to master
    if(CONFIG_DC){
    	clock_gettime(CLOCK_TO_USE, &time);
		ecrt_master_application_time(master, TIMESPEC2NS(time));
		if (sync_ref_counter) {
			sync_ref_counter--;
		} else {
			sync_ref_counter = 1; // sync every cycle
			ecrt_master_sync_reference_clock(master);
		}

		ecrt_master_sync_slave_clocks(master);
    }
    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
#if SDO_ACCESS
        // read process data SDO
        read_sdo();
#endif
    }
#if 0
    // read process data
    printf("AnaIn: state %u value %u\n",
            EC_READ_U8(domain1_pd + off_ana_in_status),
            EC_READ_U16(domain1_pd + off_ana_in_value));
#endif
clock_gettime(CLOCK_TO_USE,&endTime);
diff = calcdiff_ns(endTime, startTime);
//diff /= 1000;
}

/****************************************************************************/

void signal_handler(int signum) {
    switch (signum) {
        case SIGALRM:
            sig_alarms++;
            break;
    }
}

/****************************************************************************/

int main(int argc, char **argv)
{
	struct sched_param param, old_param;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		perror("mlockall failed");
		return -1;
	}
    int policy = sched_getscheduler(0);


	 //ec_slave_config_t *sc;
   // struct sigaction sa;

    system("/etc/init.d/ethercat start");
    master = ecrt_request_master(0);
    if (!master)
        return -1;
    if (sched_getparam(0, &old_param)) {
    		fprintf(stderr, "unable to get scheduler parameters\n");
    		return 1;
    	}
    	param = old_param;
    	/* try to change to SCHED_FIFO */
		param.sched_priority = 1;
		if (sched_setscheduler(0, SCHED_FIFO, &param)) {
			fprintf(stderr, "Unable to change scheduling policy!\n");
			fprintf(stderr, "either run as root or join realtime group\n");
			return 1;
		}
		policy = sched_getscheduler(0);
		if (policy == SCHED_FIFO) {
			printf("policy==SCHED_FIFO");

		} else {
			fprintf(stderr, "policy set failed\n");
			return 1;
		}
	set_latency_target();
	check_clock_resolution();
	check_sleep_resolution();
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
        return -1;

    if (!(sc_ana_in = ecrt_master_slave_config(
                    master, EVBoard_PBPos, EVBoard_PB))) {
        printf("Failed to get slave configuration.\n");
        return -1;
    }
    /*
    if(!(armslave	=	ecrt_master_slave_config(master,ARMBoard_Pos,FSMC)))
    {
    	printf("Failed to get slave configuration.\n");
    	return -1;
    }
*/

#if SDO_ACCESS
    fprintf(stderr, "Creating SDO requests...\n");
    if (!(sdo = ecrt_slave_config_create_sdo_request(sc_ana_in, 0x3102, 2, 2))) {
        fprintf("Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(sdo, 500); // ms
#endif

#if CONFIGURE_PDOS
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_ana_in, EC_END, evboard_syncs)) {
        printf("Failed to configure PDOs.\n");
        return -1;
    }
    /*
    if(ecrt_slave_config_pdos(armslave,EC_END,slave_0_syncs))
    {
    	printf("Failed to configure PDOs.\n");
    	return -1;
    }
    */
    /*
    if (!(sc = ecrt_master_slave_config(
                    master, AnaOutSlavePos, Beckhoff_EL4102))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc, EC_END, el4102_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (!(sc = ecrt_master_slave_config(
                    master, DigOutSlavePos, Beckhoff_EL2032))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc, EC_END, el2004_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    */
#endif

    // Create configuration for bus coupler
/*
    sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100);
    if (!sc)
        return -1;
*/
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        printf("PDO entry registration failed!\n");
        return -1;
    }

    if(CONFIG_DC)
    	ecrt_slave_config_dc(sc_ana_in, 0x0300, dc_timens, 0, 0, 0);
    else
    {
    	ecrt_slave_config_dc(sc_ana_in, 0, 0, 0, 0, 0);
    	//ecrt_slave_config_dc(armslave,0,0,0,0,0);
    }
    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

#if PRIORITY
    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, 80))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));
#endif
/*
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0)) {
        fprintf(stderr, "Failed to install signal handler!\n");
        return -1;
    }*/
#if 0
        struct timeval t;
        gettimeofday(&t, NULL);
        printf("%u.%06u\n", t.tv_sec, t.tv_usec);
#endif
    cyclic_task();
        
    return 0;
}
static void set_latency_target(void) {
	struct stat s;
	int err;
	static int latency_target_fd = -1;
	static int32_t latency_target_value = 0;

	errno = 0;
	err = stat("/dev/cpu_dma_latency", &s);
	if (err == -1) {
		printf("WARN: stat /dev/cpu_dma_latency failed\n");
		return;
	}

	errno = 0;
	latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
	if (latency_target_fd == -1) {
		printf("WARN: open /dev/cpu_dma_latency\n");
		return;
	}

	errno = 0;
	err = write(latency_target_fd, &latency_target_value, 4);
	if (err < 1) {
		printf("error setting cpu_dma_latency to %d!\n", latency_target_value);
		close(latency_target_fd);
		return;
	}
	printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
}
static int check_clock_resolution(void) {
	int clock;
	uint64_t diff;
	int k;
	uint64_t min_non_zero_diff = UINT64_MAX;
	struct timespec now;
	struct timespec prev;
	uint64_t reported_resolution = UINT64_MAX;
	struct timespec res;
	struct timespec *time;
	int times;

	clock = CLOCK_MONOTONIC;

	if (clock_getres(clock, &res)) {
		printf("clock_getres failed");
		return 1;
	} else {
		reported_resolution = (NSEC_PER_SEC * res.tv_sec) + res.tv_nsec;
	}
	times = 1000;
	clock_gettime(clock, &prev);
	for (k = 0; k < times; k++) {
		clock_gettime(clock, &now);
	}
	diff = calcdiff_ns(now, prev);
	if (diff == 0) {
		printf("No clock rollover occurred \n");
		return 1;
	} else {
		int call_time;
		call_time = diff / times; /* duration 1 call */
		times = NSEC_PER_SEC / call_time; /* calls per second */
		times /= 1000; /* calls per msec */
		if (times < 1000)
			times = 1000;
	}
	if ((times <= 0) || (times > 100000))
		times = 100000;
	time = calloc(times, sizeof(*time));
	for (k = 0; k < times; k++) {
		clock_gettime(clock, &time[k]);
	}
	prev = time[0];
	for (k = 1; k < times; k++) {

		diff = calcdiff_ns(time[k], prev);
		prev = time[k];

		if (diff && (diff < min_non_zero_diff)) {
			min_non_zero_diff = diff;
		}
		printf("%ld.%06ld  %5llu\n", time[k].tv_sec, time[k].tv_nsec,
				(unsigned long long) diff);
	}
	free(time);
	if ((min_non_zero_diff && (min_non_zero_diff > reported_resolution))) {
		/*
		 * Measured clock resolution includes the time to call
		 * clock_gettime(), so it will be slightly larger than
		 * actual resolution.
		 */
		printf("reported clock resolution: %llu nsec\n",
				(unsigned long long) reported_resolution);
		printf("measured clock resolution approximately: %llu nsec\n",
				(unsigned long long) min_non_zero_diff);
	}
	return 0;
}
static inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2) {
	int64_t diff;
	diff = NSEC_PER_SEC * (int64_t) ((int) t1.tv_sec - (int) t2.tv_sec);
	diff += ((int) t1.tv_nsec - (int) t2.tv_nsec);
	return diff;
}
static inline void check_sleep_resolution(void) {
	int clock = CLOCK_TO_USE;

	int64_t diff;
	int i,j,k,m;

	struct timespec now;
	struct timespec prev;
	struct timespec res;
	res.tv_nsec = 1000 * (100);
			res.tv_sec = 0;

	for (i = 0; i < 1000; i++) {
	clock_gettime(clock, &prev);
		usleep(100);
	clock_gettime(clock, &now);
	diff = calcdiff_ns(now, prev);
	diff /= 1000;
		printf("the real delay time of usleep 100us is :%ld \n", diff);
	}
	for (i = 0; i < 1000; i++)
	{
		clock_gettime(clock, &prev);

		nanosleep(&res, NULL);
		clock_gettime(clock, &now);
		diff = calcdiff_ns(now, prev);
		diff /= 1000;
		printf("the real delay time of nanosleep 100us is :%ld \n", diff);
	}

}
/****************************************************************************/
