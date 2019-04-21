/*1,gcc -c -fPIC ipipe_64.c
 *2,gcc -shared -fPIC -o libipipe_64.so ipipe_64.o
 */
#include "ipipe_64.h"

double cpu_mhz;
unsigned long clock_freq;

/*x86_64*/
#define ipipe_read_tsc(t)  do {         \
        unsigned int __a,__d;                   \
        asm volatile("rdtsc" : "=a" (__a), "=d" (__d)); \
        (t) = ((unsigned long)__a) | (((unsigned long)__d)<<32); \
} while(0)

#define rtai_rdtsc() ({ unsigned long long t; ipipe_read_tsc(t); t; })

static inline long rtai_imuldiv (long i, long mult, long div) {

    /* Returns (int)i = (int)i*(int)(mult)/(int)div. */
    
    int dummy;

    __asm__ __volatile__ ( \
	"mulq %%rdx\t\n" \
	"divq %%rcx\t\n" \
	: "=a" (i), "=d" (dummy)
       	: "a" (i), "d" (mult), "c" (div));

    return i;
}

static inline long long rtai_llimd(long long ll, long mult, long div) {
	return rtai_imuldiv(ll, mult, div);
}

static double get_cpu_freq(void){
   FILE *fp;
   char s[81];
   memset(s,0,81);
   fp=popen("cat /proc/cpuinfo|grep cpu\\ MHz|sed -e 's/.*:[^0-9]//'","r");
   if(fp<0){
       printf("read CPU freq failed.\n");
       return 0;
   }
   fgets(s,80,fp);
   fclose(fp);
   
   return (double)atof(s); 
}

RTIME rt_get_time(void)
{
	return rtai_rdtsc();
}

RTIME rt_get_time_ns(void)
{
	return rtai_llimd(rtai_rdtsc(), 1000000000, clock_freq);
}

/* ++++++++++++++++++++++++++ TIME CONVERSIONS +++++++++++++++++++++++++++++ */

RTIME count2nano(RTIME counts)
{
	return (counts >= 0 ? rtai_llimd(counts, 1000000000, clock_freq) : -rtai_llimd(-counts, 1000000000, clock_freq));
}

RTIME nano2count(RTIME ns)
{
	return (ns >= 0 ? rtai_llimd(ns, clock_freq, 1000000000) : -rtai_llimd(-ns, clock_freq, 1000000000));
}

/*initialize the library*/
void __attribute__ ((constructor)) my_init(void){
   cpu_mhz = get_cpu_freq();
   clock_freq = (unsigned long)((double)cpu_mhz * 1000000l);
}

/*finitialize the library*/
void __attribute__ ((destructor)) my_fini(void){
}

/*no more*/
