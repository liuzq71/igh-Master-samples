#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

typedef long long RTIME;

RTIME rt_get_time(void);
RTIME rt_get_time_ns(void);
RTIME count2nano(RTIME counts);
RTIME nano2count(RTIME ns);
/*no more*/
