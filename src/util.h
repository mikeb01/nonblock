#include <time.h>
#include <pthread.h>
#include <stdint.h>
#include "cpucounters.h"

using std::cout;
using std::endl;

class Affinity
{
    private:
    cpu_set_t old_affinity_;
    cpu_set_t new_affinity_;

    public:
    Affinity(uint32_t);
    ~Affinity();
};

Affinity::Affinity(uint32_t core_id)
{
#ifdef _LINUX_VER
    pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &old_affinity_);
    CPU_ZERO(&new_affinity_);
    CPU_SET(core_id, &new_affinity_);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &new_affinity_);
#endif
} 

Affinity::~Affinity()
{
#ifdef _LINUX_VER
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &old_affinity_);
#endif
}

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec) < 0) 
    {
        temp.tv_sec = end.tv_sec - start.tv_sec - 1;
        temp.tv_nsec = 1000000000 + end.tv_nsec-start.tv_nsec;
    }
    else 
    {
        temp.tv_sec = end.tv_sec - start.tv_sec;
        temp.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
    return temp;
}

static void measure(CoreCounterState& before_sstate, CoreCounterState& after_sstate, uint32_t core_id)
{
    cout << "CoreId: " << core_id << endl;
    cout << "L2 Cache hit ratio : " << getL2CacheHitRatio(before_sstate, after_sstate) * 100. << " %" << endl;
    cout << "L3 Cache hit ratio : " << getL3CacheHitRatio(before_sstate, after_sstate) * 100. << " %" << endl;

    cout << "Instructions retired: " << getInstructionsRetired(before_sstate, after_sstate) / 1000000 << "mln" << endl;
    cout << "CPU cycles: " << getCycles(before_sstate, after_sstate) / 1000000 << "mln" << endl;
    cout << "Instructions per cycle: " << getIPC(before_sstate, after_sstate) << endl << endl;
}


