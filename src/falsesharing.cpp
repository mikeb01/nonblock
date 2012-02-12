#include "cpucounters.h"
#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef ITERATIONS
#define ITERATIONS 100000000
#endif

using std::cout;
using std::endl;


typedef struct {
    int64_t* value;
    uint32_t core_id;
} sequence_t;

static void* run(void* arg)
{
    sequence_t* seq = (sequence_t*) arg;

#ifdef _LINUX_VER
    cpu_set_t old_affinity, new_affinity;
    pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &old_affinity);
    CPU_ZERO(&new_affinity);
    CPU_SET(seq->core_id, &new_affinity);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &new_affinity);
#endif

    for (int i = 0; i < ITERATIONS; i++)
    {
        int64_t l = *seq->value;
        l += i;
        *seq->value = l;
        asm volatile("lock addl $0x0,(%rsp)");
    }

#ifdef _LINUX_VER
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &old_affinity);
#endif

    pthread_exit((void*) seq);
    return 0;
}

static inline uint64_t rdtsc(void)
{
    uint32_t hi, lo;
    asm volatile("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)lo) | (((uint64_t)hi)<<32);
}

static void measure(CoreCounterState& before_sstate, CoreCounterState& after_sstate,
                    uint32_t core_id, uint64_t before_ts, uint64_t after_ts)
{
    cout << "CoreId: " << core_id << endl;
    cout << "L2 Cache hit ratio : " << getL2CacheHitRatio(before_sstate, after_sstate) * 100. << " %" << endl;
    cout << "L3 Cache hit ratio : " << getL3CacheHitRatio(before_sstate, after_sstate) * 100. << " %" << endl;

    cout << "Instructions retired: " << getInstructionsRetired(before_sstate, after_sstate) / 1000000 << "mln" << endl;
    cout << "CPU cycles: " << getCycles(before_sstate, after_sstate) / 1000000 << "mln" << endl;
    cout << "Instructions per cycle: " << getIPC(before_sstate, after_sstate) << endl << endl;
}

int main (int argc, const char * argv[])
{
    sequence_t seq1;
    sequence_t seq2;
    pthread_t thread1;
    pthread_t thread2;
    void* join1;
    void* join2;
    void* ptr;

    if (argc < 4)
    {
        cout << "Usage: " << argv[0] << " <pad> <core a> <core b>" << endl;
        return -1;
    }

    int pad = atoi(argv[1]);
    int alignment = pad + 1;
    size_t size = alignment * sizeof(int64_t);

    posix_memalign(&ptr, size, size * 2);
    int64_t* values = (int64_t*) ptr;

    seq1.value = &values[0];
    seq1.core_id = atoi(argv[2]);
    seq2.value = &values[alignment];
    seq2.core_id = atoi(argv[3]);

    PCM * m = PCM::getInstance();
    if (!m->good())
    {
        cout << "Can not access CPU counters" << endl;
        cout << "Try to execute 'modprobe msr' as root user and then" << endl;
        cout << "you also must have read and write permissions for /dev/cpu/?/msr devices (the 'chown' command can help).";
        return -1;
    }

    m->program();

    uint64_t t0 = rdtsc();
    CoreCounterState before1 = getCoreCounterState(seq1.core_id);
    CoreCounterState before2 = getCoreCounterState(seq2.core_id);

    pthread_create(&thread1, NULL, run, (void*) &seq1);
    pthread_create(&thread2, NULL, run, (void*) &seq2);

    pthread_join(thread1, &join1);
    pthread_join(thread2, &join2);

    CoreCounterState after1 = getCoreCounterState(seq1.core_id);
    CoreCounterState after2 = getCoreCounterState(seq2.core_id);
    uint64_t t1 = rdtsc();

    cout << endl << "Cache Alignment: " << size << endl;
    cout << "Time Taken " << (t1 - t0) / 1000000 << "ms" << endl << endl;

    measure(before1, after1, seq1.core_id, t0, t1);
    measure(before2, after2, seq2.core_id, t0, t1);
    m->cleanup();
    delete[] values;
}
