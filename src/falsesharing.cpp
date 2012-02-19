#include "cpucounters.h"
#include "util.h"
#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#ifndef ITERATIONS
#define ITERATIONS 100000000
#endif

using std::cout;
using std::endl;


typedef struct {
    int64_t* address;
    uint32_t core_id;
} sequence_t;

static void* run(void* arg)
{
    sequence_t* seq = (sequence_t*) arg;
    Affinity a(seq->core_id);

    int64_t* address = seq->address;

    for (int i = 0; i < ITERATIONS; i++)
    {
        int64_t value = *address;
        value += i;
        *address = value;
        asm volatile("lock addl $0x0,(%rsp)");
    }

    pthread_exit((void*) seq);
    return 0;
}

int main (int argc, const char * argv[])
{
    sequence_t seq1;
    sequence_t seq2;
    pthread_t thread1;
    pthread_t thread2;
    timespec t0;
    timespec t1;
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

    seq1.address = &values[0];
    seq1.core_id = atoi(argv[2]);
    seq2.address = &values[alignment];
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

    CoreCounterState before1 = getCoreCounterState(seq1.core_id);
    CoreCounterState before2 = getCoreCounterState(seq2.core_id);

    clock_gettime(CLOCK_MONOTONIC_RAW, &t0);
    pthread_create(&thread1, NULL, run, (void*) &seq1);
    pthread_create(&thread2, NULL, run, (void*) &seq2);

    pthread_join(thread1, &join1);
    pthread_join(thread2, &join2);
    clock_gettime(CLOCK_MONOTONIC_RAW, &t1);

    CoreCounterState after1 = getCoreCounterState(seq1.core_id);
    CoreCounterState after2 = getCoreCounterState(seq2.core_id);

    timespec taken = diff(t0, t1);

    cout << endl << "Cache Alignment: " << size << endl;
    cout << "Time Taken " << taken.tv_sec << "." << taken.tv_nsec/1000000  << "s" << endl << endl;

    measure(before1, after1, seq1.core_id);
    measure(before2, after2, seq2.core_id);
    m->cleanup();
    delete[] values;
}
