//
//  main.cpp
//  Bar
//
//  Created by Michael Barker on 26/12/2011.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "cpucounters.h"
#include "util.h"
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>

#define ITERATIONS 100000000

using std::cout;
using std::endl;

pthread_mutex_t lock;
pthread_cond_t condition;
volatile int64_t sequence = -1;

static void* writer_with_lock(void* arg)
{
    Affinity a(*(uint32_t*) arg);

    for (int i = 0; i <= ITERATIONS; i++)
    {
        pthread_mutex_lock(&lock);

        sequence = i;
        
        pthread_cond_signal(&condition);
        pthread_mutex_unlock(&lock);
    }
    
    pthread_exit(NULL);
    return NULL;
}

static void* writer_with_full_fence(void* arg)
{
    Affinity a(*(uint32_t*) arg);

    for (int i = 0; i <= ITERATIONS; i++)
    {
        sequence = i;
        asm volatile("lock addl $0x0,(%rsp)");
    }
    
    pthread_exit(NULL);
    return NULL;
};

static void* writer_with_soft_barrier(void* arg)
{
    Affinity a(*(uint32_t*) arg);

    for (int i = 0; i <= ITERATIONS; i++)
    {
        sequence = i;
        asm volatile("":::"memory");
    }
    
    pthread_exit(NULL);
    return NULL;
};

static void* reader_without_lock(void* arg)
{
    Affinity a(*(uint32_t*) arg);

    while (true)
    {
        if (sequence >= ITERATIONS)
        {
            break;
        }
    }
    
    pthread_exit(NULL);
    return NULL;
}

static void* reader_with_lock(void* arg)
{
    Affinity a(*(uint32_t*) arg);

    while (true)
    {
        pthread_mutex_lock(&lock);
        pthread_cond_wait(&condition, &lock);
        
        if (sequence >= ITERATIONS)
        {
            break;
        }
        
        pthread_mutex_unlock(&lock);
    }
    
    pthread_exit(NULL);
    return NULL;
}

int main (int argc, const char * argv[])
{
    if (argc != 4)
    {
        cout << "Usage: " << argv[0] << " [0,1,2] <core a> <core b>" << endl;
        cout << "0 = Lock, 1 = Full Fence, 2 Soft Barrier" <<endl;
        return -1;
    }

    int type = atoi(argv[1]);
    uint32_t core_a = atoi(argv[2]);
    uint32_t core_b = atoi(argv[3]);
    timespec t0, t1;

    void* (*reader_fun)(void*) = NULL;
    void* (*writer_fun)(void*) = NULL;

    switch (type)
    {
        case 0:
            pthread_mutex_init(&lock, NULL);
            pthread_cond_init(&condition, NULL);

            reader_fun = reader_with_lock;
            writer_fun = writer_with_lock;
            break;
        case 1:
            reader_fun = reader_without_lock;
            writer_fun = writer_with_full_fence;
            break;
        case 2:
            reader_fun = reader_without_lock;
            writer_fun = writer_with_soft_barrier;
            break;

        default:
            cout << "Unknown type: " << type << endl;
            return -1;
    }

    pthread_t writingThread;
    pthread_t readingThread;
    
    PCM * m = PCM::getInstance();
    if (!m->good())
    {
        cout << "Can not access CPU counters" << endl;
        return -1;
    }

    m->program();

    CoreCounterState before1 = getCoreCounterState(core_a);
    CoreCounterState before2 = getCoreCounterState(core_b);
    
    clock_gettime(CLOCK_MONOTONIC_RAW, &t0);
    pthread_create(&readingThread, NULL, reader_fun, (void*) &core_a);
    pthread_create(&writingThread, NULL, writer_fun, (void*) &core_b);
    
    pthread_join(readingThread, NULL);
    pthread_join(writingThread, NULL);
    clock_gettime(CLOCK_MONOTONIC_RAW, &t1);

    CoreCounterState after1 = getCoreCounterState(core_a);
    CoreCounterState after2 = getCoreCounterState(core_b);

    timespec taken = diff(t0, t1);

    cout << "Time Taken " << taken.tv_sec << "." << taken.tv_nsec/1000000  << "s" << endl;

    measure(before1, after1, core_a);
    measure(before2, after2, core_b);
    m->cleanup();
    
    return 0;
}

