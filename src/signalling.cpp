//
//  main.cpp
//  Bar
//
//  Created by Michael Barker on 26/12/2011.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include <stdlib.h>
#include <iostream>
#include <pthread.h>

#define ITERATIONS 10000

using std::cout;
using std::endl;

pthread_mutex_t lock;
pthread_cond_t condition;

static void* writer_with_lock(void* arg)
{
    int64_t* address = (int64_t*) arg;
    
    for (int i = 0; i <= ITERATIONS; i++)
    {
        pthread_mutex_lock(&lock);
        
        *address = i;
        
        pthread_cond_signal(&condition);
        pthread_mutex_unlock(&lock);
    }
    
    pthread_exit(NULL);
    return NULL;
}

static void* write_with_full_fence(void* arg)
{
    int64_t* address = (int64_t*) arg;
    
    for (int i = 0; i <= ITERATIONS; i++)
    {
        *address = i;
        asm volatile("lock addl $0x0,(%rsp)");
    }
    
    pthread_exit(NULL);
    return NULL;
};

static void* write_with_soft_barrier(void* arg)
{
    int64_t* address = (int64_t*) arg;
    
    for (int i = 0; i <= ITERATIONS; i++)
    {
        *address = i;
        asm volatile("":::"memory");
    }
    
    pthread_exit(NULL);
    return NULL;
};

static void* reader_without_lock(void* arg)
{
    int64_t* address = (int64_t*) arg;
    volatile int64_t value;

    while (true)
    {
        value = *address;
        if (value >= ITERATIONS)
        {
            break;
        }
    }
    
    pthread_exit(NULL);
    return NULL;
}

static void* reader_with_lock(void* arg)
{
    int64_t* address = (int64_t*) arg;
    
    while (true)
    {
        pthread_mutex_lock(&lock);
        pthread_cond_wait(&condition, &lock);
        
        if (*address >= ITERATIONS)
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
    pthread_t writingThread;
    pthread_t readingThread;
    int64_t value;
    
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&condition, NULL);
    
    pthread_create(&readingThread, NULL, reader_with_lock, (void*) &value);
    pthread_create(&writingThread, NULL, writer_with_lock, (void*) &value);
    
    pthread_join(readingThread, NULL);
    pthread_join(writingThread, NULL);
    
    cout << "Final value: " << value << endl;
    
    return 0;
}

