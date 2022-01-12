//
// Created by aldo on 11/25/21.
//

#include "hThread.h"

//==============================================================================
/*!
    This method creates a hThread to execute within the address space of the
    calling process. Parameters include a pointer to the function and its
    priority level.

    \param  a_function  Pointer to hThread function.
    \param  a_level     Priority level of hThread.
*/
//==============================================================================
void hThread::start(void(*a_function)(void), ThreadPriority a_level)
{
    // create hThread
    pthread_create(
          &handle,
          0,
          (void * (*)(void*)) a_function,
          0
    );

    // set hThread priority level
    setPriority(a_level);
}


//==============================================================================
/*!
    This method creates a hThread to execute within the address space of the
    calling process. Parameters include a pointer to the function, its priority
    level, and arguments passed to the function.

    \param  a_function  Pointer to hThread function.
    \param  a_level     Priority level of hThread.
    \param  a_arg       Function arguments to pass when staring hThread.
*/
//==============================================================================
void hThread::start(void(*a_function)(void*), ThreadPriority a_level, void *a_arg)
{
    // create hThread
    pthread_create(
          &handle,
          0,
          (void * (*)(void*)) a_function,
          a_arg
    );

    // set hThread priority level
    setPriority(a_level);
}


//==============================================================================
/*!
     This method terminates the hThread (not recommended!).
*/
//==============================================================================
void hThread::stop()
{
    // terminate hThread
    pthread_cancel(handle);
}


//==============================================================================
/*!
    This method adjusts the priority level of the hThread.

    \param  a_level  Priority level of the hThread.
*/
//==============================================================================
void hThread::setPriority(ThreadPriority a_level)
{
    priorityLevel = a_level;

    struct sched_param sp;
    int policy;

    pthread_getschedparam (handle, &policy, &sp);

    switch (priorityLevel)
    {
        case THREAD_PRIORITY_GRAPHICS:
        sp.sched_priority = 75;
        break;

        case THREAD_PRIORITY_HAPTICS:
        sp.sched_priority = 99;
        break;
    }

    pthread_setschedparam(handle, SCHED_FIFO, &sp);

}

