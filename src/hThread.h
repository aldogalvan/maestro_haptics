//
// Created by aldo on 11/25/21.
//

#ifndef MAESTRO_HAPTICS_HTHREAD_H
#define MAESTRO_HAPTICS_HTHREAD_H

#include <pthread.h>
#include <sched.h>

enum ThreadPriority
{
    THREAD_PRIORITY_GRAPHICS,    // lower priority
    THREAD_PRIORITY_HAPTICS      // higher priority
};


class hThread{

public:

    hThread()
    {
        // no hThread function has been defined yet
        function = NULL;

        // default value for priority level
        priorityLevel = THREAD_PRIORITY_GRAPHICS;

        // default handle
        handle = 0;
    };

    ~hThread(){};


public:

    //! This method sets the hThread parameters (and start it).
    void start(void(*a_function)(void ), const ThreadPriority a_level);

    //! This method sets the hThread parameters (and start it).
    void start(void(*a_function)(void*), const ThreadPriority a_level, void *a_arg);

    //! This method terminates the hThread (not recommended!).
    void stop();

    //! This method sets the hThread priority level.
    void setPriority(ThreadPriority a_level);

    //! This method returns the current hThread priority level.
    ThreadPriority getPriority() const { return (priorityLevel); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Thread handle.
    pthread_t handle;

    //! Pointer to hThread function.
    void* function;

    //! Thread priority level.
    ThreadPriority priorityLevel;
};

#endif //MAESTRO_HAPTICS_HTHREAD_H
