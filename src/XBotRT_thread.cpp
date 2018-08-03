#include <stdio.h>
#include <stdlib.h>

#include <XCM/XBotThread.h>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////
void * XBot::periodic_thread ( Thread_hook_Ptr th_hook ) {
    
    int                 ret = 0;
    struct timespec     starttp, periodtp;
    struct itimerspec   period_timer_conf;
    unsigned long       overruns;

    // thread specific initialization
    th_hook->th_init ( 0 );

    DPRINTF ( "%s %s period %ld us\n",
              __FUNCTION__, th_hook->name,
              th_hook->period.period.tv_usec );

    ret = pthread_setname_np ( pthread_self(), th_hook->name );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_name_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }
#ifdef __COBALT__
    // PTHREAD_WARNSW, when set, cause the signal SIGXCPU to be sent to the
    // current thread, whenever it involontary switches to secondary mode;
    ret = pthread_setmode_np ( 0, PTHREAD_WARNSW, 0 );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_mode_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }
#endif
    th_hook->fd_timer = timerfd_create(CLOCK_MONOTONIC, 0);
    if ( th_hook->fd_timer == -1 ) {
        DPRINTF ( "%s : timerfd_create() return code %d\n",
                  th_hook->name, errno );
        exit ( 1 );        
    }

    clock_gettime ( CLOCK_MONOTONIC, &starttp );
    starttp.tv_sec  += th_hook->period.period.tv_sec;
    starttp.tv_nsec += th_hook->period.period.tv_usec * 1000ULL;
    tsnorm ( &starttp );

    period_timer_conf.it_value = starttp;
    period_timer_conf.it_interval.tv_sec =  th_hook->period.period.tv_sec;
    period_timer_conf.it_interval.tv_nsec = th_hook->period.period.tv_usec * 1000ULL;
    if ( timerfd_settime( th_hook->fd_timer, TFD_TIMER_ABSTIME, &period_timer_conf, NULL) == -1 )
    {
        DPRINTF ( "%s : timerfd_settime() return code %d\n",    
                  th_hook->name, errno );
        exit ( 1 );        
    }
    
    DPRINTF ( "%s %s : start looping ...\n", 
              __FUNCTION__, th_hook->name );

    while ( th_hook->_run_loop ) {

        uint64_t ticks;
        ret = read( th_hook->fd_timer, &ticks, sizeof(ticks));
        if ( ret < 0 ) {
            printf( "fd_timer wait period failed for thread: err %d\n", ret );
        }
        if ( ticks > 1 ) {
            printf( "fd_timer wait period missed for thread: overruns: %lu\n", (long unsigned int)ticks );
        }
        // thread specific loop
        th_hook->th_loop ( 0 );

    } // end while

    DPRINTF ( "%s %s : exit thread ...\n",
              __FUNCTION__, th_hook->name );

    return 0;
}


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

void * XBot::non_periodic_thread ( Thread_hook_Ptr th_hook ) {
    int ret = 0;

    // thread specific initialization
    th_hook->th_init ( 0 );

    DPRINTF ( "%s %s, period %ld us\n",
              __FUNCTION__, th_hook->name,
              th_hook->period.period.tv_usec );

    ret = pthread_setname_np ( pthread_self(), th_hook->name );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_setname_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }

#ifdef __COBALT__
    // PTHREAD_WARNSW, when set, cause the signal SIGXCPU to be sent to the
    // current thread, whenever it involontary switches to secondary mode;
    ret = pthread_setmode_np ( 0, PTHREAD_WARNSW, 0 );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_mode_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }
#endif

    DPRINTF ( "%s %s : start looping ...\n",
              __FUNCTION__, th_hook->name );


    while ( th_hook->_run_loop ) {

        // thread specific loop
        th_hook->th_loop ( 0 );

    } // end while

    DPRINTF ( "%s %s : exit thread ...\n",
              __FUNCTION__, th_hook->name );
    
    return 0;
}



// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
