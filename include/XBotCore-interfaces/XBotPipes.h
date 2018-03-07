/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Alessio Margan, Arturo Laurenzi, Luca Muratore
 * email: alessio.margan@iit.it, arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef __X_BOT_PIPES_H__
#define __X_BOT_PIPES_H__

#include <string>
#include <memory>
#include <iostream>

#if defined( __XENO__ ) || defined( __COBALT__ )
#include <XBotCore-interfaces/XBotRT_ipc.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

#include <unistd.h>

#include <XCM/XBotUtils.h>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

namespace XBot{
  
#if defined( __XENO__ ) || defined( __COBALT__ )
    static const std::string pipe_prefix ( "/proc/xenomai/registry/rtipc/xddp/" );
#else
    static const std::string pipe_prefix ( "/tmp/" );
#endif
    
#if defined( __XENO__ ) || defined( __COBALT__ )
    static const std::string iddp_pipe_prefix ( "/proc/xenomai/registry/rtipc/iddp/" );
#else
    static const std::string iddp_pipe_prefix ( "/tmp/" );
#endif


    /**
     * @brief XDDP (Cross Domain Datagram Protocol) pipes: useful to communicate beetween N-RT and RT threads.
     *        It can be used both in Xenomai and standard linux kernel: XDDP in the former case, FIFO in the latter.
     *        Check https://xenomai.org/documentation/trunk/html/api/xddp-echo_8c-example.html for more info.
     * 
     */
    class XDDP_pipe {

    public:
        
        typedef std::shared_ptr<XDDP_pipe> Ptr;

        /**
         * @brief XDDP pipe constructor
         *
         * @param _pool_size local pool size: memory needed to convey datagrams will be pulled from this pool, instead of Xenomai's system pool. 
         * 
         */
        XDDP_pipe ( int _pool_size = 8192 ) : pool_size ( _pool_size ) { fd = 0; }

        /**
         * @brief Getter for the XDDP pipe file descriptor.
         * 
         * @return int XDDP pipe file descriptor
         */
        int get_fd() { return fd; }

        /**
         * @brief XDDP pipe initialization: xddp_bind for Xenomai system, mkfifo and open for standard linux kernel
         * 
         * @param pipe_name name of the pipe : the pipe_prefix will be automatically preponed.
         * @return void
         */
        void init ( const std::string pipe_name ) {

            #if !defined( __XENO__ ) && !defined( __COBALT__ )
            const char* env_user = std::getenv("USER");
            std::string pipe_prefix = std::string("/tmp/")+env_user+std::string("/");     
            #endif    
   
            std::string pipe = pipe_prefix + pipe_name;

            #if defined( __XENO__ ) || defined( __COBALT__ )
                    fd = xddp_bind ( pipe_name.c_str(), pool_size );
            #else
                    mkfifo ( pipe.c_str(), S_IRWXU|S_IRWXG );
                    fd = open ( pipe.c_str(), O_RDWR | O_NONBLOCK );
            #endif
                    Logger::info() << "Opened pipe " << pipe << Logger::endl();
        }

        /**
         * @brief XDDP pipe destructor: it will close the file descriptor linked to the pipe and in XENOMAI systems it will also unlik the pipe.
         * 
         */
        virtual ~XDDP_pipe() {
            
            
            if ( fd == 0 ) {
                return;
            }
            
            Logger::info() << "Closing socket fd " << fd << Logger::endl();

            close ( fd );
    #if !defined(__XENO__) && !defined(__COBALT__)
            std::string pipe = pipe_prefix + pipe_name;
            unlink ( pipe.c_str() );
    #endif
        }

        
        /**
         * @brief template XDDP pipe non-blocking write function: it writes tx data on the fd linked to the pipe
         * 
         * @param tx XddpTxTypes to write
         * @return int bytes wrote
         */
        template<typename XddpTxTypes>
        int xddp_write ( const XddpTxTypes & tx ) {
            if ( fd <= 0 ) { return 0; }
            return write ( fd, ( void* ) &tx, sizeof ( tx ) );
        }

        /**
         * @brief simple XDDP pipe non-blocking write of a buffer of byte with size size
         * 
         * @param buffer byte buffer to write
         * @param size buffer size
         * 
         * @return int bytes wrote
         */
        int xddp_write ( const uint8_t * buffer, int size ) {
            if ( fd <= 0 ) { return 0; }
            return write ( fd, ( void* ) buffer, size );
        }

        /**
         * @brief template XDDP pipe non-blocking read function: it recvfrom (XENOMAI) or reads rx data on the fd linked to the pipe
         * 
         * @param rx XddpRxTypes to read
         * @return int bytes read
         */
        template<typename XddpRxTypes>
        int xddp_read ( const XddpRxTypes & rx ) {

            if ( fd <= 0 ) { return 0; }
            /////////////////////////////////////////////////////////
            // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
    #if defined( __XENO__ ) || defined( __COBALT__ )
            return recvfrom ( fd, ( void* ) &rx, sizeof ( rx ), MSG_DONTWAIT, NULL, 0 );
    #else
            // NON-BLOCKING
            return read ( fd, ( void* ) &rx, sizeof ( rx ) );
    #endif
        }

        /**
         * @brief simple XDDP pipe non-blocking read of a buffer of byte with size size
         * 
         * @param buffer byte buffer to write
         * @param size buffer size
         * 
         * @return int bytes wrote
         */
        int xddp_read ( const uint8_t * buffer, int size ) {

            if ( fd <= 0 ) { return 0; }
            /////////////////////////////////////////////////////////
            // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
    #if defined( __XENO__ ) || defined( __COBALT__ )
            return recvfrom ( fd, (void*)buffer, size, MSG_DONTWAIT, NULL, 0 );
    #else
            // NON-BLOCKING
            return read ( fd, (void*)buffer, size );
    #endif
        }
        
        XDDP_pipe& operator=(const XDDP_pipe&) = delete;
        XDDP_pipe(const XDDP_pipe&) = delete;
        

    protected:
        std::string pipe_name;

    private:
        int fd;
        int pool_size;

    };
    
    /**
     * @brief IDDP (Intra Domain Datagram Protocol) pipes: useful to communicate beetween RT threads.
     * 
     */
    class IDDP_pipe {

    public:
        
        typedef std::shared_ptr<IDDP_pipe> Ptr;

        /**
         * @brief IDDP_pipe pipe constructor
         *
         * @param _pool_size local pool size: memory needed to convey datagrams will be pulled from this pool, instead of Xenomai's system pool. 
         * 
         */
        IDDP_pipe (  bool is_publisher, int _pool_size = 8192*2 ) : pool_size ( _pool_size ) { this->is_publisher = is_publisher; fd = 0; }

        /**
         * @brief Getter for the IDDP pipe file descriptor.
         * 
         * @return int IDDP pipe file descriptor
         */
        int get_fd() { return fd; }

        /**
         * @brief IDDP pipe initialization: iddp_bind for Xenomai system, mkfifo and open for standard linux kernel
         * 
         * @param pipe_name name of the pipe : the pipe_prefix will be automatically preponed.
         * @return void
         */
        void init ( const std::string pipe_name ) {

            #if !defined( __XENO__ ) && !defined( __COBALT__ )
            const char* env_user = std::getenv("USER");
            std::string iddp_pipe_prefix = std::string("/tmp/")+env_user+std::string("/");     
            #endif    
   
            std::string pipe = iddp_pipe_prefix + pipe_name;

            #if defined( __XENO__ ) || defined( __COBALT__ )
                    if( !is_publisher ) {
                        fd = iddp_bind ( pipe_name.c_str(), pool_size );
                    }
                    else {
                        Logger::warning() << "Trying to connect on IDDP pipe named: " << pipe_name << Logger::endl();
                        fd = iddp_connect ( pipe_name.c_str() );
                    }
            #else
                    mkfifo ( pipe.c_str(), S_IRWXU|S_IRWXG );
                    fd = open ( pipe.c_str(), O_RDWR | O_NONBLOCK );
            #endif
                    Logger::info() << "Opened IDDP pipe " << pipe << Logger::endl();
        }

        /**
         * @brief IDDP pipe destructor: it will close the file descriptor linked to the pipe and in XENOMAI systems it will also unlik the pipe.
         * 
         */
        virtual ~IDDP_pipe() {
            
            
            if ( fd == 0 ) {
                return;
            }
            
            Logger::info() << "Closing socket fd " << fd << Logger::endl();

            close ( fd );
    #if !defined(__XENO__) && !defined(__COBALT__)
            std::string pipe = iddp_pipe_prefix + pipe_name;
            unlink ( pipe.c_str() );
    #endif
        }

        
        /**
         * @brief template IDDP pipe non-blocking write function: it writes tx data on the fd linked to the pipe
         * 
         * @param tx IddpTxTypes to write
         * @return int bytes wrote
         */
        template<typename IddpTxTypes>
        int iddp_write ( const IddpTxTypes & tx ) {
            if ( fd <= 0 ) { return 0; }
        #if defined( __XENO__ ) || defined( __COBALT__ )
            return send ( fd, ( void* ) &tx, sizeof ( tx ), 0 );
        #else
            return write ( fd, ( void* ) &tx, sizeof ( tx ) );
        #endif
            
        }

        /**
         * @brief simple IDDP pipe non-blocking write of a buffer of byte with size size
         * 
         * @param buffer byte buffer to write
         * @param size buffer size
         * 
         * @return int bytes wrote
         */
        int iddp_write ( const uint8_t * buffer, int size ) {
            if ( fd <= 0 ) { return 0; }
        #if defined( __XENO__ ) || defined( __COBALT__ )
            return send ( fd, ( void* ) buffer, size, 0 );
        #else
            return write ( fd, ( void* ) buffer, size );
        #endif
        }

        /**
         * @brief template IDDP pipe non-blocking read function: it recvfrom (XENOMAI) or reads rx data on the fd linked to the pipe
         * 
         * @param rx XddpRxTypes to read
         * @return int bytes read
         */
        template<typename IddpRxTypes>
        int iddp_read ( const IddpRxTypes & rx ) {

            if ( fd <= 0 ) { return 0; }
            /////////////////////////////////////////////////////////
            // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
    #if defined( __XENO__ ) || defined( __COBALT__ )
            return recvfrom ( fd, ( void* ) &rx, sizeof ( rx ), MSG_DONTWAIT, NULL, 0 );
    #else
            // NON-BLOCKING
            return read ( fd, ( void* ) &rx, sizeof ( rx ) );
    #endif
        }

        /**
         * @brief simple IDDP pipe non-blocking read of a buffer of byte with size size
         * 
         * @param buffer byte buffer to write
         * @param size buffer size
         * 
         * @return int bytes wrote
         */
        int iddp_read ( const uint8_t * buffer, int size ) {

            if ( fd <= 0 ) { return 0; }
            /////////////////////////////////////////////////////////
            // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
    #if defined( __XENO__ ) || defined( __COBALT__ )
            return recvfrom ( fd, (void*)buffer, size, MSG_DONTWAIT, NULL, 0 );
    #else
            // NON-BLOCKING
            return read ( fd, (void*)buffer, size );
    #endif
        }
        
        IDDP_pipe& operator=(const IDDP_pipe&) = delete;
        IDDP_pipe(const IDDP_pipe&) = delete;
        

    protected:
        std::string pipe_name;

    private:
        int fd;
        int pool_size;
        bool is_publisher;

    };

};

#endif
