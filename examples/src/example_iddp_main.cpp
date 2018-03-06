/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       
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

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <IDDPExample1.h>
#include <IDDPExample2.h>

#include <XBotInterface/RtLog.hpp>

extern void main_common(__sighandler_t sig_handler);

static int main_loop = 1;


void shutdown(int sig __attribute__((unused)))
{
    main_loop = 0;
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main(int argc, char *argv[]) try {

    main_common(shutdown);

    XBot::IDDPExample1 ex1;
    XBot::IDDPExample2 ex2;
    
    
    ex1.create(true, 1);
    ex2.create(true, 2);
   
  
    while (main_loop) {
        sleep(1); 
    }
    

    ex1.stop();
    ex1.join();
    
    ex2.stop();
    ex2.join();
    

    return 0;

} catch (std::exception& e) {

    XBot::Logger::error() << "Main catch exception: " <<  e.what() << XBot::Logger::endl();

}


