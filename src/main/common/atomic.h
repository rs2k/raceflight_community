/*
 * This file is part of RaceFlight.
 *
 * RaceFlight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RaceFlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once





__attribute__( ( always_inline ) ) static inline void __set_BASEPRI_nb(uint32_t basePri)
{
   __ASM volatile ("\tMSR basepri, %0\n" : : "r" (basePri) );
}

__attribute__( ( always_inline ) ) static inline void __set_BASEPRI_MAX_nb(uint32_t basePri)
{
   __ASM volatile ("\tMSR basepri_max, %0\n" : : "r" (basePri) );
}

__attribute__( ( always_inline ) ) static inline void __set_BASEPRI_MAX(uint32_t basePri)
{
    __ASM volatile ("\tMSR basepri_max, %0\n" : : "r" (basePri) : "memory" );
}


static inline void __basepriRestoreMem(uint8_t *val)
{
    __set_BASEPRI(*val);
}


static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
    __set_BASEPRI_MAX(prio);
    return 1;
}


static inline void __basepriRestore(uint8_t *val)
{
    __set_BASEPRI_nb(*val);
}


static inline uint8_t __basepriSetRetVal(uint8_t prio)
{
    __set_BASEPRI_MAX_nb(prio);
    return 1;
}



#define ATOMIC_BLOCK(prio) for ( uint8_t __basepri_save __attribute__((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI(), \
                                     __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )







#define ATOMIC_BLOCK_NB(prio) for ( uint8_t __basepri_save __attribute__((__cleanup__(__basepriRestore))) = __get_BASEPRI(), \
                                    __ToDo = __basepriSetRetVal(prio); __ToDo ; __ToDo = 0 ) \








#if (__GNUC__ > 4)




#endif

#ifndef __UNIQL
# define __UNIQL_CONCAT2(x,y) x ## y
# define __UNIQL_CONCAT(x,y) __UNIQL_CONCAT2(x,y)
# define __UNIQL(x) __UNIQL_CONCAT(x,__LINE__)
#endif


#define ATOMIC_BARRIER(data)                                            \
    __extension__ void  __UNIQL(__barrierEnd)(typeof(data) **__d) {     \
        __asm__ volatile ("\t# barier(" #data ")  end\n" : : "m" (**__d));                          \
    }                                                                   \
    typeof(data)  __attribute__((__cleanup__(__UNIQL(__barrierEnd)))) *__UNIQL(__barrier) = &data; \
    __asm__ volatile ("\t# barier (" #data ") start\n" : "=m" (*__UNIQL(__barrier)))



#define ATOMIC_OR(ptr, val) __sync_fetch_and_or(ptr, val)
#define ATOMIC_AND(ptr, val) __sync_fetch_and_and(ptr, val)
