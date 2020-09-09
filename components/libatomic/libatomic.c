/* Basic implementation of libatomic for GCC.
   This basic implementation currently assumes everything is aligned. 

   Copyright (C) 2010, 2011
   Free Software Foundation, Inc.

   This file is part of GCC.

   GCC is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation; either version 3, or (at your option) any
   later version.

   GCC is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with GCC; see the file COPYING3.  If not see
   <http://www.gnu.org/licenses/>.  */


#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

/* These defines should be defined based on configuration of what the target
   supports.  Changing these #defines is all that is required to use this
   file.  If your target supports unsigned int type of the appropriate
   number of bytes, simply define it as 1.  Note that for I16 you may 
   also have to change the type from __int128_t to int128_t if approriate.  

   Also note that you can expect to see warning from compiler similar to :
warning: conflicting types for built-in function ‘__atomic_compare_exchange_1’
   This is expected behaviour.  */



#ifndef  __LIBATOMIC_SUPPORTS_I1
#define __LIBATOMIC_SUPPORTS_I1		1
#endif

#ifndef  __LIBATOMIC_SUPPORTS_I2
#define __LIBATOMIC_SUPPORTS_I2		1
#endif

#ifndef  __LIBATOMIC_SUPPORTS_I4
#define __LIBATOMIC_SUPPORTS_I4		1
#endif

/* not sure about 64 bit support, but to be on the safe side... */

#ifndef  __LIBATOMIC_SUPPORTS_I8
#define __LIBATOMIC_SUPPORTS_I8		1
#endif

#ifndef  __LIBATOMIC_SUPPORTS_I16
#define __LIBATOMIC_SUPPORTS_I16	0
#endif

/* Define types for all supported sizes.  */

#if __LIBATOMIC_SUPPORTS_I1	
typedef uint8_t		I1;
#endif
#if __LIBATOMIC_SUPPORTS_I2	
typedef uint16_t	I2;
#endif
#if __LIBATOMIC_SUPPORTS_I4	
typedef uint32_t	I4;
#endif
#if __LIBATOMIC_SUPPORTS_I8	
typedef uint64_t	I8;
#endif
#if __LIBATOMIC_SUPPORTS_I16	
typedef __int128_t	I16;
#endif

/* For testing the locked implementation, define this to make all routines use
   locks and run the testsuites.  */

#if __LIBATOMIC_ALWAYS_LOCKED
#define __atomic_always_lock_free(S,P)	false
#endif

/* This defines the number of unqiue locks which are available for mapping
   an address to.  */
#define __LIBATOMIC_N_LOCKS	(1 << 4)


/* Return a pointer to a boolean test_and_set flag for ADDR.  */

static bool *
__libatomic_flag_for_address (void *addr)
{
  static bool flag_table[__LIBATOMIC_N_LOCKS]
		      = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  uintptr_t p = (uintptr_t)addr;

  p += (p >> 2) + (p << 4);
  p += (p >> 7) + (p << 5);
  p += (p >> 17) + (p << 13);
  if (sizeof(void *) > 4)
    p += (p >> 31);
  p &= (__LIBATOMIC_N_LOCKS - 1);
  return flag_table + p;
}

/* If the specified memory MODEL can act as a release fence, issue the
   appropriate barrier.  Specify it such that it is a compile time constant.  */

static inline void
maybe_release_fence (int model)
{
  switch (model)
  {
    case __ATOMIC_RELEASE:
      __atomic_thread_fence (__ATOMIC_RELEASE);
      break;
    case __ATOMIC_ACQ_REL:
      __atomic_thread_fence (__ATOMIC_ACQ_REL);
      break;
    case __ATOMIC_SEQ_CST:
      __atomic_thread_fence (__ATOMIC_SEQ_CST);
      break;
    default:
      break;
  }
}

/* If the specified memory MODEL can act as an acquire fence, issue the
   appropriate barrier.  Specify it such that it is a compile time constant.  */

static inline void
maybe_acquire_fence (int model)
{
  switch (model)
  {
    case __ATOMIC_ACQUIRE:
      __atomic_thread_fence (__ATOMIC_ACQUIRE);
      break;
    case __ATOMIC_ACQ_REL:
      __atomic_thread_fence (__ATOMIC_ACQ_REL);
      break;
    case __ATOMIC_SEQ_CST:
      __atomic_thread_fence (__ATOMIC_SEQ_CST);
      break;
    default:
      break;
  }
}

/* Acquire the spin lock for ADDR, and issue any barrier which might be
   required.  */

static inline void
get_lock (void *addr, int model)
{
  bool *lock_ptr = __libatomic_flag_for_address (addr);

  maybe_release_fence (model);
  while (__atomic_test_and_set (lock_ptr, __ATOMIC_ACQUIRE) == 1)
    ;
}

/* Release the spin lock for ADDR, and issue any barrier which might be
   required.  */

static inline void
free_lock (void *addr, int model)
{
  bool *lock_ptr = __libatomic_flag_for_address (addr);

  __atomic_clear (lock_ptr, __ATOMIC_RELEASE);
  maybe_acquire_fence (model);
}


/* Return whether a size is lock free or not.  PTR is currently unused since
   we're assuming alignment for the moment.  */

bool
__atomic_is_lock_free (size_t size, void *ptr __attribute__ ((unused)))
{
  /* __atomic_always_lock_free requires a compile time constant to evalutate
     properly, so provide individual cases and simply fill in the constant.  */
  switch (size)
    {
      case 1:
	return __atomic_always_lock_free (1, 0);
      case 2:
	return __atomic_always_lock_free (2, 0);
      case 4:
	return __atomic_always_lock_free (4, 0);
      case 8:
	return __atomic_always_lock_free (8, 0);
      case 16:
	return __atomic_always_lock_free (16, 0);
      default:
        break;
    }
  return false;
}


/* If SIZE is lockfree, issue a lockfree sequence for the load, otherwise
   break from the switch element.  */
#define LOAD(SIZE)  						\
  if (__atomic_always_lock_free (SIZE, 0)) 			\
    { 								\
      I ## SIZE tmp = __atomic_load_ ## SIZE (mem, model);	\
      memcpy (ret, &tmp, SIZE); 				\
      return;							\
    }								\
  else								\
    break;


/* Implement a generic atomic load for an object of SIZE, copying the value
   from MEM into RET using MODEL.  */

void 
__atomic_load (size_t size, void *mem, void *ret, int model)
{
  switch (size)
  {	
#if __LIBATOMIC_SUPPORTS_I1
    case 1:
      LOAD (1);
#endif
#if __LIBATOMIC_SUPPORTS_I2
    case 2:
      LOAD (2);
#endif
#if __LIBATOMIC_SUPPORTS_I4
    case 4:
      LOAD (4);
#endif
#if __LIBATOMIC_SUPPORTS_I8
    case 8:
      LOAD (8);
#endif
#if __LIBATOMIC_SUPPORTS_I16
    case 16:
      LOAD (16);
#endif
    default:
      break;
  }

  /* If control gets here, a lock is needed.  */
  get_lock (mem, model);
  memcpy (ret, mem, size);
  free_lock (mem, model);
}


/* If SIZE is lockfree, issue a lockfree sequence for the store, otherwise
   break from the switch element.  */
#define STORE(SIZE)  					\
  if (__atomic_always_lock_free (SIZE, 0)) 		\
    { 							\
      I ## SIZE tmp;					\
      memcpy (&tmp, val, SIZE); 			\
      __atomic_store_ ## SIZE (mem, tmp, model); 	\
      return;						\
    }							\
  else							\
    break;

/* Perform an atomic store for an object of SIZE.  Store VAL into MEM using 
   MODEL.  */

void 
__atomic_store (size_t size, void *mem, void *val, int model)
{
  switch (size)
  {
#if __LIBATOMIC_SUPPORTS_I1
    case 1:
      STORE (1);
#endif
#if __LIBATOMIC_SUPPORTS_I2
    case 2:
      STORE (2);
#endif
#if __LIBATOMIC_SUPPORTS_I4
    case 4:
      STORE (4);
#endif
#if __LIBATOMIC_SUPPORTS_I8
    case 8:
      STORE (8);
#endif
#if __LIBATOMIC_SUPPORTS_I16
    case 16:
      STORE (16);
#endif
    default:
      break;
  }

  /* If control gets here, a lock is needed.  */
  get_lock (mem, model);
  memcpy (mem, val, size);
  free_lock (mem, model);
}


/* If SIZE is lockfree, issue a lockfree sequence for the exchange, otherwise
   break from the switch element.  */
#define EXCHANGE(SIZE)						\
  if (__atomic_always_lock_free (SIZE, 0))			\
    { 						     		\
      I ## SIZE tmp1, tmp2;					\
      memcpy (&tmp2, val, SIZE);				\
      tmp1 = __atomic_exchange_ ## SIZE (mem, tmp2, model);	\
      memcpy (ret, &tmp1, SIZE); 				\
      return;					     		\
    }								\
  else						     		\
    break;

/* Perform an atomic exchange for an object of SIZE.  Store VAL into MEM using 
   MODEL, and return the previous value of MEM in RET.  */

void 
__atomic_exchange (size_t size, void *mem, void *val, void *ret, int model)
{
  switch (size)
  {
#if __LIBATOMIC_SUPPORTS_I1
    case 1:
      EXCHANGE (1);
#endif
#if __LIBATOMIC_SUPPORTS_I2
    case 2:
      EXCHANGE (2);
#endif
#if __LIBATOMIC_SUPPORTS_I4
    case 4:
      EXCHANGE (4);
#endif
#if __LIBATOMIC_SUPPORTS_I8
    case 8:
      EXCHANGE (8);
#endif
#if __LIBATOMIC_SUPPORTS_I16
    case 16:
      EXCHANGE (16);
#endif
    default:
      break;
  }

  /* If control gets here, a lock is needed.  */
  get_lock (mem, model);
  memcpy (ret, mem, size);
  memcpy (mem, val, size);
  free_lock (mem, model);
}


/* If SIZE is lockfree, issue a lockfree sequence for the compare_exchange,
   otherwise break from the switch element.  */
#define COMPARE_EXCHANGE(SIZE)						\
  if (__atomic_always_lock_free (SIZE, 0))				\
    { 						     			\
      bool ret;								\
      I ## SIZE tmp;							\
      memcpy (&tmp, desired, SIZE); 					\
      ret = __atomic_compare_exchange_ ## SIZE (mem, expect, tmp, 0,	\
						success, failure);	\
      return ret;				     			\
    }									\
  else						     			\
    break;

/* Perform an atomic compare_exchange for an object of SIZE.  If MEM contains
   the value in EXPECT, copy DESIRED into MEM utilizing memory model SUCESS and
   return true.  Otherwise copy the contents of MEM into EXPECT using memory
   model FAILURE and return false.  */

bool
__atomic_compare_exchange (size_t size, void *mem, void *expect, void *desired, int success, int failure)
{
  switch (size)
  {
#if __LIBATOMIC_SUPPORTS_I1
    case 1:
      COMPARE_EXCHANGE (1);
#endif
#if __LIBATOMIC_SUPPORTS_I2
    case 2:
      COMPARE_EXCHANGE (2);
#endif
#if __LIBATOMIC_SUPPORTS_I4
    case 4:
      COMPARE_EXCHANGE (4);
#endif
#if __LIBATOMIC_SUPPORTS_I8
    case 8:
      COMPARE_EXCHANGE (8);
#endif
#if __LIBATOMIC_SUPPORTS_I16
    case 16:
      COMPARE_EXCHANGE (16);
#endif
    default:
      break;
  }

  /* If control gets here, a lock is needed.  */
  get_lock (mem, success);
  if (memcmp (mem, expect, size) == 0)
    {
      memcpy (mem, desired, size);
      free_lock (mem, success);
      return true;
    }
  memcpy (expect, mem, size);
  free_lock (mem, failure);
  return false;
}


/* Issue a SIZE specific __atomic_load_N function.  */
#define ATOMIC_LOAD(SIZE) I ## SIZE			\
__atomic_load_ ## SIZE   (I ## SIZE *mem, int model)	\
{							\
  I ## SIZE ret;					\
  if (__atomic_always_lock_free (sizeof (ret), 0))	\
    return __atomic_load_n (mem, model);		\
  get_lock (mem, model);				\
  ret = *mem;						\
  free_lock (mem, model);				\
  return ret;						\
}


/* Issue a SIZE specific __atomic_store_N function.  */
#define ATOMIC_STORE(SIZE) void						\
__atomic_store_ ## SIZE (I ## SIZE *mem, I ## SIZE val, int model)	\
{									\
  if (__atomic_always_lock_free (sizeof (val), 0))			\
    __atomic_store_n (mem, val, model);					\
  else									\
    {									\
      get_lock (mem, model);						\
      *mem = val;							\
      free_lock (mem, model);						\
    }									\
}

/*#define ATOMIC_STORE(SIZE) void						\
__atomic_store_ ## SIZE (I ## SIZE *mem, I ## SIZE val, int model)	\
{									\
  if (__atomic_always_lock_free (sizeof (val), 0))			\
    __atomic_store_n (mem, val, model);					\
  else									\
    {									\
      *mem = val;							\
    }									\
}*/

/* Issue a SIZE specific __atomic_exchange_N function.  */
#define ATOMIC_EXCHANGE(SIZE) I ## SIZE					\
__atomic_exchange_ ## SIZE   (I ## SIZE *mem, I ## SIZE val, int model)	\
{									\
  I ## SIZE ret;							\
  if (__atomic_always_lock_free (sizeof (ret), 0))			\
    return __atomic_exchange_n (mem, val, model);			\
  get_lock (mem, model);						\
  ret = *mem;								\
  *mem = val;								\
  free_lock (mem, model);						\
  return ret;								\
}

/* Issue a SIZE specific __atomic_compare_exchange_N function.
   Note the compiler complains when compiling these since these functions
   do not have the boolean weak parameter, so the params dont match the
   builtin exactly.  */

#define ATOMIC_COMPARE_EXCHANGE(SIZE) bool			\
__atomic_compare_exchange_ ## SIZE   (I ## SIZE *mem, I ## SIZE *expect, I ## SIZE desired, int success, int failure)				\
{								\
  if (__atomic_always_lock_free (sizeof (desired), 0))		\
    return __atomic_compare_exchange_n (mem, expect, desired, 0,\
				        success, failure);	\
  get_lock (mem, success);					\
  if (*mem == *expect)						\
    {								\
      *mem = desired;						\
      free_lock (mem, success);					\
      return true;						\
    }								\
  *expect = *mem;						\
  free_lock (mem, failure);					\
  return false;							\
}


#define ATOMIC_FETCH(SIZE,OP,SYM) I ## SIZE				\
__atomic_fetch_## OP ## SIZE (I ## SIZE *mem, I ## SIZE val, int model) \
{									\
  I ## SIZE ret;							\
  if (__atomic_always_lock_free (sizeof (ret), 0))			\
    return __atomic_fetch_ ## OP ## SIZE (mem, val, model);		\
  get_lock (mem, model);						\
  ret = *mem;								\
  *mem SYM ## =  val;							\
  free_lock (mem, model);						\
  return ret;								\
}

#define ATOMIC_FETCH_NAND(SIZE) I ## SIZE				\
__atomic_fetch_nand_ ## SIZE (I ## SIZE *mem, I ## SIZE val, int model)	\
{									\
  I ## SIZE ret;							\
  if (__atomic_always_lock_free (sizeof (ret), 0))			\
    return __atomic_fetch_nand_ ## SIZE (mem, val, model);		\
  get_lock (mem, model);						\
  ret = *mem;								\
  *mem =  ~(*mem & val);						\
  free_lock (mem, model);						\
  return ret;								\
}

#if __LIBATOMIC_SUPPORTS_I1	
ATOMIC_LOAD (1)
ATOMIC_STORE (1)
ATOMIC_EXCHANGE (1)
// ATOMIC_COMPARE_EXCHANGE (1)
ATOMIC_FETCH (1, add_, +)
ATOMIC_FETCH (1, sub_, -)
ATOMIC_FETCH (1, and_, &)
ATOMIC_FETCH (1, or_, |)
ATOMIC_FETCH (1, xor_, ^)
ATOMIC_FETCH_NAND (1)
#endif

#if __LIBATOMIC_SUPPORTS_I2	
ATOMIC_LOAD (2)
ATOMIC_STORE (2)
ATOMIC_EXCHANGE (2)
// ATOMIC_COMPARE_EXCHANGE (2)
ATOMIC_FETCH (2, add_, +)
ATOMIC_FETCH (2, sub_, -)
ATOMIC_FETCH (2, and_, &)
ATOMIC_FETCH (2, or_, |)
ATOMIC_FETCH (2, xor_, ^)
ATOMIC_FETCH_NAND (2)
#endif


#if __LIBATOMIC_SUPPORTS_I4	
ATOMIC_LOAD (4)
ATOMIC_STORE (4)
ATOMIC_EXCHANGE (4)
// ATOMIC_COMPARE_EXCHANGE (4)
ATOMIC_FETCH (4, add_, +)
ATOMIC_FETCH (4, sub_, -)
ATOMIC_FETCH (4, and_, &)
ATOMIC_FETCH (4, or_, |)
ATOMIC_FETCH (4, xor_, ^)
ATOMIC_FETCH_NAND (4)
#endif


#if __LIBATOMIC_SUPPORTS_I8	
ATOMIC_LOAD (8)
ATOMIC_STORE (8)
ATOMIC_EXCHANGE (8)
// ATOMIC_COMPARE_EXCHANGE (8)
ATOMIC_FETCH (8, add_, +)
ATOMIC_FETCH (8, sub_, -)
ATOMIC_FETCH (8, and_, &)
ATOMIC_FETCH (8, or_, |)
ATOMIC_FETCH (8, xor_, ^)
ATOMIC_FETCH_NAND (8)
#endif


#if __LIBATOMIC_SUPPORTS_I16	
ATOMIC_LOAD (16)
ATOMIC_STORE (16)
ATOMIC_EXCHANGE (16)
// ATOMIC_COMPARE_EXCHANGE (16)
ATOMIC_FETCH (16, add_, +)
ATOMIC_FETCH (16, sub_, -)
ATOMIC_FETCH (16, and_, &)
ATOMIC_FETCH (16, or_, |)
ATOMIC_FETCH (16, xor_, ^)
ATOMIC_FETCH_NAND (16)
#endif