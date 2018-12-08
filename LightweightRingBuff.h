/*
             LUFA Library
     Copyright (C) Dean Camera, 2010.

  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Ultra lightweight ring buffer, for fast insertion/deletion.
 */

#ifndef _ULW_RING_BUFF_H_
#define _ULW_RING_BUFF_H_

	/* Includes: */
		#include <util/atomic.h>

		#include <stdint.h>
		#include <stdbool.h>

	/* Defines: */

	/* Type Defines: */
		/** Type define for a new ring buffer object. Buffers should be initialized via a call to
		 *  \ref RingBuffer_InitBuffer() before use.
		 */
		typedef struct
		{
			uint8_t* Buffer; /**< Internal ring buffer data, referenced by the buffer pointers. */
      uint8_t* End;
			uint8_t* In; /**< Current storage location in the circular buffer */
			uint8_t* Out; /**< Current retrieval location in the circular buffer */
      uint8_t  Size;
			uint8_t  Count;

		} RingBuff_t;

	/* Inline Functions: */
		/** Initializes a ring buffer ready for use. Buffers must be initialized via this function
		 *  before any operations are called upon them. Already initialized buffers may be reset
		 *  by re-initializing them using this function.
		 *
		 *  \param[out] Buffer  Pointer to a ring buffer structure to initialize
		 */
		static inline void RingBuffer_InitBuffer(RingBuff_t* const Buffer,
                                              uint8_t* const DataPtr,
    		                                      const uint8_t Size)

		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
        Buffer->Buffer = &DataPtr[0];
        Buffer->End    = &DataPtr[Size];
        Buffer->In     = DataPtr;
				Buffer->Out    = DataPtr;
        Buffer->Size   = Size;
        Buffer->Count  = 0;
			}
		}

		/** Retrieves the minimum number of bytes stored in a particular buffer. This value is computed
		 *  by entering an atomic lock on the buffer while the IN and OUT locations are fetched, so that
		 *  the buffer cannot be modified while the computation takes place. This value should be cached
		 *  when reading out the contents of the buffer, so that as small a time as possible is spent
		 *  in an atomic lock.
		 *
		 *  \note The value returned by this function is guaranteed to only be the minimum number of bytes
		 *        stored in the given buffer; this value may change as other threads write new data and so
		 *        the returned number should be used only to determine how many successive reads may safely
		 *        be performed on the buffer.
		 *
		 *  \param[in] Buffer  Pointer to a ring buffer structure whose count is to be computed
		 */
		static inline uint8_t RingBuffer_GetCount(RingBuff_t* const Buffer)
		{
			uint8_t Count;

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				Count = Buffer->Count;
			}

			return Count;
		}

		/** Atomically determines if the specified ring buffer contains any free space. This should
		 *  be tested before storing data to the buffer, to ensure that no data is lost due to a
		 *  buffer overrun.
		 *
		 *  \param[in,out] Buffer  Pointer to a ring buffer structure to insert into
		 *
		 *  \return Boolean true if the buffer contains no free space, false otherwise
		 */
		static inline bool RingBuffer_IsFull(RingBuff_t* const Buffer)
		{
			return (RingBuffer_GetCount(Buffer) == Buffer->Size);
		}


		static inline uint8_t RingBuffer_GetFreeCount(RingBuff_t* const Buffer)
    {
    	return (Buffer->Size - RingBuffer_GetCount(Buffer));
    }

		/** Atomically determines if the specified ring buffer contains any data. This should
		 *  be tested before removing data from the buffer, to ensure that the buffer does not
		 *  underflow.
		 *
		 *  If the data is to be removed in a loop, store the total number of bytes stored in the
		 *  buffer (via a call to the \ref RingBuffer_GetCount() function) in a temporary variable
		 *  to reduce the time spent in atomicity locks.
		 *
		 *  \param[in,out] Buffer  Pointer to a ring buffer structure to insert into
		 *
		 *  \return Boolean true if the buffer contains no free space, false otherwise
		 */
		static inline bool RingBuffer_IsEmpty(RingBuff_t* const Buffer)
		{
			return (RingBuffer_GetCount(Buffer) == 0);
		}

		/** Inserts an element into the ring buffer.
		 *
		 *  \note Only one execution thread (main program thread or an ISR) may insert into a single buffer
		 *        otherwise data corruption may occur. Insertion and removal may occur from different execution
		 *        threads.
		 *
		 *  \param[in,out] Buffer  Pointer to a ring buffer structure to insert into
		 *  \param[in]     Data    Data element to insert into the buffer
		 */
		static inline void RingBuffer_Insert(RingBuff_t* const Buffer,
		                                     const uint8_t Data)
		{
			*Buffer->In = Data;

			if (++Buffer->In == Buffer->End)
			  Buffer->In = Buffer->Buffer;

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				Buffer->Count++;
			}
		}

		/** Removes an element from the ring buffer.
		 *
		 *  \note Only one execution thread (main program thread or an ISR) may remove from a single buffer
		 *        otherwise data corruption may occur. Insertion and removal may occur from different execution
		 *        threads.
		 *
		 *  \param[in,out] Buffer  Pointer to a ring buffer structure to retrieve from
		 *
		 *  \return Next data element stored in the buffer
		 */
		static inline uint8_t RingBuffer_Remove(RingBuff_t* const Buffer)
		{
			uint8_t Data = *Buffer->Out;

			if (++Buffer->Out == Buffer->End)
			  Buffer->Out = Buffer->Buffer;

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				Buffer->Count--;
			}

			return Data;
		}

    /** Returns the next element stored in the ring buffer, without removing it.
		 *
		 *  \param[in,out] Buffer  Pointer to a ring buffer structure to retrieve from.
		 *
		 *  \return Next data element stored in the buffer.
		 */
		static inline uint8_t RingBuffer_Peek(RingBuff_t* const Buffer)
		{
			return *Buffer->Out;
		}


#endif
