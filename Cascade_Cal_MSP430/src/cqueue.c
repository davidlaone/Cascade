/** 
 * @file cqueue.c
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief Utility to support storing data in a circular queue.
 */

/** 
 * \brief This module provides routines to manage a circular queue. It
 * will NOT malloc the cqueue_t object or the actual buffer that 
 * will be used as the circular queue. These must be passed in as
 * pointers to the initialization function cqueue_init. After
 * initialization, the cqueue_t object pointer must be passed in
 * as an argument to the other cqueue support functions
 * (cqueue_read, cqueue_write, etc). 
 *
 * The operation uses a head and tail index to track the entries
 * in the list. 
 *
 * \n Push values on the list at the tail (submitter).
 * \n Pop values from the list at the head (consumer). 
 * 
 * The action is always either push++ and pop++ (i.e. put value
 * at tail then increment tail or pull from the head then 
 * increment the head). After incrementing either the head or 
 * the tail, a check for roll over is performed.  If either the 
 * tail or the head index (zero based) becomes equal to the 
 * queue size, then the corresponding index is set to zero. At 
 * initialization, head index and tail index are set to zero. 
 * 
 * Empty and Full conditions: 
 * \n if (head == tail), list is empty 
 * \n if (tail == (head - 1)), list is full. 
 *  
 * Note - because of the operation of the queue, the max number 
 * of objects that can be put on the queue is queue size-1. 
 *          
 */

// Includes
#include "cascade.h"

/************************************ 
* Module Data Definitions and Macros
************************************/

/*********************************** 
* Module Static Data
***********************************/

/*********************************** 
* Module Static Prototypes
***********************************/

/*********************************** 
* Module Functions
***********************************/
/**
 * 
 * \brief Initialize the circular queue structure.
 *  
 * \n This routine will initialize a structure to manage a 
 * circular queue. It will NOT malloc the cqueue_t object or the 
 * actual buffer that will be used as the circular queue. These 
 * must be passed in as pointers.  After initialization, the 
 * cqueue_t object pointer must be passed in as an argument to 
 * all other cqueue support functions (read, write, etc). 
 *  
 * IMPORTANT: The circular queue will store "objects". The size
 * argument refers to max number of objects to store on the
 * queue (not bytes), and that objSize is the size of the object 
 * (in bytes). The total size of the buffer passed in must be at 
 * least size*objSize. 
 * 
 * Because of the queue mechanics, the max number of objects 
 * that can be put on the queue at the same time is size-1. 
 * After size-1 objects have been put on the queue, the queue 
 * will reflect as full. 
 *
 * @param cqP  Pointer to a structure of type cqueue_t
 * @param bufP Pointer to buffer to use as the circular buffer
 * @param size Size of queue in terms of number of objects 
 * @param objSize Size of each object in terms of bytes
 * 
 */
void cqueue_init(CQUEUE_T *cqP, uint8_t *bufP,
                 uint32_t size, uint32_t objSize) {

    // For protection.
    if (cqP == NULL || bufP == NULL) {
        return;
    }

    memset(cqP, 0, sizeof(CQUEUE_T));
    cqP->bufP = bufP;
    cqP->size = size;
    cqP->objSize = objSize;
    cqP->num_entries = 0;
    cqP->tail_index = cqP->head_index = 0;
}

/**
* \brief Reset the circular queue to a size of zero.  Handles 
*        the re-initialization of the circular queue's tail and
*        head indexes.
* 
* @param cqP  Pointer to a structure of type cqueue_t
*/
void cqueue_reset(CQUEUE_T *cqP) {

    // For protection.
    if (cqP == NULL) {
        return;
    }
    cqP->num_entries = 0;
    cqP->tail_index = cqP->head_index = 0;
}

/**
 * \brief Push objects onto the circular queue.
 *  
 * \n Write num_objs objects to the circular queue.  If the 
 * queue is full or becomes full, no additional objects will be 
 * written. The function will return the number of objects 
 * successfully written to the queue. 
 * 
 * @param cqP  Pointer to an initialized cqueue_t structure
 * @param dataP Pointer to object(s) to write to circular queue
 * @param num_objs Number of objects to write to circular queue
 * 
 * @return uint32_t Number of objects written to queue
 */
uint32_t cqueue_write(CQUEUE_T *cqP, uint8_t *dataP, uint32_t num_objs) {
    uint32_t num_objs_save = num_objs;
    uint32_t index = 0;

    // For protection.
    if (cqP == NULL || dataP == NULL) {
        return 0;
    }

    // Loop for each object to push onto the queue
    while (num_objs) {
        // Check that the queue is not full.
        // Full condition exists if tail = head-1, that is, the tail
        // is immediately behind the head.  We must use the modulo operator
        // to check for condition because the current tail index may be at
        // the very last location.
        if (((cqP->tail_index + 1) % cqP->size) != cqP->head_index) {
            // calculate index into buffer for next entry into queue
            index = cqP->tail_index * cqP->objSize;
            // copy object from data buffer to queue buffer
            memcpy(&cqP->bufP[index], dataP, cqP->objSize);
            // always increment after a push
            cqP->tail_index++;
            // track number of filled entries on the queue
            cqP->num_entries++;
            /* check for roll over */
            if (cqP->tail_index == cqP->size) {
                cqP->tail_index = 0;
            }
            // Decrement number of objects to write
            num_objs--;
            // Move to next copy location in source buffer
            dataP += cqP->objSize;

        } else {
            // Queue is full, so break out of while loop.
            break;
        }
    }

    // Return number of words that were written to the queue
    return (num_objs_save - num_objs);
}

/**
 * \brief Pop objects from the circular queue.
 * 
 * \n This functions reads num_objs objects from the circular 
 * queue and stores them in the buffer pointer passed in.  The 
 * function will continue to read unless the queue becomes 
 * empty.  The function will return the number of objects read 
 * from the circular queue and stored to the buffer. 
 * 
 * @param cqP Pointer to an initialized cqueue_t structure
 * @param dataP Pointer where to store the objects read from q.
 * @param num_objs Number of objects to read from the queue. 
 * 
 * @return uint32_t Number of objects read from the queue.
 */
uint32_t cqueue_read(CQUEUE_T *cqP, uint8_t *dataP, uint32_t num_objs) {
    uint32_t num_objs_save = num_objs;
    uint32_t index = 0;

    // For protection.
    if (cqP == NULL || dataP == NULL) {
        return 0;
    }

    // Loop for each object to read from the queue
    while (num_objs) {
        // Check that queue is not empty.
        // Empty condition exists if tail equals head.
        if (cqP->tail_index != cqP->head_index) {
            // calculate index into queue buffer to copy from
            index = cqP->head_index * cqP->objSize;
            // copy object from queue buffer to data buffer
            memcpy(dataP, &cqP->bufP[index], cqP->objSize);
            // always increment after pop
            cqP->head_index++;
            // Decrement total entries present in queue
            cqP->num_entries--;
            // check for roll over
            if (cqP->head_index == cqP->size) {
                cqP->head_index = 0;
            }
            // Decrement number of objects to read
            num_objs--;
            // Move to next copy location in destination buffer
            dataP += cqP->objSize;

        } else {
            // Queue is empty, so break out of while loop.
            break;
        }
    }

    // Return number of words that were read from the queue
    return (num_objs_save - num_objs);
}

/**
 * \brief Return number of filled entries currently on the 
 *        circular queue. 
 * 
 * @param cqP Pointer to initialized cqueue_t structure
 * 
 * @return uint32_t Number of filled entries on the queue
 */
uint32_t cqueue_getNumFilledEntries(CQUEUE_T *cqP) {
    // For protection.
    if (cqP == NULL) {
        return 0;
    }
    return cqP->num_entries;
}


/**
 * \brief Return true/false if the circular queue is full or 
 *        not.
 * 
 * @param cqP Pointer to initialized cqueue_t structure
 * 
 * @return bool Returns true if full, false otherwise
 */
bool cqueue_isQueueFull(CQUEUE_T *cqP) {
    // For protection.
    if (cqP == NULL) {
        return 0;
    }
    // Check if the queue is full.
    // Full condition exists if tail = head-1, that is, the tail
    // is immediately behind the head.  We must use the modulo operator
    // to check for condition because the current tail index may be at
    // the very last location.
    return (((cqP->tail_index + 1) % cqP->size) == cqP->head_index);
}

