/*
 *  ======== system_status.c ========
 *  Built with:
 *  Code Composer Studio 10.1.1.00004
 *  SimpleLink CC32xx SDK 4.30.0.06
 *  FreeRTOS V10.2.1
 */

#include <stdint.h>

/* Project header files */
#include "freertos/system_status.h"

//****************************************************************************
//                          GLOBAL VARIABLES
//****************************************************************************
uint32_t appStatus[MAX_STATUS_WORDS];

/*
 *  ======== setStatus ========
 *  This function sets a status bit within an array of status words
 *  arr - Pointer to array of status words
 *  index - Status bit ID number
 *  set - Assert (true) or Clear (false)
 *  return - Status code
 */
int8_t setStatus(uint32_t *arr, uint16_t index, uint8_t set) {
    int8_t code;
    uint8_t status_word;
    uint32_t status_bit;

    // Divide index by 32-bit status word
    status_word = index >> 5;
    if (status_word >= MAX_STATUS_WORDS) {
        code = -1; // index out of range
    }
    else {
        // Relative index for 32-bit status word
        status_bit = 1 << (index % 32);
        if (set!=0) {
            arr[status_word] |= status_bit;     // Assert status
            code = 1;
        } else {
            arr[status_word] &= ~status_bit;    // Clear Status
            code = 0;
        }
    }
    return code;
}
