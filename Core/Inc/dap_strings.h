/**
 * @file    dap_strings.h
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2019, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "string.h"

#define __STATIC_INLINE static inline

/** Get Vendor ID string.
\param str Pointer to buffer to store the string.
\return String length.
*/
__STATIC_INLINE uint8_t DAP_GetVendorString(char *str) {
    (void) str;
    return (0U);
}

/** Get Product ID string.
\param str Pointer to buffer to store the string.
\return String length.
*/
__STATIC_INLINE uint8_t DAP_GetProductString(char *str) {
    (void) str;
    return (0U);
}

/** Get Serial Number string.
\param str Pointer to buffer to store the string.
\return String length.
*/
__STATIC_INLINE uint8_t DAP_GetSerNumString(char *str) {
    const char unique_id[49] = "2333333333";
    const char *data = unique_id;
    uint8_t length = (uint8_t) strlen(data) + 1;
    memcpy(str, data, length);
    return length;
}

/** Get firmware version string.
\param str Pointer to buffer to store the string.
\return String length.
*/
__STATIC_INLINE uint8_t DAP_ProductFirmwareVerString(char *str) {
    const char DAPLINK_VERSION[] = "0256";
    const char *data = DAPLINK_VERSION;
    uint8_t length = (uint8_t) strlen(data) + 1;
    memcpy(str, data, length);
    return length;
}
#ifdef __cplusplus
}
#endif