/*
 * base64_encode.h
 *
 *  Created on: Mar 27, 2021
 *      Author: chrau
 */

#ifndef SRC_BASE64_ENCODE_H_
#define SRC_BASE64_ENCODE_H_

#include <stdint.h>
#include <stdlib.h>

/* data: input array
 * input_length: number of bytes in input array
 * encoded_data: output array with encoded data; must be at least 4 * ((input_length + 2) / 3) long!
 * output_length: number of characters in output array
 */
char *base64_encode(const uint8_t *data, size_t input_length, char *encoded_data, size_t *output_length);


#endif /* SRC_BASE64_ENCODE_H_ */
