//
// Created by wbai on 6/16/2022.
//

#ifndef MCU1_BARCODE_H
#define MCU1_BARCODE_H

#include <vector>
#include <iostream>

enum special_code {
    start_code_a = 103,
    start_code_b = 104,
    start_code_c = 105,
    stop_code = 106,
    code_c = 99,
    shift_b = 98,
    shift_a = 98,
    code_b = 100,
    code_a = 101
};

enum char_set{
    char_set_a = 1,
    char_set_b = 2,
    char_set_c = 3
};

std::vector<int> barcode_widths(std::string data, uint8_t min_pixels_a_bar);


#endif //MCU1_BARCODE_H
