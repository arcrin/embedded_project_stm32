//
// Created by wbai on 6/16/2022.
//

#include <vector>
#include <iostream>
#include <cstring>
#include "barcode.h"


const char* value_width_map[107] = { // TODO: why does std::string work here?
        "212222" ,
        "222122" ,
        "222221" ,
        "121223" ,
        "121322" ,
        "131222" ,
        "122213" ,
        "122312" ,
        "132212" ,
        "221213" ,
        "221312" ,
        "231212" ,
        "112232" ,
        "122132" ,
        "122231" ,
        "113222" ,
        "123122" ,
        "123221" ,
        "223211" ,
        "221132" ,
        "221231" ,
        "213212" ,
        "223112" ,
        "312131" ,
        "311222" ,
        "321122" ,
        "321221" ,
        "312212" ,
        "322112" ,
        "322211" ,
        "212123" ,
        "212321" ,
        "232121" ,
        "111323" ,
        "131123" ,
        "131321" ,
        "112313" ,
        "132113" ,
        "132311" ,
        "211313" ,
        "231113" ,
        "231311" ,
        "112133" ,
        "112331" ,
        "132131" ,
        "113123" ,
        "113321" ,
        "133121" ,
        "313121" ,
        "211331" ,
        "231131" ,
        "213113" ,
        "213311" ,
        "213131" ,
        "311123" ,
        "311321" ,
        "331121" ,
        "312113" ,
        "312311" ,
        "332111" ,
        "314111" ,
        "221411" ,
        "431111" ,
        "111224" ,
        "111422" ,
        "121124" ,
        "121421" ,
        "141122" ,
        "141221" ,
        "112214" ,
        "112412" ,
        "122114" ,
        "122411" ,
        "142112" ,
        "142211" ,
        "241211" ,
        "221114" ,
        "413111" ,
        "241112" ,
        "134111" ,
        "111242" ,
        "121142" ,
        "121241" ,
        "114212" ,
        "124112" ,
        "124211" ,
        "411212" ,
        "421112" ,
        "421211" ,
        "212141" ,
        "214121" ,
        "412121" ,
        "111143" ,
        "111341" ,
        "131141" ,
        "114113" ,
        "114311" ,
        "411113" ,
        "411311" ,
        "113141" ,
        "114131" ,
        "311141" ,
        "411131" ,
        "211412" ,
        "211214" ,
        "211232" ,
        "2331112",
};

std::vector<int> barcode_widths(std::string data, uint8_t min_pixels_a_bar){
    std::vector<int> values;
    std::vector <int> widths;
    // let's deal with single char first
    unsigned int pos = 0;
    char_set current_char_set;
    // determine start code
    if(std::isdigit(data[pos]) // TODO: still need to optimize
       and std::isdigit(data[pos + 1])
       and std::isdigit(data[pos + 2])
       and std::isdigit(data[pos + 3])){
//        start_code = start_code_c;
        values.push_back(start_code_c);
        current_char_set = char_set_c;
    } else{
        // since code128b includes all the alphanumeric characters, we use this character set by default
        // it is unlikely any test jig instrument number contains special characters between 0x00 and 0x1F
//        start_code = start_code_b;
        values.push_back(start_code_b);
        current_char_set = char_set_b;
    }
    while (pos < data.length()){
        // check double digits, if there is a double-digit, shift to code128c
        if (std::isdigit(data[pos])
        and std::isdigit(data[pos + 1])
        and std::isdigit(data[pos + 2])
        and std::isdigit(data[pos + 3])) {
            if (current_char_set != char_set_c) {
                values.push_back(code_c);
                current_char_set = char_set_c;
            }
            std::string double_digit_str(data.substr(pos, 2));
            uint8_t double_digit_int = std::strtol(double_digit_str.data(), nullptr, 0);
            values.push_back(double_digit_int);
            if (pos + 2 < data.length()) {
                pos += 2;
            } else {
                pos += 1;
            }
        }
        else if ( 0x00 <= data[pos] and data[pos] <= 0x1f){
            if (current_char_set == char_set_b) {
                values.push_back(shift_a);
            } else if (current_char_set == char_set_c) {
                values.push_back(code_a);
            }
            current_char_set = char_set_a;
            values.push_back(data[pos] + 0x40);
            pos += 1;
        } else{
            if (current_char_set != char_set_b) {
                values.push_back(code_b);
                current_char_set = char_set_b;
            }
            values.push_back(data[pos] - 0x20);
            pos += 1;
        }
    }
    uint16_t weight_sum = 0;
    for (unsigned int i = 1; i < values.size(); i++) {
        weight_sum += i * values[i];
    }
    weight_sum += values[0];
    uint8_t check_value = weight_sum % 103;
    printf("Check value: %d \n", check_value);
    values.push_back(check_value);
    values.push_back(stop_code);
    std::vector<int> bar_widths;
    for (int entry: values) {
        for (unsigned int i = 0; i < strlen(value_width_map[entry]); i++) {
            bar_widths.push_back((value_width_map[entry][i] - '0') * min_pixels_a_bar);
        }
    }
    return bar_widths;
}