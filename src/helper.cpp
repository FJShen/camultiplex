#include <chrono>
#include "camultiplex/helper.h"
#include <iostream>
#include <sstream>
#include <string>


void helper::print_time_stamp(std::string comment) {
    std::cout << comment << " "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::high_resolution_clock::now().time_since_epoch()).count()
              << "\t";
}


std::string helper::get_time_stamp_str() {
    std::stringstream ss;
    ss << std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    return ss.str();
}
