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


void helper::print_intrinsics(rs2_intrinsics const& i, std::string s) {
    std::cout << "Printing intrinsics of \"" << s << "\":\n";
    std::cout << "width=" << i.width << "\n"
              << "height=" << i.height << "\n"
              << "ppx=" << i.ppx << "\n"
              << "ppy=" << i.ppy << "\n"
              << "fx=" << i.fx << "\n"
              << "fy=" << i.fy << "\n"
              << "rs2_distortion=" << i.model << "\n"
              << "coeff[0-4]=";
    for (int idx = 0; idx < 5; ++idx) {
        std::cout << i.coeffs[idx] << " ";
    }
    std::cout << "\n\n";
}

void helper::print_extrinsics(rs2_extrinsics const& ex, std::string s){
    std::cout << "Printing extrinsics of \"" << s << "\":\n";
    std::cout<<"Rotation[0-8]=";
    for (int idx = 0; idx < 9; ++idx) {
        std::cout << ex.rotation[idx] << " ";
    }
    std::cout<<"\n";
    
    std::cout<<"Translation[0-2]=";
    for (int idx = 0; idx < 3; ++idx) {
        std::cout << ex.translation[idx] << " ";
    }
    std::cout<<"\n\n";
}