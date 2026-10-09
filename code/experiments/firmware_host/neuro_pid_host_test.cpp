#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include "Neuro_PID.h"

int main(int argc, char** argv) {
    std::uint64_t iterations = 1000000U;
    if (argc == 2) iterations = std::stoull(argv[1]);
    if (iterations < 1000U) { std::cerr << "iterations must be >= 1000\n"; return 2; }
    Neuro_PID controller;
    float minimum[3] = {std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()};
    float maximum[3] = {-std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};
    std::uint64_t grid_count=0U; bool finite=true;
    for(int ei=-8;ei<=8;++ei) for(int di=-8;di<=8;++di) for(int ii=-8;ii<=8;++ii){
        float kp=0,ki=0,kd=0; controller.computePID(ei*0.25F,di*0.25F,ii*0.25F,kp,ki,kd); const float values[3]={kp,ki,kd};
        for(int i=0;i<3;++i){ finite=finite&&std::isfinite(values[i]); minimum[i]=std::fmin(minimum[i],values[i]); maximum[i]=std::fmax(maximum[i],values[i]); } ++grid_count;
    }
    const bool bounded=minimum[0]>=0.0F&&maximum[0]<=8.0F&&minimum[1]>=0.0F&&maximum[1]<=2.0F&&minimum[2]>=0.0F&&maximum[2]<=0.2F;
    volatile float checksum=0.0F; const auto start=std::chrono::steady_clock::now();
    for(std::uint64_t index=0;index<iterations;++index){ float kp=0,ki=0,kd=0; controller.computePID((int(index%201U)-100)*0.01F,(int(index%101U)-50)*0.01F,(int(index%81U)-40)*0.01F,kp,ki,kd); checksum=checksum+kp+ki+kd; }
    const auto end=std::chrono::steady_clock::now(); const double elapsed_ns=double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count());
    std::cout<<std::setprecision(12)<<"{\n"<<"  \"finite\": "<<(finite?"true":"false")<<",\n"<<"  \"bounded\": "<<(bounded?"true":"false")<<",\n"<<"  \"grid_count\": "<<grid_count<<",\n"<<"  \"kp_min\": "<<minimum[0]<<", \"kp_max\": "<<maximum[0]<<",\n"<<"  \"ki_min\": "<<minimum[1]<<", \"ki_max\": "<<maximum[1]<<",\n"<<"  \"kd_min\": "<<minimum[2]<<", \"kd_max\": "<<maximum[2]<<",\n"<<"  \"iterations\": "<<iterations<<",\n"<<"  \"elapsed_ms\": "<<elapsed_ns/1.0e6<<",\n"<<"  \"ns_per_inference\": "<<elapsed_ns/double(iterations)<<",\n"<<"  \"inferences_per_second\": "<<double(iterations)*1.0e9/elapsed_ns<<",\n"<<"  \"checksum\": "<<checksum<<"\n}\n";
    return (finite&&bounded)?0:1;
}
