/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2022, Mech-Mind Robotics
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 *3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <iostream>
#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "OpenCVUtil.h"
#include <chrono>
using namespace std;
using namespace chrono;

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::DepthMap depth;
    mmind::api::ColorMap color;

    // New way for counting time
    //vector<double> tCount;
    //int i = 0;
    //while (i < 15) {
    //    auto start = system_clock::now();
    //    showError(device.captureDepthMap(depth));
    //    auto end = system_clock::now();
    //    auto duration = duration_cast<microseconds>(end - start);
    //    std::cout << "Capture time is " << double(duration.count()) << " mius." << std::endl;
    //    tCount.push_back(double(duration.count()));
    //    i++;
    //}
    //double sum = 0.0;
    //for (int i = 5; i < 15; i++) {
    //    sum += tCount[i];
    //}
    //double ave_time = sum / 10.0;
    //std::cout << "(NEW METHOD) Average captureDepthMap: " << ave_time << " mius." << std::endl;
    
    // Classic way for counting time
    vector<double> timeCount;
    int i = 0;
    while (i < 15) {
        clock_t start, end;
        start = clock();
        showError(device.captureDepthMap(depth));
        showError(device.captureColorMap(color));
        end = clock();
        std::cout << "captureDepthMap: " << (double)(end - start) << "ms" << std::endl;
        timeCount.push_back((double)(end - start));
        i++;
    }
    double sum = 0.0;
    for (int i = 5; i < 15; i++) {
        sum += timeCount[i];
    }
    double ave_time = sum / 10.0;
    std::cout << "Average captureDepthMap: " << ave_time << "ms" << std::endl;

    // Other works
    const std::string depthFile = "DepthMap.png";
    //saveMap(depth, depthFile);

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
