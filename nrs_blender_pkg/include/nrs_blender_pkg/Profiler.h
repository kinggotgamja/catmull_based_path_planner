/*
    This code is publicly available on GitHub.
    You are free to use, modify, and distribute it.
    However, you must clearly credit the source as follows:

    Source: https://github.com/kinggotgamja/nrs_blender_pkg
    Author: Jaeyoon Shim
*/

#ifndef PROFILER_H
#define PROFILER_H

#include <vector>
#include <string>
#include <iostream>
using namespace std;

// Basic Profiler Class
class Profiler {
protected:
    vector<double> time;
    vector<double> data;
    double SamplingTime;
    double StartingTime;
    double LastRestingTime;
    double AccelerationTime;

public:
    // Constructor
    Profiler(const vector<double>& time, const vector<double>& data, 
             double StartingTime, double LastRestingTime, 
             double AccelerationTime, double SamplingTime);

    // Virtual Function
    virtual vector<vector<double>> AccDecProfiling() = 0;

    // Utility Function
    void DisplayData(const vector<vector<double>>& outputData);

    // Save the result to a txt file
    void SaveResultToFile(const vector<vector<double>>& outputData, const string& filename);
};

// NRS Profiler Class (Corresponding the NRS_acc_dec_profiling)
class NRSProfiler : public Profiler {
public:
    // Constructor
    NRSProfiler(const vector<double>& time, const vector<double>& data, 
                double StartingTime, double LastRestingTime, 
                double AccelerationTime, double SamplingTime);

    // NRS Acc-Dec Profiling
    vector<vector<double>> AccDecProfiling() override;
};

#endif
