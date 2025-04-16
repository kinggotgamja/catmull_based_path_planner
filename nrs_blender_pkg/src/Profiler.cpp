/*
    This code is publicly available on GitHub.
    You are free to use, modify, and distribute it.
    However, you must clearly credit the source as follows:

    Source: https://github.com/kinggotgamja/nrs_blender_pkg
    Author: Jaeyoon Shim
*/

#include "nrs_blender_pkg/Profiler.h"
#include <cmath>
#include <fstream>
using namespace std;

// Profiler Constructor
Profiler::Profiler(const vector<double>& time, const vector<double>& data, 
                   double StartingTime, double LastRestingTime, 
                   double AccelerationTime, double SamplingTime)
    : time(time), data(data), StartingTime(StartingTime), 
      LastRestingTime(LastRestingTime), AccelerationTime(AccelerationTime), 
      SamplingTime(SamplingTime) {}

// NRSProfiler Constructor
NRSProfiler::NRSProfiler(const vector<double>& time, const vector<double>& data, 
                         double StartingTime, double LastRestingTime, 
                         double AccelerationTime, double SamplingTime)
    : Profiler(time, data, StartingTime, LastRestingTime, AccelerationTime, SamplingTime) {}

// NRS Acc-Dcc Profiling
vector<vector<double>> NRSProfiler::AccDecProfiling() {
    vector<vector<double>> Final_pos_interval;
    vector<double> Target_velocity(time.size(), 0.0);
    double Ti = StartingTime;
    double Ta = AccelerationTime;
    double Ts = SamplingTime;
    double Tl = LastRestingTime;
    double Tf = Ti + Ts * data.size() + Tl;

    // Target Velocity Calculation
    for (size_t j = 1; j < data.size(); j++) {
        Target_velocity[j] = (data[j] - data[j - 1]) / Ts;
    }

    vector<double> t;
    for (double i = 0; i <= Tf; i += Ts) {
        t.push_back(i);
    }

    // Interpolation
    vector<vector<double>> Interpolated(t.size(), vector<double>(2, 0.0));
    size_t Last_flag = 0;
    for (size_t i = 0; i < t.size(); i++) {
        if (t[i] <= Ti) {
            Interpolated[i] = { t[i], 0 };
            Last_flag++;
        } else if (t[i] <= Ti + Ts * data.size()) {
            Interpolated[i] = { t[i], Target_velocity[i - Last_flag] };
        } else {
            Interpolated[i] = { t[i], 0 };
        }
    }

    // Velocity Profiling
    double m = Ta / Ts;
    vector<vector<double>> Final(t.size(), vector<double>(2, 0.0));
    Final_pos_interval.push_back({ time[0], data[0] });

    for (size_t i = 1; i < t.size(); i++) {
        if (i <= m) {
            Final[i] = { t[i], Final[i - 1][1] + (Interpolated[i][1] - Interpolated[0][1]) / (i) };
        } else {
            Final[i] = { t[i], Final[i - 1][1] + (Interpolated[i][1] - Interpolated[i - (int)m][1]) / m };
        }
        Final_pos_interval.push_back({ t[i], Final_pos_interval[i - 1][1] + Final[i][1] * Ts });
    }

    return Final_pos_interval;
}

// Display function
void Profiler::DisplayData(const vector<vector<double>>& outputData) {
    for (const auto& row : outputData) {
        cout << row[0] << ", " << row[1] << endl;
    }
}

// Save the data to a file
void Profiler::SaveResultToFile(const vector<vector<double>>& outputData, const string& filename) {
    ofstream file(filename);
    if (file.is_open()) {
        for (const auto& row : outputData) {
            for (const auto& col : row)
            {
                file << col << " ";
            }
            file << endl;
        }
        file.close();
        cout << "Result was recorded at " << filename << endl;
    } else {
        cerr << "Can not open the file: " << filename << endl;
    }
}
