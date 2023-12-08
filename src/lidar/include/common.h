//
// Created by kiana on 23-11-13.
//

#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <Eigen/Core>
#include <io.h>
#include <stdlib.h>
#include <sys/types.h>
#include <dirent.h>

using namespace std;

void getIntrinsic(const string path, vector<float> &intrinsic);
void getDistortion(const string path, vector<float> &distortion);
void getExtrinsic(const string path, vector<float> &extrinsic);

// convert a int to a string
string int2str(int num)
{
    ostringstream oss;
    if (oss << num)
    {
        string str(oss.str());
        return str;
    }
    else
    {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

int str2int(string str)
{
    int d;
    stringstream sin(str);
    if (sin >> d)
    {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to int" << endl;
    exit(0);
}

string float2str(float num)
{
    ostringstream oss;
    if (oss << num)
    {
        string str(oss.str());
        return str;
    }
    else
    {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

float str2float(string str)
{
    float d;
    stringstream sin(str);
    if (sin >> d)
    {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to float" << endl;
    exit(0);
}

string double2str(double num)
{
    ostringstream oss;
    if (oss << num)
    {
        string str(oss.str());
        return str;
    }
    else
    {
        cout << "[double2str] - Code error" << endl;
        exit(0);
    }
}

double str2double(string str)
{
    double d;
    stringstream sin(str);
    if (sin >> d)
    {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to double" << endl;
    exit(0);
}

string long2str(long num)
{
    ostringstream oss;
    if (oss << num)
    {
        string str(oss.str());
        return str;
    }
    else
    {
        cout << "[long2str] - Code error" << endl;
        exit(0);
    }
}

// 读取txt文件，降内参矩阵展开为一个vector
void getIntrinsic(const string path, vector<float> &intrinsic)
{
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open())
    {
        cout << "Can not open file " << path << endl;
        exit(1);
    }

    string lineStr;
    getline(inFile, lineStr); // 读取第一行，然后忽略掉
    for (uint i = 0; i < 3; ++i)
    {
        getline(inFile, lineStr);
        stringstream line(lineStr);
        string str;

        line >> str;
        intrinsic.push_back(str2double(str));
        line >> str;
        intrinsic.push_back(str2double(str));
        line >> str;
        intrinsic.push_back(str2double(str));
    }
}

void getDistortion(const string path, vector<float> &distortion)
{
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open())
    {
        cout << "Can not open file " << path << endl;
        exit(1);
    }
    // 忽略前6行
    string lineStr;
    for (uint i = 0; i < 6; ++i)
    {
        getline(inFile, lineStr);
    }

    getline(inFile, lineStr);
    stringstream line(lineStr);
    string str;

    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
}

void getExtrinsic(const string path, vector<float> &extrinsic)
{
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open())
    {
        cout << "Can not open file " << path << endl;
        exit(1);
    }

    string lineStr;
    getline(inFile, lineStr);
    for (uint i = 0; i < 3; ++i)
    {
        getline(inFile, lineStr);
        stringstream line(lineStr);
        string str;

        line >> str;
        extrinsic.push_back(str2double(str));
        line >> str;
        extrinsic.push_back(str2double(str));
        line >> str;
        extrinsic.push_back(str2double(str));
        line >> str;
        extrinsic.push_back(str2double(str));
    }
}

#endif // PROJECT_COMMON_H
