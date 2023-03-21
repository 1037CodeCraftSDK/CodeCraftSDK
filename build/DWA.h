#pragma once
#ifndef CHHROBOTICS_CPP_DWA_H
#define CHHROBOTICS_CPP_DWA_H

#include <iostream>
#include<vector>
#include<cmath>
#include<algorithm>
#include <fstream>

void WriteString(const char* lpszText);

using namespace std;


#define PI 3.1415926

class Vector2d
{
private:
    float x,y;
public:
    Vector2d(){}
    Vector2d(float x1,float y1):x(x1),y(y1){}
    ~Vector2d(){}
    inline Vector2d operator-(const Vector2d& sub){ return Vector2d(x - sub.x, y - sub.y); }
    float operator()(int i){ return i == 0 ? x : y;}
    float norm(){ return sqrt(x * x + y * y); }
private:

};



class DWA {
private:
    float dt; //采样时间
    float v_min, v_max, w_min, w_max; //线速度角速度边界
    float predict_time;//轨迹推算时间长度
    float a_vmax, a_wmax; //线加速度和角加速度最大值
    float v_sample, w_sample; //采样分辨率
    float alpha, beta, gamma; //轨迹评价函数系数
    float radius_bench; // 用于判断是否到达工作台
    float radius_robot; // 用于判断是否碰撞机器人
    float radius_wall;  //用于判断碰撞墙壁
    float judge_distance; //若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
private:
    vector<float> calVelLimit();
    vector<float> calAccelLimit(float v, float w);
    vector<float> calObstacleLimit(vector<float> state, vector<Vector2d> obstacle);
    vector<float> DWA::calWorkBenchLimit(vector<float> state, Vector2d workBnech);
    vector<float> calDynamicWindowVel(float v, float w, vector<float> state, vector<Vector2d> obstacle);
    float _dist(vector<float> state, vector<Vector2d>obstacle);
    vector<vector<float>> trajectoryPredict(vector<float> state, float v, float w);
    vector<float> trajectoryEvaluation(vector<float>& state, Vector2d& goal, vector<Vector2d>& obstacle);

    float _heading(vector<vector<float>> trajectory, Vector2d goal);
    float _velocity(vector<vector<float>> trajectory);
    float _distance(vector<vector<float>> trajectory, vector<Vector2d> obstacle);
    int _distance_wall(vector<vector<float>> trajectory);

public:
    DWA(float dt, float vMin, float vMax, float wMin, float wMax, float predictTime, float aVmax, float aWmax,
        float vSample, float wSample, float alpha, float beta, float gamma, float radius_bench, float radius_robot,float radius_wall, float judgeDistance);

    vector<float> kinematicModel(vector<float>& state, vector<float>control, float dt);

    vector<float>dwaControl(vector<float>& state, Vector2d& goal, vector<Vector2d>& obstacle);
};


#endif 