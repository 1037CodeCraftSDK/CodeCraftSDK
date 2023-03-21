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
    float dt; //����ʱ��
    float v_min, v_max, w_min, w_max; //���ٶȽ��ٶȱ߽�
    float predict_time;//�켣����ʱ�䳤��
    float a_vmax, a_wmax; //�߼��ٶȺͽǼ��ٶ����ֵ
    float v_sample, w_sample; //�����ֱ���
    float alpha, beta, gamma; //�켣���ۺ���ϵ��
    float radius_bench; // �����ж��Ƿ񵽴﹤��̨
    float radius_robot; // �����ж��Ƿ���ײ������
    float radius_wall;  //�����ж���ײǽ��
    float judge_distance; //�����ϰ������С���������ֵ�������������õ���ֵΪrobot_radius+0.2��,����Ϊһ���ϴ�ĳ�ֵ
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