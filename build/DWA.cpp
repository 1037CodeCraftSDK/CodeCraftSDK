#include "DWA.h"


void WriteString(const char* lpszText)
{
    if (!lpszText)return;

    std::ofstream out("C:\\Users\\14626\\Desktop\\huawei\\WindowsRelease\\WindowsRelease\\1.txt", std::ios::app);
    if (out.is_open())
    {
        out << lpszText << std::endl;
        out.close();
    }
}
DWA::DWA(float dt, float vMin, float vMax, float wMin, float wMax, float predictTime, float aVmax, float aWmax,
        float vSample, float wSample, float alpha, float beta, float gamma, float radius_bench, float radius_robot, float radius_wall, float judgeDistance)
        : dt(dt), v_min(vMin), v_max(vMax), w_min(wMin), w_max(wMax), predict_time(predictTime), a_vmax(aVmax),
          a_wmax(aWmax), v_sample(vSample), w_sample(wSample), alpha(alpha), beta(beta), gamma(gamma),
          radius_bench(radius_bench),radius_robot(radius_robot),radius_wall(radius_wall),
          judge_distance(judgeDistance) {}

/**
 * �����ٶȱ߽�����Vm
 * @return �ٶȱ߽����ƺ���ٶȿռ�Vm
 */
vector<float> DWA::calVelLimit() {
    return {v_min,v_max,w_min,w_max };
}

/**
 * ������ٶ�����Vd
 * @return
 */
vector<float> DWA::calAccelLimit(float v, float w) {
    float v_low = v-a_vmax*dt;
    float v_high = v+a_vmax*dt;
    float w_low = w-a_wmax*dt;
    float w_high = w+a_wmax*dt;
    return {v_low, v_high,w_low, w_high};
}

/**
 * �����ϰ�������Va
 * @param state ��ǰ������״̬
 * @param obstacle �ϰ���λ��
 * @return �ƶ������˲�����Χ�ϰ��﷢����ײ���ٶȿռ�Va
 */
vector<float> DWA::calObstacleLimit(vector<float> state, vector<Vector2d> obstacle) {
    float v_low=v_min;
    float v_high = sqrt(2*_dist(state,obstacle)*a_vmax);
    float w_low =w_min;
    float w_high = sqrt(2*_dist(state,obstacle)*a_wmax);
    return {v_low, v_high,w_low, w_high};
}

/**
 * �ٶȲ���,�õ��ٶȿռ䴰��
 * @param v ��ǰʱ�����ٶ�
 * @param w ��ǰʱ�̽��ٶ�
 * @param state ��ǰ������״̬
 * @param obstacle �ϰ���λ��
 * @return [v_low,v_high,w_low,w_high]: ���ղ�������ٶȿռ�
 */
vector<float> DWA::calDynamicWindowVel(float v, float w,vector<float> state, vector<Vector2d> obstacle) {


    vector<float> Vm = calVelLimit();
    vector<float> Vd = calAccelLimit(v,w);
    vector<float> Va = calObstacleLimit(state,obstacle);
    float a = max({Vm[0],Vd[0],Va[0]});
    float b = min({Vm[1],Vd[1],Va[1]});
    float c = max({Vm[2], Vd[2],Va[2]});
    float d = min({Vm[3], Vd[3],Va[3]});
    return {a,b,c,d};
}



/**
 * ���㵱ǰ�ƶ������˾����ϰ�������ļ��ξ���
 * @param state ��ǰ������״̬
 * @param obstacle �����ϰ���λ��
 * @return �ƶ������˾����ϰ�������ļ��ξ���
 */
float DWA::_dist(vector<float> state, vector<Vector2d> obstacle) {
    float min_dist = 100000;
    for(Vector2d obs:obstacle){
        Vector2d location(state[0],state[1]);
        float distance = (obs-location).norm();
        min_dist = distance>min_dist?min_dist:distance;
    }
    float gap = 0;
    float mx = min(50- gap - state[0]>0? 50 - gap - state[0]:0, state[0]-gap>0? state[0] - gap:0);
    float my = min(50- gap - state[1]>0? 50 - gap - state[1]:0, state[1]-gap>0? state[1] - gap:0);
    float md = min(mx, my);

    min_dist = min_dist > md ? md : min_dist;

    return min_dist;
}

/**
 * �������˶�ѧģ��
 * @param state ״̬��---x,y,yaw,v,w
 * @param control ������---v,w,���ٶȺͽ��ٶ�
 * @param dt ����ʱ��
 * @return ��һ����״̬
 */
vector<float> DWA::kinematicModel(vector<float>& state, vector<float> control, float dt) {

    state[0] += control[0] * cos(state[2]) * dt;
    state[1] += control[0] * sin(state[2]) * dt;
    state[2] += control[1] * dt;
    state[3] = control[0];
    state[4] = control[1];

    return state;
}

/**
 * �켣����
 * @param state ��ǰ״̬---x,y,yaw,v,w
 * @param v ��ǰʱ�����ٶ�
 * @param w ��ǰʱ�����ٶ�
 * @return �����Ĺ켣
 */
vector<vector<float>> DWA::trajectoryPredict(vector<float> state, float v, float w) {
    vector<vector<float>> trajectory;
    trajectory.push_back(state);
    float time =0;
    while(time<=predict_time){
        state = kinematicModel(state,{v,w},dt);
        trajectory.push_back(state);
        time+=dt;
    }
    return trajectory;
}

/**
 * �켣���ۺ���,����Խ�ߣ��켣Խ��
 * @param state ��ǰ״̬---x,y,yaw,v,w
 * @param goal Ŀ���λ�ã�[x,y]
 * @param obstacle �ϰ���λ�ã�dim:[num_ob,2]
 * @return ���ſ����������Ź켣
 */
vector<float> DWA::trajectoryEvaluation(vector<float>& state, Vector2d& goal, vector<Vector2d>& obstacle) {
    float G_max = -10000000; //��������
    vector<vector<float>> trajectory_opt; //���Ź켣
    trajectory_opt.push_back(state);
    vector<float>control_opt={0.,0.}; // ���ſ���
    vector<float>dynamic_window_vel = calDynamicWindowVel(state[3],state[4],state,obstacle);//��1��--�����ٶȿռ�

    float sum_heading =0.0,sum_dist=0.0,sum_vel=0.0;//ͳ��ȫ�������켣�ĸ�������֮�ͣ��������۵Ĺ�һ��
    float v = dynamic_window_vel[0];
    float w = dynamic_window_vel[2];
    //while(v<dynamic_window_vel[1]){
    //    while(w<dynamic_window_vel[3]){
    //        vector<vector<float>>trajectory = trajectoryPredict(state,v,w);
    //        float heading_eval = alpha*_heading(trajectory,goal);
    //        float dist_eval = beta*_distance(trajectory,obstacle);
    //        float vel_eval = gamma*_velocity(trajectory);
    //        sum_vel+=vel_eval;
    //        sum_dist+=dist_eval;
    //        sum_heading+=heading_eval;
    //        w+=w_sample;
    //    }
    //    v+=v_sample;
    //}
    sum_heading =1.0,sum_dist=1.0,sum_vel=1.0;//�����й�һ��
    v = dynamic_window_vel[0];
    w = dynamic_window_vel[2];
    while(v<dynamic_window_vel[1]){
        bool b = false;
        while(w<dynamic_window_vel[3]){
            vector<vector<float>>trajectory = trajectoryPredict(state,v,w);//��2��--�켣����

            float heading_eval = alpha*_heading(trajectory,goal)/sum_heading;
            float dis =  _distance(trajectory, obstacle);
            float dist_eval, vel_eval, G;
            dist_eval = beta*dis/sum_dist;
            vel_eval = gamma*_velocity(trajectory)/sum_vel;
            if (dis == -1.0)
            {
                G = heading_eval*10 + dist_eval + vel_eval;
            }
            else
            {

                G = heading_eval+dist_eval+vel_eval; // ��3��--�켣����
            }

            if(G_max<=G){
                G_max = G;
                //trajectory_opt=trajectory;
                control_opt={v,w};
            }
            w+=w_sample;
        }
        v+=v_sample;
        w = dynamic_window_vel[2];
    }
    return control_opt;

}

/**
 * ��λ�����ۺ���
 * �����ڵ�ǰ�����ٶ��²����Ĺ켣�յ�λ�÷�����Ŀ������ߵļнǵ����
 * @param trajectory �켣��dim:[n,5]
 * @param goal Ŀ���λ��[x,y]
 * @return ��λ��������ֵ
 */
float DWA::_heading(vector<vector<float>> trajectory, Vector2d goal) {
    Vector2d location(trajectory[trajectory.size() - 1][0], trajectory[trajectory.size() - 1][1]);
    Vector2d dxy = goal-location;
    float error_angle = atan2(dxy(1),dxy(0));
    float cost_angle = abs(error_angle-trajectory[trajectory.size()-1][2]);
    float cost;

    cost = cost_angle > PI? 2 * PI - cost_angle:cost_angle;
    return 2*PI-cost;
}

/**
 * �ٶ����ۺ���
 * ��ʾ��ǰ���ٶȴ�С��������ģ��켣ĩ��λ�õ����ٶȵĴ�С����ʾ
 * @param trajectory �켣��dim:[n,5]
 * @return �ٶ�����ֵ
 */
float DWA::_velocity(vector<vector<float>> trajectory) {
    return  trajectory[trajectory.size()-1][3];
}

/**
 * �������ۺ���
 * ��ʾ��ǰ�ٶ��¶�Ӧģ��켣���ϰ���֮���������룻
 * ���û���ϰ�����������������趨����ֵ����ô�ͽ���ֵ��Ϊһ���ϴ�ĳ���ֵ��
 * @param trajectory �켣��dim:[n,5]
 * @param obstacle �ϰ���λ�ã�dim:[num_ob,2]
 * @return ��������ֵ
 */
float DWA::_distance(vector<vector<float>> trajectory, vector<Vector2d> obstacle) {
    float min_r = 10000000;
    for(Vector2d obs:obstacle){
        for(vector<float> state:trajectory){
            Vector2d location(state[0],state[1]);
            Vector2d dxy = obs-location;
            float r = dxy.norm();
            min_r = min_r>r?r:min_r;
        }
    }

    //for (vector<float> state : trajectory)
    //{
    //    if (state[0] >= 50 - radius_wall || state[0] <= radius_wall || state[1] >= 50 - radius_wall || state[1] <= radius_wall)
    //    {
    //        return -1.0;
    //    }
    //}
    
    if(min_r<radius_bench+0.2){
        return min_r;
    }else{
        return judge_distance;
    }
}

/**
 * ���������㷨������
 * @param state �����˵�ǰ״̬--[x,y,yaw,v,w]
 * @param goal Ŀ���λ�ã�[x,y]
 * @param obstacle �ϰ���λ�ã�dim:[num_ob,2]
 * @return  ���ſ�����[v,w]�����Ź켣
 */
vector<float> DWA::dwaControl(vector<float>& state, Vector2d& goal, vector<Vector2d>& obstacle) {
    vector<float> res = trajectoryEvaluation(state,goal,obstacle);
    return res;
}