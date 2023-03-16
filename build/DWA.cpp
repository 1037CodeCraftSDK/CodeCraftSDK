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
 * 计算速度边界限制Vm
 * @return 速度边界限制后的速度空间Vm
 */
vector<float> DWA::calVelLimit() {
    return {v_min,v_max,w_min,w_max };
}

/**
 * 计算加速度限制Vd
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
 * 环境障碍物限制Va
 * @param state 当前机器人状态
 * @param obstacle 障碍物位置
 * @return 移动机器人不与周围障碍物发生碰撞的速度空间Va
 */
vector<float> DWA::calObstacleLimit(vector<float> state, vector<Vector2d> obstacle) {
    float v_low=v_min;
    float v_high = sqrt(2*_dist(state,obstacle)*a_vmax);
    float w_low =w_min;
    float w_high = sqrt(2*_dist(state,obstacle)*a_wmax);
    return {v_low, v_high,w_low, w_high};
}

/**
 * 速度采样,得到速度空间窗口
 * @param v 当前时刻线速度
 * @param w 当前时刻角速度
 * @param state 当前机器人状态
 * @param obstacle 障碍物位置
 * @return [v_low,v_high,w_low,w_high]: 最终采样后的速度空间
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
 * 计算当前移动机器人距离障碍物最近的几何距离
 * @param state 当前机器人状态
 * @param obstacle 所有障碍物位置
 * @return 移动机器人距离障碍物最近的几何距离
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
 * 机器人运动学模型
 * @param state 状态量---x,y,yaw,v,w
 * @param control 控制量---v,w,线速度和角速度
 * @param dt 采样时间
 * @return 下一步的状态
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
 * 轨迹推算
 * @param state 当前状态---x,y,yaw,v,w
 * @param v 当前时刻线速度
 * @param w 当前时刻线速度
 * @return 推算后的轨迹
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
 * 轨迹评价函数,评价越高，轨迹越优
 * @param state 当前状态---x,y,yaw,v,w
 * @param goal 目标点位置，[x,y]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return 最优控制量、最优轨迹
 */
vector<float> DWA::trajectoryEvaluation(vector<float>& state, Vector2d& goal, vector<Vector2d>& obstacle) {
    float G_max = -10000000; //最优评价
    vector<vector<float>> trajectory_opt; //最优轨迹
    trajectory_opt.push_back(state);
    vector<float>control_opt={0.,0.}; // 最优控制
    vector<float>dynamic_window_vel = calDynamicWindowVel(state[3],state[4],state,obstacle);//第1步--计算速度空间

    float sum_heading =0.0,sum_dist=0.0,sum_vel=0.0;//统计全部采样轨迹的各个评价之和，便于评价的归一化
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
    sum_heading =1.0,sum_dist=1.0,sum_vel=1.0;//不进行归一化
    v = dynamic_window_vel[0];
    w = dynamic_window_vel[2];
    while(v<dynamic_window_vel[1]){
        bool b = false;
        while(w<dynamic_window_vel[3]){
            vector<vector<float>>trajectory = trajectoryPredict(state,v,w);//第2步--轨迹推算

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

                G = heading_eval+dist_eval+vel_eval; // 第3步--轨迹评价
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
 * 方位角评价函数
 * 评估在当前采样速度下产生的轨迹终点位置方向与目标点连线的夹角的误差
 * @param trajectory 轨迹，dim:[n,5]
 * @param goal 目标点位置[x,y]
 * @return 方位角评价数值
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
 * 速度评价函数
 * 表示当前的速度大小，可以用模拟轨迹末端位置的线速度的大小来表示
 * @param trajectory 轨迹，dim:[n,5]
 * @return 速度评价值
 */
float DWA::_velocity(vector<vector<float>> trajectory) {
    return  trajectory[trajectory.size()-1][3];
}

/**
 * 距离评价函数
 * 表示当前速度下对应模拟轨迹与障碍物之间的最近距离；
 * 如果没有障碍物或者最近距离大于设定的阈值，那么就将其值设为一个较大的常数值。
 * @param trajectory 轨迹，dim:[n,5]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return 距离评价值
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
 * 滚动窗口算法控制器
 * @param state 机器人当前状态--[x,y,yaw,v,w]
 * @param goal 目标点位置，[x,y]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return  最优控制量[v,w]、最优轨迹
 */
vector<float> DWA::dwaControl(vector<float>& state, Vector2d& goal, vector<Vector2d>& obstacle) {
    vector<float> res = trajectoryEvaluation(state,goal,obstacle);
    return res;
}