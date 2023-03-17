#include <iostream>
#include<algorithm>
#include<unordered_map>
#include<unordered_set>
#include<cstring>
#include"DWA.h"

using namespace std;

char map[100][100]; //初始地图
int  sum_money;     //总金币
int workBenchLength; //工作台总数
//unordered_map<int,vector<WorkBench>> mp; //工作台所需原料和工作台映射关系
const float  v_max = 6, v_min = -2 ,w_min = -PI,w_max = PI,\
a_vmax_without =  250 / (20*PI*0.45*0.45), a_vmax_with = 250 / (20*PI*0.53*0.53),\
a_wmax_without = 50 / (20 * PI * 0.45 * 0.45*0.45*0.45), a_wmax_with = 50 / (20 * PI * 0.53 * 0.53*0.53*0.53);
const float radius_bench_with = v_max/(2*a_vmax_with)+0.4;//与工作台碰撞半径(带物品
const float radius_bench_without = v_max / (2 * a_vmax_without) + 0.4;//与工作台碰撞半径(不带物品
const float radius_robot = 2.0; //与机器人碰撞半径
const float radius_wall = 0.53; //与墙壁碰撞半径
const int maxK = 50;         //工作台最大数量
float dt = 0.02;            //采样时间
float predict_time = 0.12;   //轨迹推算时间长度
float v_sample = 0.1, w_sample = 0.5 * PI / 180; //采样分辨率
float alpha = 3.0,bta = 1.0, gamma = 1.0; //轨迹评价函数系数
//     float radius = 1.0; // 用于判断是否到达目标点
float judge_distance = 10; //若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值

bool mp[10][10];           //记录工作台所需原材料
int material_need[10];               //记录原材料被需要程度；
vector<Vector2d>obstacleBench;
vector<Vector2d>obstacleWall;
bool readMap() {
    char line[1024];
    int row=0;
    while (fgets(line, sizeof line, stdin)) {
        //WriteString(line);
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        for (int i = 0; i < 100; i++)
        {
            map[row][i] = line[i];
            if (line[i] != '.'&& line[i]!='A')
            {
                //obstacleBench.push_back(Vector2d(0.25+0.5*i,0.25+0.5*(99.0-row)));

            }
        }
        row++;
    }
    return false;
}
bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
      
    }
    return false;
}
class WorkBench {               //工作台
public:
    int id;
    int type;                   //工作台类型
    float x,y;                  //坐标
    int remainProductionTime;   //剩余生产时间
    int materialState;          //原材料状态
    bool productionState;       //产品格状态
    bool choosed = false;       //是否被机器人选中(机器人去工作台获取产品)
    int materialSendState = 0;  //是否被机器人选中(机器人去工作台卖原料)
   // int buyPrior = 0;         //买的产品价值估计
   // int sellPrior = 0;        //卖的价值估计


    int get_material_num()
    {
        int state = materialSendState|materialState;
        int cnt = 0;
        while (state)
        {
            cnt += state&1;
            state>>=1;
        }
        return cnt;
   }
   /*
   * 
   * (x1,y1)为机器人坐标
   */
    int get_type_value(int type)
    {
        if (type <= 3)
        {
            return 1;
        }
        else if (type <= 6)
        {
            return 10;
        }
        else if (type <= 7)
        {
            return 1000;
        }
    }
    int getBuyPrior(float x1,float y1)
    {
        if (choosed)
        {
            
            return 0;
        }
        float dis = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
        if (!productionState)
        {
            // ---> s          ---->50fps * 20ms = 1s
            if (remainProductionTime == -1)
            {
                return 0;
            }
            else if (dis / v_max >= float(remainProductionTime)*0.02)
            {
                return get_type_value(type) - 10 * dis +100*material_need[type];
            }
            else
            {
                return get_type_value(type) - 10 * dis+100*(dis/v_max-remainProductionTime) +100 * material_need[type];
            }
        }
        int prior = get_type_value(type) - 10 * dis + 100 * material_need[type];
        return prior;
    }
    int getSellPrior(float x1,float y1,int pType)
    {
        if (mp[type][pType])
        {
            int prior = 0;
            if ((materialState|materialSendState) & (1 << pType))
            {
                return 0;
            }
            else return 100-sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)) + 100*get_material_num();
        }
        return 0;
    }
}workBenchArr[51];

class BenchBuy              //买产品工作台
{
public:
    BenchBuy():id(0),buyPrior(0){}
    ~BenchBuy(){}
    int id;
    int buyPrior = 0;
private:

}BenchBuyArr[51];

class BenchSell
{
public:
    BenchSell():id(0),sellPrior(0){}
    ~BenchSell(){}
    int id;
    int sellPrior;
private:

}BenchSellArr[51];

bool  buyPriorCmp(const BenchBuy& w1, const BenchBuy& w2)//买产品工作台价值比较函数
{
    return w1.buyPrior>w2.buyPrior;
}
bool SellPriorCmp(const BenchSell& w1, const BenchSell& w2)//卖产品工作台价值比较函数
{
    return w1.sellPrior>w2.sellPrior;
}


void init_need()
{
    memset(material_need,0,sizeof(material_need));
}
void init()
{
    for (int i = 0; i < maxK; i++)
    {
        BenchBuyArr[i].id = i;
        BenchSellArr[i].id = i;
        workBenchArr[i].id = i;
    }
    memset(mp,false,sizeof(mp));
    mp[4][1] = true, mp[4][2] = true;
    mp[5][1] = true, mp[5][3] = true;
    mp[6][2] = true, mp[6][3] = true;
    mp[7][4] = true, mp[7][5] = true, mp[7][6] = true;
    mp[8][7] = true;
    for (int i = 1; i <= 7; i++)
    {
        mp[9][i] = true;
    }
    //for (int i = 0; i < 100; i++)
    //{
    //    for (int j = 0; j < 10; j++)
    //    {
    //        obstacleWall.push_back(Vector2d(i,j));
    //    }
    //}
}


class Robot {
public:
    int workBenchId;                    //所处工作台Id
    int targetBuyId = -1;               //买产品目标Id
    int targetSellId = -1;              //卖产品目标Id
    int goodType;                       //携带物品Id
    float timeValueCoefficient;         //时间价值系数
    float collisioValueCoefficient;     //碰撞价值系数
    float angularVelocity;              //角速度
    float lineVelocityX,lineVelocityY;  //线速度
    float forward;                      //朝向
    float x,y;                          //坐标
    float getVelociry(){return sqrt(lineVelocityX*lineVelocityX+lineVelocityY*lineVelocityY); }
}robotArr[4];
int main() {
    init();             //初始化买卖工作台数组
    readMap();          //读取地图信息
    puts("OK");
    fflush(stdout);
    int frameID;


    while (scanf("%d", &frameID) != EOF) {
        scanf("%d",&sum_money);
        scanf("%d",&workBenchLength);

        init_need();    //初始化原材料所需程度

        for (size_t i = 0; i < workBenchLength; i++)
        {
            cin>>workBenchArr[i].type>>workBenchArr[i].x>>workBenchArr[i].y \
            >> workBenchArr[i].remainProductionTime>>workBenchArr[i].materialState>>workBenchArr[i].productionState;
            //workBenchArr[i].id = i;

            workBenchArr[i].materialSendState = 0;
            for (int j = 1; j < 8; j++)
            {
                if (mp[workBenchArr[i].type][j])
                {
                    if (((workBenchArr[i].materialState) & (1 << j)) == 0)
                    {
                        material_need[j] = ((j-1)/3+1)+material_need[j];

                    }
                }
            }
        }
        //debug
        //char buf[1024] = { 0 };
        //sprintf_s(buf, 1024, "id1 %d id2 %d id3 %d id4 %d id5 %d id6 %d id7 %d", material_need[1],material_need[2],\
        //material_need[3], material_need[4], material_need[5], material_need[6],material_need[7]);
        //WriteString(buf);
        int buyCnt = 0, buyIndex=0;
        for (size_t robotId = 0; robotId < 4; robotId++)
        {
            cin>>robotArr[robotId].workBenchId>>robotArr[robotId].goodType>> robotArr[robotId].timeValueCoefficient \
            >> robotArr[robotId].collisioValueCoefficient>> robotArr[robotId].angularVelocity>> robotArr[robotId].lineVelocityX \
            >> robotArr[robotId].lineVelocityY>> robotArr[robotId].forward>> robotArr[robotId].x>> robotArr[robotId].y;
        }
        //fflush(stdin);
        readUntilOK();
        printf("%d\n", frameID);
        int lineSpeed = 6;
        float angleSpeed = 1.5;

        
        //char buf[1024] = { 0 };
        //sprintf_s(buf, 1024, "frameId %d id %d angularVelocity %f lineVelocityX %f lineVelocityY %f forward %f", frameID, \
        //0, robotArr[0].angularVelocity,robotArr[0].lineVelocityX, \
        //robotArr[0].lineVelocityY , robotArr[0].forward);
        //WriteString(buf);
        for(int robotId = 0; robotId < 4; robotId++){
            //if(robotId!=0)continue;
            if (robotArr[robotId].goodType == 0)//机器人未携带物品
            {
                //vector<BenchBuy> buyArr(workBenchLength);
                if (robotArr[robotId].targetBuyId == -1)//机器人未选择工作台买产品
                {
                    int maxValue = -99999;
                    int maxIndex = -1;
                    for (int i = 0; i < workBenchLength; i++)
                    {
                        if(workBenchArr[i].choosed)continue;
                        //buyArr[i].id = i;
                        int tempValue = workBenchArr[i].getBuyPrior(robotArr[robotId].x,robotArr[robotId].y);

                        //if (robotId == 1)
                        //{
                        //    char buf[1024] = { 0 };
                        //    sprintf_s(buf, 1024, "frameId %d id %d x %f y %f x1 %f y1 %f dis %d", frameID,i, workBenchArr[i].x, workBenchArr[i].y, robotArr[robotId].x, robotArr[robotId].y,tempValue);
                        //    WriteString(buf);
                        //}

                        if (tempValue > maxValue)
                        {
                            maxValue = tempValue;
                            maxIndex = i;
                        }
                    }
                    if (maxIndex == -1)continue;
                    workBenchArr[maxIndex].choosed = true;
                    robotArr[robotId].targetBuyId = maxIndex;
                }
                


                vector<float> state = {robotArr[robotId].x,robotArr[robotId].y,robotArr[robotId].forward,robotArr[robotId].getVelociry(),robotArr[robotId].angularVelocity};
                Vector2d goal(workBenchArr[robotArr[robotId].targetBuyId].x, workBenchArr[robotArr[robotId].targetBuyId].y); //目标点

                //vector<Vector2d>obstacle = obstacleBench;//障碍物位置
                //vector<Vector2d>obstacle = {goal};
                vector<Vector2d>obstacle;
                for (int i = 0; i < 4; i++)
                {
                    if(i!=robotId)
                        obstacle.push_back(Vector2d(robotArr[i].x,robotArr[i].y));
                }
                //vector<vector<float> >trajectory;
               // trajectory.push_back(state);
                DWA dwa(dt, v_min, v_max, w_min, w_max, predict_time, 
                a_vmax_without, a_wmax_without, v_sample, w_sample,\
                alpha, bta, gamma, radius_bench_without,radius_robot,radius_wall, judge_distance);

               vector<float> res = dwa.dwaControl(state, goal, obstacle);
               state = dwa.kinematicModel(state, res, dt);
               // 
                //trajectory.push_back(state);
                
                printf("forward %d %f\n", robotId, state[3]);
                printf("rotate %d %f\n", robotId, state[4]);

                char buf[1024] = { 0 };
                sprintf_s(buf, 1024, "frameId %d id %d targetId %d velocity %f", frameID,robotId, robotArr[robotId].targetBuyId, robotArr[robotId].getVelociry());
                WriteString(buf);
                if (robotArr[robotId].workBenchId == robotArr[robotId].targetBuyId && workBenchArr[robotArr[robotId].workBenchId].productionState)
                {
                    printf("buy %d\n", robotId);
                    workBenchArr[robotArr[robotId].targetBuyId].choosed = false;
                    robotArr[robotId].targetBuyId = -1;
                }
            }
            else
            {
            
                    //|| (workBenchArr[robotArr[robotId].targetSellId].materialState&(1<< robotArr[robotId].goodType))
                if (robotArr[robotId].targetSellId == -1 || (workBenchArr[robotArr[robotId].targetSellId].materialState | \
                workBenchArr[robotArr[robotId].targetSellId].materialSendState)& (1 << robotArr[robotId].goodType))
                {
                    int maxValue = 0;
                    int maxIndex = -1;
                   // if(robotArr[robotId].targetSellId)
                    for (int i = 0; i < workBenchLength; i++)
                    {
                        //if (workBenchArr[i].choosed)continue;
                        //buyArr[i].id = i;
                        int tempValue = workBenchArr[i].getSellPrior(robotArr[robotId].x, robotArr[robotId].y, robotArr[robotId].goodType);
                        if (tempValue > maxValue)
                        {
                            maxValue = tempValue;
                            maxIndex = i;
                        }
                    }
                    if (maxIndex == -1)continue;
                    //workBenchArr[maxIndex].choosed = true;
                    //workBenchArr[maxIndex].materialSendState |= (1<< robotArr[robotId].goodType);
                    robotArr[robotId].targetSellId = maxIndex;
                }
                workBenchArr[robotArr[robotId].targetSellId].materialSendState |= (1 << robotArr[robotId].goodType);
            /*    char buf[1024] = { 0 };
                sprintf_s(buf, 1024, "frameId %d robotId %d targetId %d materialSendState %d goodType %d", frameID,robotId, robotArr[robotId].targetSellId, workBenchArr[robotArr[robotId].targetSellId].materialSendState, robotArr[robotId].goodType);
                WriteString(buf);*/

                vector<float> state = { robotArr[robotId].x,robotArr[robotId].y,robotArr[robotId].forward,robotArr[robotId].getVelociry(),robotArr[robotId].angularVelocity };
                Vector2d goal(workBenchArr[robotArr[robotId].targetSellId].x, workBenchArr[robotArr[robotId].targetSellId].y); //目标点

                
                //vector<Vector2d>obstacle = obstacleBench;//障碍物位置
                //vector<Vector2d>obstacle = {goal};
                vector<Vector2d>obstacle;
                for (int i = 0; i < 4; i++)
                {
                    if (i != robotId)
                        obstacle.push_back(Vector2d(robotArr[i].x, robotArr[i].y));
                }
                //for (int i = 0; i < workBenchLength; i++)
                //{
                //    if (i != robotArr[robotId].targetSellId)
                //        obstacle.push_back(Vector2d(robotArr[i].x, robotArr[i].y));
                //}
                //vector<vector<float> >trajectory;
               // trajectory.push_back(state);
                DWA dwa(dt, v_min, v_max, w_min, w_max, predict_time, a_vmax_with, a_wmax_with, v_sample, w_sample, \
                    alpha, bta, gamma, radius_bench_with, radius_robot, radius_wall, judge_distance);

                vector<float> res = dwa.dwaControl(state, goal, obstacle);
                state = dwa.kinematicModel(state, res, dt);
                // 
                 //trajectory.push_back(state);

                printf("forward %d %f\n", robotId, state[3]);
                printf("rotate %d %f\n", robotId, state[4]);
                //char buf[1024] = { 0 };
                //sprintf_s(buf, 1024, "frameId %d id %d forward %f rotate %f gx %f gy %f", frameID, robotId, state[3], state[4], workBenchArr[robotArr[robotId].targetBuyId].x, workBenchArr[robotArr[robotId].targetBuyId].y);
                //WriteString(buf);
                if (robotArr[robotId].workBenchId == robotArr[robotId].targetSellId && (workBenchArr[robotArr[robotId].workBenchId].materialState&(1<< robotArr[robotId].goodType))==0)
                {
                    printf("sell %d\n", robotId);
                   //workBenchArr[robotArr[robotId].targetSellId].choosed = false;
                    robotArr[robotId].targetSellId = -1;
                }
            }
        }
        printf("OK\n", frameID);
        fflush(stdout);
    }
    return 0;
}
