#include <iostream>
#include<algorithm>
#include<unordered_map>
#include<unordered_set>
#include<cstring>
#include"DWA.h"

using namespace std;
#define MIN_VALUE -99999    //������С��ֵ
char map[100][100]; //��ʼ��ͼ
int  sum_money;     //�ܽ��
int workBenchLength; //����̨����
//unordered_map<int,vector<WorkBench>> mp; //����̨����ԭ�Ϻ͹���̨ӳ���ϵ
const float  v_max = 6, v_min = -2 ,w_min = -PI,w_max = PI,\
a_vmax_without =  250 / (20*PI*0.45*0.45), a_vmax_with = 250 / (20*PI*0.53*0.53),\
a_wmax_without = 2*50 / (20 * PI * 0.45 * 0.45*0.45*0.45), a_wmax_with = 2*50 / (20 * PI * 0.53 * 0.53*0.53*0.53);
const float radius_bench_with = v_max*v_max/(2*a_vmax_with)+0.4;//�빤��̨��ײ�뾶(����Ʒ
const float radius_bench_without = v_max*v_max / (2 * a_vmax_without) + 0.4;//�빤��̨��ײ�뾶(������Ʒ
const float radius_robot = 1.5; //���������ײ�뾶
const float radius_wall = 0.53; //��ǽ����ײ�뾶
const int maxK = 50;         //����̨�������
float dt = 0.02;            //����ʱ��
float predict_time = 0.18 ;   //�켣����ʱ�䳤��
float v_sample = 0.1, w_sample = a_vmax_without*0.02*0.03; //�����ֱ���
float alpha = 0.15,bta = 1.0, gamma = 1.0; //�켣���ۺ���ϵ��
//     float radius = 1.0; // �����ж��Ƿ񵽴�Ŀ���
float judge_distance = 10; //�����ϰ������С���������ֵ�������������õ���ֵΪrobot_radius+0.2��,����Ϊһ���ϴ�ĳ�ֵ

bool mp[10][10];           //��¼����̨����ԭ����
int material_need_value[10];//����ԭ���ϱ���Ҫ�̶�
int material_need[10];     //��¼ԭ���ϱ���Ҫ����
//int material_target[10];   //��¼ԭ�����ʹ�����
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
class WorkBench {               //����̨
public:
    int id;
    int type;                   //����̨����
    float x,y;                  //����
    int remainProductionTime;   //ʣ������ʱ��
    int materialState;          //ԭ����״̬
    bool productionState;       //��Ʒ��״̬
    bool choosed = false;       //�Ƿ񱻻�����ѡ��(������ȥ����̨��ȡ��Ʒ)
    int materialSendState = 0;  //�Ƿ񱻻�����ѡ��(������ȥ����̨��ԭ��)
   // int buyPrior = 0;         //��Ĳ�Ʒ��ֵ����
   // int sellPrior = 0;        //���ļ�ֵ����


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
   * (x1,y1)Ϊ����������
   */
    int static get_type_value(int type)
    {
        if (type <= 3)
        {
            return 1;
        }
        else if (type <= 6)
        {
            return 10000;
        }
        else if (type <= 7)
        {
            return 1000000;
        }
    }
    int getBuyPrior(float x1,float y1)
    {
        char buf[1024] = { 0 };
        sprintf_s(buf, 1024, "id %d choosed %d  productionState %d remainTime %d material_need[%d] %d",id, choosed,productionState,remainProductionTime,type, material_need[type]);
        WriteString(buf);
        if (material_need[type] <=0)
        {
            return MIN_VALUE;
        }
        float dis = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
        if (!productionState)
        {
            // ---> s          ---->50fps * 20ms = 1s
            if (remainProductionTime == -1)
            {
                return MIN_VALUE;
            }
            else if (0.3+(dis-0.9) / v_max >= float(remainProductionTime)*0.02)
            {
                char buf[1024] = { 0 };
                sprintf_s(buf, 1024, "time %f remianTime %f", 0.3 + (dis - 0.9) / v_max, float(remainProductionTime) * 0.02);
                WriteString(buf);
                return get_type_value(type) - dis +material_need_value[type];
            }
            else
            {
                return MIN_VALUE;
                //return get_type_value(type) - dis+100*(dis/v_max-remainProductionTime) +material_need_value[type];
            }
        }
        int prior = get_type_value(type) - dis + material_need_value[type];
        return prior;
    }


    int getSellPrior(float x1,float y1,int pType)
    {
        if (mp[type][pType])
        {
            int prior = 0;
            if ((materialState|materialSendState) & (1 << pType))
            {
                return MIN_VALUE;
            }
            else return 100-sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)) + 100*get_material_num();
        }
        return MIN_VALUE;
    }
}workBenchArr[51];

class BenchBuy              //���Ʒ����̨
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

bool  buyPriorCmp(const BenchBuy& w1, const BenchBuy& w2)//���Ʒ����̨��ֵ�ȽϺ���
{
    return w1.buyPrior>w2.buyPrior;
}
bool SellPriorCmp(const BenchSell& w1, const BenchSell& w2)//����Ʒ����̨��ֵ�ȽϺ���
{
    return w1.sellPrior>w2.sellPrior;
}


void init_need()
{
    memset(material_need,0,sizeof(material_need));
    memset(material_need_value, 0, sizeof(material_need_value));
   // memset(material_target, 0, sizeof(material_target));
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
    int workBenchId;                    //��������̨Id
    int targetBuyId = -1;               //���ƷĿ��Id
    int targetSellId = -1;              //����ƷĿ��Id
    int goodType;                       //Я����ƷId
    float timeValueCoefficient;         //ʱ���ֵϵ��
    float collisioValueCoefficient;     //��ײ��ֵϵ��
    float angularVelocity;              //���ٶ�
    float lineVelocityX,lineVelocityY;  //���ٶ�
    float forward;                      //����
    float x,y;                          //����
    float getVelociry(){return sqrt(lineVelocityX*lineVelocityX+lineVelocityY*lineVelocityY); }
}robotArr[4];
int main() {
    init();             //��ʼ����������̨����
    readMap();          //��ȡ��ͼ��Ϣ
    puts("OK");
    fflush(stdout);
    int frameID;


    while (scanf("%d", &frameID) != EOF) {
        scanf("%d",&sum_money);
        scanf("%d",&workBenchLength);

        init_need();    //��ʼ��ԭ��������̶�

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
                        int value = WorkBench::get_type_value(j);
                        material_need_value[j] = value+material_need_value[j];
                        material_need[j]++;
                        if (j == 4)
                        {
                            material_need_value[1] = 100 + material_need_value[1];
                            material_need_value[2] = 100 + material_need_value[2];
 
                        }
                        else if (j == 5)
                        {
                            material_need_value[1] = 100 + material_need_value[1];
                            material_need_value[3] = 100 + material_need_value[3];
                        }
                        else if (j == 6)
                        {
                            material_need_value[3] = 100 + material_need_value[3];
                            material_need_value[2] = 100 + material_need_value[2];
                        }
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


            if (robotArr[robotId].goodType != 0)
            {
                //material_target[robotArr[robotId].goodType]++;
                material_need_value[robotArr[robotId].goodType] -= WorkBench::get_type_value(robotArr[robotId].goodType);
                material_need[robotArr[robotId].goodType]--;

                workBenchArr[robotArr[robotId].targetSellId].materialSendState |= (1<< robotArr[robotId].goodType);
            }
            else
            {
                int type = workBenchArr[robotArr[robotId].targetBuyId].type;
                material_need_value[type] -= WorkBench::get_type_value(type);
                material_need[type]--;
            }
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
            if (robotArr[robotId].goodType == 0)//������δЯ����Ʒ
            {
                //vector<BenchBuy> buyArr(workBenchLength);
                if (robotArr[robotId].targetBuyId == -1)//������δѡ����̨���Ʒ
                {
                    int maxValue = MIN_VALUE;
                    int maxIndex = -1;
                    for (int i = 0; i < workBenchLength; i++)
                    {
                        if(workBenchArr[i].choosed)continue;
                        //buyArr[i].id = i;
                        int tempValue = workBenchArr[i].getBuyPrior(robotArr[robotId].x,robotArr[robotId].y);


                        char buf[1024] = { 0 };
                        sprintf_s(buf, 1024, "robotId %d i %d tempValue %d ", robotId,i,tempValue);
                        WriteString(buf);
                       

                        if (tempValue > maxValue)
                        {
                            maxValue = tempValue;
                            maxIndex = i;
                        }
                    }
                    if (maxIndex == -1)continue;
                    workBenchArr[maxIndex].choosed = true;
                    robotArr[robotId].targetBuyId = maxIndex;

                    material_need[workBenchArr[maxIndex].type]--;
                }
                


                vector<float> state = {robotArr[robotId].x,robotArr[robotId].y,robotArr[robotId].forward,robotArr[robotId].getVelociry(),robotArr[robotId].angularVelocity};
                Vector2d goal(workBenchArr[robotArr[robotId].targetBuyId].x, workBenchArr[robotArr[robotId].targetBuyId].y); //Ŀ���

                //vector<Vector2d>obstacle = obstacleBench;//�ϰ���λ��
                vector<Vector2d>obstacle = {goal};
                //vector<Vector2d>obstacle;
                //for (int i = 0; i < 4; i++)
                //{
                //    if(i!=robotId)
                //        obstacle.push_back(Vector2d(robotArr[i].x,robotArr[i].y));
                //}
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
                sprintf_s(buf, 1024, "frameId %d id %d targetId %d velocity %f anglevelocity %f forward %f v %f w %f\n", frameID, robotId, robotArr[robotId].targetBuyId, robotArr[robotId].getVelociry(), robotArr[robotId].angularVelocity, robotArr[robotId].forward, state[3], state[4]);
                WriteString(buf);
                if (robotArr[robotId].workBenchId == robotArr[robotId].targetBuyId && workBenchArr[robotArr[robotId].workBenchId].productionState)
                {
                    printf("buy %d\n", robotId);
                    workBenchArr[robotArr[robotId].targetBuyId].choosed = false;
                    workBenchArr[robotArr[robotId].targetBuyId].productionState = 0;
                    robotArr[robotId].targetBuyId = -1;
                }
            }
            else
            {
            
                    //|| (workBenchArr[robotArr[robotId].targetSellId].materialState&(1<< robotArr[robotId].goodType))
                if (robotArr[robotId].targetSellId == -1 || (workBenchArr[robotArr[robotId].targetSellId].materialState | \
                    (workBenchArr[robotArr[robotId].targetSellId].materialState & (1 << robotArr[robotId].goodType)))&(1 << robotArr[robotId].goodType))
                {
                    int maxValue = MIN_VALUE;
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
                Vector2d goal(workBenchArr[robotArr[robotId].targetSellId].x, workBenchArr[robotArr[robotId].targetSellId].y); //Ŀ���

                
                //vector<Vector2d>obstacle = obstacleBench;//�ϰ���λ��
                vector<Vector2d>obstacle = {goal};
                //vector<Vector2d>obstacle;
                //for (int i = 0; i < 4; i++)
                //{
                //    if (i != robotId)
                //        obstacle.push_back(Vector2d(robotArr[i].x, robotArr[i].y));
                //}
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
                char buf[1024] = { 0 };
                sprintf_s(buf, 1024, "frameId %d id %d targetId %d velocity %f anglevelocity %f forward %f v %f w %f\n", frameID, robotId, robotArr[robotId].targetSellId, robotArr[robotId].getVelociry(), robotArr[robotId].angularVelocity,robotArr[robotId].forward, state[3], state[4]);
                WriteString(buf);
                if (robotArr[robotId].workBenchId == robotArr[robotId].targetSellId && (workBenchArr[robotArr[robotId].workBenchId].materialState&(1<< robotArr[robotId].goodType))==0)
                {
                    printf("sell %d\n", robotId);
                   //workBenchArr[robotArr[robotId].targetSellId].choosed = false;
                    robotArr[robotId].targetSellId = -1;
                    workBenchArr[robotArr[robotId].targetSellId].materialState |=(1<<robotArr[robotId].goodType);
                    //robotArr[robotId].goodType = 0;
                }
            }
        }
        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}
