/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算


#include <math.h>
#include <stdio.h>
#include <INS_task.h>
#include "SolveTrajectory.h"

struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.1f; // 飞行时间
float dz_see[20];



/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
//    printf("model %f %f\n", t, z);
    return z;
}


/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float completeAirResistanceModel(float s, float v, float angle)
{
    


}



/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;//调整的枪口z_temp , 根据模型出来的真实z_actual , 误差dz
    float angle_pitch;	//theta角
    int i = 0;
    z_temp = z;	//初始化高度
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad弧度
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);	//求解模型理论真实抵达z值
				dz = 0.3*(z - z_actual);	//通过dz来调整枪口，使得理论模型真实抵达的z值和实际上目标的z值相同，这个参数可调
				dz_see[i] = dz;	//调试观测窗口
        z_temp = z_temp + dz;
        //printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
        //    i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 直接解算Pitch(单方向空气阻力模型)
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation_new(float s, float z, float v)
{
	//测试用
//	s = 5;
//	z = 0;
//	v = 30;
	
	float angle_pitch;
	float k1 = 0.019;//小弹丸的空气阻力系数
	float a = (exp(k1*s) - 1) / k1;
	float b = (GRAVITY*pow(exp(k1*s)-1,2)) / (2*pow(k1,2)*pow(v,2));
	float delta = pow(a,2) - 4*b*(z+b);
	float tan_angle_1 = (a+sqrt(delta)) / (2*b);
	float tan_angle_2 = (a-sqrt(delta)) / (2*b);
	float angle_init = atan2(z, s);	//rad弧度，补偿前的角度
	float angle_actual_1 = -atan(tan_angle_1) * 57.3f;
	float angle_actual_2 = -atan(tan_angle_2) * 57.3f;//rad
	angle_pitch = (fabs(angle_actual_1 - INS_angle[1]) > fabs(angle_actual_2 - INS_angle[1])) ? angle_actual_2 : angle_actual_1;//取绝对值小的那个 
	t = (float)((exp(k1 * s) - 1) / (k1 * v * cos(angle_pitch/57.3f)));//更新飞行时间
	return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
    int idx = 0;
float tar_yaw_test;
float yaw_test;
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
    // 线性预测
		float timeDelay = st.bias_time/1000.0 + t; 	//偏置时间(移动)加传输时间
    float yaw_Delay = st.v_yaw * timeDelay;		//v_yaw是目标车yaw转速，乘时间后得到下一时刻目标的航向角
		float tar_yaw = st.tar_yaw + yaw_Delay;
		tar_yaw_test = tar_yaw * 57.3f;
		yaw_test = *yaw * 57.3f;
    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
			idx = 0;
 // 选择的装甲板
	
		//接下来将目标车中心的值转换到4块装甲板上，得到4个装甲板在云台坐标系下的x,y,z，并得到4块装甲板基于目标车中心(我方坐标系平移)
		//下的偏航角，用我方与目标装甲板基于其中心偏航角的差值来选择装甲板
	
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


    } else {

        for (i = 0; i<4; i++) {
            float tmp_yaw = tar_yaw + i * PI/2.0;
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
//        	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
//        	//int idx = 0;
//        	for (i = 1; i<4; i++)
//        	{
//						//这里不知道chenjunnnn为啥写得[i]*[0]
//        		//float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
//						float temp_dis_diff = sqrt(tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y);
//        		if (temp_dis_diff < dis_diff_min)
//        		{
//        			dis_diff_min = temp_dis_diff;
//        			idx = i;
//        		}
//        	}
        

            //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//        for (i = 1; i<4; i++) {
//            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }
				 //新 计算枪管到目标装甲板yaw最小的那个装甲板
        float yaw_diff_min = cos(tar_position[0].yaw - *yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = cos(tar_position[i].yaw - *yaw);
            if (temp_yaw_diff > yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    }
    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.vyw * timeDelay;	//预测下一刻的位置
		
//		*aim_z = tar_position[0].z + st.vzw * timeDelay;
//    *aim_x = tar_position[0].x + st.vxw * timeDelay;
//    *aim_y = tar_position[0].y + st.vyw * timeDelay;	//预测下一刻的位置
		
		//不用yaw，只用xyz和他们的速度来预测
//		*aim_z = st.zw + st.vzw * timeDelay;
//    *aim_x = st.xw + st.vxw * timeDelay;
//    *aim_y = st.yw + st.vyw * timeDelay;	//预测下一刻的位置
    //这里符号给错了
		//float s_bias;  枪口前推的距离
    //float z_bias;  yaw轴电机到枪口水平面的垂直距离
		//float current_v;  当前弹速
		
		//单方向空气阻力模型
    *pitch = pitchTrajectoryCompensation_new(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
            *aim_z + st.z_bias, st.current_v);//param: s , z , v0
    *yaw = (float)(atan2(*aim_y, *aim_x)) * 57.3f;
		
		//对准车中心下一位置
		//*pitch = -(float)(atan2(*aim_z+st.z_bias , sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y))))*57.3f;
		
		//瞄准车中心
		//*pitch = -(float)(atan2(st.zw + st.z_bias,sqrt(st.xw*st.xw+st.yw*st.yw)))*57.3f;
		//*yaw = (float)(atan2(st.yw,st.xw))*57.3f;

}

// 从坐标轴正向看向原点，逆时针方向为正

//int main()
//{
//    float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 落点，传回上位机用于可视化
//    float pitch = 0; //输出控制量 pitch绝对角度 弧度
//    float yaw = 0;   //输出控制量 yaw绝对角度 弧度

//    //定义参数
//    st.k = 0.092;
//    st.bullet_type =  BULLET_17;
//    st.current_v = 18;
//    st.current_pitch = 0;
//    st.current_yaw = 0;
//    st.xw = 3.0;
//    // st.yw = 0.0159;
//    st.yw = 0;
//    // st.zw = -0.2898;
//    st.zw = 1.5;

//    st.vxw = 0;
//    st.vyw = 0;
//    st.vzw = 0;
//    st.v_yaw = 0;
//    st.tar_yaw = 0.09131;
//    st.r1 = 0.5;
//    st.r2 = 0.5;
//    st.dz = 0.1;
//    st.bias_time = 100;
//    st.s_bias = 0.19133;
//    st.z_bias = 0.21265;
//    st.armor_id = ARMOR_INFANTRY3;
//    st.armor_num = ARMOR_NUM_NORMAL;


//    autoSolveTrajectory(&pitch, &yaw, &aim_x, &aim_y, &aim_z);


//    printf("main pitch:%f° yaw:%f° ", pitch * 180 / PI, yaw * 180 / PI);
//    printf("\npitch:%frad yaw:%frad aim_x:%f aim_y:%f aim_z:%f", pitch, yaw, aim_x, aim_y, aim_z);

//    return 0;
//}
