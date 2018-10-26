#ifndef _PATHFOLLOWING_H
#define _PATHFOLLOWING_H
#include "calculate.h"
#include "stdint.h"

uint8_t OpenLoopTZ1ToTZ2(Point_t startPoint);
void ClearVelStage(void);

int PathFollowing(float percent, int viewMode);
float AngleControl(float anglePresent,float angleTarget);
void getFinalPoint(float x, float y, float angle);
Pose_t GetEndPoint(void);
/*********************************************************************************
* @name 	LineClose2Point
* @brief	给定两点沿直线减速
* @param	startPoint 起点坐标
* @param  endPoint  终点坐标
* @param	startVel 起点速度
* @param  endVel  终点速度
* @param  posAngle  姿态角度
* @retval	无
**********************************************************************************/
void EndPointCloseLoop(Pose_t endPoint , Pose_t actualPoint);
void LineClose2Point(Point_t startPoint, Point_t endPoint,float startVel , float endVel , float posAngle);
/*********************************************************************************
* @name 	LineVelCloseLoop
* @brief	给定速度直线闭环，不控制车的姿态，用来靠墙
* @param	lineA:直线通用表达式中X前的系数
* @param	lineB：直线通用表达式中Y前的系数
* @param	lineC：直线通用表达式中的常数项
* @param	vel：走直线的速度,小于零为反向走
* @retval	无
**********************************************************************************/
void LineVelCloseLoop(float lineA, float lineB,float lineC , float vel);
#endif
