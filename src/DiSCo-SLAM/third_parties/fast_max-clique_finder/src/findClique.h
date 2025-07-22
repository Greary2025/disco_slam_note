/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*   Description:  一个用于查找图的最大团的库                                                      */                                                   
/*                                                                                         */
/*                                                                                         */
/*   Authors: Bharath Pattabiraman 和 Md. Mostofa Ali Patwary                            */
/*            美国西北大学电子工程与计算机科学系                                               */
/*            邮箱: {bpa342,mpatwary}@eecs.northwestern.edu                                */
/*                                                                                         */
/*   Copyright, 2014, 西北大学                                                              */
/*   请查看顶级目录中的 COPYRIGHT 声明。                                                      */
/*                                                                                         */
/*   如果使用此软件包，请引用以下出版物:                                                        */
/*   Bharath Pattabiraman, Md. Mostofa Ali Patwary, Assefaw H. Gebremedhin2,               */
/*   Wei-keng Liao, and Alok Choudhary.                                                     */
/*   "Fast Algorithms for the Maximum Clique Problem on Massive Graphs with                */
/*   Applications to Overlapping Community Detection"                                      */
/*   http://arxiv.org/abs/1411.7460                                                        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// 头文件保护宏，防止头文件被重复包含
#ifndef FINDCLIQUE_H_INCLUDED
#define FINDCLIQUE_H_INCLUDED

// 包含图输入输出相关的头文件
#include "graphIO.h"
// 包含标准库头文件，用于处理基本数据类型
#include <cstddef>
// 包含输入输出流头文件，用于标准输入输出操作
#include <iostream>
// 包含时间相关的头文件，用于获取系统时间
#include <sys/time.h>
// 包含 Unix 标准库头文件，提供通用工具函数
#include <unistd.h>
// 包含标准库头文件，提供动态内存分配等功能
#include <stdlib.h>
// 包含数值计算相关的头文件，提供数值算法
#include <numeric>
// 包含算法相关的头文件，提供通用算法
#include <algorithm>
// 使用标准命名空间
using namespace std;

// 如果定义了 _DEBUG 宏，开启调试模式
#ifdef _DEBUG
// 调试标志，值为 1 表示开启调试
int DEBUG=1;
#endif

// 定义 FMC 命名空间，将相关函数和类封装在此命名空间下
namespace FMC {
    
// 函数声明部分

/**
 * @brief 检查指定文件是否存在
 * 
 * @param filename 要检查的文件的名称
 * @return bool 如果文件存在返回 true，否则返回 false
 */
bool fexists(const char *filename);

/**
 * @brief 获取当前的系统时间（以秒为单位）
 * 
 * @return double 当前系统时间
 */
double wtime();

/**
 * @brief 打印程序的使用说明
 * 
 * @param argv0 程序的名称
 */
void usage(char *argv0);

/**
 * @brief 获取指定顶点的度
 * 
 * @param ptrVtx 指向顶点邻接表的指针
 * @param idx 顶点的索引
 * @return int 该顶点的度
 */
int getDegree(vector<int>* ptrVtx, int idx);

/**
 * @brief 打印最大团的数据
 * 
 * @param max_clique_data 存储最大团顶点的向量
 */
void print_max_clique(vector<int>& max_clique_data);

/**
 * @brief 查找图的最大团
 * 
 * @param gio 图的输入输出对象，包含图的信息
 * @param l_bound 最大团的下界
 * @param max_clique_data 存储最大团顶点的向量
 * @return int 最大团的大小
 */
int maxClique( CGraphIO& gio, int l_bound, vector<int>& max_clique_data );

/**
 * @brief 最大团查找的辅助函数
 * 
 * @param gio 图的输入输出对象，包含图的信息
 * @param U 候选顶点集合
 * @param sizeOfClique 当前团的大小
 * @param maxClq 最大团的大小
 * @param max_clique_data_inter 存储中间最大团顶点的向量
 */
void maxCliqueHelper( CGraphIO& gio, vector<int>* U, int sizeOfClique, int& maxClq, vector<int>& max_clique_data_inter );

/**
 * @brief 使用启发式算法查找图的最大团，仅返回最大团的大小
 * 
 * @param gio 图的输入输出对象，包含图的信息
 * @return int 最大团的大小
 */
int maxCliqueHeu( CGraphIO& gio );

/**
 * @brief 使用启发式算法查找图的最大团，返回最大团的大小并存储最大团顶点
 * 
 * @param gio 图的输入输出对象，包含图的信息
 * @param max_clique_data 存储最大团顶点的向量
 * @return int 最大团的大小
 */
int maxCliqueHeu(CGraphIO& gio, vector<int>& max_clique_data);

/**
 * @brief 启发式最大团查找的辅助函数
 * 
 * @param gio 图的输入输出对象，包含图的信息
 * @param U 候选顶点集合
 * @param sizeOfClique 当前团的大小
 * @param maxClq 最大团的大小
 * @param max_clique_data_inter 存储中间最大团顶点的向量
 */
void maxCliqueHelperHeu( CGraphIO& gio, vector<int>* U, int sizeOfClique, int& maxClq, vector<int>& max_clique_data_inter );

}
// 结束头文件保护
#endif 
