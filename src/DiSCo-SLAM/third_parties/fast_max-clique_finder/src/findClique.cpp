/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*   Description:  a library for finding the maximum clique of a graph     		   */                                                   
/*                                                                           		   */
/*                                                                           		   */
/*   Authors: Bharath Pattabiraman and Md. Mostofa Ali Patwary               		   */
/*            EECS Department, Northwestern University                       		   */
/*            email: {bpa342,mpatwary}@eecs.northwestern.edu                 		   */
/*                                                                           		   */
/*   Copyright, 2014, Northwestern University			             		   */
/*   See COPYRIGHT notice in top-level directory.                            		   */
/*                                                                           		   */
/*   Please site the following publication if you use this package:           		   */
/*   Bharath Pattabiraman, Md. Mostofa Ali Patwary, Assefaw H. Gebremedhin2, 	   	   */
/*   Wei-keng Liao, and Alok Choudhary.	 					   	   */
/*   "Fast Algorithms for the Maximum Clique Problem on Massive Graphs with           	   */
/*   Applications to Overlapping Community Detection"				  	   */
/*   http://arxiv.org/abs/1411.7460 		 					   */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// 包含自定义的头文件，声明了相关函数和类
#include "findClique.h"

// 进入 FMC 命名空间
namespace FMC {
// 记录不同剪枝策略的剪枝次数
int pruned1;
int pruned2;
int pruned3;
int pruned5;

/**
 * @brief 算法 2: CLIQUE，算法 1 的递归子例程，用于递归查找最大团
 * 
 * @param gio 图的输入输出对象，包含图的顶点和边信息
 * @param U 候选顶点集合，当前可添加到团中的顶点
 * @param sizeOfClique 当前团的大小
 * @param maxClq 最大团的大小，通过引用传递，会在递归过程中更新
 * @param max_clique_data_inter 存储中间最大团顶点的向量
 */
void maxCliqueHelper( CGraphIO& gio, vector<int>* U, int sizeOfClique, int& maxClq, vector<int>& max_clique_data_inter )
{
    // 中间变量，用于记录位置和索引
    int iPos, index = 0, maxClq_prev;
    // 获取图的顶点指针和边指针
    vector <int>* ptrVertex = gio.GetVerticesPtr();
    vector <int>* ptrEdge = gio.GetEdgesPtr();
    // 用于存储新的候选顶点集合
    vector <int> U_new;
    // 预先分配内存，提高性能
    U_new.reserve(gio.GetVertexCount());

    // 如果候选顶点集合为空
    if( U->size() == 0  )
    {
        // 如果当前团的大小大于最大团的大小
        if( sizeOfClique > maxClq )
        {
            // 更新最大团的大小
            maxClq = sizeOfClique;
            // 清空中间最大团顶点向量
            max_clique_data_inter.clear();
        }
        // 递归返回
        return;
    }

    // 当候选顶点集合不为空时，继续循环
    while( U->size() > 0 )
    {
        // 旧的剪枝策略：如果当前团大小加上候选顶点数量小于等于最大团大小，直接返回
        if( sizeOfClique + U->size() <= maxClq )
            return;

        // 取出候选顶点集合的最后一个顶点
        index = U->back();
        // 从候选顶点集合中移除该顶点
        U->pop_back();

        // 遍历当前顶点的所有邻居顶点
        for(int j = (*ptrVertex)[index]; j < (*ptrVertex)[index + 1]; j++ )
        {
            // 剪枝策略 5：如果邻居顶点的度大于等于最大团大小
            if( getDegree(ptrVertex, (*ptrEdge)[j]) >=  maxClq )
            {
                // 遍历候选顶点集合
                for(int i = 0; i < U->size(); i++)
                {
                    // 如果邻居顶点在候选顶点集合中，将其添加到新的候选顶点集合中
                    if( (*ptrEdge)[j] == (*U)[i] )
                        U_new.push_back( (*ptrEdge)[j]);
                }
            }
            else
                // 记录剪枝策略 3 的剪枝次数
                pruned3++;
        }

        // 记录更新前的最大团大小
        maxClq_prev = maxClq;

        // 递归调用 maxCliqueHelper 函数
        maxCliqueHelper( gio, &U_new, sizeOfClique + 1, maxClq, max_clique_data_inter );

        // 如果最大团大小更新了，将当前顶点添加到中间最大团顶点向量中
        if(maxClq > maxClq_prev)
            max_clique_data_inter.push_back(index);

        // 清空新的候选顶点集合
        U_new.clear();
    }
}

/**
 * @brief 算法 1: MAXCLIQUE，用于查找给定图的最大团
 * 
 * @param gio 图的输入输出对象，包含图的顶点和边信息
 * @param l_bound 最大团的下界
 * @param max_clique_data 存储最大团顶点的向量
 * @return int 最大团的大小
 */
int maxClique( CGraphIO& gio, int l_bound, vector<int>& max_clique_data )
{
    // 获取图的顶点指针和边指针
    vector <int>* ptrVertex = gio.GetVerticesPtr();
    vector <int>* ptrEdge = gio.GetEdgesPtr();
    // 候选顶点集合
    vector <int> U;
    // 预先分配内存，提高性能
    U.reserve(gio.GetVertexCount());
    // 存储中间最大团顶点的向量
    vector<int> max_clique_data_inter;
    max_clique_data_inter.reserve(gio.GetVertexCount());
    // 存储最终最大团顶点的向量
    max_clique_data.reserve(gio.GetVertexCount());
    // 最大团的大小，初始化为下界
    int maxClq = l_bound;
    // 记录更新前的最大团大小
    int prev_maxClq;

    // 打印提示信息，注释掉避免输出干扰
    //cout << "Computing Max Clique... with lower bound " << maxClq << endl;
    // 初始化不同剪枝策略的剪枝次数
    pruned1 = 0;
    pruned2 = 0;
    pruned3 = 0;
    pruned5 = 0;

    // 位向量，用于跟踪顶点是否已经被考虑过
    int *bitVec = new int[gio.GetVertexCount()];
    // 初始化位向量为 0
    memset(bitVec, 0, gio.GetVertexCount() * sizeof(int));

    // 从最后一个顶点开始遍历
    for(int i = gio.GetVertexCount()-1; i >= 0; i--)
    {
        // 标记当前顶点已被考虑
        bitVec[i] = 1;
        // 记录更新前的最大团大小
        prev_maxClq = maxClq;

        // 清空候选顶点集合
        U.clear();
        // 剪枝策略 1：如果当前顶点的度小于最大团大小，跳过该顶点
        if( getDegree(ptrVertex, i) < maxClq)
        {
            // 记录剪枝策略 1 的剪枝次数
            pruned1++;
            continue;
        }

        // 遍历当前顶点的所有邻居顶点
        for( int j = (*ptrVertex)[i]; j < (*ptrVertex)[i + 1]; j++ )
        {	
            // 剪枝策略 2：如果邻居顶点已经被考虑过
            if(bitVec[(*ptrEdge)[j]] != 1)
            {
                // 剪枝策略 3：如果邻居顶点的度大于等于最大团大小，将其添加到候选顶点集合中
                if( getDegree(ptrVertex, (*ptrEdge)[j]) >=  maxClq )			
                    U.push_back((*ptrEdge)[j]);
                else
                    // 记录剪枝策略 3 的剪枝次数
                    pruned3++;
            }
            else 
                // 记录剪枝策略 2 的剪枝次数
                pruned2++;
        }

        // 调用递归子例程查找最大团
        maxCliqueHelper( gio, &U, 1, maxClq, max_clique_data_inter );

        // 如果最大团大小更新了
        if(maxClq > prev_maxClq)
        {
            // 将当前顶点添加到中间最大团顶点向量中
            max_clique_data_inter.push_back(i);
            // 更新最终最大团顶点向量
            max_clique_data = max_clique_data_inter;
        }
        // 清空中间最大团顶点向量
        max_clique_data_inter.clear();
    }

    // 释放位向量的内存
    delete [] bitVec;
    // 清空中间最大团顶点向量
    max_clique_data_inter.clear();

    // 如果定义了 _DEBUG 宏，打印不同剪枝策略的剪枝次数
#ifdef _DEBUG
    cout << "Pruning 1 = " << pruned1 << endl;
    cout << "Pruning 2 = " << pruned2 << endl;
    cout << "Pruning 3 = " << pruned3 << endl;
    cout << "Pruning 5 = " << pruned5 << endl;
#endif

    // 返回最大团的大小
    return maxClq;
}

/**
 * @brief 打印最大团的顶点信息
 * 
 * @param max_clique_data 存储最大团顶点的向量
 */
void print_max_clique(vector<int>& max_clique_data)
{
    // 打印提示信息，注释掉避免输出干扰
    //cout << "Maximum clique: ";
    // 遍历最大团顶点向量并打印顶点信息
    for(int i = 0; i < max_clique_data.size(); i++)
        cout << max_clique_data[i] + 1 << " ";
    cout << endl;
}
}