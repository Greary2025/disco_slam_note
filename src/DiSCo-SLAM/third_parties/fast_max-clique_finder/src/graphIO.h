/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*   Description:  一个用于读取图的输入输出库                                                    */                                                   
/*                                                                           		   */
/*                                                                           		   */
/*   Authors: Md. Mostofa Ali Patwary 和 Bharath Pattabiraman             		   */
/*            美国西北大学电子工程与计算机科学系                                               */
/*            邮箱: {mpatwary,bpa342}@eecs.northwestern.edu                 		   */
/*                                                                           		   */
/*   Copyright, 2014, 西北大学                                                              */
/*   请查看顶级目录中的 COPYRIGHT 声明。                                                      */
/*                                                                           		   */
/*   如果使用此软件包，请引用以下出版物:                                                        */
/*   Bharath Pattabiraman, Md. Mostofa Ali Patwary, Assefaw H. Gebremedhin2, 	   	   */
/*   Wei-keng Liao, and Alok Choudhary.                                                     */
/*   "Fast Algorithms for the Maximum Clique Problem on Massive Graphs with                */
/*   Applications to Overlapping Community Detection"                                      */
/*   http://arxiv.org/abs/1411.7460                                                        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// 头文件保护宏，防止头文件被重复包含
#ifndef _graphIO_
#define _graphIO_

// 包含标准输入输出流头文件，用于输入输出操作
#include <iostream>
// 包含向量容器头文件，用于存储动态数组
#include <vector>
// 包含映射容器头文件，用于存储键值对
#include <map>
// 包含文件流头文件，用于文件读写操作
#include <fstream>
// 包含字符串流头文件，用于字符串处理
#include <sstream>
// 包含浮点数相关的常量头文件，用于获取浮点数的最大值等信息
#include <float.h>
// 包含字符串处理函数头文件，用于字符串操作
#include <string.h>

// 定义每行的最大长度
#define LINE_LENGTH 256

// 使用标准命名空间，避免每次使用标准库中的类和函数都要加 std::
using namespace std;

// 定义 FMC 命名空间，将图相关的类和函数封装起来
namespace FMC {
// 定义 IntVector 为 std::vector<int> 的别名，方便后续使用
typedef std::vector<int> IntVector;

/**
 * @brief 图输入输出类，用于读取不同格式的图文件并处理图的相关信息
 */
class CGraphIO
{
public:
    /**
     * @brief 默认构造函数
     */
    CGraphIO(){ }

    /**
     * @brief 虚析构函数，用于释放对象占用的资源
     */
    virtual ~CGraphIO();

    /**
     * @brief 读取图文件
     * 
     * 根据文件扩展名判断文件格式，调用相应的读取函数读取图数据
     * 
     * @param s_InputFile 输入图文件的路径
     * @param connStrength 连接强度阈值，用于过滤边，默认为负的双精度浮点数最大值
     * @return bool 读取成功返回 true，失败返回 false
     */
    bool readGraph(string s_InputFile, float connStrength = -DBL_MAX);

    /**
     * @brief 获取文件的扩展名
     * 
     * 从文件路径中提取文件的扩展名
     * 
     * @param fileName 文件名或文件路径
     * @return string 文件的扩展名
     */
    string getFileExtension(string fileName);

    /**
     * @brief 读取 Matrix Market 格式的邻接图文件
     * 
     * 解析 Matrix Market 格式的文件，构建图的顶点和边信息，并计算顶点度
     * 
     * @param s_InputFile 输入的 Matrix Market 格式文件路径
     * @param connStrength 连接强度阈值，用于过滤边，默认为负的双精度浮点数最大值
     * @return bool 读取成功返回 true，失败返回 false
     */
    bool ReadMatrixMarketAdjacencyGraph(string s_InputFile, float connStrength = -DBL_MAX);

    /**
     * @brief 读取 MeTiS 格式的邻接图文件
     * 
     * @param s_InputFile 输入的 MeTiS 格式文件路径
     * @return bool 读取成功返回 true，失败返回 false
     */
    bool ReadMeTiSAdjacencyGraph(string s_InputFile);

    /**
     * @brief 计算图中顶点的度信息
     * 
     * 计算图中每个顶点的度，并统计最大度、最小度和平均度
     */
    void CalculateVertexDegrees();

    /**
     * @brief 获取图的顶点数量
     * 
     * @return int 图的顶点数量
     */
    int GetVertexCount(){ return m_vi_Vertices.size() - 1; }

    /**
     * @brief 获取图的边数量
     * 
     * @return int 图的边数量
     */
    int GetEdgeCount(){ return m_vi_Edges.size()/2; }

    /**
     * @brief 获取图中顶点的最大度
     * 
     * @return int 图中顶点的最大度
     */
    int GetMaximumVertexDegree(){ return m_i_MaximumVertexDegree; }

    /**
     * @brief 获取图中顶点的最小度
     * 
     * @return int 图中顶点的最小度
     */
    int GetMinimumVertexDegree(){ return m_i_MinimumVertexDegree; }

    /**
     * @brief 获取图中顶点的平均度
     * 
     * @return double 图中顶点的平均度
     */
    double GetAverageVertexDegree(){ return m_d_AverageVertexDegree; }

    /**
     * @brief 获取输入图文件的路径
     * 
     * @return string 输入图文件的路径
     */
    string GetInputFile(){ return m_s_InputFile; }

    /**
     * @brief 获取存储顶点信息的向量的指针
     * 
     * @return vector <int>* 指向存储顶点信息向量的指针
     */
    vector <int>* GetVerticesPtr(){ return &m_vi_Vertices; }

    /**
     * @brief 获取存储边信息的向量的指针
     * 
     * @return vector <int>* 指向存储边信息向量的指针
     */
    vector <int>* GetEdgesPtr(){ return &m_vi_Edges; }

public:
    // 图中顶点的最大度
    int m_i_MaximumVertexDegree;
    // 图中顶点的最小度
    int m_i_MinimumVertexDegree;
    // 图中顶点的平均度
    double m_d_AverageVertexDegree;
    // 输入图文件的路径
    string 	m_s_InputFile;
    // 存储顶点信息的向量
    vector<int> 	m_vi_Vertices;
    // 存储有序顶点信息的向量
    vector<int>   m_vi_OrderedVertices;
    // 存储边信息的向量
    vector<int> 	m_vi_Edges;
    // 存储边权重值的向量
    vector<double> 	m_vd_Values;
};
}
// 结束头文件保护
#endif
