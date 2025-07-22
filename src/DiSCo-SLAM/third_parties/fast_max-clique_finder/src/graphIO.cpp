/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*   Description:  an I/O library for reading a graph    			 	   */                                                   
/*                                                                           		   */
/*                                                                           		   */
/*   Authors: Md. Mostofa Ali Patwary and Bharath Pattabiraman             		   */
/*            EECS Department, Northwestern University                       		   */
/*            email: {mpatwary,bpa342}@eecs.northwestern.edu                 		   */
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

// 包含图输入输出相关的头文件
#include "graphIO.h"

// 进入 FMC 命名空间
namespace FMC {
/**
 * @brief CGraphIO 类的析构函数
 * 
 * 用于清理 CGraphIO 对象中存储图信息的向量，释放内存。
 */
CGraphIO::~CGraphIO()
{
    // 清空存储顶点信息的向量
    m_vi_Vertices.clear();
    // 清空存储有序顶点信息的向量
    m_vi_OrderedVertices.clear();
    // 清空存储边信息的向量
    m_vi_Edges.clear();
    // 清空存储边权重值的向量
    m_vd_Values.clear();
}

/**
 * @brief 读取图文件
 * 
 * 根据文件扩展名判断文件格式，调用相应的读取函数读取图数据。
 * 
 * @param s_InputFile 输入图文件的路径
 * @param connStrength 连接强度阈值，用于过滤边
 * @return bool 读取成功返回 true，失败返回 false
 */
bool CGraphIO::readGraph(string s_InputFile, float connStrength)
{
    // 获取文件扩展名
    string fileExtension = getFileExtension(s_InputFile);		
    if(fileExtension == "mtx")
    {
        // 如果是 Matrix Market 格式文件
        return ReadMatrixMarketAdjacencyGraph(s_InputFile, connStrength);
    }
    else if(fileExtension == "gr")
    {
        // 如果是 gr 格式文件
        return ReadMeTiSAdjacencyGraph(s_InputFile);
    }
    else
        // 不支持的文件格式
        return false;	
}

/**
 * @brief 读取 Matrix Market 格式的邻接图文件
 * 
 * 解析 Matrix Market 格式的文件，构建图的顶点和边信息，并计算顶点度。
 * 
 * @param s_InputFile 输入的 Matrix Market 格式文件路径
 * @param connStrength 连接强度阈值，用于过滤边
 * @return bool 读取成功返回 true，失败返回 false
 */
bool CGraphIO::ReadMatrixMarketAdjacencyGraph(string s_InputFile, float connStrength)
{
    // 字符串输入流，用于解析行数据
    istringstream in2;
    // 存储读取的行数据
    string line="";
    // 存储节点邻接关系的映射
    map<int,vector<int> > nodeList;
    // 存储边权重值的映射
    map<int,vector<double> > valueList;
    // 矩阵的列数
    int col=0, 
    // 矩阵的行数
    row=0, 
    // 当前行索引
    rowIndex=0, 
    // 当前列索引
    colIndex=0;
    // 已读取的条目数
    int entry_counter = 0, 
    // 文件中条目的总数
    num_of_entries = 0;
    // 边的权重值
    double value;

    // 打开输入文件
    ifstream in (s_InputFile.c_str());	
    if(!in) 
    {
        // 文件打开失败
        cout<<m_s_InputFile<<" not Found!"<<endl;
        return false;
    }

    // 存储读取的行数据
    char data[LINE_LENGTH];
    // 存储文件头部信息
    char banner[LINE_LENGTH];
    // 存储矩阵类型信息
    char mtx[LINE_LENGTH];
    // 存储坐标类型信息
    char crd[LINE_LENGTH];
    // 存储数据类型信息
    char data_type[LINE_LENGTH];
    // 存储存储方案信息
    char storage_scheme[LINE_LENGTH];
    // 字符指针，用于处理字符串
    char* p;
    // 是否获取边权重值的标志
    bool b_getValue = true;
    // 上三角部分重复条目的数量
    int num_upper_triangular = 0;

    // 读取文件的第一行（banner 行）
    getline(in, line);
    strcpy(data, line.c_str());

    // 解析 banner 行
    if (sscanf(data, "%s %s %s %s %s", banner, mtx, crd, data_type, storage_scheme) != 5)
    {    
        // banner 行格式错误
        cout << "Matrix file banner is missing!!!" << endl;
        return false;
    }

    // 将数据类型转换为小写
    for (p=data_type; *p!='\0'; *p=tolower(*p),p++);

    // 如果数据类型为 pattern，则不获取边权重值
    if (strcmp(data_type, "pattern") == 0)
        b_getValue = false;

    // 读取下一行
    getline(in, line);
    // 跳过注释行
    while(line.size()>0&&line[0]=='%') 
        getline(in,line);

    // 解析矩阵的行数、列数和条目总数
    in2.str(line);
    in2 >> row >> col >> num_of_entries;

    if(row!=col) 
    {
        // 矩阵不是方阵，无法处理
        cout<<"* WARNING: GraphInputOutput::ReadMatrixMarketAdjacencyGraph()"<<endl;
        cout<<"*\t row!=col. This is not a square matrix. Can't process."<<endl;
        return false;
    }

    // 逐行读取文件，直到文件结束或读取完所有条目
    while(!in.eof() && entry_counter<num_of_entries) 
    {
        getline(in,line);
        entry_counter++;

        if(line!="")
        {
            // 清空字符串输入流并重新设置数据
            in2.clear();
            in2.str(line);

            // 解析行中的行索引、列索引和边权重值
            in2 >> rowIndex >> colIndex >> value;
            // 调整索引为 0 起始
            rowIndex--;
            colIndex--;

            if(rowIndex < 0 || rowIndex >= row)
                // 行索引越界
                cout << "Something wrong rowIndex " << rowIndex << " row " << row << endl;

            if(colIndex < 0 || colIndex >= col)
                // 列索引越界
                cout << "Something wrong colIndex " << colIndex << " col " << col << endl;

            if(rowIndex == colIndex)
            {
                // 忽略自环边
                continue;
            }

            // 检查边是否已经存在
            int exists=0;
            for(int k=0; k<nodeList[rowIndex].size(); k++) {
                if(colIndex == nodeList[rowIndex][k]) {
                    exists = 1;
                    break;
                }
            }

            if(exists==1) {
                // 边已存在，记录上三角部分重复条目数
                num_upper_triangular++;
            } else {
                if(b_getValue)
                {
                    if(value > connStrength)
                    {
                        // 添加边到邻接关系映射
                        nodeList[rowIndex].push_back(colIndex);
                        nodeList[colIndex].push_back(rowIndex);
                    }
                } 
                else 
                {
                    // 添加边到邻接关系映射
                    nodeList[rowIndex].push_back(colIndex);
                    nodeList[colIndex].push_back(rowIndex);
                }

                if(b_getValue && value > connStrength) 
                {
                    // 添加边权重值到映射
                    valueList[rowIndex].push_back(value);
                    valueList[colIndex].push_back(value);
                }
            }
        }
    }

    // 输出上三角部分重复条目的数量，注释掉避免干扰
    //cout << "No. of upper triangular pruned: " << num_upper_triangular << endl;
    // 初始化顶点向量
    m_vi_Vertices.push_back(m_vi_Edges.size());

    // 构建顶点和边的向量
    for(int i=0;i < row; i++) 
    {
        m_vi_Edges.insert(m_vi_Edges.end(),nodeList[i].begin(),nodeList[i].end());
        m_vi_Vertices.push_back(m_vi_Edges.size());
    }

    if(b_getValue) 
    {
        // 构建边权重值向量
        for(int i=0;i<row; i++) 
        {
            m_vd_Values.insert(m_vd_Values.end(),valueList[i].begin(),valueList[i].end());
        }
    }

    // 清空临时映射
    nodeList.clear();
    valueList.clear();
    // 计算顶点度
    CalculateVertexDegrees();
    return true;
}

/**
 * @brief 读取 MeTiS 格式的邻接图文件
 * 
 * 目前该函数仅返回 true，未实现具体读取逻辑。
 * 
 * @param s_InputFile 输入的 MeTiS 格式文件路径
 * @return bool 固定返回 true
 */
bool CGraphIO::ReadMeTiSAdjacencyGraph(string s_InputFile)
{
    return true;
}

/**
 * @brief 计算图中顶点的度信息
 * 
 * 计算图中每个顶点的度，并统计最大度、最小度和平均度。
 */
void CGraphIO::CalculateVertexDegrees()
{
    // 顶点的数量
    int i_VertexCount = m_vi_Vertices.size() - 1;

    // 初始化最大顶点度为 -1
    m_i_MaximumVertexDegree =  -1;
    // 初始化最小顶点度为 -1
    m_i_MinimumVertexDegree = -1;

    // 遍历每个顶点
    for(int i = 0; i < i_VertexCount; i++)
    {
        // 计算当前顶点的度
        int i_VertexDegree = m_vi_Vertices[i + 1] - m_vi_Vertices[i];

        if(m_i_MaximumVertexDegree < i_VertexDegree)
        {
            // 更新最大顶点度
            m_i_MaximumVertexDegree = i_VertexDegree;
        }

        if(m_i_MinimumVertexDegree == -1)
        {
            // 初始化最小顶点度
            m_i_MinimumVertexDegree = i_VertexDegree;
        }
        else if(m_i_MinimumVertexDegree > i_VertexDegree)
        {
            // 更新最小顶点度
            m_i_MinimumVertexDegree = i_VertexDegree;
        }
    }

    // 计算平均顶点度
    m_d_AverageVertexDegree = (double)m_vi_Edges.size()/i_VertexCount;

    return;
}

/**
 * @brief 获取文件的扩展名
 * 
 * 从文件路径中提取文件的扩展名。
 * 
 * @param fileName 文件名或文件路径
 * @return string 文件的扩展名
 */
string CGraphIO::getFileExtension(string fileName)
{
    // 存储查找结果的位置
    string::size_type result;
    // 存储文件扩展名
    string fileExtension = "";

    // 1. 此处注释掉的代码用于处理文件全路径，提取文件名
    /*result = fileName.rfind("/", fileName.size() - 1);
      if(result != string::npos)
      {
    //found the path (file prefix)
    //get the path, including the last DIR_SEPARATOR
    path = fileName.substr(0,result+1);
    //remove the path from the fileName
    fileName = fileName.substr(result+1);
    }
    */

    // 2. 查找文件扩展名
    result = fileName.rfind('.', fileName.size() - 1);
    if(result != string::npos)
    {
        // 提取文件扩展名，不包含 '.'
        fileExtension = fileName.substr(result+1);
        // 此处注释掉的代码用于移除文件扩展名
        //fileName = fileName.substr(0,result);	
    }

    // 3. 此处注释掉的代码用于获取输入文件的名称
    //name = fileName;

    return fileExtension;
}
}