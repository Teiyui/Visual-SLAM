#include "/usr/local/include/sophus/se3.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unistd.h>

using namespace std;
using namespace Eigen;

void ReadFile(string FileName ,vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> &poses);
double ErrorTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_g,
                       vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_e);
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_g,
                    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_e);

int main(int argc, char **argv)
{
    string GroundFile = "/home/zhengyouwei/slam/week3-error/groundtruth.txt";
    string ErrorFile = "/home/zhengyouwei/slam/week3-error/estimated.txt";
    double trajectory_error_RMSE = 0;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_g;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_e;

    ReadFile(GroundFile,poses_g);
    ReadFile(ErrorFile,poses_e);
    trajectory_error_RMSE = ErrorTrajectory(poses_g, poses_e);
    cout<<"trajectory_error_RMSE = "<< trajectory_error_RMSE<<endl;
    DrawTrajectory(poses_g,poses_e);

}


// 读取文件
void ReadFile(string FileName ,vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> &poses)
{
    ifstream tra_file(FileName);

    while(!tra_file.eof()) {
        double time, tx, ty, tz, qx, qy, qz, w;
        tra_file >> time >> tx >> ty >> tz >> qx >> qy >> qz >> w;
        // 读取位移向量
        Eigen::Vector3d t(tx, ty, tz);
        // 读取旋转矩阵的四元数
        Eigen::Quaterniond q = Eigen::Quaterniond(w, qx, qy, qz).normalized();
        // 获取SE(3)
        Sophus::SE3d SE3_qt(q, t);
        poses.push_back(SE3_qt);
    }

}


// 计算轨迹误差
double ErrorTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_g,
                       vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_e )
{
    // 两条轨迹的均方根
    double RMSE = 0;
    Matrix<double,6,1> se3;
    // 每个位姿的误差集合
    vector<double> error;
    // 每个位姿的误差二范式
    double twoNF;
    double size = poses_g.size();
    for(int i=0;i<size;i++){
        // 计算真实轨迹和估计轨迹之间的误差
        se3=(poses_g[i].inverse()*poses_e[i]).log();
        // 计算误差的二范式
        twoNF = se3.squaredNorm();
        error.push_back(twoNF);
    }

    // 计算两条轨迹的均方根
    for(int i=0; i<size;i++){
        RMSE += error[i];
    }
    RMSE = sqrt(RMSE/double(error.size()));
    return RMSE;
}


// 绘制轨迹
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_g,
                    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_e) {
    if (poses_g.empty() || poses_e.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);  //建立一個視窗
    glEnable(GL_DEPTH_TEST);   //啟動深度測試
    glEnable(GL_BLEND);       //啟動混合
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//混合函式glBlendFunc( GLenum sfactor , GLenum dfactor );sfactor 源混合因子dfactor 目標混合因子

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0) //對應的是gluLookAt,攝像機位置,參考點位置,up vector(上向量)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses_g.size() - 1; i++) {
            glColor3f(1 - (float) i / poses_g.size(), 0.0f, (float) i / poses_g.size());
            glBegin(GL_LINES);
            auto p1 = poses_g[i], p2 = poses_g[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t j = 0; j < poses_e.size() - 1; j++) {
            glColor3f(1 - (float) j / poses_e.size(), 0.0f, (float) j / poses_e.size());
            glBegin(GL_LINES);
            auto p1 = poses_e[j], p2 = poses_e[j + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}