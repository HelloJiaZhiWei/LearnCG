#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
Eigen::Vector3f Transform2D(Eigen::Vector3f point,float angle,float xtrans,float ytrans);
Eigen::Vector3f Transform2D(Eigen::Vector3f point,Eigen::Matrix3f rotation,Eigen::Matrix3f translation);
int main()
{
    Eigen::Vector3f point(2.0f,1.0f,1.0f);
    Eigen::Vector3f answer = Transform2D(point,45.0f,1.0f,2.0f);
    std::cout <<"("<< answer[0] << "," << answer[1] << ")" << std::endl;
    return 0;

}
Eigen::Vector3f Transform2D(Eigen::Vector3f point,float angle,float xtrans,float ytrans)
{
    float theta = angle/180*M_PI;
    Eigen::Matrix3f rotaMatri;
    rotaMatri << cos(theta),-sin(theta),0,
                 sin(theta),cos(theta),0,
                 0,0,1;
    Eigen::Matrix3f transMatri;
    transMatri << 1,0,xtrans,
                  0,1,ytrans,
                  0,0,1;
    return Transform2D(point,rotaMatri,transMatri);
}
Eigen::Vector3f Transform2D(Eigen::Vector3f point,Eigen::Matrix3f rotation,Eigen::Matrix3f translation)
{
    return translation * rotation * point;
}