#include <iostream>
using namespace std;
#include <eigen3/Eigen/Dense>

int main() {
    Eigen::Vector3d u;
    u(0) = 3;
    u(1) = 4;
    u(2) = 5;
    Eigen::Vector3d v = u/2;
    std::cout << v(0) <<" " <<v(1)<< " " << v(2 ) << std::endl;
    return 0;
}