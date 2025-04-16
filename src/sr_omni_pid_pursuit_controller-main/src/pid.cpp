#include "sr_omni_pid_pursuit_controller/pid.hpp"

int main() {
    PID pid(0.1, 1.0, -1.0, 0.5, 0.1, 0.01);
    double set_point = 10.0;
    double pv = 0.0;

    for (int i = 0; i < 100; ++i) {
        double output = pid.calculate(set_point, pv);
        pv += output * 0.1; // 模拟系统响应
        std::cout << "Iteration " << i << ": pv=" << pv << ", output=" << output << std::endl;
    }

    return 0;
}