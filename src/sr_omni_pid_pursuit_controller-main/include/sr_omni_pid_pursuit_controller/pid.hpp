#ifndef SR_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_
#define SR_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_
#include <iostream>
#include <cmath>

class PID {
public:
    PID(double dt, double max, double min, double kp, double kd, double ki)
    : dt_(dt), max_(max), min_(min), kp_(kp), kd_(kd), ki_(ki), pre_error_(0), integral_(0), set_point_(0), pv_(0) {}

    double calculate(double set_point, double pv) {
        // 更新设定值和当前值
        double error = set_point - pv;
        double p_out = kp_ * error;

        // 积分项
        integral_ += error * dt_;
        double i_out = ki_ * integral_;

        // 积分防饱和
        if (integral_ > max_) {
            integral_ = max_;
        } else if (integral_ < min_) {
            integral_ = min_;
        }

        // 微分项
        double derivative = (error - pre_error_) / dt_;
        double d_out = kd_ * derivative;

        // 计算总输出
        double output = p_out + i_out + d_out;

        // 限制输出
        if (output > max_) {
            output = max_;
        } else if (output < min_) {
            output = min_;
        }

        // 保存误差
        pre_error_ = error;

        // 更新设定值和当前值
        set_point_ = set_point;
        pv_ = pv;

        return output;
    }

    void setGains(double kp, double kd, double ki) {
        kp_ = kp;
        kd_ = kd;
        ki_ = ki;
    }

    void setSumError(double sum_error) {
        integral_ = sum_error;
    }

    void reset() {
        pre_error_ = 0;
        integral_ = 0;
    }

    void log()  {
        std::cout << "PID: p_out=" << kp_ * (set_point_ - pv_) 
                  << ", i_out=" << ki_ * integral_ 
                  << ", d_out=" << kd_ * ((set_point_ - pv_) - pre_error_) / dt_ 
                  << ", output=" << calculate(set_point_, pv_) << std::endl;
    }

    ~PID() {}

private:
    double dt_;
    double max_;
    double min_;
    double kp_;
    double kd_;
    double ki_;
    double pre_error_;
    double integral_;
    double set_point_;
    double pv_;
};

#endif