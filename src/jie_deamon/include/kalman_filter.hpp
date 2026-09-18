/**
 * @file kalman_filter.hpp
 * @brief 2D卡尔曼滤波器 - 用于平滑目标追踪位置
 * 
 * 采用匀速运动模型(Constant Velocity)，状态向量为 [x, y, vx, vy]。
 * 不依赖定时器，通过雷达数据的到达间隔推算dt。
 */

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <array>
#include <cmath>
#include <chrono>

/**
 * @class KalmanFilter2D
 * @brief 二维匀速运动模型卡尔曼滤波器
 * 
 * 状态向量: [x, y, vx, vy]
 * 观测向量: [x, y]
 * 
 * 状态转移:
 *   x  = x  + vx * dt
 *   y  = y  + vy * dt
 *   vx = vx
 *   vy = vy
 */
class KalmanFilter2D {
public:
    /**
     * @brief 构造函数
     * @param process_noise 过程噪声系数，越大表示越信任观测值（跟踪更灵敏）
     * @param measurement_noise 观测噪声系数，越大表示越信任预测（滤波更平滑）
     */
    KalmanFilter2D(double process_noise = 0.1, double measurement_noise = 0.05)
        : q_(process_noise), r_(measurement_noise) {
        reset();
    }

    /**
     * @brief 设置噪声参数
     */
    void setProcessNoise(double q) { q_ = q; }
    void setMeasurementNoise(double r) { r_ = r; }

    /**
     * @brief 重置滤波器状态
     */
    void reset() {
        initialized_ = false;
        // 状态: [x, y, vx, vy]
        x_ = {0.0, 0.0, 0.0, 0.0};
        // 协方差矩阵P（4x4，用一维数组展平，对角初始化）
        P_.fill(0.0);
        P_[0]  = 1.0;   // P(0,0) = var(x)
        P_[5]  = 1.0;   // P(1,1) = var(y)
        P_[10] = 10.0;  // P(2,2) = var(vx)，速度初始不确定性较大
        P_[15] = 10.0;  // P(3,3) = var(vy)
    }

    /**
     * @brief 用观测值更新滤波器（预测+更新一体化）
     * @param meas_x 观测到的x坐标
     * @param meas_y 观测到的y坐标
     * @param[out] filtered_x 滤波后的x坐标
     * @param[out] filtered_y 滤波后的y坐标
     */
    void update(double meas_x, double meas_y,
                double& filtered_x, double& filtered_y) {
        auto now = std::chrono::steady_clock::now();

        if (!initialized_) {
            // 首次调用，直接用观测值初始化
            x_[0] = meas_x;
            x_[1] = meas_y;
            x_[2] = 0.0;
            x_[3] = 0.0;
            last_time_ = now;
            initialized_ = true;
            filtered_x = meas_x;
            filtered_y = meas_y;
            return;
        }

        // 计算dt（秒）
        double dt = std::chrono::duration<double>(now - last_time_).count();
        last_time_ = now;

        // 防止dt异常（过大或过小）
        if (dt <= 0.0 || dt > 1.0) {
            dt = 0.1;  // 默认值，对应约10Hz
        }

        // === 预测步骤 ===
        predict(dt);

        // === 更新步骤 ===
        correct(meas_x, meas_y);

        filtered_x = x_[0];
        filtered_y = x_[1];
    }

    /**
     * @brief 仅执行预测（无观测值时调用，如目标丢失时）
     * @param dt 时间间隔（秒）
     * @param[out] pred_x 预测的x坐标
     * @param[out] pred_y 预测的y坐标
     */
    void predictOnly(double dt, double& pred_x, double& pred_y) {
        if (!initialized_) {
            pred_x = 0.0;
            pred_y = 0.0;
            return;
        }
        predict(dt);
        pred_x = x_[0];
        pred_y = x_[1];
    }

    /**
     * @brief 强制设置状态（如用户手动设置目标时）
     */
    void setState(double x, double y) {
        x_[0] = x;
        x_[1] = y;
        x_[2] = 0.0;  // 重置速度
        x_[3] = 0.0;
        // 重置协方差
        P_.fill(0.0);
        P_[0]  = 0.5;
        P_[5]  = 0.5;
        P_[10] = 10.0;
        P_[15] = 10.0;
        initialized_ = true;
        last_time_ = std::chrono::steady_clock::now();
    }

    bool isInitialized() const { return initialized_; }

    // 获取滤波后的速度估计
    double getVelocityX() const { return x_[2]; }
    double getVelocityY() const { return x_[3]; }

private:
    double q_;  // 过程噪声系数
    double r_;  // 观测噪声系数
    bool initialized_ = false;

    // 状态向量 [x, y, vx, vy]
    std::array<double, 4> x_;
    // 协方差矩阵 P (4x4展平)
    std::array<double, 16> P_;

    std::chrono::steady_clock::time_point last_time_;

    /**
     * @brief 预测步骤
     * 
     * 状态转移: x' = F * x
     * F = [1, 0, dt, 0 ]
     *     [0, 1, 0,  dt]
     *     [0, 0, 1,  0 ]
     *     [0, 0, 0,  1 ]
     * 
     * 过程噪声: Q = q * [dt^3/3, 0,      dt^2/2, 0     ]
     *                    [0,      dt^3/3, 0,      dt^2/2]
     *                    [dt^2/2, 0,      dt,     0     ]
     *                    [0,      dt^2/2, 0,      dt    ]
     */
    void predict(double dt) {
        // 状态预测: x' = F * x
        x_[0] += x_[2] * dt;
        x_[1] += x_[3] * dt;
        // vx, vy 不变

        // 协方差预测: P' = F * P * F^T + Q
        // 直接展开计算，避免矩阵库依赖
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;

        // 先计算 F*P （临时存储）
        std::array<double, 16> FP;
        // F*P 第0行: P[0]+dt*P[8], P[1]+dt*P[9], P[2]+dt*P[10], P[3]+dt*P[11]
        FP[0]  = P_[0]  + dt * P_[8];
        FP[1]  = P_[1]  + dt * P_[9];
        FP[2]  = P_[2]  + dt * P_[10];
        FP[3]  = P_[3]  + dt * P_[11];
        // F*P 第1行: P[4]+dt*P[12], P[5]+dt*P[13], P[6]+dt*P[14], P[7]+dt*P[15]
        FP[4]  = P_[4]  + dt * P_[12];
        FP[5]  = P_[5]  + dt * P_[13];
        FP[6]  = P_[6]  + dt * P_[14];
        FP[7]  = P_[7]  + dt * P_[15];
        // F*P 第2行: P[8..11] 不变
        FP[8]  = P_[8];
        FP[9]  = P_[9];
        FP[10] = P_[10];
        FP[11] = P_[11];
        // F*P 第3行: P[12..15] 不变
        FP[12] = P_[12];
        FP[13] = P_[13];
        FP[14] = P_[14];
        FP[15] = P_[15];

        // P' = FP * F^T + Q
        // F^T的列: [1,0,0,0], [0,1,0,0], [dt,0,1,0], [0,dt,0,1]
        P_[0]  = FP[0]  + FP[2]  * dt + q_ * dt3 / 3.0;
        P_[1]  = FP[1]  + FP[3]  * dt;
        P_[2]  = FP[2]  + q_ * dt2 / 2.0;
        P_[3]  = FP[3];

        P_[4]  = FP[4]  + FP[6]  * dt;
        P_[5]  = FP[5]  + FP[7]  * dt + q_ * dt3 / 3.0;
        P_[6]  = FP[6]  + q_ * dt2 / 2.0;
        P_[7]  = FP[7];

        P_[8]  = FP[8]  + FP[10] * dt + q_ * dt2 / 2.0;
        P_[9]  = FP[9]  + FP[11] * dt;
        P_[10] = FP[10] + q_ * dt;
        P_[11] = FP[11];

        P_[12] = FP[12] + FP[14] * dt;
        P_[13] = FP[13] + FP[15] * dt + q_ * dt2 / 2.0;
        P_[14] = FP[14] + q_ * dt;
        P_[15] = FP[15];
    }

    /**
     * @brief 更新步骤
     * 
     * 观测矩阵: H = [1, 0, 0, 0]
     *               [0, 1, 0, 0]
     * 
     * 新息: y = z - H*x = [meas_x - x[0], meas_y - x[1]]
     * 新息协方差: S = H*P*H^T + R
     * 卡尔曼增益: K = P*H^T * S^(-1)
     * 状态更新: x = x + K*y
     * 协方差更新: P = (I - K*H) * P
     */
    void correct(double meas_x, double meas_y) {
        // 新息
        double y0 = meas_x - x_[0];
        double y1 = meas_y - x_[1];

        // S = H*P*H^T + R (2x2矩阵)
        double s00 = P_[0]  + r_;
        double s01 = P_[1];
        double s10 = P_[4];
        double s11 = P_[5]  + r_;

        // S的逆 (2x2矩阵求逆)
        double det = s00 * s11 - s01 * s10;
        if (std::abs(det) < 1e-12) return;  // 奇异矩阵保护
        double inv_det = 1.0 / det;
        double si00 =  s11 * inv_det;
        double si01 = -s01 * inv_det;
        double si10 = -s10 * inv_det;
        double si11 =  s00 * inv_det;

        // K = P*H^T * S^(-1)  (4x2矩阵)
        // P*H^T = P的前两列: [P[0],P[1]; P[4],P[5]; P[8],P[9]; P[12],P[13]]
        double k00 = P_[0]  * si00 + P_[1]  * si10;
        double k01 = P_[0]  * si01 + P_[1]  * si11;
        double k10 = P_[4]  * si00 + P_[5]  * si10;
        double k11 = P_[4]  * si01 + P_[5]  * si11;
        double k20 = P_[8]  * si00 + P_[9]  * si10;
        double k21 = P_[8]  * si01 + P_[9]  * si11;
        double k30 = P_[12] * si00 + P_[13] * si10;
        double k31 = P_[12] * si01 + P_[13] * si11;

        // 状态更新: x = x + K * y
        x_[0] += k00 * y0 + k01 * y1;
        x_[1] += k10 * y0 + k11 * y1;
        x_[2] += k20 * y0 + k21 * y1;
        x_[3] += k30 * y0 + k31 * y1;

        // 协方差更新: P = (I - K*H) * P
        // I - K*H 是4x4矩阵
        // (I-KH)[i][j] = I[i][j] - K[i][0]*H[0][j] - K[i][1]*H[1][j]
        // H[0][j] = j==0?1:0,  H[1][j] = j==1?1:0
        std::array<double, 16> P_new;
        // 保存旧P用于计算
        auto P_old = P_;

        // 第0行
        P_new[0]  = (1.0 - k00) * P_old[0]  - k01 * P_old[4];
        P_new[1]  = (1.0 - k00) * P_old[1]  - k01 * P_old[5];
        P_new[2]  = (1.0 - k00) * P_old[2]  - k01 * P_old[6];
        P_new[3]  = (1.0 - k00) * P_old[3]  - k01 * P_old[7];
        // 第1行
        P_new[4]  = -k10 * P_old[0]  + (1.0 - k11) * P_old[4];
        P_new[5]  = -k10 * P_old[1]  + (1.0 - k11) * P_old[5];
        P_new[6]  = -k10 * P_old[2]  + (1.0 - k11) * P_old[6];
        P_new[7]  = -k10 * P_old[3]  + (1.0 - k11) * P_old[7];
        // 第2行
        P_new[8]  = -k20 * P_old[0]  - k21 * P_old[4]  + P_old[8];
        P_new[9]  = -k20 * P_old[1]  - k21 * P_old[5]  + P_old[9];
        P_new[10] = -k20 * P_old[2]  - k21 * P_old[6]  + P_old[10];
        P_new[11] = -k20 * P_old[3]  - k21 * P_old[7]  + P_old[11];
        // 第3行
        P_new[12] = -k30 * P_old[0]  - k31 * P_old[4]  + P_old[12];
        P_new[13] = -k30 * P_old[1]  - k31 * P_old[5]  + P_old[13];
        P_new[14] = -k30 * P_old[2]  - k31 * P_old[6]  + P_old[14];
        P_new[15] = -k30 * P_old[3]  - k31 * P_old[7]  + P_old[15];

        P_ = P_new;
    }
};

#endif // KALMAN_FILTER_HPP
