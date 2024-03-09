#ifndef ALGORITHM_H
#define ALGORITHM_H
typedef struct filter
{
    double ProcessNiose_Q;
    double MeasureNoise_R;
    double InitialPrediction;
    double x_last;
    double p_last;
} Kf;

/**
 * @brief 卡尔曼滤�?
 *
 * @param[in] ResrcData
 * @param[in] ProcessNiose_Q
 * @param[in] MeasureNoise_R
 * @param[in] InitialPrediction
 * @return double
 */
double KalmanFilter(const double ResrcData, Kf *kf);

#endif
