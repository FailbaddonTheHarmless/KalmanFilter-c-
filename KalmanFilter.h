//
// Created by Dave on 25.05.2017.
//

#ifndef KALMAN_FILTER_KALMANFILTER_H
#define KALMAN_FILTER_KALMANFILTER_H


#include <eigen3/Eigen/Dense>

class KalmanFilter {
private:

    /*********************************************************/

    /*Computed covariance matrix of the given measurements of speed and position using matlab cov() function. */
    const double measureCov11=1.0806745120268212e-01;
    const double measureCov12=-1.3175717668886839e-03;
    const double measureCov21=-1.3175717668886839e-03;
    const double measureCov22=4.5589780326132856e-01;

    /********************************************************/
    /*Reasonable process noise covariance values*/
    const double compCov11=0.50d;
    const double compCov12=0.50d;
    const double compCov21=0.50d;
    const double compCov22=0.50d;

    /*********************************************************/
    /*Estimate error covariance*/

    const double estCov11=0.50d;
    const double estCov12=0.50d;
    const double estCov21=0.50d;
    const double estCov22=0.50d;


    /**********************************************************/

    /*Time between samples*/
    const double dt=1.0d/100;

    double offset=0.00005d;
    /*********************************************************/

    /*A set of eigen matrices*/
    Eigen::Matrix2d F,Q,R,H;
    Eigen::Matrix2d P,G,PN,I;

    Eigen::Vector2d B;

    /*Guessed vectors*/
    Eigen::Vector2d x_guessed, x_new_guess;


public:
    KalmanFilter();


    void initialize(const Eigen::VectorXd& x);

    void compute(const Eigen::VectorXd& v);
    void setoffset(double o);
    double returnPos();

    Eigen::VectorXd returnState();


};


#endif //KALMAN_FILTER_KALMANFILTER_H
