//
// Created by Dave on 25.05.2017.
//

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
    /* setting B vector dt and 1 so after multiplying by offset we get pos and speed*/
    double vel =dt*offset;
    double pos= vel*dt/2;
    B(0)=pos;
    B(1)=vel;
    /*initiating computing and measurement covariance matrices with default values*/
    Q(0,0)=compCov11;
    Q(0,1)=compCov12;
    Q(1,0)=compCov21;
    Q(1,1)=compCov22;

    R(0,0)=measureCov11;
    R(0,1)=measureCov12;
    R(1,0)=measureCov21;
    R(1,1)=measureCov22;

    P(0,0)=estCov11;
    P(0,1)=estCov12;
    P(1,0)=estCov21;
    P(1,1)=estCov22;

    F(0,0)=1.0d;
    F(0,1)=dt;
    F(1,0)=0.0d;
    F(1,1)=1.0d;
    I.setIdentity();


}




/*setting initial states*/
void KalmanFilter::initialize(const Eigen::VectorXd &x) {
    x_guessed = x;

}
/*whole algorithm of predicting and updating states*/
void KalmanFilter::compute(const Eigen::VectorXd &v) {
    /*prediction*/
    x_new_guess= F*x_guessed + B ;
    PN= F*P*F.transpose() + Q;
    /*update*/
    G= PN*H.transpose()*(H*PN*H.transpose() +R).inverse();
    x_guessed = x_new_guess + G*(v -H*x_new_guess);
    P=(I-G*H)*PN;

}

Eigen::VectorXd KalmanFilter::returnState() {
    return x_guessed;
}


void KalmanFilter::setoffset(double o) {
    offset = o;
}

double KalmanFilter::returnPos() {
    return x_guessed(0);

}

