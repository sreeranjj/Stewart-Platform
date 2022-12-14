function X_corr = stateEstimator(X_old,uk,Y_meas,Ad,Bd,Bias_discrete)
global Pk_kf
%This does linear kalman filter over the coming data
%Since many sensors are involved at different rates. Its not easy to
%Simulate for different C matrix and stuff. So here an Identity matrix is
%used for C. In real life kalman filter will given estimates of Xk that
%depends on specific set of input available at that time.(System is fully observable anyways)

%% NOISE Variance:))
    SD_xy=0.2; %Sd- standard derivation.THis is not possible with GPS. But possible with other sensors like
    %Poszyx and so on for indoor navigation
    SD_z=0.05; %Z measurements are extremely easy
    SD_vel=0.05; %THis is not possible with GPS. But possible with other sensors like
    %Poszyx and so on for indoor navigation
    SD_angles=0.01;
    SD_omega=sqrt(2.3675e-04);
    noiseSD=[SD_xy,SD_xy,SD_z,SD_vel,SD_vel,SD_vel,SD_omega,SD_omega,SD_omega,SD_angles,SD_angles,SD_angles]';
    R=diag(noiseSD.*noiseSD);%measurement covariance
    Q=ones(12,12)*0.0001; % covariance of model
%% kalman filter
C=eye(12); %Its hard to generate data for each sensor separately as each sensor spits data at different time rates. 
[X_pred,P_pred]=predictFunc(X_old,uk,Pk_kf,Q,Ad,Bd,Bias_discrete);
KalmanGain=getK(P_pred,R,C);
[X_corr,P_corr]=correctFunc(X_pred,P_pred,KalmanGain,C,Y_meas);
Pk_kf=P_corr;%update the Covariance
end