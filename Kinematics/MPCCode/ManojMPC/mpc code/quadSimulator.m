function Yk_new=quadSimulator(uk)
    global Ad Bd Bias_discrete Xk_simulation
    %% SIMULATE FUTURE
    Xk_simulation=Ad*Xk_simulation+Bd*uk+Bias_discrete;
    C=eye(12); %Its hard to generate data for each sensor separately as each sensor spits data at different time rates. 
    %For EKF implementation, we have used actual data from a quadcopter.
    %But generating such data on computer is difficult. So a simple
    %equivalent imaginary case is taken here for C matrix.
    %% ADDING NOISE :))
    SD_xy=0.2; %Sd- standard derivation.THis is not possible with GPS. But possible with other sensors like
    %Poszyx and so on for indoor navigation
    SD_z=0.05; %Z measurements are extremely easy
    SD_vel=0.05^2; %THis is not possible with GPS. But possible with other sensors like
    %Poszyx and so on for indoor navigation
    SD_angles=0.001;
    SD_omega=sqrt(2.3675e-04);
    noise=[SD_xy,SD_xy,SD_z,SD_vel,SD_vel,SD_vel,SD_omega,SD_omega,SD_omega,SD_angles,SD_angles,SD_angles]'.*randn(12,1);
    %% Adding disturbance Bias(unknown)
    biasUnknown=[0.5 0.5 0.1 0.05 0.05 0.05 0.001 0.001 0.001 0.001 0.001 0.001]';
    %% OUTPUT
    Yk_new=C*Xk_simulation+noise+biasUnknown;
end