clear all
close all
global Ts Pk_kf Xk_simulation
%% Controller parameters
Ts=0.020;
P=20;    %10 works
M=7;    %5 works
Qx=diag([1, 1, 1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 1]);%Importance given to states
lambda=0.06; %Importance given to inputs

%% Model, Initial Condition and Setpoint
fetchModel;
Xk  = [0 0 0 0 0 0 0 0 0 0 0 0]';%(NED frame is followed!!!(NEGATIVE Z IS POSITIVE HEIGHT))
XsetPoint= [3, -5, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
%X is [x y z vx vy vz  wx wy wz phi theta psi]'
Pk_kf=ones(12,12)*0.1;
Xk_simulation=Xk;%Xk_simulation is what the system knows while running but inaccessible by us.;
%% MPC formulation in QP form
[H,f_func]=fetchQPformulation(Ad,Bd,Bias_discrete,Qx,lambda,P,M);
LB=zeros(M*4,1);UB=1.0+zeros(M*4,1); %Constraints 0-1.0

%% Quadcopter Simulation with MPC controller
Tend=10; %Simulate for 10seconds
Kfinal=round(Tend/Ts); %K denote time step
XkStore=zeros(12,Kfinal+1);XkStore(:,1)=Xk;
ukStore=zeros(4,Kfinal);
tic %Timing the code to check its speed
SSoffset=[0 0 0 0 0 0 0 0 0 0 0 0]';
predictionErrorX=[0 0 0 0 0 0 0 0 0 0 0 0]';
for k=1:Kfinal
    SSoffset(3)=(XsetPoint(3)-Xk(3))*0.5; %Refer Report
    [uk,Xexpected] = runMpcController(Xk,XsetPoint+SSoffset,H,f_func,LB,UB,predictionErrorX,Ad,Bd,Bias_discrete);
    Y = quadSimulator(uk);%This is y for next instant
    XkPlusOne = stateEstimator(Xk,uk,Y,Ad,Bd,Bias_discrete);%give old X, get new X
    predictionErrorX=(XkPlusOne-Xexpected);
    Xk=XkPlusOne;%Prepare for next iteration
    ukStore(:,k)=uk;%Keep storing the data as we do simulation
    XkStore(:,k+1)=Xk_simulation;%Keep storing the data as we do simulation
end
toc
%% Plotting Data
t=(1:(Kfinal+1))*Ts;
%Xk data
for i=1:12
    subplot(4,3,i);plot(t,XkStore(i,:));title(i)
end
figure
t=(1:(Kfinal))*Ts;
%uk data
for i=1:4
    subplot(2,2,i);plot(t,ukStore(i,:));title(i)
end