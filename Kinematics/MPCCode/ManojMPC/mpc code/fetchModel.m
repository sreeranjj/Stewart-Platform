
syms x y z vx vy vz  wx wy wz phi theta psi 
X_syms = [x; y; z; vx; vy; vz;  wx; wy; wz; phi; theta; psi];
global Ad Bd Bias_discrete
%% MODEL PARAMETERS
Cth =0.75*9.8;%Newton/unit throttleInput 750gm thrust C2830 motor+10X4.5 
%inch propeller(http://www.rcbazaar.com/product.aspx?productid=1905)
%Note that all inputs(to esc--pulse width(uS)) mentioned here are scaled 
%between 0.0-1.0 for conveniance sake
L =0.45/2; %(half length of quad's diagonal)
torqByThrust= 0.0162;%(Obtained from UIUC Propeller database)

mass = 1.5;
Ixx=0.0142;
Iyy=Ixx;
Izz=0.0284;
u0=0.56;%Average Control input given to each ESC(Electronic speed 
%Controller) at hover condition(taken from pixhawk log)

%% Linearised Model about Hover condition for quadcopter
%X_dot=A_l*x+B_l*u+C    Model is in this form
A_l=zeros(12);%first defining a zero matrix anf filling it up
A_l(1,4)=1;
A_l(2,5)=1;
A_l(3,6)=1;
A_l(4,11)=-4*Cth*u0/mass;
A_l(5,10)=4*Cth*u0/mass;
A_l(10,7)=1;
A_l(11,8)=1;
A_l(12,9)=1;%A_linearised matrix is done
A_l;

Km = L/sqrt(2)*Cth;
B_l=zeros(12,4);%first defining a zero matrix anf filling it up
B_l(6,1:4)=-Cth/mass;
B_l(7,1)=-Km/Ixx;
B_l(7,2)=Km/Ixx;
B_l(7,3)=Km/Ixx;
B_l(7,4)=-Km/Ixx;
B_l(8,1)=Km/Iyy;
B_l(8,2)=-Km/Iyy;
B_l(8,3)=Km/Iyy;
B_l(8,4)=-Km/Iyy;
B_l(9,1)=Km*torqByThrust/Izz;
B_l(9,2)=Km*torqByThrust/Izz;
B_l(9,3)=-Km*torqByThrust/Izz;
B_l(9,4)=-Km*torqByThrust/Izz;
B_l;

CnstBias=[0 0 0 0 0 9.81 0 0 0 0 0 0]'; %Constant offset in the model

Ts=0.02; %Time for Each MPC Iteration. Since 20mS is usually used for 
%regular PID based controllers in quadcopters. the same number is chosen
%for MPC also
Ad=expm(A_l*Ts);
syms tau;
temp1=expm(A_l*tau);
Bd = int(temp1,tau,0,Ts)*B_l; %discretised model
Bd=double(Bd);
Bias_discrete=int(temp1,tau,0,Ts)*CnstBias;
Bias_discrete=double(Bias_discrete);
%Now we have obtained Ad,Bd and Bias term for discrete model of linearised
%system equations. 
Ad
Bd
Bias_discrete
