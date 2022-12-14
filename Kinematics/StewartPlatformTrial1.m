%All vectors related to the actuators are formatted as follows
%                 VECTOR(IACT,ICOMP)
%                                   IACT is the actuator index (IACT=1-6)
%                                   ICOMP is the component index (1-3)
%                'b' represents base coordinate frame
%                'p' represents platform coordinate frame

% Coordinate systems are defined in function "CONFIG"

BASBAS= zeros(6,3);            %These are the base vectors:Vectors from 
                               %base frame to the joints
PLAPLA=zeros(6,3);             %These are the platform vectors:Vectors from
                               %platform frame to the joints
Pb=zeros(1,3);                 %Platform vector:Vector from base frame to 
                               %platform frame.
TRANS=zeros(3,3);              %Forward trnasformation matrix:from platform       
                               %base
R_TRANS=zeros(3,3);            %Reverese transformation matrix:from base to
                               %platform
FOACTb=zeros(6,3);             %Actuator force vector
ACTLEN=zeros(6,1);             %Magnitudes of the actuator lengths
ACTVEC=zeros(6,3);             %Actuator vector:from base joint to platform joint
TOACTp=zeros(6,3);             %Actuator torque vector:Torque applied to 
                               %platform per actuator
BSWIVb=zeros(6,3);             %Base to swivel joint vector:from base frame
                               %to platform swivel joint


COM_EUz=0;                     %Euler angles of COM and  
COM_EUy=0;
COM_EUx=0;
EUz=0;
EUy=0;
EUx=0;
ACT_LEG_COM=zeros(6,1);
FORCEB=zeros(1,3);             %Net platform force vector:Force vector applied to platform
TORQp=zeros(1,3);              %Net platform torque 
FOACTp=zeros(6,3);             %Actuator force
ANGACp=zeros(1,3);            %Platform angular acceleration
ACTUNI=zeros(6,3);           %Unit actuator vector
ANGVLp=zeros(6,3);            %Angular Velocity
ANGVLb=zeros(6,3);            %Angular velocity
WXRp=zeros(6,3);              %Angular Velocity cross platform vector in 
                              %platform coordinate
WXRb=zeros(6,3);              %same thing in base coordinate

PVLb=zeros(1,3);               %Platform velocity vector
PVLxyz=zeros(6,3);             %Point velocity representing each platform joints
ACTVEL=zeros(6,1);             %Magnitude of actuator velocities


RMASS=0.5;                     %KG
DNEUTRAL=0.3;                  %Actuator length for neutral position


[BASBAS,PLAPLA,HGT]= CONFIG();  %This calls CONFIG which gives the base 
                                %and platform configurations.

%************Problem 1: Given the 6 degrees of     *********************
%************orientation determination of length of each actuator*****


[EUy,EUx,EUz,Y,X,Z]=ASK_ORIENT(); %Gives current orientation of the platform




   




