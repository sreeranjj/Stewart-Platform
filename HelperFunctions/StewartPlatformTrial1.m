%All vectors related to the actuators are formatted as follows
%                 VECTOR(IACT,ICOMP)
%                                   IACT is the actuator index (IACT=1-6)
%                                   ICOMP is the component index (1-3)
%                'b' represents base coordinate frame
%                'p' represents platform coordinate frame

% Coordinate systems are defined in function "CONFIG"

%BASBAS= zeros(6,3);            %These are the base vectors:Vectors from 
                               %base frame to the joints
%PLAPLA=zeros(6,3);             %These are the platform vectors:Vectors from
                               %platform frame to the joints
%Pb=zeros(1,3);                 %Platform vector:Vector from base frame to 
                               %platform frame.
TRANS=zeros(3,3);              %Forward trnasformation matrix:from platform       
                               %base
R_TRANS=zeros(3,3);            %Reverese transformation matrix:from base to
                               %platform
FOACTb=zeros(6,3);             %Actuator force vector
ACTLEN=zeros(6,1);             %Magnitudes of the actuator lengths
%ACTVEC=zeros(6,3);             %Actuator vector:from base joint to platform joint
TOACTp=zeros(6,3);             %Actuator torque vector:Torque applied to 
                               %platform per actuator
%BSWIVb=zeros(6,3);             %Base to swivel joint vector:from base frame
                               %to platform swivel joint


%COM_EUz=0;                     %Euler angles of COM and  
%COM_EUy=0;
%COM_EUx=0;
%EUz=0;
%EUy=0;
%EUx=0;
ACT_LEG_COM=zeros(6,1);
FORCEb=zeros(1,3);             %Net platform force vector:Force vector applied to platform
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

%************PROBLEM 1: Given the 6 degrees of     *********************
%************orientation determination of length of each actuator*****


[EUx,EUy,EUz,X,Y,Z]=ASK_ORIENT(); %Gives current orientation of the platform

[Pb,BSWIVb,ACTVEC]=ACTUATOR(EUx,EUy,EUz,X,Y,Z,BASBAS,PLAPLA,HGT);

%Now we have to calculate the magnitiude of lengths of each actuator
%We define function MAGNITUDE to 

for i=1:6
  [ACTLEN(i,1),ACTUNI(i,:)]=MAGNITUDE(ACTVEC(i,:));
end

for i=1:6
    
   fprintf('L',i);
   ACTLEN(i,1)
end



%**********PROBLEM 2:Given the actuator forces and *******************
%**********the 6degree of orientation,determine the ******************
%**********total force and torque on the platform*********************


[EUx,EUy,EUz,X,Y,Z]=ASK_ORIENT();
%Giving the values of the forces on each actuator
FMAG=zeros(6,1);
FMAG(:,1)=[0.5,0.5,0.5,0.5,0.5,0.5]';

%Now we have to determine the force in vector
%Use ACTVEC for this which is the vector describing the actuator 
%length and orientation

[Pb,BSWIVb,ACTVEC]=ACTUATOR(EUx,EUy,EUz,X,Y,Z,BASBAS,PLAPLA,HGT);

for i=1:6
     [ACTLEN(i,1),ACTUNI(i,:)]=MAGNITUDE(ACTVEC(i,:));
end

%Now we have to create the force vectors
%We are considering these force vectors in base coordinates

for i=1:3
    FOACTb(1,i)=FMAG(1,1)*ACTUNI(1,i);
    FOACTb(2,i)=FMAG(2,1)*ACTUNI(2,i);
    FOACTb(3,i)=FMAG(3,1)*ACTUNI(3,i);
    FOACTb(4,i)=FMAG(4,1)*ACTUNI(4,i);
    FOACTb(5,i)=FMAG(5,1)*ACTUNI(5,i);
    FOACTb(6,i)=FMAG(6,1)*ACTUNI(6,i);
    
end

%Now we transform these forces from base coordinates to platform 
%coordinates
for i=1:6
    FOACTp(i,:)=REVERSETRANS(EUx,EUy,EUz,FOACTb,i);
end

%Now force on platform will be sum of forces in each direction in each
%actuator - weight of the platform.

for i=1:3
   FORCEb(1,i)=0;    %Initialising force to zero
   TORQp(1,i)=0;     %Initialising torque to zero
end

for i=1:6
  FORCEb(1,1)=FORCEb(1,1)+FOACTb(i,1);   %X component of force  
  FORCEb(1,2)=FORCEb(1,2)+FOACTb(i,2);   %Y component of force
  FORCEb(1,3)=FORCEb(1,3)+FOACTb(i,3);   %Z component of force
  
end

%Now we have to determine the torque from wach actuator in platform 
%coordinates
for i=1:6
    TOACTp(i,:)=CROSS(PLAPLA(i,:),FOACTp(i,:));
end

for i=1:6
   TORQp(1,1)=TORQp(1,1)+TOACTp(i,1);     %X component of torque
   TORQp(1,2)=TORQp(1,2)+TOACTp(i,2);     %X component of torque
   TORQp(1,3)=TORQp(1,3)+TOACTp(i,3);     %X component of torque
end
FOACTb

%If inertia of the platform is known, the angular acceleration can 
%be found out as ANGACp=Sum of TORQp/Inertia


%***************PROBLEM 3: Given the 6 degree orientations**************
%**********and 6 degree rates, determine the actuator velocities***


[EUx,EUy,EUz,X,Y,Z]=ASK_ORIENT();
%Defining angular velocities in platform coordinates

ANGVLp=[0.1,0.1,0.1];      %In radians per second
PVLb=[0.05,0.05,0.05];     %In metres per second
WXRp=zeros(6,3);
WXRb=zeros(6,3);
PVLxyz=zeros(6,3);
for i=1:6
    WXRp(i,:)=CROSS(ANGVLp,PLAPLA(i,:));  %wxr in platform coordinates
    WXRb(i,:)=FORWARD_TRANS(EUx,EUy,EUz,WXRp,i); %wxr in base coordinate
  
    for j=1:3
      PVLxyz(i,j)=PVLb(1,j)+WXRb(i,j);   %BSib_dot=Vb+(Trans*(WXPpip)p)b
    end                                  %Platform joint velocities
end



[Pb,BSWIVb,ACTVEC]=ACTUATOR(EUx,EUy,EUz,X,Y,Z,BASBAS,PLAPLA,HGT);


for i=1:6
     [ACTLEN(i,1),ACTUNI(i,:)]=MAGNITUDE(ACTVEC(i,:));
end

%Taking dot product of ACTUNI and PVLxyz gives actuator velocity
for i=1:6
    ACTVEL(i,1)=0;
end
for i=1:6
    for j=1:3
        ACTVEL(i,1)=ACTVEL(i,1)+ACTUNI(i,j)*PVLxyz(i,j);
    end
end

for i=1:6
    ACTVEL(i,:)
end


%Programme to calculate servo angles required
s=0.3;                                         %Length of operating leg
%ACTLEN contains magnitude of the length of each actuator
a=0.05;                                        %Servo arm length

A=zeros(6,3);
%We have to find vector joining base origin to joint on the servo
for i=1:6
    A(i,:)=(ACTLEN(i,1)-s).*ACTUNI(i,:)+BASBAS(i,:);  %From vector algebra
end








   




