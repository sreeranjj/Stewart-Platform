%This program calculated platform rates when actuator rates are given. This
%is a problem of forward kinematics

%First position of platform is determined given the actuator heights

%We decide an initial guess for position of Stewart Platform
x=0.0;
y=0.0;
z=0.4;
phi=0;%.0872665;                    %5 degree
theta=0.087;%.0872665;                  %5 degree
psi=0;%.0872665;                    %5 degree


[BASBAS,PLAPLA,HGT]= CONFIG();               %This calls CONFIG which gives the base 
                                             %and platform configurations.
                                             
Ho=[0.05,0.05,0.05,0.05,0.05,0.05]';
%PLAPLA_T=PLAPLA';
%Pi=[PLAPLA_T;[1,1,1,1,1,1]]
%BASBAS_T=BASBAS';
%Bi=[BASBAS_T;[1,1,1,1,1,1]];

[EUx,EUy,EUz,X,Y,Z]=ASK_ORIENT(); %Gives current orientation of the platform

ACTLEN=zeros(6,1);             %Magnitudes of the actuator lengths
ACTUNI=zeros(6,3);           %Unit actuator vector

%q=[x,y,z,phi,theta,psi]';
MSE=2;                                        %Random initial error
flag=0;
while(flag==0)
    
%R=[cos(phi)*cos(theta) ,cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),x;
 %   sin(phi)*cos(theta),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) , y;
  %  -sin(theta)        ,             cos(theta)*sin(psi)              ,           cos(theta)*cos(psi)                 ,z;
   % 0                  ,                     0                        ,                    0                          ,1]
%R*Pi
%S=R*Pi-Bi ;                                  %Vector connecting base joint to platform joint


[Pb,BSWIVb,ACTVEC]=ACTUATOR(EUx,EUy,EUz,X,Y,Z,BASBAS,PLAPLA,HGT);

%Now we have to calculate the magnitiude of lengths of each actuator
%We define function MAGNITUDE to 
ACTVEC
for i=1:6
  [ACTLEN(i,1),ACTUNI(i,:)]=MAGNITUDE(ACTVEC(i,:));
end

for i=1:6
    
   fprintf('L',i);
   ACTLEN(i,1)
end


Hi=zeros(6,1);
L=0.8;                                        %Link length
%for i=1:6
 %   Hi(i,1)=S(3,i)-sqrt(L*L-(S(1,i)^2+S(2,i)^2));
%
%end
for i=1:6
    Hi(i,1)=ACTVEC(3,i)-sqrt(L*L-(ACTVEC(1,i)^2+ACTVEC(2,i)^2));
end
Hi
deltah=zeros(6,1);

for i=1:6
    delta_h(i,1)=Hi(i,1)-Ho(i,1);
end
   D = abs(Hi-Ho).^2;
   MSE = sum(D(:))/numel(Ho);            %Calculating error
   
   if(MSE<0.0001)
       flag=0;
   else
      DGbyDs=zeros(4,6);
        for i=1:6
          k=1/(sqrt(L*L-(S(1,i)^2+S(2,i)^2)));
          DGbyDs(:,i)=[0,0,1,1]'+[k*S(1,i),k*S(2,i),0,0]';
        end
   
   
     Rx=[0,0,0,1;
         0,0,0,0;
         0,0,0,0;
         0,0,0,0];
     Ry=[0,0,0,0;
         0,0,0,1;
         0,0,0,0;
         0,0,0,0];
     Rz=[0,0,0,0;
         0,0,0,0;
         0,0,0,1;
         0,0,0,0];

     Rphi=[-cos(theta)*sin(phi),-cos(phi)*cos(psi)-sin(psi)*sin(theta)*sin(phi),cos(phi)*sin(psi)-cos(psi)*sin(theta)*sin(phi),0;
           cos(theta)*cos(phi), cos(phi)*sin(psi)*sin(theta)+cos(psi)*sin(theta),cos(psi)*cos(phi)*sin(theta)+sin(psi)*sin(phi),0;
           0,0,0,0;
           0,0,0,0];
     Rtheta=[cos(phi)*sin(theta),-cos(theta)*cos(phi)*sin(psi),cos(theta)*cos(phi)*cos(psi),0;
            -sin(theta)*cos(psi),cos(theta)*sin(psi)*sin(phi),cos(psi)*cos(theta)*sin(phi),0;
            -cos(theta),-sin(psi)*sin(theta),-cos(psi)*sin(theta),0;
             0,0,0,0];
     Rpsi=[0,cos(psi)*cos(phi)*sin(theta)+sin(psi)*sin(phi),cos(phi)*sin(psi)-cos(psi)*sin(theta)*sin(phi),0;
           0,-cos(phi)*sin(psi)+cos(psi)*sin(theta)*sin(phi),-cos(phi)*cos(psi)-sin(psi)*sin(theta)*sin(phi),0;
           0,cos(psi)*cos(theta),-sin(psi)*cos(theta),0;
           0,0,0,0];
  
    % R_q=Rx+Ry+Rz+Rphi+Rtheta+Rpsi
 
     %G_s=DGbyDs'

     %G_q=G_s*R_q*Pi;
     %deltaq=G_q\delta_h;
     %[EUx,EUy,EUz,X,Y,Z]=[EUx,EUy,EUz,X,Y,Z]+deltaq;
     %q=q+deltaq;
     %flag=1;
   end
  
end
q
    
