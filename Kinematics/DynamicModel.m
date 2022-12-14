clc;
clear all;

X=zeros(6,31);                %Initial states
Xdot=zeros(6,31);             %Velocity
Xddot=zeros(6,31);            %Acceleration
F=zeros(6,31);
X(1,1)=0.01;
X(2,1)=0.01;
X(3,1)=0.01;
X(4,1)=0.65;
X(5,1)=0.77;
X(6,1)=0.55;
Xdot(1,1)=0.001;
Xdot(2,1)=0.001;
Xdot(3,1)=0.001;
Xdot(4,1)=0.023;
Xdot(5,1)=0.043;
Xdot(6,1)=0.034;

%Force on each leg
%bRp=zeros(3,3);
for jj =0:0.01:0.3 
    
    www=50;
    AA1=3.1;
    AA2=3.05;
    AA3=3.0;
    AA4=-2.95;
    AA5=-2.9;
    AA6=-2.8;
    
    a=fix(jj*100+1);
    b=fix(jj*100+2);
    
    F(1,a)=AA1*sin(jj*www);
    F(2,a)=AA2*sin(jj*www);
    F(3,a)=AA3*sin(jj*www);
    F(4,a)=AA4*sin(jj*www);
    F(5,a)=AA5*sin(jj*www);
    F(6,a)=AA6*sin(jj*www);
    
   
% System parameters
gammapl = 53.45 ;
gammap = [gammapl,120-gammapl, 120+gammapl, -120-gammapl,-120+gammapl,-gammapl];
gammabl = 7.75 ;
gammab = [gammabl,120-gammabl, 120+gammabl, -120-gammabl,-120+gammabl,-gammabl];
alphap = [0 , 120, 120, -120 , -120 , 0] ;
betab = [60,60,180,180,-60,-60];

vert=0.2;                 %Distance between center of platform and base

%rp = 0.668;
%rb = 1.133;
%hp = 0.203;
rp = 0.15;
rb = 0.20;
hp = 0.02;
g=9.8 ; % g acceleration

    

mplatform = 2.0 ;
%pIp = mplatform *[((0.6^2)/4) + (1.0^2)/12, 0, 0; 0, ((0.6*2)/4)+((1.0^2)/12), 0; 0, 0, (0.6^2)/2] ;
pIp = mplatform *[((0.15^2)/4) , 0, 0; 0, ((0.15*2)/4), 0; 0, 0, (0.15^2)/2] ;
%velocity
%bVp = [dotx ; doty ; dotz] ;
bVp= [Xdot(1,a),Xdot(2,a),Xdot(3,a)];
%acceleration
%bACCp = [ddotx ; ddoty ; ddotz];
bACCp=[Xddot(1,a),Xddot(2,a),Xddot(3,a)]';

euler=zeros(3,1);



psi=X(4,a);
the=X(5,a);
phi=X(6,a);



%spsi = sin(psi*pi/180);
%cpsi = cos(psi*pi/180);
%sthe = sin(the*pi/180);
%cthe = cos(the*pi/180);
%sphi = sin(phi*pi/180);
%cphi = cos(phi*pi/180);

spsi = sin(psi);
cpsi = cos(psi);
sthe = sin(the);
cthe = cos(the);
sphi = sin(phi);
cphi = cos(phi);
 tpsi= tan(psi);
 tphi= tan(phi);
 tthe= tan(the);

 R2(1,1)=1;
    R2(1,2)=tthe*spsi;                                      %R2 is the transformation matrix for transforming Body rates to Euler rates
    R2(1,3)=tthe*cpsi;
    R2(2,1)=0;
    R2(2,2)=cpsi;
    R2(2,3)=-spsi;
    R2(3,1)=0;
    R2(3,2)=spsi/cthe;
    R2(3,3)=cpsi/cthe;
    
    
bRp(1,1 ) = cphi * cthe ;
bRp(1,2 ) = -sphi*cpsi + cphi * sthe * spsi ;
bRp(1,3 ) = sphi * spsi + cphi * sthe * cpsi ;
bRp(2,1 ) = sphi * cthe ;
bRp(2,2 ) = cphi * cpsi + sphi * sthe * spsi ;
bRp(2,3 ) = -cphi * spsi + sphi * sthe * cpsi ;
bRp(3,1 ) = -sthe ;
bRp(3,2 ) = cthe * spsi ;
bRp(3,3 ) = cthe * cpsi ;

pp(1,: ) = rp*cos(gammap*pi/180);
pp(2,: ) = rp*sin(gammap*pi/180) ;
pp(3,: ) = [hp/2,hp/2,hp/2,hp/2,hp/2,hp/2 ] ;        %hp is the thickness of the platform

bb(1,: ) = rb*cos(gammab*pi/180);
bb(2,: ) = rb*sin(gammab*pi/180);
bb(3,: ) = [0 , 0, 0, 0 , 0, 0] ;

bdp = [X(1,a),X(2,a),X(3,a)+vert]' ;                        %Position of platform frame wrt base
bDp = [bdp,bdp,bdp,bdp,bdp,bdp] ;        %This is same for all 6 actuators
bA = bRp*pp + bDp - bb ;                %Actuator vector

% inertia
%Iui = mu*(2*lu)^2/3 ;
%B is the transformation matrix for transforming Euler rates to angular
%velocity
B(1,1 ) = cthe * cphi;
B(1,2 ) = -sphi ;
B(1,3 ) = 0;
B(2,1 ) = cthe * sphi;
B(2,2 ) = cphi ;
B(2,3 ) = 0;
B(3,1 ) = -sthe ;
B(3,2 ) = 0;
B(3,3 ) = 1 ;


%angular velocity of platform
%bWp = B * [dotpsi ; dotthe ; dotphi] ;
eulerrate=R2*[Xdot(4,a) ; Xdot(5,a) ; Xdot(6,a)];
bWp = B * [eulerrate(1,1);eulerrate(2,1);eulerrate(3,1)] ;
dotpsi=eulerrate(1,1);
dotthe=eulerrate(2,1);
dotphi= eulerrate(3,1);

%dotB is used for transforming Euler rates to angular acceleration
dotB(1,1 ) = -sthe*cphi*dotthe - cthe*sphi*dotphi ;
dotB(1,2 ) = -cphi*dotphi ;
dotB(1,3 ) = 0;
dotB(2,1 ) = -sthe*sphi*dotthe + cthe*cphi*dotphi;
dotB(2,2 ) = -sphi*dotphi ;
dotB(2,3 ) = 0;
dotB(3,1 ) = -cthe*dotthe ;
dotB(3,2 ) = 0;
dotB(3,3 ) = 0;
%angular acceleration
%balphap = dotB * [dotpsi ; dotthe ; dotphi ] + B * [ddotpsi ; ddotthe ; ddotphi] ;
balphap=dotB*[Xdot(4,a) ; Xdot(5,a) ; Xdot(6,a)]+ B*[Xddot(4,a);Xddot(5,a);Xddot(6,a)];

%bIp = zeros(3,3);
bIp=bRp*pIp*bRp';

bRp;
bFp = mplatform * bACCp;
bFp = bFp - mplatform * [0, 0, -g]';
bTAUp = bIp * balphap + cross(bWp,(bIp*bWp));

Fvec=zeros(3,6);

 for i=1:6
       Fvec(:,i)=F(i,a)*bA(:,i);
 end
   
    U=zeros(6,1);                                              %Combined effect of gravity and force on actuators
     for i=1:6
       U(i,1)= F(i,a)+dot(Fvec(:,i),G);
     end
     
     
% Platform dynamics
Jt=zeros(6,6);

  for i=1:6
   Jt(:,i ) = [ ( bRp*pp(:,i) + bdp - bb(:,i) ); cross( bRp*pp(:,i),bdp-bb(:,i) ) ] / norm(bA(:,i)) ;
  
  end
Jt;

 Uin=Jt*U;                                                %Input u in state space form
     
K=Jt*F(:,a);
F;
%Now using state space form we can calculate Xdot and Xddot
%Constructing M and H
m=mplatform*(eye(3));

z=zeros(3,3);

M=[m,z;z,bIp];
    M;
bWpskew=[ 0 , -1*bWp(3,1) , bWp(2,1) ; 
         bWp(3,1) , 0 , -1*bWp(1,1)  ;
         -1*bWp(2,1), bWp(1,1), 0 ];
  inv(M);   
h= bWpskew*bIp;                             %WxI
H=[z,z;z,h];
%State space form
% X1=X......X1dot=X2.........X2dot=Xddot

 
A=[zeros(6,6),eye(6);zeros(6,6),-1*(M\H)];

C=[zeros(6,6);inv(M)];

%Xmatdot=zeros(12,1);
Xmat=[X(:,a);Xdot(:,a)];
C*K;
Xmatdot=A*Xmat+C*K;

for i=1:6
    Xdot(i,b)=Xmatdot(i,1);
end

for i=1:6
    Xddot(i,b)=Xmatdot(i+6,1);
end



Xddot(:,a);
dt=0.01;
for i=1:6
 X(i,b)=Xdot(i,b)*b+X(i,1);
end

end
X
%plot(0:0.01:0.30 ,X(1,:),'yo ')


 
    

