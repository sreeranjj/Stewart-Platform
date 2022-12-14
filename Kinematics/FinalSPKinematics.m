function [length,bA,alpa] = FinalSPKinematics(x,y,z,psi,the,phi)
%Define the system parameters

%gammap1 = 53.45 ; % unit in degree
gammap1=8;
gammap = [gammap1,120-gammap1, 120+gammap1, -120-gammap1,-120+gammap1,-gammap1];
gammab1 = 32.63; % unit in degree %onsidering servo arm length of 4.5cm
%gammab1 = 21.1495 ; % unit in degree
gammab = [gammab1,120-gammab1, 120+gammab1, -120-gammab1,-120+gammab1,-gammab1];
beta=12.63*pi/180;
rp = 0.10; % radius of platform ,in meter
rb = 0.20;  % radius of base,in meter
hp = 0.0;   % platform nominal height ,in meter


length=zeros(6,1);

spsi = sin(psi*pi/180 );
cpsi = cos(psi*pi/180 );
sthe = sin(the*pi/180 );
cthe = cos(the*pi/180 );
sphi = sin(phi*pi/180 );
cphi = cos(phi*pi/180 );
% rotation matrix R
bRp(1,1 ) = cphi * cthe ;
bRp(1,2 ) = -sphi * cpsi + cphi * sthe * spsi ;
bRp(1,3 ) = sphi * spsi + cphi * sthe * cpsi ;
bRp(2,1 ) = sphi * cthe ;
bRp(2,2 ) = cphi * cpsi + sphi * sthe * spsi ;
bRp(2,3 ) = -cphi * spsi + sphi * sthe * cpsi ;
bRp(3,1 ) = -sthe ;
bRp(3,2 ) = cthe * spsi ;
bRp(3,3 ) = cthe * cpsi ;
% fixed vector of pp
pp(1,: ) = rp*cos(gammap*pi/180);
pp(2,: ) = rp*sin(gammap*pi/180) ;
pp(3,: ) = [hp/2,hp/2,hp/2,hp/2,hp/2,hp/2 ] ;
pp;
%fixed vector of bb
bb(1,: ) = rb*cos(gammab*pi/180);
bb(2,: ) = rb*sin(gammab*pi/180) ;
bb(3,: ) = [0,0,0,0,0,0] ;
% bdp
bb
bdp = [x,y,z ]' ;
bDp = [bdp,bdp,bdp,bdp,bdp,bdp] ;
% actuator vector
qq=bRp*pp+bDp;
bA = bRp*pp +bDp - bb ;
qq
bA;
for i =1:6 
length(i,1) = norm(bA(:,i)) ;
end
L=zeros(6,1);
M=zeros(6,1);
N=zeros(6,1);
alpa=zeros(6,1);
for i=1:6
    
    L(i)= (length(i,1))^2 -0.100464;    % L=l^2-(s^2-a^2)
    M(i)=0.088*(qq(3,i)-bb(3,i));
    N(i)=0.088*(cos(beta)*(qq(1,i)-bb(1,i))+sin(beta)*(qq(2,i)-bb(2,i)));
    
    k=sqrt(L(i)^2+M(i)^2);
    alpa(i)=asin(L(i)/k)-atan(N(i)/M(i));
end


%Now we have toJfind deltal to get the servo angle rotation
%Servo only moves in the 