%This function returen the servoangles for any specific orientation and 
%position
function [length,bA,servoang] = servoangles(x,y,z,psi,the,phi)
%Define the system parameters

gammap1=8;
gammap = [gammap1,120-gammap1, 120+gammap1, -120-gammap1,-120+gammap1,-gammap1];
gammab1 = 20; %unit in degree %onsidering servo arm length of 4.5cm
gammab = [gammab1,120-gammab1, 120+gammab1, -120-gammab1,-120+gammab1,-gammab1];
rp = 0.10; % radius of platform ,in meter
rb = 0.20;  % radius of base,in meter
hp = 0.01;   % platform nominal height ,in meter

s=0.045;   %Servo arm length in cm
len=0.3202;  %Leg length in cm
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
bRp;
pp(1,: ) = rp*cos(gammap*pi/180);
pp(2,: ) = rp*sin(gammap*pi/180) ;
pp(3,: ) = [hp/2,hp/2,hp/2,hp/2,hp/2,hp/2 ] ;
pp;
%fixed vector of bb
bb(1,: ) = rb*cos(gammab*pi/180);
bb(2,: ) = rb*sin(gammab*pi/180) ;
bb(3,: ) = [0,0,0,0,0,0];
% bdp
bb;
bdp = [x,y,z ]' ;
bDp = [bdp,bdp,bdp,bdp,bdp,bdp] ;

bDp;
    
% actuator vector
qq=bRp*pp+bDp;
qq;
bA = bRp*pp +bDp - bb ;
bA;
for i=1:6
   L(:,i)= rotationmat(gammab(1,i))*bA(:,i)-[rb,0,0]'; %Origin is shifted each time to the servo location
  rotationmat(gammab(1,i)) ;
end
% The offset comes as we haven't defined the home position
for i =1:6 
length(i,1) = norm(bA(:,i)) ;
end
servoangt=zeros(6,2);
offset1=[10.2575,18.9046,10.2575,18.9046,10.2575,18.9046]';
offset2=[-180,-180,-180,-180,-180,-180]';
offset=[-8.1635,8.1635,-8.1635,8.1635,-8.1635,8.1635]';
for i=1:6
    a=2*s*L(2,i);
    b=2*s*L(3,i);
    c=len^2-(length(i,1))^2;
  
      
     servoangt(i,1)=(acos(-c/(sqrt(a^2+b^2)))+atan2(b,a))*180/pi+offset2(i,1)-10.2575;
 
    servoangt(i,2)=(-acos(-c/(sqrt(a^2+b^2)))+atan2(b,a))*(180/pi)+offset1(i,1);
end

servoang=zeros(6,1);
for i=1:6
    if(mod(i,2)==0)
        
    servoang(i,1)=servoangt(i,1)+offset(i,1);
    
    else
      servoang(i,1)=servoangt(i,2)+ offset(i,1);
    end
    
end


