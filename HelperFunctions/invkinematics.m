function [length,bA] = invkinematics(x,y,z,psi,the,phi )
%Define the system parameters

gammapl = 53.45 ; % unit in degree
gammap = [gammapl,120-gammapl, 120+gammapl, -120-gammapl,-120+gammapl,-gammapl];
gammabl = 7.75 ; % unit in degree
%gammabl = 21.1495 ; % unit in degree
gammab = [gammabl,120-gammabl, 120+gammabl, -120-gammabl,-120+gammabl,-gammabl];

rp = 0.15; % radius of platform ,in meter
rb = 0.20;  % radius of base,in meter
hp = 0.02;   % platform nominal height ,in meter


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
bA = bRp*pp +bDp - bb ;
bA;
for i =1:6 
length(i,1) = norm(bA(:,i)) ;
end