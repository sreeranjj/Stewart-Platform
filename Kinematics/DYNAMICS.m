function [F] = DYNAMICS(ddotx,ddoty,ddotz,ddotpsi,ddotthe,ddotphi,dotx,doty,dotz,dotpsi,dotthe,dotphi,x,y,z,psi,the,phi)


% System parameters
gammapl = 53.45 ;
gammap = [gammapl,120-gammapl, 120+gammapl, -120-gammapl,-120+gammapl,-gammapl];
gammabl = 7.75 ;
gammab = [gammabl,120-gammabl, 120+gammabl, -120-gammabl,-120+gammabl,-gammabl];
alphap = [0 , 120, 120, -120 , -120 , 0] ;
betab = [60,60,180,180,-60,-60];



%rp = 0.668;
%rb = 1.133;
%hp = 0.203;
rp = 0.15;
rb = 0.20;
hp = 0.02;
g=9.8 ; % g acceleration
% switch =1 or =0 for with or without considering legs ;
%ml = mleg*0.2 ; % mass of lower part of leg
%mu = mleg*0.8 ; % mass of upper part of leg
%lu = 0.4 ; % distance between upper center and connection
%11 = 0.8 ; % distance between lower center and connection

%pIp=zeros(3,3);

% assume the load to be 250 kg
mplatform = 2.0 ;
%pIp = mplatform *[((0.6^2)/4) + (1.0^2)/12, 0, 0; 0, ((0.6*2)/4)+((1.0^2)/12), 0; 0, 0, (0.6^2)/2] ;
pIp = mplatform *[((0.15^2)/4) , 0, 0; 0, ((0.15*2)/4), 0; 0, 0, (0.15^2)/2] ;
%velocity


bVp = [dotx ; doty ; dotz] ;
%acceleration
bACCp = [ddotx ; ddoty ; ddotz];


spsi = sin(psi*pi/180);
cpsi = cos(psi*pi/180);
sthe = sin(the*pi/180);
cthe = cos(the*pi/180);
sphi = sin(phi*pi/180);
cphi = cos(phi*pi/180);

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

bdp = [x,y,z ]' ;                        %Position of platform frame wrt base
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
bWp = B * [dotpsi ; dotthe ; dotphi] ;
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
balphap = dotB * [dotpsi ; dotthe ; dotphi ] + B * [ddotpsi ; ddotthe ; ddotphi] ;

%Leg Dynamics:
%for i=l:6 
% length of legs

%lleg = norm(bA(:,i)) ;
% Calculations of variables as shown in dynamic equations
%dotbA(:,i ) = cross ( bWp, bRp*pp(:,i ) ) + bVp;
%ddotbA(:,i ) = cross(balphap , bRp*pp(:,i) ) +cross(bWp , cross(bWp,bRp*pp(:,i)) ) + bACCp;
                
               

%dotlleg = (bA(:,i)'*dotbA(:,i))/lleg ;

%Ili(i) = ml*( lleg-2 + (lleg-2*ll)*lle g + (lleg-2*ll) ' 2 )/3;

%Ili(i ) = ml*( 3*lleg*2 + (2*11)*2 - 3*(2*ll)*lleg )/3;
%dotlli(i ) = ml * (2*lleg-(2*ll)) * dotlleg;
%bWi(:,i) = cross( bA(:,i), dotbA(:,i) ) / (norm(bA(:,i))*norm(bA(:,i)));

%ti = cross( bA(:,i), [eye(3), cross(eye(3), bRp*pp(:,i)) ] )/ (norm(bA(:,i))*norm(bA(:,i)));
%balphai =( ( bA(:,i)'*bA(:,i) )*( cross(bA(:,i), ddotbA(:,i)) )- 2*( bA(:,i)'*dotbA(:,i) )*( cross(bA(:,i), dotbA(:,i)) ) )/ norm(bA(:,i))*4;



%bTAUi = (Ili(i ) + Iui) * balphai + dotlli(i ) * bWi(:,i);
%titimesbTAUi(:,i) = ti'*bTAUi;
%hil = (11/norm(bA(:,i))^3)*bA(:,i)*bA(:,i)'*[eye(3), cross(eye(3), ...
 %      bRp*pp(:,i))] + ( (norm(bA(:,i)) - ll)/norm(bA(:,i)) )* ...
  %     [eye(3), cross(eye(3), bRp*pp(:,i)) ];
   
%hiu = (-lu/norm(bA(:,i))^3)*bA(:,i)*bA(:,i)'*[eye(3), cross(eye(3) , ...
 %      bRp*pp(:,i)) ] + ( (lu)/norm(bA(:,i)) )* ...
  %     [eye(3), cross(eye(3), bRp*pp(:,i)) ];
%hmg(:,i) = hil'*ml*[0, 0, -g]' + hiu'*mu*[0, 0, -g]';
%end;

bIp = bRp * pIp * bRp';
bFp = mplatform * bACCp;
bFp = bFp - mplatform * [0, 0, -g]';
bTAUp = bIp * balphap + cross(bWp, (bIp*bWp));
% Platform dynamics
Jt=zeros(6,6);
for i=1:6,
Jt(:,i ) = [ ( bRp*pp(:,i) + bdp - bb(:,i) ); cross( bRp*pp(:,i),bdp-bb(:,i) ) ] / norm(bA(:,i)) ;

end


% The required actuator forces for certain trajectory
%F = Jt\([bFp; bTAUp]+titimesbTAUi*[1;1;1;1;1;1] - hmg*[1;i;1;1;1;1]);
F = Jt\([bFp; bTAUp])


end