%Cartesian space variables
x=0 ; y=0 ; z=0.4 ;
dotx = 0; doty = 0; dotz = 0;
ddotx = 0; ddoty = 0; ddotz = 0;
psi=0 ; the=0 ; phi=0 ;
dotpsi=0 ; dotthe=0 ; dotphi=0 ;
ddotpsi=0 ; ddotthe=0 ; ddotphi=0 ;
force=zeros(6,31);
%for jj =0:0.01:0.3 
 for jj =0:0.01:0.3 
    
% acceleration i n y direction
%ddoty = 9.8 ;
%doty = jj*9.8 ;
%y = 0.5*jj~2*9.8 ;

% acceleration in x direction
%ddotx = 9.8 ;
%dotx = jj*9.8 ;
%x = 0.5* j j~2*9.8 ;

www=5;
aaa=0.1;
%x=aaa*sin(jj*www) ;
%dotx = aaa*www*cos(jj*www);
%ddotx=-aaa*www*www*sin(jj*www);

% sinusoidal the angle
the=aaa*sin(jj*www);
dotthe= aaa*www*cos(jj*www);
ddotthe=-aaa*www*www*sin(jj*www);
%switch=0 ;
    
 %Call the Dynamics function without considering leg dynamics
F = DYNAMICS(ddotx,ddoty,ddotz,ddotpsi,ddotthe,ddotphi,dotx,doty,dotz,dotpsi,dotthe,dotphi,x,y,z,psi,the,phi) ;

jj
force(1,jj*100+1 ) = F(1) ;
force(2,jj*100+1 ) = F(2) ;
force(3,jj*100+1 ) = F(3) ;
force(4,jj*100+1 ) = F(4) ;
force(5,jj*100+1 ) = F(5) ;
force(6,jj*100+1 ) = F(6) ;




%switch=l ;
    
% Call the Dynamics function with considering leg dynamics
%F = Dynamics(ddotx,ddoty,ddotz,ddotpsi,ddotthe,ddotphi,dotx,doty,dotz,dotpsi,dotthe,dotphi,x,y,z,psi,the,phi) ;

%forceleg(1,jj*100+l ) = F(l) ;
%forceleg(2,jj*100+l ) = F(2) ;
%forceleg(3,jj*100+l ) = F(3) ;
%forceleg(4,jj*100+l ) = F(4) ;
%forceleg(5,jj*100+l ) = F(5) ;
%forceleg(6,jj*100+l ) = F(6) ;
end


% Plot the required actuator forces for both cases

force(1,:)

hold on 
plot(0:0.01:0.30 , force(1,:),'yo ' ) 
plot(0:0.01:0.30 , force(2,:),'w— ' )
plot(0:0.01:0.30 , force(3,:),'gx ' ) 
plot(0:0.01:0.30 , force(4,:),'r: ' ) 
plot(0:0.01:0.30 , force(5,:),'c-') 
plot(0:0.01:0.30 , force(6,:),'m-. ' ) 
%plot(0:0.01:0.3 , forceleg(l,:),'yo ' ) ;
%plot(0:0.01:0.3 , forceleg(2,:),'w— ' ) ;
%plot(0:0.01:0.3 , forceleg(3,:),'gx ' ) ;
%plot(0:0.01:0.3 , forceleg(4,:),'r: ' ) ;
%plot(0:0.01:0.3 , forceleg(5,:) , 'c-') ;
%plot(0:0.01:0.3 , forceleg(6,:),'m-. ' ) ;
%legend('Actuator A' , 'Actuator B','Actuator C','Actuator D','Actuator E','Actuator F' ,-1 )

