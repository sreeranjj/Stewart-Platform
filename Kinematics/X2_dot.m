function Xdot=X2_dot(~,X)


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
     pIp = mplatform *[((rp^2)/4) , 0, 0; 0, ((rp*2)/4), 0; 0, 0, (rp^2)/2] ;

    psi=X(4,a);
    the=X(5,a);
    phi=X(6,a);
    
    x=X(1,a);
    y=X(2,a);
    z=X(3,a);
    
    spsi = sin(psi);
    cpsi = cos(psi);
    sthe = sin(the);
    cthe = cos(the);
    sphi = sin(phi);
    cphi = cos(phi);
    tpsi= tan(psi);
    tphi= tan(phi);
    tthe= tan(the);
    
    
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
    bDp = [bdp,bdp,bdp,bdp,bdp,bdp] ;                           %This is same for all 6 actuators
    bA = bRp*pp + bDp - bb ;                                    %Actuator vector
    
    
    
    bIp=bRp*pIp*bRp';                                           %Inertia tensor varying due to orientation of platform wrt base
    
    
    G=[0,0,-g]';
    Fvec=zeros(3,6);
    %We are going to add g in vector form to the input force
    for i=1:6
       Fvec(:,i)=F(i,a)*bA(:,i);
    end
    
    U=zeros(6,1);                                              %Combined effect of gravity and force on actuators
     for i=1:6
       U(i,1)= F(i,a)+dot(Fvec(:,i),G);
     end
     
    Jt=zeros(6,6);
    
    R2(1,1)=1;
    R2(1,2)=tthe*spsi;                                      %R2 is the transformation matrix for transforming Body rates to Euler rates
    R2(1,3)=tthe*cpsi;
    R2(2,1)=0;
    R2(2,2)=cpsi;
    R2(2,3)=-spsi;
    R2(3,1)=0;
    R2(3,2)=spsi/cthe;
    R2(3,3)=cpsi/cthe;
    
    
    
   % bWp = B * [Xdot(4,a) ; Xdot(5,a) ; Xdot(6,a)] ;
     bWp = [Xdot(4,a) ; Xdot(5,a) ; Xdot(6,a)] ;
    dotpsi=Xdot(4,a);
    dotthe=Xdot(5,a);
    dotphi= Xdot(6,a);
    
    %dotB is used for transforming Euler rates to angular acceleration
   %dotB(1,1 ) = -sthe*cphi*dotthe - cthe*sphi*dotphi ;
   %dotB(1,2 ) = -cphi*dotphi ;
    %dotB(1,3 ) = 0;
   % dotB(2,1 ) = -sthe*sphi*dotthe + cthe*cphi*dotphi;
   % dotB(2,2 ) = -sphi*dotphi ;
   % dotB(2,3 ) = 0;
   % dotB(3,1 ) = -cthe*dotthe ;
   % dotB(3,2 ) = 0;
    %dotB(3,3 ) = 0;
    %angular acceleration
    %balphap = dotB * [dotpsi ; dotthe ; dotphi ] + B * [ddotpsi ; ddotthe ; ddotphi] ;
    %balphap=dotB*[Xdot(4,a) ; Xdot(5,a) ; Xdot(6,a)]+ B*[Xddot(4,a);Xddot(5,a);Xddot(6,a)];

    
      for i=1:6
      Jt(:,i ) = [ ( bRp*pp(:,i) + bdp - bb(:,i) ); cross( bRp*pp(:,i),bdp-bb(:,i) ) ] / norm(bA(:,i)) ;
  
      end
    
     Uin=Jt*U;                                                %Input u in state space form
     
     z=zeros(3,3);
     m=mplatform*eye(3);
    
     M=[ m , z ; z , bIp];                                    %6X6 M matrix in state space form
     
     Minv=inv(M);
     
     bWpskew=[ 0 , -1*bWp(3,1) , bWp(2,1) ; 
               bWp(3,1) , 0 , -1*bWp(1,1)  ;
               -1*bWp(2,1), bWp(1,1), 0 ];
           
           
           
    h=bWpskew*bIp;
    H=[z,z;z,h];

     Xdot = Minv*Uin-Minv*H*X; % Evaluate ODE at time t

  end