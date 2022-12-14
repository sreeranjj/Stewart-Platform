% u = verificationData.u;
 %   time = verificationData.Time;
 %   omega_exp =  verificationData.omega;
  %  y_dot = @( t, y ) [b*y(2) - a*y(1) ; 0];
   % Y0 = [ omega_exp(1) ; u(1) ];
    %t_i = [time(1) time(2)];
    %omega_sim(1) = Y0(1);
    %for i = 1:length(time)-1
     %   [t_bla,y_bla] = ode15s( y_dot, t_i,Y0 );
      %  if i< length(time)-1
       %     Y0 = [ y_bla(end,1); u(i+1) ];
         %   t_i = [time(i+1), time(i+2)];
        %end
        %omega_sim(i+1) = y_bla(end,1);
    %end
   %omega_sim = omega_sim';
   
   
  % Y_dot=@(t,Y) [Minv(1)*y(2)-Minv(1)*H(1)*y(1);0];
   
  % Yo=[Xdot(1);Uin(1)];
   
  % t_i=[time(1) time(2)];
   %Xdot_sim(1)=Yo(1);
   
   
   
   for i=0:0.01:0.3
       
      
    a=fix(i*100+1);
    b=fix(i*100+2);
       
       
   
    
     www=10;
    AA1=3.1;
    AA2=3.05;
    AA3=3.0;
    AA4=-2.95;
    AA5=-2.9;
    AA6=-2.8;
    
  
    
    F(1,a)=AA1*sin(i*www);
    F(2,a)=AA2*sin(i*www);
    F(3,a)=AA3*sin(i*www);
    F(4,a)=AA4*sin(i*www);
    F(5,a)=AA5*sin(i*www);
    F(6,a)=AA6*sin(i*www);
    
    
    
   
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
    for j=1:6
       Fvec(:,j)=F(j,a)*bA(:,j);
    end
    
    U=zeros(6,1);                                              %Combined effect of gravity and force on actuators
     for j=1:6
       U(j,1)= F(j,a)+dot(Fvec(:,j),G);
     end
     
    Jt=zeros(6,6);
   
    
  
     bWp = [Xdot(4,a) ; Xdot(5,a) ; Xdot(6,a)] ;
    dotpsi=Xdot(4,a);
    dotthe=Xdot(5,a);
    dotphi= Xdot(6,a);
    
   

    
      for i=1:6
      Jt(:,i ) = [ ( bRp*pp(:,i) + bdp - bb(:,i) ); cross( bRp*pp(:,i),bdp-bb(:,i) ) ] / norm(bA(:,i)) ;
  
      end
    
     Uin=Jt*U;                                                %Input u in state space form
     
    
    
   
   Yo=[Xdot(:,a);Uin(:,a)];
   Y_dot=@(t,Y) [Minv*Y(2)-Minv*H*Y(1);0];
       [t,Y]=ode45(Y_dot,[i i+0.01],Yo);
       
       
        %if i<0.3
         % Yo=[y(end,1);Uin(b)];
          %t_i=[time(i+),time(i+2)];
        %end
     
      Xdot(:,b)=Y(end,1);
   
      
     R2(1,1)=1;
    R2(1,2)=tthe*spsi;                                      %R2 is the transformation matrix for transforming Body rates to Euler rates
    R2(1,3)=tthe*cpsi;
    R2(2,1)=0;
    R2(2,2)=cpsi;
    R2(2,3)=-spsi;
    R2(3,1)=0;
    R2(3,2)=spsi/cthe;
    R2(3,3)=cpsi/cthe;
    
    Eulerdot=R2*[Xdot(b,4);Xdot(b,5);Xdot(b,6)];
   
    for j=1:3
        X(b,j)=X(a,j)+i*Xdot_sim(b,j);
    end
    
    for j=1:3
        X(b,j+3)=X(a,j+3)+Eulerdot(j);
    end
     
   end
   