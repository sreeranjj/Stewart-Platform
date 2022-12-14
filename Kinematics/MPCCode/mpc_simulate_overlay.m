%%%%%%%%%%%%%%                         (NO T-filter!!)
%%%%%%%%%%%%%%  Simulates MIMO GPC with constraint handling - no
%%%%%%%%%%%%%%  advance knowledge of target assumed
%%%%   OVERLAYS THE UNCONSTRAINED CONTROL CHOICE
%%%
%%%%%  [y,u,Du,r] = mpc_simulate_overlay(B,A,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise)
%              y, u, Du, r are dimensionally compatible 
% closed-loop outputs/inputs/input increments and supplied set-point and disturbance
%
% MFD model     Ay(k) = Bu(k-1) + dist
%
% ny is output horizon
% nu is the input horizon
% Wu is the diagonal control weighting 
% Wy is the diagonal output weighting
% sizey no. outputs and inputs (assumed square)
% dist,noise are the disturbance and noise signals
% ref is the reference signal
% Dumax is a vector of limits on input increments (assumed symetric)
% umax, umin are vectors of limits on the inputs
%
% [y,u,Du,r,d] = mpc_simulate_overlay(B,A,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise)
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function [y,u,Du,r] = mpc_simulate_overlay(B,A,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Error checks
sizey = size(A,1);
if size(B,2)==sizey;B=[B,zeros(sizey,sizey)];end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%   Find prediction matrices 
%%%%    yfut = H *Dufut + P*Dupast + Q*ypast
[H,P,Q] = mpc_predmat(A,B,ny);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%   Find control law and parameters of the cost function
%%%%   Dufut = Pr*rfut - Dk*Dupast - Nk*ypast 
%%%%    J = Dufut'*S*Dufut + Dufut'*2X*[Dupast;ypast;rfut]
[Nk,Dk,Pr,S,X] = mpc_law(H,P,Q,nu,Wu,Wy,sizey);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%   Define constraint matrices
%%%%%%   CC*Dufut - dd - du*ut <= 0
[CC,dd,du]  = mpc_constraints(Dumax,umax,umin,sizey,nu);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Set up simulation parameters
nNk = size(Nk,2)/sizey;
nDk = size(Dk,2)/sizey;
init = max([nNk,nDk])+2;
y = zeros(sizey,init);
u = y; ufast=u;
Du = u;
r = u;
d=u;
opt = optimset('quadprog');
opt.Diagnostics='off';    %%%%% Switches of unwanted MATLAB displays
opt.LargeScale='off';     %%%%% However no warning of infeasibility
opt.Display='off';
opt.Algorithm='active-set';

runtime = size(ref,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1); clf reset

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Closed-loop simulation
for i=init:runtime-ny-1;

%%% Update unconstrained control law
d(1:sizey,i+1)=dist(:,i+1);
ypast = y(:, i:-1:i+1-nNk)+noise(:, i:-1:i+1-nNk);
Dupast = Du(:, i-1:-1:i-nDk) ;
upast = u(:, i-1);
rfut = ref(:,i+1); 


%%%%%%% Unconstrained law - for overlay plots
Dufast(:,i) = Pr*rfut - Nk*ypast(:) - Dk*Dupast(:);
ufast(:,i)=u(:,i-1)+Dufast(1:sizey,i);

% Form constraint matrices and solve constrained optimisation
%  CC*Dufast-dd-du*upast <=0;
%  J = Dufut'S*Dufut+2*X*[Dupast(:);ypast(:);rfut(:)]*Dufut
dt = dd+du*upast;
f=X*[Dupast(:);ypast(:);rfut(:)];
Duqp = quadprog(S,f,CC,dt,[],[],[],[],[],opt);
Du(:,i) = Duqp(1:sizey);
u(:,i) = u(:,i-1)+Du(:,i);


%  Ensure the constraints satisfied by proposed control law   
for j=1:sizey;
   if u(j,i)>u(j,i-1)+Dumax(j);u(j,i)=u(j,i-1)+Dumax(j);end
   if u(j,i)<u(j,i-1)-Dumax(j);u(j,i)=u(j,i-1)-Dumax(j);end
   if u(j,i)>umax(j); u(i)=umax(j);end
   if u(j,i)<umin(j); u(i)=umin(j);end
end
Du(:,i) = u(:,i)-u(:,i-1);
%%% End of update to the control law


%%% Simulate the process
upast2 = u(:,i:-1:i-nDk);
ypast2 = y(:, i:-1:i+2-nNk);
y(:,i+1) = -A(:,sizey+1:nNk*sizey)*ypast2(:) + B*[upast2(:)] + d(:,i+1);
r(:,i+1) = ref(:,i+1);

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Ensure all outputs are dimensionally compatible
u(:,i+1) = u(:,i);
ufast(:,i+1)=ufast(:,i);
Du(:,i+1) = Du(:,i)*0;
Dufast(:,i+1)=Dufast(:,i)*0;
noise = noise(:,1:i+1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%  Produce a neat plot
time=0:size(u,2)-1;
for i=1:sizey;
    figure(i);clf reset
    plotall(y(i,:),r(i,:),u(i,:),ufast(i,:),Du(i,:),Dufast(i,:),d(i,:),noise(i,:),umax(i),umin(i),Dumax(i),time,i);
end

disp('*******************************************************************************');
disp(['***    For GPC there are ',num2str(sizey),' figures beginning at figure 1   ***']);
disp('*******************************************************************************');






%%%%% Function to do plotting in the MIMO case and 
%%%%% allow a small boundary around each plot

function plotall(y,r,u,ufast,Du,Dufast,d,noise,umax,umin,Dumax,time,loop)

uupper = [umax,umax]';
ulower = [umin,umin]';
Dulim = [Dumax,Dumax]';
time2 = [0,time(end)];
rangeu = (max(umax)-min(umin))/20;
rangey = (max(max(y))-min(min(y)))/20;
ranged = (max(max([d,noise]))-min(min([d,noise])))/20;if ranged==0;ranged=1;end

subplot(221);plot(time,y','-',time,r','--');
axis([time2,min(min(y))-rangey,max(max(y))+rangey]);
xlabel(['GPC - Outputs and set-point in loop ',num2str(loop)]);
subplot(222);plot(time,Du','-',time, Dufast,':',time2,Dulim,'--',time2,-Dulim,'--');
axis([time2,min(-Dumax)-rangeu,max(Dumax)+rangeu]);
legend('Constrained','Unconstrained');
xlabel(['GPC - Input increments in loop ',num2str(loop)]);
subplot(223);plot(time,u','-',time,ufast,':',time2,uupper,'--',time2,ulower,'--');
axis([time2,min(umin)-rangeu,max(umax)+rangeu]);
xlabel(['GPC - Inputs in loop ',num2str(loop)]);
subplot(224);plot(time,d','b',time,noise,'g');
axis([time2,min(min([d,noise]))-ranged,max(max([d,noise]))+ranged]);
xlabel(['GPC - Disturbance/noise in loop ',num2str(loop)]);
