%%%%%  Typical data entry required for a 
%%%%%  MIMO MFD model
%%%%%    [Ao + A1z^{-1}+...] y = [B1 z^{-1} + B2 z^{-2}+...] u
%%%%%    A = [A0,A1,A2,...]      B = [B1,B2,...]
%%%%%
%%%%%  Illustrates open-loop prediction
%%%%   for MFD model   A y(k+1) = B u(k)
%%%%
%%%%  yfut = H *Dufut + P*Dupast + Q*ypast
%%%%
%%%%  [H,P,Q] = mpc_predmat(A,B,ny)  - uses recursive methods
%%%%  [H1,P1,Q1] = mpc_predmat_toeplitz(A,B,ny); %%% Toeplitz matrices method
%%%%%
%%%%%   THIS IS A SCRIPT FILE. CREATES ITS OWN DATA AS REQUIRED
%%%%%   EDIT THIS FILE TO ENTER YOUR OWN MODELS, ETC.
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

%%% SISO Model
a=[1 -1.2 0.32];
b=[0.41,0.65];
ny=5;
[H,P,Q] = mpc_predmat(a,b,ny)   %%% Historical recursive method
[H1,P1,Q1] = mpc_predmat_toeplitz(a,b,ny) %%% Toeplitz matrices method
[norm(H-H1),norm(Q-Q1),norm(P-P1)]

%%% MIMO Model
A=[];B=[];
A(1,1:3:12) = poly([.5,.8,-.2]);
A(2,2:3:12) = poly([-.2,.9,.5]);
A(3,3:3:12) = poly([-.1,.4,.6]);
A(1,5:3:12) = [ -.2 .1 .02];
A(2,4:3:12) = [ .4 0 -.1];
A(3,4:3:12) = [0 .2 .2];
A(2,6:3:12) = [-1 .1 .3];
A(:,4:12) = A(:,4:12)*.8;
B = [.5 0.2 -.5 1 2 1;2 0 .3 -.8 .6 .5;0 .9 -.4 1 .3 .5];
ny=5;

[H,P,Q] = mpc_predmat(A,B,ny)  ; %%% Historical recursive method
[H1,P1,Q1] = mpc_predmat_toeplitz(A,B,ny); %%% Toeplitz matrices method
[norm(H-H1),norm(Q-Q1),norm(P-P1)]

