function [H,f]=fetchQPformulation(Ad,Bd,Bias_discrete,Qx,lambda,P,M)
%This function takes in the model and Weights(Qx for states and lambda for
%input signals->lambda*I=Qu). Note that Lambda is a scalar value. This function returns
%the Hessian which only depends on the model AND function handle for 'f'
%that depends on desired X and Current X states.
%Since our quadcopter system is fully observable(By means of different 
%sensors and C matrixes-refer ekf part), we care taking the states instead
%of outputs in the mpc optimization problem.
%P,M denote the prediction horizon and control horizon respectively

%FINDING HESSIAN
    dim=size(Bd);dim=dim(2);
    H=lambda*eye(dim*M);%This is the input part, wait for next step for beta part
    for j=1:P
        beta=getBeta(j,M,Ad,Bd);
        H= H +beta'*Qx*beta;
    end

%FINDING F
    f1=0;f2=0;f3=0;f4=0;
    for j=1:P
        beta=getBeta(j,M,Ad,Bd);
        f1=f1+beta'*Qx'*(Ad^j);
        f2=f2-beta'*Qx';
        f3=f3+beta'*Qx'*(Ad^(j-1))*Bias_discrete;
        f4=f4+beta'*Qx'*(Ad^(j-1));
    end

f=@(Xk,Xref,predictionErrorX) f1*Xk+f2*Xref+f3+f4*predictionErrorX;
%DONE. Use the above function handle to get f