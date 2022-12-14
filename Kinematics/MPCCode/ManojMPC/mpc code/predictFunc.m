function [X_pred,P_pred] = predictFunc(Xk,uk,Pk,Q,Ad,Bd,Bias_discrete)

X_pred=Ad*Xk+Bd*uk+Bias_discrete;
P_pred=Ad*Pk*Ad'+Q;

end



%%HOW TO TEST
%Run this in command window to validate this function
%  Xk=zeros(12,1);uk=[.5,.5,.5,.5]';Pk=eye(12)*1;Q=eye(12)*0.00001;
%  predictFunc(Xk,uk,Pk,Q)