%function to correct state prediction and P prediction with measurements using kalman filter
%correctX(k,X_pred,Y_measured,C)
function [X_corrected,P_corrected]=correctFunc(X_pred,P_pred,k,C,Y_measured)
    Y_pred=C*X_pred;
    X_corrected=X_pred+k*(Y_measured-Y_pred);
    P_corrected=(eye(size(k*C))-k*C)*P_pred;
    
end