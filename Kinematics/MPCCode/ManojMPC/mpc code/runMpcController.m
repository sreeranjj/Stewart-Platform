function  [uk,Xexpected] = runMpcController(Xk,XsetPoint,H,f_func,LB,UB,predictionErrorX,Ad,Bd,Bias_discrete)
    f = f_func(Xk,XsetPoint,predictionErrorX);
    U = quadprog(H,f,[],[],[],[],LB,UB);
    uk = U(1:4);
    Xexpected=Ad*Xk+Bd*uk+Bias_discrete;%Prediction of next state
end