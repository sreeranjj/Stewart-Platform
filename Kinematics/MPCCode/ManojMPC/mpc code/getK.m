%function to find k matrix using kalman filter
%getK(P,R,C)
function k=getK(P,R,C)
  % k=(P*C')*inv(C*P*C'+R);
    k=(P*C')/(C*P*C'+R);
end