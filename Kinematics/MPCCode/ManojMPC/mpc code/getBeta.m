function beta = getBeta(j,M,A,B)
%input discretized model 
%This beta that we find is need for formulating the quadratic programming
%problem.
siz=size(B);
zm=zeros(siz(1),siz(2));
beta=[];
if j<M
    for index=1:M-j
        beta=[zm,beta]; %Assembling from right
    end
    for index=0:j-1
        beta=[A^(index)*B,beta]; %Assembling from right
    end
elseif j==M
    for index=0:j-1
        beta=[A^(index)*B,beta]; %Assembling from right
    end
elseif j>M
    beta=0;
    for index=0:j-M
        beta=beta+(A^index)*B;
    end
    for index=j-M+1:j-1
        beta=[(A^index)*B,beta];
    end
end

end
        
  