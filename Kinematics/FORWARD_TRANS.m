function VECT_OUT= FORWARD_TRANS(EUx,EUy,EUz,VECT_INP,IACT)
%This function converts vector in platform coordinates to base coordinates
%by using Euler angle transformation
%Returns one vector at each call with 3 components

Trans_mat=Trans_MAT(EUx,EUy,EUz);
VECT_OUT=zeros(1,3);


%VEC_INP is the vector from base origin to platform joints

%PLAb=Trans*PLAPLA
% VEC_OUT=Trans*VEC_INP

  for i=1:3
      VECT_OUT(1,i)=Trans_mat(i,1)*VECT_INP(IACT,1)+Trans_mat(i,2)*VECT_INP(IACT,2)+Trans_mat(i,3)*VECT_INP(IACT,3);
    
  end
end
