function VECT_OUT= REVERSETRANS(EUx,EUy,EUz,VECT_INP,IACT)
%This function converts a vector in base coordinates to a vector in
%platform coordinates
%Returns one vector at each call with 3 components
%We use inverse of transformation matrix(transpose of original
%Transformation matrix)


RTrans_mat=R_Trans_MAT(EUx,EUy,EUz);
VECT_OUT=zeros(1,3);

%PLatform_VECT=RTransform*Base_VECT
  for i=1:3
    
    VECT_OUT(1,i)=RTrans_mat(i,1)*VECT_INP(IACT,1)+RTrans_mat(i,2)*VECT_INP(IACT,2)+RTrans_mat(i,3)*VECT_INP(IACT,3);
  end
end



    