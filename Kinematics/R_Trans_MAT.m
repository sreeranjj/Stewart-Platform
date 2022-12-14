function R_TRANS=R_Trans_MAT(EUx,EUy,EUz)
%This function calculates the reverese tranformation matrix derived from
%Euler angles
%Returns a 3 by 3 matrix
R_TRANS=zeros(3,3);

R_TRANS(1,1)=cos(EUz)*cos(EUy);
R_TRANS(2,1)=-sin(EUz)*cos(EUx)+sin(EUy)*cos(EUz)*sin(EUx);
R_TRANS(3,1)=sin(EUz)*sin(EUx)+sin(EUy)*cos(EUz)*cos(EUx);
R_TRANS(1,2)=cos(EUy)*sin(EUz);
R_TRANS(2,2)=cos(EUz)*cos(EUx)+sin(EUz)*sin(EUy)*sin(EUx);
R_TRANS(3,2)=-cos(EUz)*sin(EUx)+sin(EUz)*sin(EUy)*cos(EUx);
R_TRANS(1,3)=-sin(EUy);
R_TRANS(2,3)=cos(EUy)*sin(EUx);
R_TRANS(3,3)=cos(EUy)*cos(EUx);

end