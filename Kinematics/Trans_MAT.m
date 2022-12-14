function TRANS=Trans_MAT(EUx,EUy,EUz)
%This function calculates the forward transformation matrix derived from
%Euler angles
%Returns a 3 by 3 matrix
TRANS=zeros(3,3);

TRANS(1,1)=cos(EUz)*cos(EUy);
TRANS(1,2)=-sin(EUz)*cos(EUx)+sin(EUy)*cos(EUz)*sin(EUx);
TRANS(1,3)=sin(EUz)*sin(EUx)+sin(EUy)*cos(EUz)*cos(EUx);
TRANS(2,1)=cos(EUy)*sin(EUz);
TRANS(2,2)=cos(EUz)*cos(EUx)+sin(EUz)*sin(EUy)*sin(EUx);
TRANS(2,3)=-cos(EUz)*sin(EUx)+sin(EUz)*sin(EUy)*cos(EUx);
TRANS(3,1)=-sin(EUy);
TRANS(3,2)=cos(EUy)*sin(EUx);
TRANS(3,3)=cos(EUy)*cos(EUx);

end

