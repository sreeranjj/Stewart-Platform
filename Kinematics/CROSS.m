%This function determines the cross product of two vectors
function PRODUCT=CROSS(VECT_1,VECT_2)

PRODUCT=zeros(1,3);

PRODUCT(1,1)=VECT_1(1,2)*VECT_2(1,3)-VECT_2(1,2)*VECT_1(1,3);   %X
PRODUCT(1,2)=-1*VECT_1(1,1)*VECT_2(1,3)-VECT_2(1,1)*VECT_1(1,3);%Y
PRODUCT(1,3)=VECT_1(1,1)*VECT_2(1,2)-VECT_1(1,2)*VECT_2(1,1);   %Z

end