function [MAG,UNIT_VECTOR]= MAGNITUDE(VECTOR)
%This function normalises each vector by dividing with its magnitude
%and we get a unit vector
%Returns a scalar(magnitude) and a unit vector with 3 components

MAG=0;
UNIT_VECTOR=zeros(1,3);

MAG=sqrt((VECTOR(1,1))^2+ (VECTOR(1,2))^2+ (VECTOR(1,3))^2);

%Determination of the unit vector
UNIT_VECTOR(1,1)=VECTOR(1,1)/MAG;
UNIT_VECTOR(1,2)=VECTOR(1,2)/MAG;
UNIT_VECTOR(1,3)=VECTOR(1,3)/MAG;

end
