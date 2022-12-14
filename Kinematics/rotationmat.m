function[rot]=rotationmat(gamma)

rot(1,1)=cos(gamma*pi/180);
rot(1,2)=sin(gamma*pi/180);
rot(1,3)=0;
rot(2,1)=-sin(gamma*pi/180);
rot(2,2)=cos(gamma*pi/180);
rot(2,3)=0;
rot(3,1)=0;
rot(3,2)=0;
rot(3,3)=1;

end
