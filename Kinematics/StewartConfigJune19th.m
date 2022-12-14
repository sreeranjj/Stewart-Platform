x=0.0;
y=0.0;
vert=0.0;
z=0.2+vert; %20cm is the distance between two platforms
phi=0;
theta=0;
psi=0;

        
%length=zeros(6,1);
%[length,bA]=invkinematics(x,y,z,psi,theta,phi);
[length,bA]=NewSPKinematics(x,y,z,psi,theta,phi);
length;
bA;
lo=[0.2062,0.2062,0.2062,0.2062,0.2062,0.2062]';
%lo=[0.2544,0.2544,0.2544,0.2544,0.2544,0.2544]';
%lo=[0.3793,0.3793,0.3793,0.3793,0.3793,0.3793]';
deltal=length-lo;
deltal;
a=0.05;
alpha=zeros(6,1);
for i=1:6
 if(deltal(i,1)<0)
     deltal(i,1)=-1*deltal(i,1);
 end
 
 alpha(i,1)=asin(deltal(i,1)/a);
end
alpha;
deltal;
%***********************************************************************

n=5;  % Total time of the simulation
x=zeros(n);
y=zeros(n);
vert=zeros(n);
z=0.2+vert; %20cm is the distance between two platforms
phi=zeros(n);
theta=zeros(n);
psi=zeros(n);

for i=0:0.1:5 
     
    x(i)=0.01*sin(5*i*pi/180);
    y(i)=0.01*sin(5*i*pi/180);
    vert(i)=0.01*sin(5*i*pi/180);
    phi(i)=0.0*sin(5*i*pi/180);
    theta(i)=0.0*sin(5*i*pi/180);
    psi(i)=0.0*sin(5*i*pi/180);
    
%length=zeros(6,1);
%[length,bA]=invkinematics(x,y,z,psi,theta,phi);
[length(fix(i*10)),bA(fix(i*10))]=NewSPKinematics(x(i),y(i),z(i),psi(i),theta(i),phi(i));
%length;
%bA;
%lo=[0.2062,0.2062,0.2062,0.2062,0.2062,0.2062]';
%lo=[0.2544,0.2544,0.2544,0.2544,0.2544,0.2544]';
%lo=[0.3793,0.3793,0.3793,0.3793,0.3793,0.3793]';
deltal=length-lo;
deltal;
a=0.05;
alpha=zeros(6,1);
for i=1:6
 if(deltal(i,1)<0)
     deltal(i,1)=-1*deltal(i,1);
 end
 
 alpha(i,1)=asin(deltal(i,1)/a);
end

end
alpha;
deltal;