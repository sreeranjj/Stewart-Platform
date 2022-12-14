function [Pb,BSWIVb,ACTVEC]= ACTUATOR(EUx,EUy,EUz,X,Y,Z,BASBAS,PLAPLA,HGT)

% This function calculates the actuator vector describing the actuator 
%length and orientation in space by base coordinates
    
  PLAb=zeros(6,3);
    
  Pb=zeros(1,3);
  BSWIVb=zeros(6,3);
  ACTVEC=zeros(6,3);
    
    
  %Pb is from origin of base to origin of platform
  
  Pb(1,1)=X;
  Pb(1,2)=Y;
  Pb(1,3)=HGT+Z;
  
  %Now we have to find the vector from base to joint
  %From derivation, Lib=Pb+(Trans*Ppip)b-Bbib
  %BSib=Pb+(Trans*Ppip)b
  
  for IACT=1:6
      PLAb(IACT,:)= FORWARD_TRANS(EUx,EUy,EUz,PLAPLA,IACT);  %Trans*Ppib 
                                                             %Check notes
      for ICOMP=1:3
           BSWIVb(IACT,ICOMP)=PLAb(IACT,ICOMP)+Pb(1,ICOMP); %Gives vector between base origin and platform joint 
      end
  end
 
%Now we have to calculate vector for each actuator:Li (in base coordinates
%Lib=BSib-Bbib
%We already calculated BSib

for IACT=1:6
    for i=1:3
        ACTVEC(IACT,i)=BSWIVb(IACT,i)-BASBAS(IACT,i);
    end
end


end






  
           
    
    