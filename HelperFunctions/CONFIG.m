function [BASBASr,PLAPLAr,HGT1]= CONFIG()
 
 %This function contains the geometry configuration of the platform and base
 %Thus the platform coordinate system and base coordinate system are derived
 %here by means of determining the vectors from the origin (center) to the
 %joints

 %HGT is the neutral position distance between the base and platform
 %coordinate system

   BASBASr= zeros(6,3);
   PLAPLAr=zeros(6,3);
   HGT1=0.5 ;               %Initial distance between the base and platform
   
   IACT=1;                 %Actuator index
   ANGLE=0;                %Initial angle
   RB=0.40;                %Radius of the base ***Not sure of the value***
   DELB=0.0698132;             %Small angle between two joints 4 degree
    for i=1:3                          %We measure 1 and 2 first,3 and 4 next and so on
        BASBASr(IACT,1)=RB*cos(ANGLE-DELB); %X component
        BASBASr(IACT,2)=RB*sin(ANGLE-DELB); %Y component
        BASBASr(IACT,3)=0;                  %Z component
        
        BASBASr(IACT+1,1)=RB*cos(ANGLE-2.0944+DELB); %X component
        BASBASr(IACT+1,2)=RB*sin(ANGLE-2.0944+DELB); %Y component*** 2.0944 radians=120 degree***
        BASBASr(IACT+1,3)=0;                      %Z component
        IACT=IACT+2;
        ANGLE=ANGLE-2.0944;
    end
    
   RP=0.15;                    %Radius of the platform 23cm-Law of sines

   IACT=1;                     %Index
   ANGLE=-1.0472;                  %Initial angle 60degree
   DELP=0.0872665;                %Small angle 5 degree
     for  i=1:3
         PLAPLAr(IACT,1)=RP*cos(ANGLE+DELP); %X component
         PLAPLAr(IACT,2)=RP*sin(ANGLE+DELP); %Y component
         PLAPLAr(IACT,3)=0;                  %Z component
         
         PLAPLAr(IACT+1,1)=RP*cos(ANGLE-DELP); %X component
         PLAPLAr(IACT+1,2)=RP*sin(ANGLE-DELP); %Y component
         PLAPLAr(IACT+1,3)=0;                      %Z component
         IACT=IACT+2;
         ANGLE=ANGLE-120;
     end
     
end


