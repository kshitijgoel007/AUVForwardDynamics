
function I=integration2(X)

k=0:(5.3/9):5.3;

u = X(1);
v = X(2);
w = X(3);
p = X(4);
q = X(5);
r = X(6);

syms x;
h=[2*sqrt(.0544*(x+2.65)),.42,.545*(5.26-(x-2.65))];
b=[2*sqrt(.14694*(x+2.65)),.69,.896*(5.26-(x-2.65))];
Ucf=sqrt((v+(x-2.65)*r)^2+(w-(x-2.65)*q)^2);
Ucf1=@(x)Ucf;

Cdy=.5;
Cdz=.6;
sway=zeros(1,10);
heave=zeros(1,10);
pitch=zeros(1,10);
yaw=zeros(1,10);

for i=1:1:10
    if(subs(Ucf1,x,k(i))==0)
       sway(i)=0;
       heave(i)=0;
       pitch(i)=0;
       yaw(i)=0;
       continue;
    end  
        
  if (k(i)<=.81)
   
       F= @(x)(Cdy*h(1)*(v+(x-2.65)*r)^2+Cdz*b(1)*(w-(x-2.65)*q)^2)*(v+(x-2.65)*r)/Ucf; 
       sway(i)=subs(F,x,k(i)); 
   
       F=@(x)(Cdy*h(1)*(v+(x-2.65)*r)^2+Cdz*b(1)*(w-(x-2.65)*q)^2)*(v+(x-2.65)*r)/Ucf; 
       heave(i)=subs(F,x,k(i));
    
       F= @(x)(Cdy*h(1)*(v+(x-2.65)*r)^2+Cdz*b(1)*(w-(x-2.65)*q)^2)*(w+(x-2.65)*q)*(x-2.65)/Ucf;
       pitch(i)=subs(F,x,k(i));  
    
       F=@(x)(Cdy*h(1)*(v+(x-2.65)*r)^2+Cdz*b(1)*(w-(x-2.65)*q)^2)*(v+(x-2.65)*r)*(x-2.65)/Ucf;
       yaw=subs(F,x,k(i));
    
  
  elseif (k(i)<=4.49)
   F= @(x)(Cdy*h(2)*(v+(x-2.65)*r)^2+Cdz*b(2)*(w-(x-2.65)*q)^2)*(v+(x-2.65)*r)/Ucf; 
   sway(i)=subs(F,x,k(i));            
   
   F=@(x)(Cdy*h(2)*(v+(x-2.65)*r)^2+Cdz*b(2)*(w-(x-2.65)*q)^2)*(v+(x-2.65)*r)/Ucf; 
   heave(i)=subs(F,x,k(i));           
   
   F=@(x)(Cdy*h(2)*(v+(x-2.65)*r)^2+Cdz*b(2)*(w-(x-2.65)*q)^2)*(w+(x-2.65)*q)*(x-2.65)/Ucf;
   pitch(i)=subs(F,x,k(i));
              
   F=@(x)(Cdy*h(2)*(v+(x-2.65)*r)^2+Cdz*b(2)*(w-(x-2.65)*q)^2)*(v+(x-2.65)*r)*(x-2.65)/Ucf;
    yaw(i)=subs(F,x,k(i));
              
 else F=@(x)(Cdy*h(3)*(v+(x-2.65)*r)^2+Cdz*b(3)*(w-(x-2.65)*q)^2)*(v+(x-2.65)*r)/Ucf ;   
       sway(i)=subs(F,x,k(i));  
       
       F= @(x)(Cdy*h(3)*(v+(x-2.65)*r)^2+Cdz*b(3)*(w-(x-2.65)*q)^2)*(v+(x-2.65)*r)/Ucf;
       heave(i)=subs(F,x,k(i));  
       
       F=@(x)(Cdy*h(3)*(v+(x-2.65)*r)^2+Cdz*b(3)*(w-(x-2.65)*q)^2)*(w+(x-2.65)*q)*(x-2.65)/Ucf;
       pitch(i)=subs(F,x,k(i));
       
       F= @(x)(Cdy*h(3)*(v+(x-2.65)*r)^2+Cdz*b(3)*(w-(x-2.65)*q)^2)*(v+(x-2.65)*r)*(x-2.65)/Ucf;
       yaw(i)=subs(F,x,k(i));
  end
  
end

I(1)=(3*(5.3/9)/8)*(sway(1)+3*sway(2)+3*sway(3)+2*sway(4)+3*sway(5)+3*sway(6)+2*sway(7)+3*sway(8)+3*sway(9)+sway(10));
I(2)=(3*(5.3/9)/8)*(heave(1)+3*heave(2)+3*heave(3)+2*heave(4)+3*heave(5)+3*heave(6)+2*heave(7)+3*heave(8)+3*heave(9)+heave(10));
I(3)=(3*(5.3/9)/8)*(pitch(1)+3*pitch(2)+3*pitch(3)+2*pitch(4)+3*pitch(5)+3*pitch(6)+2*pitch(7)+3*pitch(8)+3*pitch(9)+pitch(10));
I(4)=(3*(5.3/9)/8)*(yaw(1)+3*yaw(2)+3*yaw(3)+2*yaw(4)+3*yaw(5)+3*yaw(6)+2*yaw(7)+3*yaw(8)+3*yaw(9)+yaw(10));

end