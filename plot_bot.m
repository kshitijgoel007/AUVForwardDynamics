n=0;

for i=-2:.2:2
    
    for j=0:90
        V(n+j+1,1)=i;
        V(n+j+1,2)=.2*cos(4*j*pi/180);
        V(n+j+1,3)=.2*sin(4*j*pi/180);
    end
n=n+91;    
end

for i=2:.1:3.3
    for j=0:90
        V(n+j+1,1)=i;
        V(n+j+1,2)=.2*(3.3-i)*cos(4*j*pi/180)/1.3;
        V(n+j+1,3)=.2*(3.3-i)*sin(4*j*pi/180)/1.3;
    end
n=n+91;
end

 fname=['Bot_coordiantes','.csv'] ;
 fid=fopen(fname,'w') ;    
 csvwrite(fname,V);
 fclose(fid);