function Y=euler(fname)
t=0:.1:50;
dt=.1;
count=1;
geoprop;                       %MAT FILE CONTAING PROPERTIES OF BOT
X=zeros(18,1);
X(1,1)=1;
Y=zeros(length(t),length(X)+1);
Y(1,1)=0;                      %first column stores time

for i=1:length(X)
    Y(1,i+1)=X(i);            %INITIAL VALUES SAVED IN Y
end


%OPEN FILE CONTAINING LOCAL BOT COORDINATES

fname1=['Bot_coordiantes','.csv'] ;%LOCAL BOT COORDINATES
fid=fopen(fname1,'r') ; 
input_data = csvread(fname1) ;%//
fclose(fid);

switch fname
case {'1'}
  id=1;  
  del_o = 10*pi/180.0;
 for i=2:length(t)
 %SOLVING USING EULER METHOD
    DX=forwarddynamics2(t(i),X);
    temp=X+DX*dt;
    X=temp;
    Y(i,1)=t(i);
    for j=1:length(X)
    Y(i,j+1)=X(j);
    end
  %EULER METHOD IMPLEMENTED 
    
  %UPDATING BOT COORDINATES AND SAVING IN FILE
    if rem(t(i),10)==0|| i==2  %UPDATE COORDINATES EVERY 10 SECONDS 
       plot_bot_global(count,id,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       count=count+1;
    end
  %BOT COORDINATES UPDATED AND SAVED

  
 end
          
          
          
case {'2'}
id=2;
del_o=15*pi/180.0;
for i=2:length(t)
 %SOLVING USING EULER METHOD
    DX=forwarddynamics2(t(i),X);
    temp=X+DX*dt;
    X=temp;
    Y(i,1)=t(i);
    for j=1:length(X)
    Y(i,j+1)=X(j);
    end
  %EULER METHOD IMPLEMENTED 
    
  %UPDATING BOT COORDINATES AND SAVING IN FILE
    if rem(t(i),10)==0|| i==2  %UPDATE COORDINATES EVERY 10 SECONDS 
       plot_bot_global(count,id,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       count=count+1;
    end
  %BOT COORDINATES UPDATED AND SAVED

  
end
  
  
  
case {'3'}
id=3;
del_o=10*pi/180.0;
for i=2:length(t)
    %SOLVING USING EULER METHOD
      
    DX=forwarddynamics2(t(i),X);
    temp=X+DX*dt;
    X=temp;
    Y(i,1)=t(i);
    for j=1:length(X)
    Y(i,j+1)=X(j);
    end
  %EULER METHOD IMPLEMENTED 
   
  if (X(16)>10*pi/180.0 && del_o==10*pi/180.0) 
       del_o=-10*pi/180.0;
   else if (X(16)<-10*pi/180.0 && del_o==-10*pi/180.0)
       del_o=10*pi/180.0;    
       end
  end
   %UPDATING BOT COORDINATES AND SAVING IN FILE
    if rem(t(i),10)==0|| i==2  %UPDATE COORDINATES EVERY 10 SECONDS 
       plot_bot_global(count,id,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       count=count+1;
    end
  %BOT COORDINATES UPDATED AND SAVED

  
 end
 
case {'4'}
id=4;
del_o=-10*pi/180.0;
for i=2:length(t)
    %SOLVING USING EULER METHOD
    DX=forwarddynamics2(t(i),X);
    temp=X+DX*dt;
    X=temp;
    Y(i,1)=t(i);
    for j=1:length(X)
    Y(i,j+1)=X(j);
    end
  %EULER METHOD IMPLEMENTED 
   
  if (X(16)>10*pi/180.0 && del_o==10*pi/180.0) 
       del_o=-10*pi/180.0;
   else if (X(16)<-10*pi/180.0 && del_o==-10*pi/180.0)
       del_o=10*pi/180.0;    
       end
  end
   %UPDATING BOT COORDINATES AND SAVING IN FILE
    if rem(t(i),10)==0|| i==2  %UPDATE COORDINATES EVERY 10 SECONDS 
       plot_bot_global(count,id,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       count=count+1;
    end
  %BOT COORDINATES UPDATED AND SAVED

  
end       
    case {'5'}


    case {'6'}
        
    case {'7'}
        
    case {'8'}
        
end



%SAVING Y VECTOR  INCLUDE TIME ALSO444444444444444444444444444
fname=['case_',fname,'.csv'] ;
fid=fopen(fname,'w') ;
csvwrite(fname,Y);
fclose(fid);
end


  
    