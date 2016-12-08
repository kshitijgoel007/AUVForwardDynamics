%function for solving 6DOF equations using 1st order euler method

function Y=euler(fname,input_data)
%fname=case no to be run
%input_dta=bot coordinates in body frame
t=0:.1:30;
dt=.1;


%MAT FILE CONTAING PROPERTIES OF BOT
geoprop;                       
X=zeros(18,1);
X(1,1)=1;
Y=zeros(length(t),length(X)+1);

%first column stores time
Y(1,1)=0;                      

%INITIAL VALUES SAVED IN Y
for i=1:length(X)
    Y(1,i+1)=X(i);            
end

global del_o;

%CASES/////////////////////////////////////////////////

switch fname

    case {'1'}
    %when del_o is positive, bot turns to port(-Y initially points to port
    %also si is -ve.
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
    
    
    %UPDATING BOT COORDINATES AND SAVING IN FILE
    if rem(t(i),10)==0|| i==2  %UPDATE COORDINATES EVERY 10 SECONDS 
       plot_bot_global(fname,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       
    end
    %BOT COORDINATES UPDATED AND SAVED
    
    end
    figure;
    plot(Y(:,8),Y(:,9));
    legend('case 1');
    
    

%///////////////////////////////////////////////////////////           
          
    case {'2'}

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
       plot_bot_global(fname,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       
    end
    %BOT COORDINATES UPDATED AND SAVED
    end
    
    figure;
    plot(Y(:,8),Y(:,9));
    legend('case 2');
    
  
%///////////////////////////////////////////////////////////
  
  
  
    case {'3'}
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

        if (X(12)>10*pi/180.0) 
            del_o=10*pi/180.0;
        else if (X(12)<-10*pi/180.0)
            del_o=-10*pi/180.0;    
            end
        end
    
        %UPDATING BOT COORDINATES AND SAVING IN FILE
        %UPDATE COORDINATES EVERY 10 SECONDS
        if rem(t(i),10)==0|| i==2   
        plot_bot_global(fname,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       
        end
    end
    %BOT COORDINATES UPDATED AND SAVED

    figure;
    plot(Y(:,8),Y(:,9));
    legend('case 3');
    

   %///////////////////////////////////////////////////////////////////

case {'4'}
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

    if (X(12)<-10*pi/180.0) 
       del_o=-10*pi/180.0;
    else if (X(12)>10*pi/180.0)
       del_o=10*pi/180.0;    
        end
    end
    
   
   %UPDATING BOT COORDINATES AND SAVING IN FILE
   %UPDATE COORDINATES EVERY 10 SECONDS
   if rem(t(i),10)==0|| i==2   
       plot_bot_global(fname,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       
   end
   end
    %BOT COORDINATES UPDATED AND SAVED
    
    
    figure;
    plot(Y(:,8),Y(:,9));
    legend('case 4');
    

   %///////////////////////////////////////////////////////////////////
    %rudder deflection to port side -ve
    
    case {'5'}
    del_o=-20*pi/180.0;
    
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

    if (X(12)>20*pi/180.0) 
       del_o=20*pi/180.0;
    else if (X(12)<-20*pi/180.0)
       del_o=-20*pi/180.0;    
        end
    end
    
   %UPDATING BOT COORDINATES AND SAVING IN FILE
   %UPDATE COORDINATES EVERY 10 SECONDS
   if rem(t(i),5)==0|| i==2   
       plot_bot_global(fname,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       
   end
   end
    %BOT COORDINATES UPDATED AND SAVED


%     figure;
%     plot(Y(:,8),Y(:,9));
%     legend('case 5');
%     
   %///////////////////////////////////////////////////////////////////

case {'6'}
del_o=20*pi/180.0;
    
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

    if (X(12)<-20*pi/180.0) 
       del_o=-20*pi/180.0;
    else if (X(12)>20*pi/180.0)
       del_o=20*pi/180.0;    
        end
    end
    
   
   %UPDATING BOT COORDINATES AND SAVING IN FILE
   %UPDATE COORDINATES EVERY 10 SECONDS
   if rem(t(i),10)==0|| i==2   
       plot_bot_global(fname,X(10),X(11),X(12),X(7),X(8),X(9),input_data);
       
   end
   end
   %BOT COORDINATES UPDATED AND SAVED

    figure;
    plot(Y(:,8),Y(:,9));
    legend('case 6');
    

   %///////////////////////////////////////////////////////////////////


    case {'7'}
    %vertical zig-zag    
    case {'8'}
    %vertical zig-zag    
end



%SAVING Y VECTOR  
fname1=['case_',fname,'.csv'] ;
fid=fopen(fname1,'w') ;
dlmwrite(fname1,Y);
fclose(fid);
end


  
    