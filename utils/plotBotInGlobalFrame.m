
  function plotBotInGlobalFrame(caseNo,phi,theta,si,Xcm,Ycm,Zcm,botCoordinatesInBodyFrame)
% Function for drawing the bot at different time instants
% Input_data=3185*3 constant matrix containing coordinates of bot in body frame 
% INPUT :
% OUTPUT : trajectory 
% function plot_bot_global(caseNo,phi,theta,si,Xcm,Ycm,Zcm,botCoordinatesInBodyFrame)


% R=rotation matrix
  R=[cos(si)*cos(theta) -sin(si)*cos(phi)+cos(si)*sin(theta)*sin(phi) sin(si)*sin(phi)+cos(si)*cos(phi)*sin(theta);
     sin(si)*cos(theta) cos(si)*cos(phi)+sin(phi)*sin(theta)*sin(si) -cos(si)*sin(phi)+sin(theta)*sin(si)*cos(phi);
     -sin(theta)         cos(theta)*sin(phi)                          cos(theta)*cos(phi)                         ;];

% CM coordinates
  poscm=[Xcm;Ycm;Zcm];

% Initialising matix for storing transformed bot coordinates
  [row1,col1]=size(botCoordinatesInBodyFrame) ;
  output_data=zeros(row1,col1);
    
  for i=1:row1
       temp(1,1)=botCoordinatesInBodyFrame(i,1);
       temp(2,1)=botCoordinatesInBodyFrame(i,2);
       temp(3,1)=botCoordinatesInBodyFrame(i,3);
       
       temp1=((R*temp)+poscm);
       output_data(i,1)=temp1(1,1);
       output_data(i,2)=temp1(2,1);
       output_data(i,3)=temp1(3,1);  
   end

% Plotting bot
  scatter(output_data(:,1),output_data(:,2));
% xlim([-40 40]);
% ylim([-20 20]);
  grid;
  xlabel('X');
  ylabel('Y');
  hold on;
%   Legend('case %d',caseNo);

% Storing bot coordinates
  fname2=['Global_bot_coordinates_case_',caseNo,'.csv'] ;
  fid2=fopen(fname2,'a');
  dlmwrite(fname2,output_data,'roffset',1,'coffset',0,'-append');
  fclose(fid2) ;

end

