function plot_bot_global(fname,phi,theta,si,Xcm,Ycm,Zcm,input_data)
%function for drawing the bot at different time instants
%input_data=3185*3 constant matrix containing coordinates of bot in body frame 
%R=rotation matrix

R=[cos(si)*cos(theta) -sin(si)*cos(phi)+cos(si)*sin(theta)*sin(phi) sin(si)*sin(phi)+cos(si)*cos(phi)*sin(theta);
   sin(si)*cos(theta) cos(si)*cos(phi)+sin(phi)*sin(theta)*sin(si) -cos(si)*sin(phi)+sin(theta)*sin(si)*cos(phi);
  -sin(theta)         cos(theta)*sin(phi)                          cos(theta)*cos(phi)                         ;];


 poscm=[Xcm;Ycm;Zcm];

    [row1,col1]=size(input_data) ;
    output_data=zeros(row1,col1);
    
   for i=1:row1
       
       %rotating the bot
       output_data(i,:)=(R*input_data(i,:)')';
       
       %now translating the bot
       output_data(i,:)=output_data(i,:)+poscm';
   end
   
%plotting bot   
scatter(output_data(:,1),output_data(:,2));
% xlim([-40 40]);
% ylim([-20 20]);
grid;
xlabel('X');
ylabel('Y');
hold on;


%storing bot coordinates
fname2=['Global_bot_coordinates_case_',fname,'.csv'] ;
fid2=fopen(fname2,'a');
dlmwrite(fname2,output_data,'roffset',1,'coffset',0,'-append');
fclose(fid2) ;
end

