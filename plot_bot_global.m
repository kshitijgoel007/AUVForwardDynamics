function plot_bot_global(fname,phi,theta,si,Xcm,Ycm,Zcm,input_data)
%function for drawing the bot at different time instants

%R=rotation matrix
R_phi=[1 0 0;0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
R_theta=[cos(theta) 0 sin(theta);0 1 0; -sin(theta) 0 cos(theta)];
R_si=[cos(si) -sin(si) 0; sin(si) cos(si) 0;0 0 1];
R=R_phi*R_theta*R_si;  

poscm=[Xcm;Ycm;Zcm];

if det(R)~=0
   [row1,col1]=size(input_data) ;
   output_data=zeros(row1,col1);
   
   for i=1:row1
       temp(1,1)=input_data(i,1);
       temp(2,1)=input_data(i,2);
       temp(3,1)=input_data(i,3);
       
       %output_data(i,:)=((inv(R)*temp)+poscm)';
       output_data(i,:)=(temp+poscm)';
   end
   
fname2=['Global_bot_coordinates_case_',fname,'.csv'] ;
fid2=fopen(fname2,'a');
dlmwrite(fname2,output_data,'roffset',1,'coffset',0,'-append');
fclose(fid2) ;
end
end