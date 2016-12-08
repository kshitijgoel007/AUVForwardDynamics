function output_data=rotation_check(phi,theta,si,poscm,input_data)
%function for drawing the bot at different time instants

%input_data=draw_rectangle();

%R=rotation matrix
R=[cos(si)*cos(theta) -sin(si)*cos(phi)+cos(si)*sin(theta)*sin(phi) sin(si)*sin(phi)+cos(si)*cos(phi)*sin(theta);
   sin(si)*cos(theta) cos(si)*cos(phi)+sin(phi)*sin(theta)*sin(si) -cos(si)*sin(phi)+sin(theta)*sin(si)*cos(phi);
  -sin(theta)         cos(theta)*sin(phi)                          cos(theta)*cos(phi)                         ;];



%if det(R)~=0
   [row1,col1]=size(input_data) ;
   output_data=zeros(row1,col1);
   
   for i=1:row1
       temp(1,1)=input_data(i,1);
       temp(2,1)=input_data(i,2);
       temp(3,1)=input_data(i,3);
       
       output_data(i,:)=((R*temp)+poscm)';

   end

scatter(output_data(:,1),output_data(:,2));
xlim([-40 40]);
ylim([-20,20]);
grid;
hold on;
% fname2=['test_rotation','.csv'] ;
% fid2=fopen(fname2,'w+');
% dlmwrite(fname2,output_data);
% fclose(fid2) ;
end