function plot_bot_global(i_main,id,phi,theta,si,Xcm,Ycm,Zcm,input_data)

R_phi=[1 0 0;0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
R_theta=[cos(theta) 0 sin(theta);0 1 0; -sin(theta) 0 cos(theta)];
R_si=[cos(si) -sin(si) 0; sin(si) cos(si) 0;0 0 1];
R=R_phi*R_theta*R_si;  %R=ROTATION MATRIX
poscm=[Xcm;Ycm;Zcm];

if det(R)~=0
   [row1,col1]=size(input_data) ;
   output_data=zeros(row1,col1);
   
   for i=1:row1
       temp(1,1)=input_data(i,1);
       temp(2,1)=input_data(i,2);
       temp(3,1)=input_data(i,3);
       
       output_data(i,:)=((inv(R)*temp)+poscm)';
   end
   
   fname2=['Bot_coordiantes_global_case_',num2str(id,'%0d'),'.csv'] ;
   fid2=fopen(fname2,'a') ;
   csvwrite(fname2,output_data);%writing data to different columns!!modify
   fclose(fid2) ;
end
end