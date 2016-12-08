function call_rotation_check(phi,theta,si,Xcm,Ycm,Zcm)

fname1=['Bot_coordiantes','.csv'] ;
fid=fopen(fname1,'r') ; 
input_data = csvread(fname1);
fclose(fid);
figure;

poscm=[Xcm;Ycm;Zcm];

for i=1:4
    output_data=rotation_check(phi,theta,si,poscm,input_data);
    
    poscm(1)=poscm(1)+5;
    poscm(2)=poscm(2);
    poscm(3)=poscm(3);
    input_data=output_data;
end

end