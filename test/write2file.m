function write2file()
%persistent n;
n=0;
output_data=zeros(5,3);
[row col]=size(output_data);
for i=1:4
fname=['lmn','.csv'] ;
fid=fopen(fname,'a');
if n==0
    dlmwrite(fname,output_data,'roffset',0,'coffset',0,'-append');
    n=n+1;
else
    dlmwrite(fname,output_data,'roffset',1,'coffset',0,'-append');
    n=n+1;
end

fclose(fid) ;
end
end
