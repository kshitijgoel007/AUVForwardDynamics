%THIS FUNCTION GIVES VARIOUS CASES AS INPUT
function cases()

%CASE1:DEL_R_ORDERED=10DEGREE
%CASE2:DEL_R_ORDERED=15DEGREE
%CASE3:ZIG-ZAG MOTION: DEL_R_ORDERED=10DEGREE,WHEN PSI=10DEGREE
%        DEL_R_ORDERED=-10DEGREE
%CASE4:SAME AS CASE 3 WITH DEL_R_ORDERED=-10DEGREE INTIALLY
%CASE5:AKA CASE3:DEL_R_ORDERED=20DEGREE
%CASE6:AKA CASE4_2:DEL_R_ORDERED=-20DEGREE
%CASE7:zig zag pitch motion in vertical plane
%CASE8:??


%OPEN FILE CONTAINING LOCAL BOT COORDINATES
fname1=['Bot_coordiantes','.csv'] ;
fid=fopen(fname1,'r') ; 
input_data = csvread(fname1);
fclose(fid);

n=8;%TOTAL NO OF CASES
 
i=5;
%for i=2:n
fname=num2str(i,'%0d') ;
euler(fname,input_data);
%end

%code for taking input from user
% prompt = 'input case no(input 0 to quit):';
% x = input(prompt);
% while(x)
%     fname=num2str(X,'%0d') ;
%     Y(i)=euler(fname,input_data);
%     prompt = 'input case no(input 0 to quit):';
%     x = input(prompt);

%end




end
