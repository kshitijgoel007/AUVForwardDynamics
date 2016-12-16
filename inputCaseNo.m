
  function inputCaseNo()
% Input function for specifying which maneuver to perform
% CASE1:DEL_R_ORDERED=10DEGREE
% CASE2:DEL_R_ORDERED=15DEGREE
% CASE3:ZIG-ZAG MOTION: DEL_R_ORDERED=10DEGREE,WHEN PSI=10DEGREE
%        DEL_R_ORDERED=-10DEGREE
% CASE4:SAME AS CASE 3 WITH DEL_R_ORDERED=-10DEGREE INTIALLY
% CASE5:AKA CASE3:DEL_R_ORDERED=20DEGREE
% CASE6:AKA CASE4_2:DEL_R_ORDERED=-20DEGREE
% CASE7:zig zag pitch motion in vertical plane
% 
%
% INPUT : An integer specifying case number to be run
% OUTPUT : Trajectory of bot
% function inputCasenotoRun() 

% time for which the simultion runs
  timespan=0:.1:50;

% OPENING FILE CONTAINING BOT COORDINATES IN BODY FRAME
  fname1=['Bot_coordiantes','.csv'] ;
  fid=fopen(fname1,'r') ; 
  botCoordinatesInBodyFrame = csvread(fname1);
  fclose(fid);

% Code for taking input from user
  prompt = 'input case no(input 0 to quit):';
  x = input(prompt);
  while(x)
    fname=num2str(x,'%0d') ;
  % Calling euler function
    Y=euler(fname,botCoordinatesInBodyFrame,timespan);
    x=input(prompt);
  end

end
