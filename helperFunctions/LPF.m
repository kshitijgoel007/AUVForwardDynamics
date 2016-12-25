function [ Output ] = LPF( Input, PrevOutput, tinc, f2 )
%LPF is a low pass filter
%   Input - Current input
%   PrevOutput - previous filter output.
%   tinc - step size
%   f2
Output = ( tinc/(1+f2*tinc) )*Input + PrevOutput*( 1/(1+f2*tinc) );


end

