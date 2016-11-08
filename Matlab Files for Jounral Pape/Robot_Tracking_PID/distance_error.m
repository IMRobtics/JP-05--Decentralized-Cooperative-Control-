function [ output_args ] = distance_error( input_args )
%DISTANCE_ERROR Summary of this function goes here
%   Detailed explanation goes here
x_error=input_args(1);
y_error=input_args(2);


output_args=sqrt(x_error^2+y_error^2);

if output_args<1e-2
    output_args=0;
end

    
end

