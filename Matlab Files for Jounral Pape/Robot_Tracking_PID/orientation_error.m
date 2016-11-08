function [ output_args ] = orientation_error( input_args )
%ORIENTATION_ERROR Summary of this function goes here
%   Detailed explanation goes here

x_error=input_args(1);
y_error=input_args(2);
orientation_actual=input_args(3);

orientation_desired=atan2(y_error,x_error);

orientation_error=orientation_desired-orientation_actual;

if orientation_error>pi
    orientation_error=orientation_error-pi;
end

output_args=orientation_error;


end

