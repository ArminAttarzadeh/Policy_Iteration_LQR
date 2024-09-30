% Finding optimal values (v_opt) by solving nonlinear equations
clear; clc; close all;

InitialGuess = randn(2,1);
[OptimalValues, FunctionValues] = fsolve(@mySystem, InitialGuess);

v_star_A=OptimalValues(1)
v_star_B=OptimalValues(2)

function result = mySystem(Variables)

    discountRate = 0.5; 

    OptimalValueA = Variables(1);
    OptimalValueB = Variables(2);

    result = zeros(2,1);

    result(1) = OptimalValueA - max(-1 + discountRate * OptimalValueA, 0 + discountRate * OptimalValueB);
    result(2) = OptimalValueB - (2 + discountRate * OptimalValueA);

end
