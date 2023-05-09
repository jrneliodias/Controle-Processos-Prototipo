function [J, MVI] = JMVI(y_real,y_model)
% Cálculo de J
J = sum((y_real-y_model).^2)

% Cálculo do MVI (Model Validation Index)
MVI = 1-abs(y_model'-y_real')/abs(y_real')

end