% Init
clc
clear

% Connect to arduino
a = arduino('COM10', 'Uno');

%% Let the motor turn left
writePWMDutyCycle(a,'D3', 0.25)
writeDigitalPin(a,'D2',0)
writeDigitalPin(a,'D4',1)
pause(1)

% Increase speed
writePWMDutyCycle(a,'D3', 0.5)
pause(1)

% Increase speed
writePWMDutyCycle(a,'D3', 0.75)
pause(1)

% Increase speed
writePWMDutyCycle(a,'D3', 1.0)
pause(1)

%% Let the motor turn right
writePWMDutyCycle(a,'D3', 0.25)
writeDigitalPin(a,'D2',1)
writeDigitalPin(a,'D4',0)
pause(1)

% Increase speed
writePWMDutyCycle(a,'D3', 0.5)
pause(1)

% Increase speed
writePWMDutyCycle(a,'D3', 0.75)
pause(1)

% Increase speed
writePWMDutyCycle(a,'D3', 1.0)
pause(1)

%% Stop motor
writeDigitalPin(a,'D2',0)
writeDigitalPin(a,'D4',0)



