% Assume you have the final feedback gain stored in the variable K_history
finalFeedbackGain = [-0.1352   -0.1501    0.4329];

% Define the simulation time
tspan = 0:0.01:10; % time vector from 0 to 10 seconds with a step of 0.01 seconds

% Define the initial state
x0 = [10; -10; -3]; % Initial state vector

% Function to represent the system dynamics (x'(t) = Ax(t) + Bu(t))
systemDynamics = @(t, x) (A - B * finalFeedbackGain) * x;

% Solve the system of ODEs using ode45
[t, x] = ode45(systemDynamics, tspan, x0);

% Plot the state response
figure;
plot(t, x);
title('State Response Over Time');
xlabel('Time (seconds)');
ylabel('State Variables');
legend('x1', 'x2', 'x3');
grid on;
