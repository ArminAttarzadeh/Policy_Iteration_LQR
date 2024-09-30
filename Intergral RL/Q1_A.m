clc
clear
close all

A = [-1.01887 0.90506 -0.00215; 0.82225 -1.07741 -0.17555; 0 0 -1];
B = [0; 0; 1];

% Define cost matrices
Q = eye(3);
R = 1;

%% Variables
numIterations = 100;
K = [0.5 0.5 0.5];  % Initial feedback gain
tolerance = 1e-6;   % Tolerance for convergence

P_prev_PI = zeros(3, 3); % Initial P for policy iteration

% Arrays to store converged K and P values
K_history = zeros(numIterations, 3);
P_history = zeros(3, 3, numIterations);

%% Perform policy iteration
convergedIteration_PI = -1;  % Initialize to -1 if not converged

for iteration = 1:numIterations
    % Compute the optimal state cost matrix P for policy iteration
    P_PI = sdpvar(3, 3);
    equ_PI = (A - B * K)' * P_PI + P_PI * (A - B * K) + Q + K' * R * K == 0;
    optimize(equ_PI);
    P_sol_PI = value(P_PI);
    
    % Update the feedback gain matrix K for policy iteration
    K_new_PI = inv(R) * B' * P_sol_PI;
    
    % Store converged K and P values
    K_history(iteration, :) = K_new_PI;
    P_history(:, :, iteration) = P_sol_PI;
    
    % Calculate the 2-norm error and store it in the array for policy iteration
    norm2K_PI(iteration) = norm(K_new_PI - K, 2);
    
    % Calculate the 2-norm of the difference between consecutive P matrices for policy iteration
    norm2P_PI(iteration) = norm(P_sol_PI - P_prev_PI, 2);
    
    % Check for convergence in policy iteration
    if norm2K_PI(iteration) < tolerance
        disp(['Policy Iteration Converged at Iteration ', num2str(iteration), '. Stopping iterations.']);
        convergedIteration_PI = iteration;
        break;
    end
    
    % Update K for the next iteration in policy iteration
    K = K_new_PI;
    
    % Store the current P for the next iteration in policy iteration
    P_prev_PI = P_sol_PI;
end

% Trim arrays to store only up to the converged iteration
K_history = K_history(1:convergedIteration_PI, :)
P_history = P_history(:, :, 1:convergedIteration_PI)

% Display final results
disp('Final Feedback Gain (K):');
disp(K_history(end, :));

disp('Final State Cost Matrix (P):');
disp(P_history(:, :, end));
