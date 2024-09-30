clc
clear
close all

%% Define System 
A = [0.9065 0.0816 -0.0005; 0.0743 0.9012 -0.0007; 0 0 0.1327];
B = [-0.0027; -0.0068; 1];

% Define cost matrices
Q = eye(3);
R = 1;      

%% Variables 
numIterations = 100;
K = [0.5 0.5 0.5];  % Initial feedback gain 
tolerance = 1e-5;   % Tolerance for convergence

% Initialize arrays to store norms
norm2K_PI = zeros(numIterations, 1);
norm2P_PI = zeros(numIterations, 1);
norm2K_VI = zeros(numIterations, 1);
norm2P_VI = zeros(numIterations, 1);
norm2K_L_PI1 = zeros(numIterations, 1);
norm2P_L_PI1 = zeros(numIterations, 1);
norm2K_L_PI2 = zeros(numIterations, 1);
norm2P_L_PI2 = zeros(numIterations, 1);

P_prev_PI = zeros(3, 3); % Initial P for policy iteration
P_prev_VI = zeros(3, 3); % Initial P for value iteration
P_prev_L_PI1 = zeros(3, 3); % Initial P for L_PI1
P_prev_L_PI2 = zeros(3, 3); % Initial P for L_PI2

%% Perform policy iteration
convergedIteration_PI = -1;  % Initialize to -1 if not converged

for iteration = 1:numIterations
    % Compute the optimal state cost matrix P for policy iteration
    P_PI = sdpvar(3, 3);
    equ_PI = (A - B * K)' * P_PI * (A - B * K) - P_PI + Q + K' * R * K == 0;
    optimize(equ_PI);
    P_sol_PI = value(P_PI);
    
    % Update the feedback gain matrix K for policy iteration
    K_new_PI = inv(R + B' * P_sol_PI * B) * B' * P_sol_PI * A;
    
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

%% Perform Value Iteration
convergedIteration_VI = 0;

% Reset variables for value iteration
K = [0.5 0.5 0.5];  % Initial feedback gain 
P_prev_VI = zeros(3, 3); % Initial P for value iteration

for iteration = 1:numIterations
    % Compute the optimal state cost matrix P for value iteration
    P_VI = sdpvar(3, 3);
    equ_VI = (A - B * K)' * P_prev_VI * (A - B * K)  + Q + K' * R * K - P_VI == 0;
    optimize(equ_VI);
    P_sol_VI = value(P_VI);
    
    % Update the feedback gain matrix K for value iteration
    K_new_VI = inv(R + B' * P_sol_VI * B) * B' * P_sol_VI * A;
    
    % Calculate the 2-norm error and store it in the array for value iteration
    norm2K_VI(iteration) = norm(K_new_VI - K, 2);
    
    % Calculate the 2-norm of the difference between consecutive P matrices for value iteration
    norm2P_VI(iteration) = norm(P_sol_VI - P_prev_VI, 2);
    
    % Check for convergence in value iteration
    if norm2K_VI(iteration) < tolerance
        disp(['Value Iteration Converged at Iteration ', num2str(iteration), '. Stopping iterations.']);
   
        break;
    end
    convergedIteration_VI = convergedIteration_VI + 1;
    
    % Update K for the next iteration in value iteration
    K = K_new_VI;
    
    % Store the current P for the next iteration in value iteration
    P_prev_VI = P_sol_VI;
end

%% Perform L_PI1
lambda_L_PI1 = 0.5;
convergedIteration_L_PI1 = 0;

% Reset variables for L_PI1
K = [0.5 0.5 0.5];  % Initial feedback gain 
P_prev_L_PI1 = zeros(3, 3); % Initial P for L_PI1

for iteration = 1:numIterations
    % Compute the optimal state cost matrix P for L_PI1
    P_L_PI1 = sdpvar(3, 3);
    equ_L_PI1 = (1 - lambda_L_PI1) * ((A - B * K)' * P_prev_L_PI1 * (A - B * K)  + Q + K' * R * K - P_L_PI1) + (lambda_L_PI1) * ((A - B * K)' * P_L_PI1 * (A - B * K) - P_L_PI1 + Q + K' * R * K) == 0;
    optimize(equ_L_PI1);
    P_sol_L_PI1 = value(P_L_PI1);
    
    % Update the feedback gain matrix K for L_PI1
    K_new_L_PI1 = inv(R + B' * P_sol_L_PI1 * B) * B' * P_sol_L_PI1 * A;
    
    % Calculate the 2-norm error and store it in the array for L_PI1
    norm2K_L_PI1(iteration) = norm(K_new_L_PI1 - K, 2);
    
    % Calculate the 2-norm of the difference between consecutive P matrices for L_PI1
    norm2P_L_PI1(iteration) = norm(P_sol_L_PI1 - P_prev_L_PI1, 2);
    
    % Check for convergence in L_PI1
    if norm2K_L_PI1(iteration) < tolerance
        disp(['L_PI1 Converged at Iteration ', num2str(iteration), '. Stopping iterations.']);
        convergedIteration_L_PI1 = iteration;
        break;
    end
    
    % Update K for the next iteration in L_PI1
    K = K_new_L_PI1;
    
    % Store the current P for the next iteration in L_PI1
    P_prev_L_PI1 = P_sol_L_PI1;
end

%% Perform L_PI2
lambda_L_PI2 = 0.9;
convergedIteration_L_PI2 = 0;

% Reset variables for L_PI2
K = [0.5 0.5 0.5];  % Initial feedback gain 
P_prev_L_PI2 = zeros(3, 3); % Initial P for L_PI2

for iteration = 1:numIterations
    % Compute the optimal state cost matrix P for L_PI2
    P_L_PI2 = sdpvar(3, 3);
    equ_L_PI2 = (1 - lambda_L_PI2) * ((A - B * K)' * P_prev_L_PI2 * (A - B * K)  + Q + K' * R * K - P_L_PI2) + (lambda_L_PI2) * ((A - B * K)' * P_L_PI2 * (A - B * K) - P_L_PI2 + Q + K' * R * K) == 0;
    optimize(equ_L_PI2);
    P_sol_L_PI2 = value(P_L_PI2);
    
    % Update the feedback gain matrix K for L_PI2
    K_new_L_PI2 = inv(R + B' * P_sol_L_PI2 * B) * B' * P_sol_L_PI2 * A;
    
    % Calculate the 2-norm error and store it in the array for L_PI2
    norm2K_L_PI2(iteration) = norm(K_new_L_PI2 - K, 2);
    
    % Calculate the 2-norm of the difference between consecutive P matrices for L_PI2
    norm2P_L_PI2(iteration) = norm(P_sol_L_PI2 - P_prev_L_PI2, 2);
    
    % Check for convergence in L_PI2
    if norm2K_L_PI2(iteration) < tolerance
        disp(['L_PI2 Converged at Iteration ', num2str(iteration), '. Stopping iterations.']);
        convergedIteration_L_PI2 = iteration;
        break;
    end
    
    % Update K for the next iteration in L_PI2
    K = K_new_L_PI2;
    
    % Store the current P for the next iteration in L_PI2
    P_prev_L_PI2 = P_sol_L_PI2;
end

%% Compare Norm Errors
disp(['Policy Iteration Converged at Iteration ', num2str(convergedIteration_PI)]);
disp(['Value Iteration Converged at Iteration ', num2str(convergedIteration_VI)]);
disp(['L_PI 0.5 Converged at Iteration ', num2str(convergedIteration_L_PI1)]);
disp(['L_PI 0.8 Converged at Iteration ', num2str(convergedIteration_L_PI2)]);

% Plot the 2-norm of K for all methods
figure;

subplot(4, 1, 1);
plot(1:convergedIteration_PI, norm2K_PI(1:convergedIteration_PI), '-o', 'LineWidth', 1.5, 'DisplayName', 'Policy Iteration');
ylabel('2-norm error of K');
xlabel('Iteration');
title('Convergence Plot for Policy Iteration');
legend('show');
grid on;

subplot(4, 1, 2);
plot(1:convergedIteration_VI, norm2K_VI(1:convergedIteration_VI), '-o', 'LineWidth', 1.5, 'DisplayName', 'Value Iteration');
ylabel('2-norm error of K');
xlabel('Iteration');
title('Convergence Plot for Value Iteration');
legend('show');
grid on;

subplot(4, 1, 3);
plot(1:convergedIteration_L_PI1, norm2K_L_PI1(1:convergedIteration_L_PI1), '-o', 'LineWidth', 1.5, 'DisplayName', 'PI \lambda=0.5');
ylabel('2-norm error of K');
xlabel('Iteration');
title('Convergence Plot for PI \lambda=0.5');
legend('show');
grid on;

subplot(4, 1, 4);
plot(1:convergedIteration_L_PI2, norm2K_L_PI2(1:convergedIteration_L_PI2), '-o', 'LineWidth', 1.5, 'DisplayName', 'PI \lambda=0.9');
ylabel('2-norm error of K');
xlabel('Iteration');
title('Convergence Plot for PI \lambda=0.9');
legend('show');
grid on;

% Plot the 2-norm of P for all methods
figure;

subplot(4, 1, 1);
plot(1:convergedIteration_PI, norm2P_PI(1:convergedIteration_PI), '-o', 'LineWidth', 1.5, 'DisplayName', 'Policy Iteration');
ylabel('2-norm of norm2P');
xlabel('Iteration');
title('Norm of Consecutive P Matrices for Policy Iteration');
legend('show');
grid on;

subplot(4, 1, 2);
plot(1:convergedIteration_VI, norm2P_VI(1:convergedIteration_VI), '-o', 'LineWidth', 1.5, 'DisplayName', 'Value Iteration');
ylabel('2-norm of norm2P');
xlabel('Iteration');
title('Norm of Consecutive P Matrices for Value Iteration');
legend('show');
grid on;

subplot(4, 1, 3);
plot(1:convergedIteration_L_PI1, norm2P_L_PI1(1:convergedIteration_L_PI1), '-o', 'LineWidth', 1.5, 'DisplayName', 'PI \lambda=0.5');
ylabel('2-norm of norm2P');
xlabel('Iteration');
title('Norm of Consecutive P Matrices for PI \lambda=0.5');
legend('show');
grid on;

subplot(4, 1, 4);
plot(1:convergedIteration_L_PI2, norm2P_L_PI2(1:convergedIteration_L_PI2), '-o', 'LineWidth', 1.5, 'DisplayName', 'PI \lambda=0.9');
ylabel('2-norm of norm2P');
xlabel('Iteration');
title('Norm of Consecutive P Matrices for PI \lambda=0.9');
legend('show');
grid on;
