%clc; clear; close all;
% Define the size of the grid
grid_size = 4;

% Create a matrix to represent the grid world
grid_world = reshape(1:grid_size^2, grid_size, grid_size);

% Define rewards
rewards = zeros(grid_size, grid_size);  % Initialize rewards with zeros

% Set the rewards for specific cells
rewards(2, 3) = -10;  % Cell 7 is a trap with reward -10
rewards(4, 4) = 10;   % Cell 16 is the target with reward +10

% Define the discount factor
gamma = 0.9;

% Initialize the value function
V = zeros(grid_size, grid_size);

% Convergence threshold
epsilon = 1e-6;

% Define the set of possible actions (left, up, right, down)
actions = ["left", "up", "right", "down"];

while true
    delta = 0;
    
    for i = 1:grid_size
        for j = 1:grid_size
            % Skip the target and trap cells
            if (i == 2 && j == 3) || (i == 4 && j == 4)
                continue;
            end
            
            % Store the current value for the state
            v = V(i, j);
            
            % Calculate the expected values for all actions
            expected_values = zeros(1, numel(actions));
            
            for a = 1:numel(actions)
                action = actions(a);
                next_i = i;
                next_j = j;
                
                % Determine the next state after taking the action
                if action == "left"
                    next_j = max(1, j - 1);
                elseif action == "up"
                    next_i = max(1, i - 1);
                elseif action == "right"
                    next_j = min(grid_size, j + 1);
                elseif action == "down"
                    next_i = min(grid_size, i + 1);
                end
                
                % Calculate the expected value using the Bellman equation
                expected_values(a) = rewards(next_i, next_j) + gamma * V(next_i, next_j);
            end
            
            % Update the value function with the maximum expected value
            V(i, j) = max(expected_values);
            
            % Calculate the change in value for this state
            delta = max(delta, abs(v - V(i, j)));
        end
    end
    
    % Check for convergence
    if delta < epsilon
        break;
    end
end

% Display the optimal value function
disp("Optimal Value Function:");
disp(V);

% Initialize the optimal policy
optimal_policy = cell(grid_size, grid_size);

% Calculate the optimal policy based on the optimal value function
for i = 1:grid_size
    for j = 1:grid_size
        if (i == 2 && j == 3) || (i == 4 && j == 4)
            % Skip the target and trap cells
            continue;
        end
        
        % Calculate the expected values for all actions
        expected_values = zeros(1, numel(actions));
        
        for a = 1:numel(actions)
            action = actions(a);
            next_i = i;
            next_j = j;
            
            % Determine the next state after taking the action
            if action == "left"
                next_j = max(1, j - 1);
            elseif action == "up"
                next_i = max(1, i - 1);
            elseif action == "right"
                next_j = min(grid_size, j + 1);
            elseif action == "down"
                next_i = min(grid_size, i + 1);
            end
            
            % Calculate the expected value using the Bellman equation
            expected_values(a) = rewards(next_i, next_j) + gamma * V(next_i, next_j);
        end
        
        % Determine the optimal action for this state
        optimal_action = actions(find(expected_values == max(expected_values), 1));
        
        % Store the optimal action in the policy
        optimal_policy{i, j} = optimal_action;
    end
end

% Display the optimal policy
disp("Optimal Policy:");
disp(optimal_policy);
