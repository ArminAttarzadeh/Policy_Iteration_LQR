clc; clear; close all;

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

% Initialize the policy with a random policy
policy = cell(grid_size, grid_size);

% Define the set of possible actions (left, up, right, down)
actions = ["left", "up", "right", "down"];

for i = 1:grid_size
    for j = 1:grid_size
        if (i == 2 && j == 3) || (i == 4 && j == 4)
            % Skip the target and trap cells
            continue;
        end
        policy{i, j} = actions(randi(4));  % Initialize with a random action
    end
end

% Convergence threshold for policy evaluation
epsilon = 1e-6;

while true
    % Policy Evaluation
    V = zeros(grid_size, grid_size);
    while true
        delta = 0;
        for i = 1:grid_size
            for j = 1:grid_size
                if (i == 2 && j == 3) || (i == 4 && j == 4)
                    continue;
                end
                v = V(i, j);
                action = policy{i, j};
                next_i = i;
                next_j = j;
                if action == "left"
                    next_j = max(1, j - 1);
                elseif action == "up"
                    next_i = max(1, i - 1);
                elseif action == "right"
                    next_j = min(grid_size, j + 1);
                elseif action == "down"
                    next_i = min(grid_size, i + 1);
                end
                V(i, j) = rewards(next_i, next_j) + gamma * V(next_i, next_j);
                delta = max(delta, abs(v - V(i, j)));
            end
        end
        if delta < epsilon
            break;
        end
    end
    
    policy_stable = true;
    
    % Policy Improvement
    for i = 1:grid_size
        for j = 1:grid_size
            if (i == 2 && j == 3) || (i == 4 && j == 4)
                continue;
            end
            old_action = policy{i, j};
            
            % Calculate the expected values for all actions
            expected_values = zeros(1, numel(actions));
            for a = 1:numel(actions)
                action = actions(a);
                next_i = i;
                next_j = j;
                if action == "left"
                    next_j = max(1, j - 1);
                elseif action == "up"
                    next_i = max(1, i - 1);
                elseif action == "right"
                    next_j = min(grid_size, j + 1);
                elseif action == "down"
                    next_i = min(grid_size, i + 1);
                end
                expected_values(a) = rewards(next_i, next_j) + gamma * V(next_i, next_j);
            end
            
            % Determine the optimal action for this state
            optimal_action = actions(find(expected_values == max(expected_values), 1));
            
            % Update the policy
            policy{i, j} = optimal_action;
            
            if ~isequal(old_action, optimal_action)
                policy_stable = false;
            end
        end
    end
    
    if policy_stable
        break;  % If the policy is stable, terminate
    end
end

% Display the optimal value function
disp("Optimal Value Function:");
disp(V);

% Display the optimal policy
disp("Optimal Policy:");
disp(policy);
