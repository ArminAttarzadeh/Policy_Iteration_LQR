syms v_x;

gamma = 0.8;
actions = {'Greedy Right', 'Greedy Left','Random'}; % Possible actions

num_iterations = 10;
vx = zeros(1, num_iterations);  % Store the estimated value of state x

% Initialize v(x) with the given random policy


for i = 1:num_iterations
    
    if i==1
        eqn1 = v_x == 1/2*(1/4 * (-3 + gamma * -1) + 3/4 * (4 + gamma * v_x))+1/2*(1/5*(-6+gamma*7)+4/5*(3+gamma*2));
        sol = solve(eqn1, v_x);
        vx(i) = sol;
        
         fprintf('Iteration %d: Selected Policy: %s, Estimated v(x): %.4f\n', i, actions{3}, vx(i));
    else
        % Policy improvement
        action_R_value = 1/4 * (-3 + gamma * -1) + 3/4 * (4 + gamma * vx(i-1));
        action_L_value = 1/5 * (-6 + gamma * 7) + 4/5 * (3 + gamma * 2);
        
        % Determine the maximum value and selected policy
        [max_value, max_index] = max([action_R_value, action_L_value]);
        next_policy = actions{max_index};
        
        if next_policy=='Greedy Right'
            eqn = v_x == 1/4 * (-3 + gamma * -1) + 3/4 * (4 + gamma * v_x);  % Define the equation
            sol = solve(eqn, v_x);  % Solve the equation for vx
            vx(i)=sol;
        elseif next_policy=='Greedy Left'
            vx(i)=vx(i-1); % no update (assume)
        end
            
            
        
        % Display the results for the current iteration
        fprintf('Iteration %d: Selected Policy: %s, Estimated v(x): %.4f\n', i, next_policy, vx(i));
    end
end


disp('===Estimated v(x) for All Iterations:===');
disp(vx(1:num_iterations));
