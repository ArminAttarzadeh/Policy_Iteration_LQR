clear
clc
close all

A = [-1.01887 0.90506 -0.00215; 0.82225 -1.07741 -0.17555; 0 0 -1];
B2 = [0; 0; 1];
B1 = [1; 0; 0];

Q = eye(3); R = 1; beta = 6;

% Set the number of iterations
numIterations =20;
tolerance = 1e-5;

n = size(A, 1); % order
K1 = zeros(numIterations, n);
K2 = zeros(numIterations, n);

% target policy
K1(1, :) = [0.5 0.5 0.5];
K2(1, :) = [0 0 0];

% behavior policy
K1b = K1(1, :);
K2b = K2(1, :);

Time = 10;
dt = 0.001; % sampling time
N = Time / dt; % number of samples for integral


integral_reward = 0;
integral_xu = [0 0 0];
integral_xd = [0 0 0];

P = zeros(3, 3, numIterations+1);
P(:,:,1)=zeros(3,3);

% Policy Iteration
for j = 1:numIterations
    
    x = zeros(3, N);
    x(:, 1) = [10; -10; -3];
    PHI = [];
    SAI = [];
    for t = 1:N
        ub = -K1b * x(:, t) + 0.01 * randn;
        db = K2b * x(:, t);
        
        u(j) = -K1(j, :) * x(:, t);
        d(j) = K2(j, :) * x(:, t);
        
        integral_reward = integral_reward + dt * (-x(:, t)' * Q * x(:, t) - ub' * R * ub + beta^2 * db' * db);
        e1 = ub - u(j);
        e2 = db - d(j);
        
        integral_xu = integral_xu + dt * (kron(x(:, t), e1)');
        integral_xd = integral_xd + dt * (kron(x(:, t), e2)');
        
        x(:, t + 1) = x(:, t) + dt * (A * x(:, t) + B2 * ub + B1 * db);
       
    end
    PHI = [PHI; integral_reward];
    SAI = [SAI; [ComputeXbar(x(:, 1))' - ComputeXbar(x(:, Time))', 2 * integral_xu * kron(eye(n), R), 2 * beta^2 * integral_xd]];
    
    z = SAI / PHI;
    Pbar = [z(1) z(2)/2 z(3)/2 z(4)/2 z(5)/2 z(6)];
    P(:,:,j+1) = ConvertPbar2P(Pbar);
    K1(j+1, :) = [z(7) z(8) z(9)];
    K2(j+1, :) = [z(10) z(11) z(12)];
    
    if norm(P(:,:,j+1)-P(:,:,j)) < tolerance
        disp(['Converged at Iteration ', num2str(j), '. Stopping iterations.']);
        break;
    end
end

disp('solution for H_infiinty (P):');
disp(P(:,:,numIterations));

disp('Final Gain:');
disp(K1(numIterations,:));



function Xbar = ComputeXbar(X)
    x1 = X(1); x2 = X(2); x3 = X(3);
    Xbar = [x1 * x1, x1 * x2, x1 * x3, x2 * x2, x2 * x3, x3 * x3]';
end

function P = ConvertPbar2P(Pbar)
    P = [Pbar(1) Pbar(2)/2 Pbar(3)/2;
         Pbar(2)/2 Pbar(4) Pbar(5)/2;
         Pbar(3)/2 Pbar(5)/2 Pbar(6)];
end
