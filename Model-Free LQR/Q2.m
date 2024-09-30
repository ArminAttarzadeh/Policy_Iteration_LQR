clear
clc
close all
%% define system and costs

A = [0.9065 0.0816 -0.0005; 0.0743 0.9012 -0.0007; 0 0 0.1327];
B = [-0.0027; -0.0068; 1];

% Define cost matrices
Q = eye(3);  % State cost
R = 1;       % Control cost

% Set the number of iterations
numIterations = 100;
tolerance = 1e-5;

n=size(A,1);
K=zeros(numIterations , n); K(1,:)=place(A,B,[0.3 0.4 0.8]);
P=cell(numIterations); P{1}=zeros(n);

M=120; %number of sampling data

E = eye(n);
S = zeros(n , 1) ;
[P_lqr , K_lqr , L] = idare(A , B , Q , R , S , E);
%% Policy Iteration

for i =1:numIterations
    
    PHI=[];
    SAI=[];
    for j=1:M
        xk=randn(n,1);
        uk=-K(i , :)*xk + 0.01*randn; %noise probe
        xk_next=A*xk+B*uk;
        PHI=[PHI ; ComputeXbar(xk)-ComputeXbar(xk_next)]; %#ok
        SAI=[SAI ; xk'*Q*xk+uk'*R*uk]; %#ok
    end
    
    Pbar=PHI\SAI;
    P{i+1}=ConvertPbar2P(Pbar);
    %policy improvment
    K(i+1 , :)=inv(R+B'*P{i+1}*B)*(B'*P{i+1}*A);
    
    disp(['Iteration(' num2str(i) ')']);
    
    if norm(K(i+1,:)-K(i,:)) < tolerance
        break;
    end
end

disp(['K LQR = ' num2str(K_lqr)]);
disp(['K PI = ' num2str(K(i+1 , :))]);
disp(['P_lqr']);
disp(P_lqr);
disp(['P PI']);disp(ConvertPbar2P(Pbar));


%% functions
function Xbar = ComputeXbar(X)
    X=X(:)';
    
    Xbar=[];
    for i = 1:numel(X)
        Xbar=[Xbar X(i)*X(i:end)];
        
    end
end

function P=ConvertPbar2P(Pbar)
P=[Pbar(1) Pbar(2)/2 Pbar(3)/2 
    Pbar(2)/2 Pbar(4) Pbar(5)/2
    Pbar(3)/2 Pbar(5)/2 Pbar(6)];
end