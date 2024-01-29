function X = care_sda(A, B, H, R)
% Structure-preserving Doubling Algorithm
n_controller_state = length(A);

gamma = 2.4; % parameter in the Cayley transform, SDA's author suggested the value between 2.1~2.6
I = eye(n_controller_state);
G = B*inv(R)*transpose(B);
% A_transpose = transpose(A);
A_gamma = A - (gamma*I);

iteration_times = 0;

%solve CARE with SDA
% step 1
A_hat_last = I + 2*gamma*inv(A_gamma + G*inv(transpose(A_gamma))*H);
G_hat_last = 2*gamma*inv(A_gamma)*G*inv(transpose(A_gamma) + H*inv(A_gamma)*G);
H_hat_last = 2*gamma*inv(transpose(A_gamma) + H*inv(A_gamma)*G)*H*inv(A_gamma);

while 1
    % do until convergence
    iteration_times = iteration_times + 1;
    
    %reduce redundent calculation by pre-calculating repeated terms
    inv_I_H_G = inv(I + (H_hat_last * G_hat_last));
    transpose_A_hat_last = transpose(A_hat_last);
    
    %update
    A_hat_new = A_hat_last * inv(I + G_hat_last * H_hat_last) * A_hat_last;
    G_hat_new = G_hat_last + (A_hat_last * G_hat_last * inv_I_H_G * transpose_A_hat_last);
    H_hat_new = H_hat_last + (transpose_A_hat_last * inv_I_H_G * H_hat_last * A_hat_last);
    
    %matrix norms
    norm_H_last = norm(H_hat_last);
    norm_H_now = norm(H_hat_new);
    
    %prepare next iteration
    A_hat_last = A_hat_new;
    G_hat_last = G_hat_new;
    H_hat_last = H_hat_new;
    
    %stop iteration if converged
    if abs(norm_H_now - norm_H_last) < 0.000000001 %* abs(norm_H_now)
        break;
    end
    
    %disp(iteration_times);
end

X = H_hat_new; % set X <- H_hat_new
end