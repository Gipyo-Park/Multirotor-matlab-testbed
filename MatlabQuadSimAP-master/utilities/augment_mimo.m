function [A_aug, B_aug, C_aug] = augment_mimo(Ad, Bd, Cd, num_of_states, num_of_outputs)

    A_aug = [Ad, zeros(num_of_states, num_of_outputs); 
             Cd*Ad, eye(num_of_outputs)];
    B_aug = [Bd; 
             Cd*Bd];
    C_aug = [zeros(num_of_outputs, num_of_states), eye(num_of_outputs)];

end