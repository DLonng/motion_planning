function x_rand = random_sample(p_goal, goal)
    p = unifrnd(0, 1);
    if p < p_goal
        x_rand = goal;
    else
        x_rand = [unifrnd(0, goal(1)), unifrnd(0, goal(2))];
    end
end