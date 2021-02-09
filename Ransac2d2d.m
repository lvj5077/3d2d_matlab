function [x_out] = Ransac2d2d(d1,d2)
%%
x = 0;
x_out = 0;
maxInlierNum = 0;

for i = 1:400
    x = d2(i)-d1(i);
    error = vecnorm(d1+x - d2);
    inlierNum = length(find(error<1));
    if (inlierNum > maxInlierNum)
        maxInlierNum = inlierNum;
        x_out = x;
    end
end
% minMeanError
%%
end

