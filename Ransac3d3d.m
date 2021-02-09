function [R_out,t_out] = Ransac3d3d(obj_input_fix,obj_input_mv)
%%
R_out = zeros(3,3);
t_out = zeros(3,1);
maxInlierNum = 0;
minMeanError = 0;
for i = 1:300
    idx = randperm(length(obj_input_fix),8);
    fixhp = obj_input_fix(idx,:);
    mvhp = obj_input_mv(idx,:);
    fix_ctr = mean(fixhp);
    mv_ctr = mean(mvhp);

    q1 = fixhp - fix_ctr;
    q2 = mvhp - mv_ctr;
    W = zeros(3,3);
    for j = 1:length(fixhp)
        W = W+q1(j,:)'.*q2(j,:);
    end
    [ U , S , V ] = svd( W );
    R = U* ( V');
    t = fix_ctr' - R*mv_ctr';
    
    error = vecnorm((obj_input_fix' - (R*obj_input_mv'+t)),2);
    inlierNum = length(find(error<0.02));
    meanError = mean(error(error<0.02));
    if (inlierNum > maxInlierNum)
        maxInlierNum = inlierNum;
%         if(inlierNum == maxInlierNum)
%             if ( minMeanError > meanError )
%                 minMeanError = meanError;
%                 R_out = R;
%                 t_out = t;
%             end
%         else
%             R_out = R;
%             t_out = t;
%             minMeanError = meanError;
%         end
        R_out = R;
        t_out = t;
    end

end
inlierRatio = maxInlierNum/length(obj_input_fix);
% minMeanError
%%
end

