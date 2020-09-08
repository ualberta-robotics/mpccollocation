filename = 't';

for i=1:7   
    
        [demos{i}.pos] = csvimport([filename, num2str(i),  ... 
            '/_slash_zeus_slash_wam_slash_pose.csv'], ... 
                                 'columns', {'tool_pose_x', 'tool_pose_y', 'tool_pose_z'})';
                             
        [posdemos{i}.pos] = csvimport([filename,num2str(i), '/_slash_zeus_slash_wam_slash_pose.csv'], ... 
                                 'columns', {'tool_pose_x', 'tool_pose_y', 'tool_pose_z'})';                    
        [veldemos{i}.vel] = csvimport([filename,num2str(i), '/_slash_zeus_slash_wam_slash_pose.csv'], ... 
                                 'columns', {'tool_pose_x', 'tool_pose_y', 'tool_pose_z'})';                     
    nb_knots = 20;
    nb_clean_data_per_demo = 100;

    data = demos{i}.pos;

    t = [1:size((demos{i}.pos),2)]*0.05;
    
    if(i>1)
        if(t > (size((demos{i-1}.pos),2)))
            t = size((demos{i-1}.pos),2);
        end
    end

    nb_data = size(data,2);
    skip = round(nb_data/100);
    knots = data(:,1:skip:end);
    knots = knots(:,1:98);
    tt = t(:,1:skip:end);
    tt = tt(:,1:98);
    ppx = spline(tt,knots);
    
   
    % % and get first derivative
    ppxd = differentiate_spline(ppx);
    pos = ppval(ppx,tt);
    vel = ppval(ppxd,tt);
    
    
    posdemos{i}.pos = pos;
    veldemos{i}.vel = vel;
   
end

save new_traj
