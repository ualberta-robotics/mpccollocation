filename = 't';

for i=1:10   
    
        [demos{i}.pos] = csvimport([filename, num2str(i),  ... 
            '/_slash_my_gen3_slash_base_feedback.csv'], ... 
                                 'columns', {'thetax', 'thetay', 'thetaz'})';
                             
        [posdemos{i}.pos] = csvimport([filename,num2str(i), '/_slash_my_gen3_slash_base_feedback.csv'], ... 
                                 'columns', {'thetax', 'thetay', 'thetaz'})';                    
        [veldemos{i}.vel] = csvimport([filename,num2str(i), '/_slash_my_gen3_slash_base_feedback.csv'], ... 
                                 'columns', {'thetax', 'thetay', 'thetaz'})';                   
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
    knots = knots(:,1:90);
    tt = t(:,1:skip:end);
    tt = tt(:,1:90);
    ppx = spline(tt,knots);
    
    % % and get first derivative
    ppxd = differentiate_spline(ppx);
    pos = ppval(ppx,tt);
    vel = ppval(ppxd,tt);
    
    posdemos{i}.pos = pos;
    veldemos{i}.vel = vel;
   
end

save z_traj_kinova_obj2
