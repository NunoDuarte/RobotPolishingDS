function [F3, F2] = preprocess_data(data, varargin)

    if nargin > 2
        plotting = varargin{1};
    else
        plotting = 2;
    end

    % standard size
    size = [1, 200];

    for i=1:length(data)
        if ~isempty(data{i})

            b1 = data{i}(1,:);
            b2 = data{i}(2,:);
            b3 = data{i}(3,:);

            dataFy = imresize(b1, size);
            dataFx = imresize(b2, size);       
            dataFz = imresize(b3, size);      

            F3{i} = [dataFx; dataFy; dataFz];

            % reduce dimension
            z1 = b1;
            z2 = sqrt(b2.^2 + b3.^2);

            dataFy = imresize(z1, size); 
            dataFz = imresize(z2, size);  
            
            F2{i} = [dataFy; dataFz];

        end 
    end
    
    %% plot
    if plotting == 2
        for i=1:length(F2)
            figure()
            plot(F2{i}(1,:), F2{i}(2,:), '.');
        end

    elseif plotting == 3
        for i=1:length(F3)
            figure()
            plot3(F3{i}(1,:), F3{i}(2,:), F3{i}(3,:), '.');
        end
    end

end