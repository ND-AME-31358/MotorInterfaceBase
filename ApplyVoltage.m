function output_data = ApplyVoltage(voltage)
    if nargin == 0
        % Set voltage to 3V if run directly
        voltage = 3;
    end
    
    figure(1);  clf;       % Create an empty figure to update later
    subplot(211)
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Current (A)');
    subplot(212)
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Angle (deg)');
    
    % This function will get called any time there is new data from
    % the FRDM board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        t = new_data(:,1);          % time
        current = new_data(:,2);    % current
        angle = new_data(:,3);      % angle
        N = length(current);
        
        h1.XData(end+1:end+N) = t;  % Update subplot 1
        h1.YData(end+1:end+N) = current;
        h2.XData(end+1:end+N) = t;  % Update subplot 2
        h2.YData(end+1:end+N) = angle;
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout

    ExperimentTime = 5;             % in seconds
    % Pack experiment parameters
    input = [voltage, ExperimentTime];
    output_size = 3;                % time, current, angle
   
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    
end



