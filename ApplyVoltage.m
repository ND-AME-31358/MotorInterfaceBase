function output_data = ApplyVoltage(voltage)
    if nargin == 0
        % Set voltage to 3V if run directly
        voltage = 3;
    end
    
    figure(1);  clf;       % Create an empty figure to update later
    subplot(411)
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Position (rad)');
    
    subplot(412)
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity (rad/s)');
    
    subplot(413)
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Voltage (V)');
    
    subplot(414)
    h4 = plot([0],[0]);
    h4.XData = []; h4.YData = [];
    ylabel('Current (A)');
    
    % This function will get called any time there is new data from
    % the FRDM board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        t = new_data(:,1);   % time
        angle = new_data(:,2); % position
        vel = new_data(:,3); % velocity
        voltage = new_data(:,4); % voltage
        current = new_data(:,5); % current
        N = length(angle);
        
        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = angle;
        h2.XData(end+1:end+N) = t;   % Update subplot 2
        h2.YData(end+1:end+N) = vel;
        h3.XData(end+1:end+N) = t;   % Update subplot 3
        h3.YData(end+1:end+N) = voltage;
        h4.XData(end+1:end+N) = t;   % Update subplot 4
        h4.YData(end+1:end+N) = current;
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout

    ExperimentTime = 5;             % in seconds
    % Pack experiment parameters
    input = [voltage, ExperimentTime];
    output_size = 5;                % time, current, angle
   
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    
end



