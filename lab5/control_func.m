function [exectime, data] = control_func(seg, data)

switch seg
    case 1
        % Get pend idx
        pend_idx = data.pend_idx;


        % PEND A
        % extract states
        x1 = ttAnalogIn( (pend_idx-1)*4 + 1 ); % Read signal
        x2 = ttAnalogIn( (pend_idx-1)*4 + 2 ); % Read signal
        x3 = ttAnalogIn( (pend_idx-1)*4 + 3 ); % Read signal
        x4 = ttAnalogIn( (pend_idx-1)*4 + 4 ); % Read signal
        
        x0 = [x1; x2; x3; x4];
        
        % Get args
        F = data.pend_data(pend_idx).F;
        
        % apply & exec time
        data.u = -F * x0;

        exectime = data.pend_data(pend_idx).exec_time;
    case 2
        ttAnalogOut(data.pend_idx, data.u); % Output control signal
        exectime = -1;
end
