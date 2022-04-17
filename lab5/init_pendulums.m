function init_pendulums(arg)

% Task scheduling and control.
%
% This example extends the simple PID control example (located in
% $DIR/examples/servo) to the case of three PID-tasks running
% concurrently on the same CPU controlling three different servo
% systems. The effect of the scheduling policy on the global control
% performance is demonstrated.

% Initialize TrueTime kernel
start_index = arg.start_index;
end_index = arg.end_index;

schedule_algo_num = arg.schedule_algo;

switch schedule_algo_num
 case 1   
  ttInitKernel('prioFP')
 case 2     
  ttInitKernel('prioDM')
 case 3
  ttInitKernel('prioEDF')
 otherwise
  error('Illegal init argument')
end

% Task parameters
starttimes = [0 0 0];
tasknames = {'cartA', 'cartB', 'cartC'};

% Create the three tasks
for i = start_index:end_index
    pend_data = arg.pend_data(i);
    arg.pend_idx = i;    
    ttCreatePeriodicTask(tasknames{i}, starttimes(i), pend_data.period, 'control_func', arg);

    % Set priority (val, name)
%     ttSetPriority(1/pend_data.period, tasknames{i});
% TODO: Priority C, B, A (highest C)
%     ttSetPriority(4-i, tasknames{i});    
    ttSetPriority(i, tasknames{i});
end

ttCreateHandler('dl_miss_handler', 1, 'hdlcode')
for i = start_index:end_index
  ttAttachDLHandler(tasknames{i}, 'dl_miss_handler')
end

