function matlabLcmRpc(rpc_chan,extra_data)
if nargin<1
    rpc_chan = 'MATLAB_LCM_RPC';
end
if nargin<2
    extra_data =[];
end

% add the lcm jar file
javaaddpath /usr/local/share/java/lcm.jar

% add all the pods lcmtypes
prefix = pods_path();
jarprefix = [prefix '/share/java/'];
jarfiles = dir([jarprefix '/lcmtypes_*.jar']);
for i= 1:length(jarfiles)
    javaaddpath([jarprefix jarfiles(i).name])
end
lc = lcm.lcm.LCM.getSingleton();


cmd_aggregator = lcm.lcm.MessageAggregator();
ret_ack_aggregator = lcm.lcm.MessageAggregator();

lc.subscribe([rpc_chan '_CMD'], cmd_aggregator);
lc.subscribe([rpc_chan '_RET_ACK'], ret_ack_aggregator);

prev_cmd = [];
while true
    fprintf('waiting for cmd\n')
    while true
        millis_to_wait = 1000;
        msg = cmd_aggregator.getNextMessage(millis_to_wait);
        if ~isempty(msg)
            break
        end
    end
    
    try
        cmd = eigen_utils.matlab_rpc_command_t(msg.data);
    catch ME
        fprintf('WARNING: could not decode msg on channel %s\n',char(msg.channel))
        continue;
    end
    ack = eigen_utils.matlab_rpc_ack_t();
    ack.nonce = cmd.nonce;
    lc.publish([rpc_chan '_CMD_ACK'], ack);
    
    if ~isempty(prev_cmd) && prev_cmd.nonce==cmd.nonce
        fprintf('received duplicate command: %s %d!\n',char(cmd.command), cmd.nonce);
        continue;
    end
    prev_cmd = cmd;
     
    ret = eigen_utils.matlab_rpc_return_t();
    ret.nonce = cmd.nonce;
    ret.error_msg = '';
    
    try
        cmdArgs = cell(cmd.numArgs,1);
        for i=1:cmd.numArgs
            argFlat = typecast(cmd.args(i).data,'double');
            arg = reshape(argFlat,cmd.args(i).rows,cmd.args(i).cols);
            cmdArgs{i} = arg;
        end
        if ~isempty(extra_data)
            cmdArgs{end+1} = extra_data;
        end
        if cmd.numReturnVals<0
            nrets = nargout(char(cmd.command));
            retArgs = cell(1, nrets);
        else
            retArgs = cell(1, cmd.numReturnVals);
        end
        
        [retArgs{:}] = feval(char(cmd.command),cmdArgs{:});
        
        ret.numReturnVals = length(retArgs);
        ret.returnVals = javaArray('eigen_utils.eigen_dense_t', ret.numReturnVals);
        for i = 1:ret.numReturnVals
            edmsg = eigen_utils.eigen_dense_t();
            [edmsg.rows, edmsg.cols] = size(retArgs{i});
            edmsg.type = 'double';
            bytes = typecast(retArgs{i}(:),'uint8');
            edmsg.data_sz = length(bytes);
            edmsg.data = bytes;
            ret.returnVals(i) = edmsg;
        end
        ret.return_status = 1;
        printmsg = sprintf('rpc command: "%s" %d succeeded!',char(cmd.command), cmd.nonce);
    catch ME
        ret.return_status = 0;
        ret.error_msg = ME.message;
        printmsg = sprintf('rpc command %s failed with %d in-args, and %d out-args\n ',char(cmd.command), cmd. numArgs, cmd.numReturnVals);
    end
    
    disp(printmsg);
    while true
        lc.publish([rpc_chan '_RETURN'],ret)
        
        millis_to_wait = 100;
        msg = ret_ack_aggregator.getNextMessage(millis_to_wait);
        if isempty(msg)
            continue;
        end
        
        ret_ack =eigen_utils.matlab_rpc_ack_t(msg.data);
        if (ret_ack.nonce == cmd.nonce)
            break;
        end
    end
    
end