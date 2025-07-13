function checkCoppeliaSimConnection(host, port, timeout)
% Check if CoppeliaSim can be connected via ZeroMQ.
%
% Parameters:
%   host    - IP address of CoppeliaSim (string, e.g., '127.0.0.1').
%   port    - Port number CoppeliaSim is listening on (integer, e.g., 23000).
%   timeout - Timeout duration for connection in seconds (numeric, e.g., 5).
%
% Returns:
%   can_connected - Boolean value (true if connected, false otherwise).
    try
        % Attempt to establish a TCP connection
        TCP_Client = tcpclient(host, port, "Timeout", timeout);
        
        % Log success and connection details
        fprintf('[LOG INFO] -- CoppeliaSim Can be Connected\n');
        fprintf('[LOG INFO] -- CoppeliaSim IP Address %s\n', TCP_Client.Address);
        
        % Clean up and set return value
        clear TCP_Client;
    catch
        % Log error and set return value
        fprintf('[WARNING] -- CONNECTION FAILED -- Ensure CoppeliaSim is opened & Port is correct\n'); 
    end
end
