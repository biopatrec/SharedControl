function test_tl()

% This is used to test the communication to ArduPilot in simulation
% DO NOT USE THIS IN A REAL VEHICLE

t = tcpip('127.0.0.1', 8080, 'NetworkRole', 'Client');
fopen(t)
flushinput(t)

%
% Motors:
%  Move North/South     1,A
%  Turn Right/Left      2,B
%  Move Up/Down         3,C
%  Move East/West       4,D
%

REST    = 0;
PITCH   = 'A';
YAW     = 'B';
THRUST  = 'C';
ROLL    = 'D';

HALT    = 'H';
ARM     = 'T';

% Test the connection
fwrite(t, 'AC');
rsp = char(fread(t, 1));
if(rsp == 'C')
    fprintf('Connection Successful\n');
else
    fprintf('Conn Test Failure\n')
    fprintf('RSP: %d,0x%X\n', rsp, rsp)
    fclose(t)
    return
end

% Wait up to 45 seconds for initialization 
for i = 1:45
    fwrite(t, ['M',0,REST,0,0]);
    rsp = fread(t, 1)
    if(rsp == ARM)
        pause(1)
    elseif(rsp == REST)
        break
    else
        fprintf('Initialization Failure: %d\n', i)
        fprintf('RSP: %d,0x%X\n', rsp, rsp)
        fclose(t)
        return
    end
end
fprintf('Initialization Successful\n');

% Move North
fprintf('Moving North\n');
for i = 1:5
    fwrite(t, ['M',0,PITCH,1,0])
    rsp = fread(t, 1);
    if(isempty(rsp) | rsp ~= PITCH)
        fprintf('Invalid response %d\n', i)
        fprintf('RSP: %d,0x%X\n', rsp, rsp)
        fclose(t)
        return
    end
    pause(1)
end

% Turn east
fprintf('Turning East\n');
for i = 1:5
    fwrite(t, ['M',0,YAW,1,0])
    rsp = fread(t, 1);
    if(isempty(rsp) | rsp ~= YAW)
        fprintf('Invalid response %d\n', i)
        fprintf('RSP: %d,0x%X\n', rsp, rsp)
        fclose(t)
        return
    end
    pause(1)
end

% Move east
fprintf('Moving East\n');
for i = 1:5
    fwrite(t, ['M',0,ROLL,1,0])
    rsp = fread(t, 1);
    if(isempty(rsp) | rsp ~= ROLL)
        fprintf('Invalid response %d\n', i)
        fprintf('RSP: %d,0x%X\n', rsp, rsp)
        fclose(t)
        return
    end
    pause(1)
end

% Move west
fprintf('Moving West\n');
for i = 1:5
    fwrite(t, ['M',0,ROLL,0,0])
    rsp = fread(t, 1);
    if(isempty(rsp) | rsp ~= ROLL)
        fprintf('Invalid response %d\n', i)
        fprintf('RSP: %d,0x%X\n', rsp, rsp)
        fclose(t)
        return
    end
    pause(1)
end

fprintf('Landing\n');
for i = 1:45
    fwrite(t, ['M',0,THRUST,0,0])
    rsp = fread(t, 1);
    if(rsp == 'H')
        fprintf('Vehicle Landed\n')
        break
    elseif(rsp == THRUST)
        pause(1)
    else
        fprintf('Landing failure %d\n')
        fprintf('RSP: %d,0x%X\n', rsp, rsp)
        fclose(t)
        return
    end
end

fclose(t)
fprintf('Success!\n')

end
