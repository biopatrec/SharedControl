function GUI_TestMotorControl
    f = figure('Visible', 'off', 'Position', [680, 560, 430, 190]);
    
    handles.obj = tcpip('127.0.0.1', 8080, 'NetworkRole', 'Client');
    
    statusLabel = uicontrol('Style', 'text', ...
        'String', 'Ready...', 'Position', [10, 10, 300, 20]);
    handles.statusLabel = statusLabel;
    
    connButton = uicontrol('Style', 'pushbutton', ...
        'String', 'Connect', 'Position', [10, 140, 100, 30], ...
        'Callback', {@connectCallback, handles});
    connButton = uicontrol('Style', 'pushbutton', ...
        'String', 'Disconnect', 'Position', [120, 140, 100, 30], ...
        'Callback', {@disconnectCallback, handles});
    
    prnButton = uicontrol('Style', 'pushbutton', ...
        'String', 'Pronate Arm', 'Position', [320, 50, 100, 30], ...
        'Callback', {@pushButtonCallback, handles});
    supButton = uicontrol('Style', 'pushbutton', ...
        'String', 'Supinate Arm', 'Position', [320, 90, 100, 30], ...
        'Callback', {@pushButtonCallback, handles});
    extButton = uicontrol('Style', 'pushbutton', ...
        'String', 'Extend Wrist', 'Position', [210, 70, 100, 30], ...
        'Callback', {@pushButtonCallback, handles});
    flexButton = uicontrol('Style', 'pushbutton', ...
        'String', 'Flex Wrist', 'Position', [10, 70, 100, 30], ...
        'Callback', {@pushButtonCallback, handles});
    openButton = uicontrol('Style', 'pushbutton', ...
        'String', 'Open Hand', 'Position', [110, 100, 100, 30], ...
        'Callback', {@pushButtonCallback, handles});
    closeButton = uicontrol('Style', 'pushbutton', ...
        'String', 'Close Hand', 'Position', [110, 40, 100, 30], ...
        'Callback', {@pushButtonCallback, handles});
    
    set(f, 'Visible', 'on');
    set(f, 'CloseRequestFcn', {@CloseWindow, handles});
end

function connectCallback(~, ~, handles)
    if ~strcmp(handles.obj.Status, 'open')
        fopen(handles.obj);
        if strcmp(handles.obj.Status, 'open')
            % Arm the copter
            fwrite(handles.obj, 'S');
            old_timeout = handles.obj.Timeout;
            handles.obj.Timeout = 20; % Initialization can take some time
            rsp = fread(handles.obj, 1);
            handles.obj.Timeout = old_timeout;
            if(rsp ~= 0)
                fprintf('Invalid response on arm\n');
                fclose(handles.obj);
                handles.statusLabel.string = 'Error';
                return;
            end
            handles.statusLabel.String = 'Connected';
        else
            fprintf('Unable to connect\n');
            handles.statusLabel.string = 'Error';
        end
    else
        fprintf('Connection already open...\n');
    end
end

function disconnectCallback(~, ~, handles)
    if strcmp(handles.obj.Status, 'open')
        fprintf('Landing...\n');
        fwrite(handles.obj, 'H');
        rsp = fread(handles.obj, 1);
        if rsp ~= 0
            fprintf('Invalid response\n');
        end
        fclose(handles.obj);
        handles.statusLabel.String = 'Ready...';
    else
        fprintf('Connection not open...\n');
    end
end

function CloseWindow(hObject, ~, handles)
    fprintf('Exiting...\n');
    
    if strcmp(handles.obj.Status, 'open')
        dixconnectCallback(0,0,handles);
    end
    delete(hObject);
end

function pushButtonCallback(source, ~, handles)
    if ~strcmp(handles.obj.Status, 'open')
        fprintf('Connection not open...\n');
        return;
    end
    
    switch(source.String)
        case 'Open Hand' % Move Forward
            fwrite(handles.obj, ['M',0,1,1,0]);
            rsp = fread(handles.obj, 1);
            if rsp ~= 1
                fprintf('Invalid response\n');
            end
        case 'Close Hand' % Move Backwards
            fwrite(handles.obj, ['M',0,1,0,0]);
            rsp = fread(handles.obj, 1);
            if rsp ~= 1
                fprintf('Invalid response\n');
            end
        case 'Pronate Arm' % Move Down
            fwrite(handles.obj, ['M',0,3,0,0]);
            rsp = fread(handles.obj, 1);
            if rsp ~= 3
                fprintf('Invalid response\n');
            end
        case 'Supinate Arm' % Move Up
            fwrite(handles.obj, ['M',0,3,1,0]);
            rsp = fread(handles.obj, 1);
            if rsp ~= 3
                fprintf('Invalid response\n');
            end
        case 'Flex Wrist' % Turn Left
            fwrite(handles.obj, ['M',0,2,0,0]);
            rsp = fread(handles.obj, 1);
            if rsp ~= 2
                fprintf('Invalid response\n');
            end
        case 'Extend Wrist' % Turn Right
            fwrite(handles.obj, ['M',0,2,1,0]);
            rsp = fread(handles.obj, 1);
            if rsp ~= 2
                fprintf('Invalid response\n');
            end
        otherwise
            fprintf('Unknown command...\n');
    end
end