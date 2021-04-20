function varargout = Serial_communication(varargin)
% SERIAL_COMMUNICATION MATLAB code for Serial_communication.fig
%      SERIAL_COMMUNICATION, by itself, creates a new SERIAL_COMMUNICATION or raises the existing
%      singleton*.
%
%      H = SERIAL_COMMUNICATION returns the handle to a new SERIAL_COMMUNICATION or the handle to
%      the existing singleton*.
%
%      SERIAL_COMMUNICATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SERIAL_COMMUNICATION.M with the given input arguments.
%
%      SERIAL_COMMUNICATION('Property','Value',...) creates a new SERIAL_COMMUNICATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Serial_communication_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Serial_communication_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Serial_communication

% Last Modified by GUIDE v2.5 01-Dec-2020 18:09:20

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Serial_communication_OpeningFcn, ...
                   'gui_OutputFcn',  @Serial_communication_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Serial_communication is made visible.
function Serial_communication_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Serial_communication (see VARARGIN)

% Choose default command line output for Serial_communication
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Serial_communication wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% s = serial('COM8', 'BaudRate', 115200)
%  delete(s);



% flag = true
% handles.flag = flag;
% guidata(hObject,handles);

% --- Outputs from this function are returned to the command line.
function varargout = Serial_communication_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_Inc_velocity.
function pushbutton_Inc_velocity_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Inc_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pushbutton_Inc_velocity

handles.tx_inc=true;
guidata(hObject,handles);

% --- Executes on button press in pushbutton_Dec_velocity.
function pushbutton_Dec_velocity_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Dec_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pushbutton_Dec_velocity

handles.tx_dec=true;
guidata(hObject,handles);

% s = serial('COM8', 'BaudRate', 115200);
% fopen(s);
% fwrite(s,'0')
% fclose(s)
% delete(s)

function edit_velocity_Callback(hObject, eventdata, handles)
% hObject    handle to edit_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_velocity as text
%        str2double(get(hObject,'String')) returns contents of edit_velocity as a double


% --- Executes during object creation, after setting all properties.
function edit_velocity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_start.
function pushbutton_start_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla(handles.Position)
cla(handles.PWM)
cla(handles.Current)
cla()

handles.tx_inc=false;
guidata(hObject,handles);

handles.tx_dec=false;
guidata(hObject,handles);

handles.stop=false;
guidata(hObject,handles);

handles.clockwise=false;
guidata(hObject,handles);

handles.counterclockwise=false;
guidata(hObject,handles);

s = serial('COM8', 'BaudRate', 115200, 'Databits', 8,'Parity', "even", 'StopBits', 1)
% s = serial('COM8', 'BaudRate', 115200);
% ;

% fopen(s)       
% fwrite(s,'s');        
% fclose(s)

i=1;
uint16 control;
while(1)

    fopen(s);
    
  start = fread(s,1,'char');
    
    if(start == 'b')
    
        %data(i,:) = fread(s,1,'uint16');
        data_position(i,:) = fread(s,4,'int16');
%         data_force(i,:) = fread(s,1,'uint32')
%         control = data(:,2)+data(:,3);
        ended = fread(s,1,'char');
        if(ended == 'e')
%             if (data(:,4)==control & ended == 'e')     
                axes(handles.PWM)
                
                % Torque
                % 0.004
                plot(data_position(:,1)*0.0033);
%                 plot(data_force(:,1))
                grid on 
                
                disp('ok')
                
                axes(handles.Current)
                plot(data_position(:,1),'r');
                grid on, hold on
                            
                axes(handles.Position)
                plot(data_position(:,2),'g');
                grid on, hold on
                axes(handles.Position)
                plot(data_position(:,3)/24,'r');
                
                axes(handles.Torque)
                plot(data_position(:,4),'r');
                grid on, hold on
%                 legend('RPM Target', 'RPM Read');
                i = i+1;
%             end
        end
    end
    fclose(s);
        

    drawnow()
    handles = guidata(hObject);
    
    if(handles.tx_dec==true)
        
        fopen(s);      
        fwrite(s,'d');        
        fclose(s);
        handles.tx_dec=false;
        guidata(hObject,handles)
        
    end
    
    if(handles.tx_inc==true)
        
        fopen(s)
        fwrite(s,'i');
        fclose(s)
        handles.tx_inc=false
        guidata(hObject,handles);
        
   end
    
    if(handles.counterclockwise==true)   
        fopen(s)
        fwrite(s,'k');
        fclose(s)
        handles.counterclockwise =false
        guidata(hObject,handles);
        
    end
    
    if(handles.clockwise==true) 
            
        fopen(s)
        fwrite(s,'c');
        fclose(s)
        handles.clockwise=false
        guidata(hObject,handles);
        
    end

    if(handles.stop==true)
        fopen(s)
        fwrite(s,'x');
        fclose(s)
        fopen(s)
        fwrite(s,'x');
        fclose(s)
        fopen(s)
        fwrite(s,'x');
        fclose(s)
        handles.tx_inc=false
        guidata(hObject,handles);
        break;
    end   
end

 


% --- Executes on button press in pushbutton_Stop.
function pushbutton_Stop_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.stop=true;
guidata(hObject,handles);


% --- Executes on button press in pushbutton_Counterclockwise.
function pushbutton_Counterclockwise_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Counterclockwise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.counterclockwise=true;
guidata(hObject,handles);


% --- Executes on button press in pushbutton_clockwise.
function pushbutton_clockwise_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_clockwise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.clockwise=true;
guidata(hObject,handles);
