function varargout = kin_visualization(varargin)
% KIN_VISUALIZATION MATLAB code for kin_visualization.fig
%      KIN_VISUALIZATION, by itself, creates a new KIN_VISUALIZATION or raises the existing
%      singleton*.
%
%      H = KIN_VISUALIZATION returns the handle to a new KIN_VISUALIZATION or the handle to
%      the existing singleton*.
%
%      KIN_VISUALIZATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KIN_VISUALIZATION.M with the given input arguments.
%
%      KIN_VISUALIZATION('Property','Value',...) creates a new KIN_VISUALIZATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before kin_visualization_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to kin_visualization_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help kin_visualization

% Last Modified by GUIDE v2.5 02-Feb-2025 20:57:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @kin_visualization_OpeningFcn, ...
                   'gui_OutputFcn',  @kin_visualization_OutputFcn, ...
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


% --- Executes just before kin_visualization is made visible.
function kin_visualization_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to kin_visualization (see VARARGIN)

% Choose default command line output for kin_visualization
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes kin_visualization wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = kin_visualization_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function theta1box_Callback(hObject, eventdata, handles)
% hObject    handle to theta1box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta1box as text
%        str2double(get(hObject,'String')) returns contents of theta1box as a double


% --- Executes during object creation, after setting all properties.
function theta1box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta2box_Callback(hObject, eventdata, handles)
% hObject    handle to theta2box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta2box as text
%        str2double(get(hObject,'String')) returns contents of theta2box as a double


% --- Executes during object creation, after setting all properties.
function theta2box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta2box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta3box_Callback(hObject, eventdata, handles)
% hObject    handle to theta3box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta3box as text
%        str2double(get(hObject,'String')) returns contents of theta3box as a double


% --- Executes during object creation, after setting all properties.
function theta3box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta3box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta4box_Callback(hObject, eventdata, handles)
% hObject    handle to theta4box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta4box as text
%        str2double(get(hObject,'String')) returns contents of theta4box as a double


% --- Executes during object creation, after setting all properties.
function theta4box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta4box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta5box_Callback(hObject, eventdata, handles)
% hObject    handle to theta5box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta5box as text
%        str2double(get(hObject,'String')) returns contents of theta5box as a double


% --- Executes during object creation, after setting all properties.
function theta5box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta5box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function X_Callback(hObject, eventdata, handles)
% hObject    handle to X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of X as text
%        str2double(get(hObject,'String')) returns contents of X as a double


% --- Executes during object creation, after setting all properties.
function X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Y_Callback(hObject, eventdata, handles)
% hObject    handle to Y_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Y_text as text
%        str2double(get(hObject,'String')) returns contents of Y_text as a double


% --- Executes during object creation, after setting all properties.
function Y_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Y_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Z_Callback(hObject, eventdata, handles)
% hObject    handle to Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Z as text
%        str2double(get(hObject,'String')) returns contents of Z as a double


% --- Executes during object creation, after setting all properties.
function Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonForward.
function pushbuttonForward_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonForward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Take theta inputs
Th_1 = str2double(handles.theta1box.String)*pi/180;
Th_2 = str2double(handles.theta2box.String)*pi/180;
Th_3 = str2double(handles.theta3box.String)*pi/180;
% Th_4 = str2double(handles.theta4box.String)*pi/180;
% Th_5 = str2double(handles.theta5box.String)*pi/180;

% Link lengths
L_1 = 3.95/2;
L_2 = 21.9;
L_3 = 21.7;
% L_4 = 21.7;
% L_5 = 21.9;

% [theta, d, a, alpha]
L(1) = Link([0 0 L_1 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
% L(4) = Link([0 0 L_4 0]);
% L(5) = Link([0 0 L_5 0]);

Robot = SerialLink(L);
% Robot.base = troty(pi/2);
Robot.base = trotx(-pi/2);
Robot.name = 'Pantograph';
Robot.plot([Th_1 Th_2 Th_3]);

T = Robot.fkine([Th_1 Th_2 Th_3]);
handles.X.String = num2str(round(T.t(1), 4));
handles.Y.String = num2str(round(T.t(2), 4));
handles.Z.String = num2str(round(T.t(3), 4));

% --- Executes on button press in pushbuttonInverse.
function pushbuttonInverse_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonInverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
