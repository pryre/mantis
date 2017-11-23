h = actxserver('WScript.Shell');
h.Run('C:\Users\kyemo\Datcom\bin\datcom.bat C:\Users\kyemo\Datcom\exmaples\ASW-20.dcm');
pause(3); %Waits for the application to load.
%kh.AppActivate('Select DATCOM'); %Brings notepad to focus
pause(10)

h.SendKeys('k'); %Sends keystrokes