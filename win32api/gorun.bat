rem call as 'gorun.bat <inifile> <module>', e.g.,
rem gorun.bat ..\..\etc\gowin.ini gomain.exe

setlocal

cd debug

@echo off

if [%1]==[] ( set inifile=..\..\etc\gowin.ini ) else ( set inifile=%1 )
if [%2]==[] ( set module=gomain.exe ) else ( set module=%2 )

for /F %%a in ('inifind SHM_KEY TRAJ %inifile%') do set TRAJ_SHM_KEY=%%a
for /F %%a in ('inifind SHM_KEY SERVO %inifile%') do set SERVO_SHM_KEY=%%a
for /F %%a in ('inifind SEM_KEY SERVO %inifile%') do set SERVO_SEM_KEY=%%a
for /F %%a in ('inifind EXT_INIT_STRING GOMOTION %inifile%') do set EXT_INIT_STRING=%%a
for /F %%a in ('inifind KINEMATICS TRAJ %inifile%') do set KINEMATICS=%%a
for /F %%a in ('inifind SHM_KEY GO_LOG %inifile%') do set GO_LOG_SHM_KEY=%%a
for /F %%a in ('inifind SHM_KEY GO_IO %inifile%') do set GO_IO_SHM_KEY=%%a

set TOOL_SHM_KEY=0
for /F %%a in ('inifind SHM_KEY TOOL %inifile%') do set TOOL_SHM_KEY=%%a
set TASK_SHM_KEY=0
for /F %%a in ('inifind SHM_KEY TASK %inifile%') do set TASK_SHM_KEY=%%a

rem spawn the controller
start /B %module% DEBUG=1 TRAJ_SHM_KEY=%TRAJ_SHM_KEY% SERVO_SHM_KEY=%SERVO_SHM_KEY% SERVO_SEM_KEY=%SERVO_SEM_KEY% EXT_INIT_STRING=%EXT_INIT_STRING% KINEMATICS=%KINEMATICS% GO_LOG_SHM_KEY=%GO_LOG_SHM_KEY% GO_IO_SHM_KEY=%GO_IO_SHM_KEY%

rem wait a few seconds
ping -n 2 127.0.0.1 > nul

rem spawn the tool controller, if indicated
if not "%TOOL_SHM_KEY%"=="0" (start /B toolmain.exe TOOL_SHM_KEY=%TOOL_SHM_KEY%)

rem spawn the task controller, if indicated
if not "%TASK_SHM_KEY%"=="0" (start /B taskmain.exe -i %inifile%)

rem configure the controller
gocfg.exe -i %inifile%

rem spawn the GUI
start /B gotk.exe ..\..\bin\pendant.tcl -- -i %inifile% -k

endlocal

