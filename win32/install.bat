@ECHO OFF

IF NOT EXIST "C:\Program Files\gomotion" MKDIR "C:\Program Files\gomotion"
IF NOT EXIST "C:\Program Files\gomotion\include" MKDIR "C:\Program Files\gomotion\include"
IF NOT EXIST "C:\Program Files\gomotion\bin" MKDIR "C:\Program Files\gomotion\bin"
IF NOT EXIST "C:\Program Files\gomotion\lib" MKDIR "C:\Program Files\gomotion\lib"
IF NOT EXIST "C:\Program Files\gomotion\etc" MKDIR "C:\Program Files\gomotion\etc"


COPY ..\src\*.h "C:\Program Files\gomotion\include"
COPY Debug\*.exe "C:\Program Files\gomotion\bin"
COPY ..\bin\pendant.tcl "C:\Program Files\gomotion\bin"
COPY gorun.bat "C:\Program Files\gomotion\bin"
COPY Debug\*.lib "C:\Program Files\gomotion\lib"
COPY ..\etc\*.ini "C:\Program Files\gomotion\etc"

IF NOT EXIST "Debug" MKDIR "Debug"
COPY ..\bin\pendant.tcl "Debug"

PAUSE
