setlocal
cd %~dp0\out\src\UnitTest
set path=%path;%C:\Users\marco\OneDrive\Documents\PROJECTS\ThirdParties\vtk_installRelease\bin
call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars32.bat"
##%VS140COMNTOOLS%vsvars32.bat"
devenv C:\Users\marco\OneDrive\Documents\PROJECTS\polytriagnulation\out\Project.sln
exit 0