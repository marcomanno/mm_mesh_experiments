setlocal
cd %~dp0\out\src\UnitTest
set path=%path;%C:\Users\marco\OneDrive\Documents\PROJECTS\ThirdParties\vtk_installRelease\bin
%VS140COMNTOOLS%vsvars32.bat"
start C:\Users\marco\OneDrive\Documents\PROJECTS\polytriagnulation\out\Project.sln
exit 0