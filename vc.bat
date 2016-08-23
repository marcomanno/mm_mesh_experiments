setlocal
set path=%path%;C:\Users\marco\OneDrive\Documents\PROJECTS\ThirdParties\vtk_installRelease\bin
call "%VS140COMNTOOLS%vsvars32.bat"
call devenv.exe C:\Users\marco\OneDrive\Documents\PROJECTS\polytriagnulation\out\Project.sln
exit