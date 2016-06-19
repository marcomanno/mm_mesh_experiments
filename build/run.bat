cd %~dp0\..
mkdir out
cd out
cmake -G "Visual Studio 14 2015 Win64" ..\main
cd %~dp0
