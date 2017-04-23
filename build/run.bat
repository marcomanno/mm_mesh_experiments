cd %~dp0\..
mkdir out
cd out
cmake -G "Visual Studio 15 2017 Win64" ..\main
cd %~dp0
