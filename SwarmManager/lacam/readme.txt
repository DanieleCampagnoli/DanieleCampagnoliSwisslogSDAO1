installed with:
MINGW64 platform and command line.

1) install https://www.msys2.org/
2) cmake -B build -G "MinGW Makefiles"
3) workaround to compile in windows
sed -i '1i using uint = unsigned int;' $(git ls-files "*.hpp" "*.cpp")
4) cmake --build build
