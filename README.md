# Swisslog SDAO1 Coding Challenge. 

SwarmGridlock contains the challenge requirements and the initial files. 

SwarmManager is the project I built.

The core of the project is implemented in python.

```mermaid
flowchart TD

    Dataset["Dataset in/out files"]

    Converter["Dataset Converter<br/>• Load dataset into data structure<br/>• Save data structure to file"]

    BOStart["Business Objects<br/>(start)"]

    Persistence["Persistence: SQLite<br/>• Store simulation state<br/>• Store robot steps<br/>• Store obstacles<br/>• Store current robot positions<br/>• Store starting positions<br/>• Store end positions"]

    BOCurrent["Business Objects<br/>(current state)"]

    Algorithm["Algorithm<br/>• Strategy to move robots"]

    BONext["Business Objects<br/>(next state)"]

    BOEnd["Business Objects<br/>(end)"]

    BOState["Business Objects<br/>(state)"]

    Visualizer["Visualization: Tkinter<br/>• Visualize simulation state<br/>• Playback simulation"]

    Dataset --> Converter
    Converter --> BOStart
    BOStart --> Persistence

    Persistence --> BOCurrent
    BOCurrent --> Algorithm
    Algorithm --> BONext
    BONext --> Persistence

    Persistence --> BOEnd
    BOEnd --> Converter

    Persistence --> BOState
    BOState --> Visualizer
```

I tried different strategies to move the robots:  
OR-Tools  
https://developers.google.com/optimization  
imported as a python project  
LaCAM  
https://github.com/Kei18/lacam  
compiled locally and invoked from python as an external process  
LaCAM3  
https://github.com/Kei18/lacam3
compiled locally and invoked from python as an external process  

I committed the alreay complied c++ sources.
I can you need to recompile them:  
Install mingw-w64 https://www.mingw-w64.org/  
on mingw-64 console

```
cmake -B build 
```

```
cmake --build build
```
collect dependencies with ldd command

```
$ ldd mapf.exe
        ntdll.dll => /c/WINDOWS/SYSTEM32/ntdll.dll (0x7ff9ba180000)
        KERNEL32.DLL => /c/WINDOWS/System32/KERNEL32.DLL (0x7ff9b87e0000)
        KERNELBASE.dll => /c/WINDOWS/System32/KERNELBASE.dll (0x7ff9b7830000)
        msvcrt.dll => /c/WINDOWS/System32/msvcrt.dll (0x7ff9b7f30000)
        libgcc_s_seh-1.dll => /mingw64/bin/libgcc_s_seh-1.dll (0x7ff9b31a0000)
        libstdc++-6.dll => /mingw64/bin/libstdc++-6.dll (0x7ff972d10000)
        libwinpthread-1.dll => /mingw64/bin/libwinpthread-1.dll (0x7ff9b3180000)
        libwinpthread-1.dll => /mingw64/bin/libwinpthread-1.dll (0x1dc59670000)

```
it may be necessary to adjust some classes to make it compile.  
I have not documented all the changes, but in my case where quite minimal.  
example:  
third_party/grid-pathfinding/graph/src/graph.cpp add in the include section  
```
#include <cstring>
```


