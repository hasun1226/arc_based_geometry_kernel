This is a repository for my thesis "Geometry kernel based on G1-continuous circular arcs"

To build this project in Visual Studio, you must follow as belows:

1. VC++ Directories > Include Directories > add "$(SolutionDir)Dependencies\freeglut\include"
2. VC++ Directories > Library Directories > add "$(SolutionDir)Dependencies\freeglut\lib\$(Platform)"
3. Linker > Input > Additional Dependncies > add "freeglut.lib"
4. Build Event > Post-Build Event > Command Line > add "copy "$(SolutionDir)Dependencies\freeglut\bin\$(Platform)\freeglut.dll" "$(OutDir)""