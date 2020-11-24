# run in scripts/ directory

clang-format -style=Google -i ../src/*.cpp
clang-format -style=Google -i ../include/*.hpp
clang-format -style=Google -i ../include/*.h

black *.py
