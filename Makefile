CXX = g++
CXXFLAGS = -std=c++17 -O3 -I/usr/local/include
LDFLAGS = -L/usr/local/lib -lraylib

SRC = tester.cpp \
      $(wildcard math/*.cpp) \
	  $(wildcard UI/*.cpp) \
      $(shell find physics -name "*.cpp")

OBJ = $(SRC:.cpp=.o)

app: $(OBJ)
	$(CXX) $(OBJ) $(LDFLAGS) -o tester

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: app
	./tester

clean:
	rm -f $(OBJ) tester

# app:
# 	g++ tester.cpp \
# 	math/*.cpp \
# 	$(shell find physics -name "*.cpp" ! -name "world.cpp") \
# 	-std=c++17 \
# 	-O3 \
# 	-I/usr/local/include \
# 	-L/usr/local/lib \
# 	-lraylib \
# 	-o tester

# run:
# 	./tester