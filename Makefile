app:
	g++ tester.cpp \
	math/*.cpp \
	$(shell find physics -name "*.cpp" ! -name "world.cpp") \
	-std=c++17 \
	-I/usr/local/include \
	-L/usr/local/lib \
	-lraylib \
	-o tester

run:
	./tester