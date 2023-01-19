CXX = g++
CFLAGS = -O2 -std=c++11 -Wall
LIBS = 
EXCUTABLE = main

SOURCE = $(wildcard ./*cpp)
OBJECTS = $(patsubst %.cpp,%.o,$(SOURCE))
INCLUDE = 

$(EXCUTABLE): $(OBJECTS)
	$(CXX) $(INCLUDE) $(CFLAGS) $^ -o $@ $(LIBS)
$(OBJECTS): %.o : %.cpp
	$(CXX) $(INCLUDE) -c $(CFLAGS) $< -o $@ $(LIBS)

.PHONY: clean example1 example2
clean: 
	rm -r $(OBJECTS) $(EXCUTABLE)

example1:
	./main -s ../pic/test1_src.png -t ../pic/test1_target.png -m ../pic/test1_mask.png -r example1 -x -120 -y -80

example2:
	./main -s ../pic/test2_src.png -t ../pic/test2_target.png -m ../pic/test2_mask.png -r example2 -x 160 -y 140
