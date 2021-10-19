CC:= g++
LIBS:= -lstdc++ -lwiringPi 
CXXFLAGS:= -std=c++11 -g -pthread
DIR_SRC:= ./src/move
DIR_OBJ:= ./obj/move
TARGET:= test_move
OBJECTS := test_move.o 
OBJECTS := $(addprefix $(DIR_OBJ)/,$(OBJECTS))

all: $(TARGET)

$(shell mkdir obj)
$(shell mkdir $(DIR_OBJ))

$(TARGET): $(OBJECTS)
	$(CC) -o $(TARGET) $(OBJECTS) $(LIBS)

$(DIR_OBJ)/%.o: $(DIR_SRC)/%.c  
	$(CC) -c $(CXXFLAGS) $(INCLUDE) $< -o $@
  
.PHONY : clean
clean:   
	-rm -f $(DIR_OBJ)\*.o
	-rm -f $(TARGET) 
