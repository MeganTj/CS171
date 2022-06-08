CC = g++
FLAGS = -g -std=c++11

INCLUDE = -I/usr/X11R6/include -I/usr/include/GL -I/usr/include -isystem.
LIBDIR = -L/usr/X11R6/lib -L/usr/local/lib
SHADED_SOURCES = shaded_renderer.cpp parsing.cpp
TEXTURE_SOURCES = texture_renderer.cpp readpng.cpp parsing.cpp
LIBS = -lGLEW -lGL -lGLU -lglut -lm -lpng

SHADED_EXENAME = shadedRenderer
TEXTURE_EXENAME = textureRenderer

all: clean $(SHADED_EXENAME) $(TEXTURE_EXENAME)

shadedRenderer: $(SHADED_SOURCES)
	$(CC) $(FLAGS) -o $(SHADED_EXENAME) $(INCLUDE) $(LIBDIR) $(SHADED_SOURCES) $(LIBS)

textureRenderer: $(TEXTURE_SOURCES)
	$(CC) $(FLAGS) -o $(TEXTURE_EXENAME) $(INCLUDE) $(LIBDIR) $(TEXTURE_SOURCES) $(LIBS)

runShaded:
	./$(SHADED_EXENAME)

runTex:
	./$(TEXTURE_EXENAME)

.PHONY: clean
clean:
	rm -f *.o $(SHADED_EXENAME) $(TEXTURE_EXENAME)  

.PHONY: all