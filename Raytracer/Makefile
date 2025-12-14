# Makefile pour un unique exécutable

# liste des variables à renseigner
#   CIBLE : nom du programme ( $(CIBLE).c doit contenir main() )
#   SRCS : ensemble des fichiers sources 
#   LIBS : liste des bibliothèques utiles à l'édition des liens 
#          (format : -lnom1 -lnom2 ...) 
#   PREFIX : chemin de la hiérarchie 
#
# NE PAS OUBLIER D'AJOUTER LA LISTE DES DEPENDANCES A LA FIN DU FICHIER

# --- CONFIGURATION IMGUI ---
IMGUI_DIR = imgui
# ---------------------------

CIBLE = main

# Vos sources existantes
SRCS = src/Camera.cpp main.cpp src/Trackball.cpp src/imageLoader.cpp src/Mesh.cpp

# Ajout des sources ImGui (Coeur)
SRCS += $(IMGUI_DIR)/imgui.cpp $(IMGUI_DIR)/imgui_draw.cpp $(IMGUI_DIR)/imgui_tables.cpp $(IMGUI_DIR)/imgui_widgets.cpp

# Ajout des sources ImGui (Backends pour GLUT et OpenGL2)
# Vérifiez que ces fichiers sont bien dans le dossier 'imgui' (sinon ajustez le chemin, ex: $(IMGUI_DIR)/backends/...)
SRCS += $(IMGUI_DIR)/imgui_impl_glut.cpp $(IMGUI_DIR)/imgui_impl_opengl2.cpp

LIBS =  -lglut -lGLU -lGL -lm -lpthread 

#########################################################

INCDIR = .
LIBDIR = .
BINDIR = .

# nom du compilateur
CC = g++
CPP = g++

# options du compilateur          
CFLAGS = -Wall -O3 
CXXFLAGS = -Wall -O3 

# option du preprocesseur
# On ajoute le dossier imgui aux includes (-I)
CPPFLAGS =  -I$(INCDIR) -I$(IMGUI_DIR)

# options du linker et liste des bibliothèques à charger
LDFLAGS = -L/usr/X11R6/lib              
LDLIBS = -L$(LIBDIR) $(LIBS)  

# construire la liste des fichiers objets une nouvelle chaine à partir
# de SRCS en substituant les occurences de ".c" ou ".cpp" par ".o" 
OBJS = $(SRCS:.cpp=.o)   

# cible par défaut
$(CIBLE): $(OBJS)
	$(CPP) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $(CIBLE)

install:  $(CIBLE)
	cp $(CIBLE) $(BINDIR)/

installdirs:
	test -d $(INCDIR) || mkdir $(INCDIR)
	test -d $(LIBDIR) || mkdir $(LIBDIR)
	test -d $(BINDIR) || mkdir $(BINDIR)

clean:
	rm -f  *~  $(CIBLE) $(OBJS)

veryclean: clean
	rm -f $(BINDIR)/$(CIBLE)

dep:
	gcc $(CPPFLAGS) -MM $(SRCS)

# liste des dépendances générée par 'make dep'
Camera.o: src/Camera.cpp src/Camera.h src/Vec3.h src/Trackball.h
main.o: main.cpp src/Vec3.h src/Camera.h src/Trackball.h
Trackball.o: src/Trackball.cpp src/Trackball.h