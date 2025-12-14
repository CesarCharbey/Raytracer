// -------------------------------------------
// gMini : a minimal OpenGL/GLUT application
// for 3D graphics.
// Copyright (C) 2006-2008 Tamy Boubekeur
// All rights reserved.
// -------------------------------------------

// -------------------------------------------
// Disclaimer: this code is dirty in the
// meaning that there is no attention paid to
// proper class attribute access, memory
// management or optimisation of any kind. It
// is designed for quick-and-dirty testing
// purpose.
// -------------------------------------------
#include <chrono>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>

#include <algorithm>
#include "src/Vec3.h"
#include "src/Camera.h"
#include "src/Scene.h"
#include <GL/glut.h>

#include "src/matrixUtilities.h"

using namespace std;

#include "src/imageLoader.h"

#include "src/Material.h"

// Pour ImGui
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glut.h"
#include "imgui/imgui_impl_opengl2.h"

// 1. CONFIGURATION DE LA FENETRE
static int sidePanelWidth = 350; // Largeur du panneau ImGui
static int windowWidth = 830;    // Largeur totale (Scene + Panel)
static int windowHeight = 480;
static int savedWidth = 830;
static int savedHeight = 480;

static int targetPhotonCount = 1'000'000;
static int renderSamples = 5;
static float lastTime = 0.0f;
// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 480;
static unsigned int SCREENHEIGHT = 480;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX = 0, lastY = 0, lastZoom = 0;
static unsigned int FPS = 0;
static bool fullScreen = false;

std::vector<Scene> scenes;
unsigned int selected_scene;
std::vector<std::pair<Vec3, Vec3>> rays;

void ray_trace_from_camera();

void printUsage()
{
    cerr << endl
         << "gMini: a minimal OpenGL/GLUT application" << endl
         << "for 3D graphics." << endl
         << "Author : Tamy Boubekeur (http://www.labri.fr/~boubek)" << endl
         << endl
         << "Usage : ./gmini [<file.off>]" << endl
         << "Keyboard commands" << endl
         << "------------------" << endl
         << " ?: Print help" << endl
         << " w: Toggle Wireframe Mode" << endl
         << " g: Toggle Gouraud Shading Mode" << endl
         << " f: Toggle full screen mode" << endl
         << " <drag>+<left button>: rotate model" << endl
         << " <drag>+<right button>: move model" << endl
         << " <drag>+<middle button>: zoom" << endl
         << " q, <esc>: Quit" << endl
         << endl;
}

void usage()
{
    printUsage();
    exit(EXIT_FAILURE);
}

// ------------------------------------
void initLight()
{
    GLfloat light_position[4] = {0.0, 1.5, 0.0, 1.0};
    GLfloat color[4] = {1.0, 1.0, 1.0, 1.0};
    GLfloat ambient[4] = {1.0, 1.0, 1.0, 1.0};

    glLightfv(GL_LIGHT1, GL_POSITION, light_position);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, color);
    glLightfv(GL_LIGHT1, GL_SPECULAR, color);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHTING);
}

void init()
{
    camera.resize(SCREENWIDTH, SCREENHEIGHT);
    initLight();

    // Initialisation ImGui
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Optionnel : navigation au clavier
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    // Important : InstallFuncs = false car nous allons gérer les inputs manuellement
    ImGui_ImplGLUT_Init();
    ImGui_ImplOpenGL2_Init();

    // glCullFace (GL_BACK);
    glDisable(GL_CULL_FACE);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.2f, 0.2f, 0.3f, 1.0f);
}

// ------------------------------------
// Replace the code of this
// functions for cleaning memory,
// closing sockets, etc.
// ------------------------------------

void clear()
{
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();
    ImGui::DestroyContext();
}

// ------------------------------------
// Replace the code of this
// functions for alternative rendering.
// ------------------------------------

void draw()
{
    glEnable(GL_LIGHTING);
    scenes[selected_scene].draw();

    // draw rays : (for debug)
    // std::cout << rays.size() << std::endl;

    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glLineWidth(6);
    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
    for (unsigned int r = 0; r < rays.size(); ++r)
    {
        glVertex3f(rays[r].first[0], rays[r].first[1], rays[r].first[2]);
        glVertex3f(rays[r].second[0], rays[r].second[1], rays[r].second[2]);
    }
    glEnd();
}

void display()
{
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
    ImGui::NewFrame();
    // CONFIGURATION DU PANNEAU LATERAL
    ImGui::SetNextWindowPos(ImVec2(windowWidth - sidePanelWidth, 0));
    ImGui::SetNextWindowSize(ImVec2(sidePanelWidth, windowHeight));
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    ImGui::Begin("SidePanel", nullptr, window_flags);
    ImGui::Text("Général");
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Separator();

    // Choix de la scène
    const char *scene_names[] = {"Sphere", "Square", "Cornell Box", "Underwater"};
    int current_scene_int = (int)selected_scene;
    if (ImGui::Combo("Scene", &current_scene_int, scene_names, IM_ARRAYSIZE(scene_names)))
    {
        selected_scene = (unsigned int)current_scene_int;
        scenes[selected_scene].emitPhotons(targetPhotonCount);
    }

    ImGui::Separator();
    ImGui::Text("Gestion des Photons");

    // SLIDER POUR LE NOMBRE DE PHOTONS
    ImGui::InputInt("Nb Photons", &targetPhotonCount, 50'000, 1'000'000);
    if (targetPhotonCount < 50'000)
        targetPhotonCount = 50'000; // Sécurité

    // CHECKBOX AVEC RECALCUL AUTOMATIQUE
    bool prevShow = scenes[selected_scene].showPhotons;
    if (ImGui::Checkbox("Afficher Photons (p)", &scenes[selected_scene].showPhotons))
    {
        if (scenes[selected_scene].showPhotons && !prevShow)
        {
            scenes[selected_scene].emitPhotons(targetPhotonCount);
        }
    }

    if (ImGui::Button("Recalculer Photons Maintenant"))
    {
        scenes[selected_scene].emitPhotons(targetPhotonCount);
    }

    ImGui::Separator();
    ImGui::Text("Paramètres de Rendu");

    ImGui::InputInt("Samples/Pixel", &renderSamples, 1, 5);
    if (renderSamples < 1)
        renderSamples = 1; // Sécurité

    ImGui::Separator();
    ImGui::Checkbox("Afficher Kd-Tree (k)", &scenes[selected_scene].showKdTree);
    ImGui::Checkbox("Position Lumière", &scenes[selected_scene].showLights);

    // GESTION DE LA LUMIERE ET RECALCUL DYNAMIQUE
    if (!scenes[selected_scene].lights.empty())
    {
        ImGui::Separator();
        ImGui::Text("Lumière");
        // BOUTON RESET
        if (ImGui::Button("Reset Position Lumière"))
        {
            scenes[selected_scene].resetLightPositions();
            if (scenes[selected_scene].showPhotons)
            {
                scenes[selected_scene].emitPhotons(targetPhotonCount);
            }
        }
        Light &l = scenes[selected_scene].lights[0];
        float pos[3] = {l.pos[0], l.pos[1], l.pos[2]};
        // DragFloat3 renvoie true si la valeur a changé
        if (ImGui::DragFloat3("Position", pos, 0.1f))
        {
            l.pos = Vec3(pos[0], pos[1], pos[2]);
        }
        // Si la position a changé, on recalcule les photons
        if (ImGui::IsItemDeactivatedAfterEdit() && scenes[selected_scene].showPhotons)
        {
            std::cout << "Lumiere bougee -> Recalcul des photons..." << std::endl;
            scenes[selected_scene].emitPhotons(targetPhotonCount);
        }

        ImGui::SliderFloat("Puissance", &l.powerCorrection, 0.0f, 50.0f);
    }

    ImGui::Separator();
    ImGui::Text("Rendu Final");
    if (ImGui::Button("Render (r)", ImVec2(-1, 40))) // -1 width = prend toute la largeur
    {
        ray_trace_from_camera();
    }

    ImGui::End();

    // 3. RENDU 3D
    glViewport(0, 0, windowWidth - sidePanelWidth, windowHeight);

    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply();
    draw();

    // 4. RENDU IMGUI
    // On remet le viewport global pour qu'ImGui puisse dessiner partout si besoin (tooltips, popups)
    glViewport(0, 0, windowWidth, windowHeight);

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glFlush();
    glutSwapBuffers();
}

void idle()
{
    // Initialisation de sûreté
    if (lastTime == 0.0f)
        lastTime = glutGet((GLenum)GLUT_ELAPSED_TIME);

    static unsigned int counter = 0;
    counter++;
    float currentTime = glutGet((GLenum)GLUT_ELAPSED_TIME);

    if (currentTime - lastTime >= 1000.0f)
    {
        FPS = counter;
        counter = 0;
        if (!ImGui::GetIO().WantCaptureKeyboard)
        {
            static char winTitle[64];
            sprintf(winTitle, "Raytracer - FPS: %d", FPS);
            glutSetWindowTitle(winTitle);
        }

        lastTime = currentTime;
    }
    glutPostRedisplay();
}

void ray_trace_from_camera()
{
    // 1. CALCUL DES DIMENSIONS DE LA ZONE 3D UNIQUEMENT
    // On ignore la partie occupée par ImGui
    int renderW = windowWidth - sidePanelWidth;
    int renderH = windowHeight;
    // Sécurité pour éviter de crasher si la fenêtre est trop petite
    if (renderW <= 0)
        renderW = 1;
    // Construction des photons (inchangé)
    scenes[selected_scene].emitPhotons(targetPhotonCount);

    std::cout << "Ray tracing a " << renderW << " x " << renderH << " image " << std::endl;

    // Important : La caméra a déjà le bon aspect ratio grâce à votre fonction reshape() modifiée
    camera.apply();

    auto start = std::chrono::high_resolution_clock::now();
    Vec3 pos, dir;
    unsigned int nsamples = (unsigned int)renderSamples;

    // 2. LE BUFFER IMAGE A LA TAILLE REDUITE
    std::vector<Vec3> image(renderW * renderH, Vec3(0, 0, 0));

    // 3. BOUCLES SUR LA ZONE 3D UNIQUEMENT
    for (int y = 0; y < renderH; y++)
    {
        for (int x = 0; x < renderW; x++)
        {
            for (unsigned int s = 0; s < nsamples; ++s)
            {
                float u = ((float)(x) + (float)(rand()) / (float)(RAND_MAX)) / renderW;
                float v = ((float)(y) + (float)(rand()) / (float)(RAND_MAX)) / renderH;
                screen_space_to_world_space_ray(u, v, pos, dir);
                Vec3 color = scenes[selected_scene].rayTrace(Ray(pos, dir));
                image[x + y * renderW] += color;
            }
            image[x + y * renderW] /= nsamples;
        }
        // Affichage de la progression
        if (y % 10 == 0)
        {
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - start).count();
            double progress = double((y + 1) * renderW) / double(renderW * renderH);
            double estimatedTotal = elapsed / progress;
            double remaining = estimatedTotal - elapsed;
            std::cout << "\rProgress: " << int(progress * 100) << "% "
                      << "Remaining: ~" << int(remaining) << "s     " << std::flush;
        }
    }
    std::cout << "\tDone" << std::endl;
    std::string filename = "./rendu.ppm";
    ofstream f(filename.c_str(), ios::binary);
    if (f.fail())
    {
        cout << "Could not open file: " << filename << endl;
        return;
    }
    f << "P3" << std::endl
      << renderW << " " << renderH << std::endl
      << 255 << std::endl;

    for (int i = 0; i < renderW * renderH; i++)
        f << (int)(255.f * std::min<float>(1.f, image[i][0])) << " "
          << (int)(255.f * std::min<float>(1.f, image[i][1])) << " "
          << (int)(255.f * std::min<float>(1.f, image[i][2])) << " ";
    f << std::endl;
    f.close();
    lastTime = glutGet((GLenum)GLUT_ELAPSED_TIME);
}

void key(unsigned char keyPressed, int x, int y)
{
    ImGui_ImplGLUT_KeyboardFunc(keyPressed, x, y);
    // Si ImGui veut le clavier (ex: taper du texte), on ignore les raccourcis 3D
    if (ImGui::GetIO().WantCaptureKeyboard)
        return;

    Vec3 pos, dir;
    switch (keyPressed)
    {
    case 'f':
        if (fullScreen == true)
        {
            glutReshapeWindow(savedWidth, savedHeight);
            glutPositionWindow(0, 0);
            fullScreen = false;
        }
        else
        {
            savedWidth = windowWidth;
            savedHeight = windowHeight;
            glutFullScreen();
            fullScreen = true;
        }
        break;
    case 'q':
    case 27:
        clear();
        exit(0);
        break;
    case 'w':
        GLint polygonMode[2];
        glGetIntegerv(GL_POLYGON_MODE, polygonMode);
        if (polygonMode[0] != GL_FILL)
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        break;

    case 'r':
        camera.apply();
        rays.clear();
        ray_trace_from_camera();
        break;
    case '+':
        selected_scene++;
        if (selected_scene >= scenes.size())
            selected_scene = 0;
        scenes[selected_scene].emitPhotons(100'000); // petit chiffre pour le debug rapide
        break;

    // debug photon map
    case 'p':
        scenes[selected_scene].showPhotons = !scenes[selected_scene].showPhotons;
        break;
    // debug kd-tree
    case 'k':
        scenes[selected_scene].showKdTree = !scenes[selected_scene].showKdTree;
        break;
    // debug lights
    case 'l':
        scenes[selected_scene].showLights = !scenes[selected_scene].showLights;
        break;
    default:
        printUsage();
        break;
    }
    idle();
}

void special(int key, int x, int y)
{
    ImGui_ImplGLUT_SpecialFunc(key, x, y);
    if (ImGui::GetIO().WantCaptureKeyboard)
        return;
}

void specialUp(int key, int x, int y)
{
    ImGui_ImplGLUT_SpecialUpFunc(key, x, y);
}

void keyUp(unsigned char key, int x, int y)
{
    ImGui_ImplGLUT_KeyboardUpFunc(key, x, y);
}

void mouse(int button, int state, int x, int y)
{
    ImGui_ImplGLUT_MouseFunc(button, state, x, y);
    // Si ImGui utilise la souris (ex: clic sur un bouton), on ignore la caméra
    if (x > (windowWidth - sidePanelWidth) || ImGui::GetIO().WantCaptureMouse)
        return;

    if (state == GLUT_UP)
    {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    }
    else
    {
        if (button == GLUT_LEFT_BUTTON)
        {
            camera.beginRotate(x, y);
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
        }
        else if (button == GLUT_RIGHT_BUTTON)
        {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
        }
        else if (button == GLUT_MIDDLE_BUTTON)
        {
            if (mouseZoomPressed == false)
            {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
            }
        }
    }
    idle();
}

void motion(int x, int y)
{
    ImGui_ImplGLUT_MotionFunc(x, y); // Transmettre à ImGui
    if (ImGui::GetIO().WantCaptureMouse)
        return;

    if (mouseRotatePressed == true)
    {
        camera.rotate(x, y);
    }
    else if (mouseMovePressed == true)
    {
        camera.move((x - lastX) / static_cast<float>(SCREENWIDTH), (lastY - y) / static_cast<float>(SCREENHEIGHT), 0.0);
        lastX = x;
        lastY = y;
    }
    else if (mouseZoomPressed == true)
    {
        camera.zoom(float(y - lastZoom) / SCREENHEIGHT);
        lastZoom = y;
    }
}

void reshape(int w, int h)
{
    windowWidth = w;
    windowHeight = h;
    ImGui_ImplGLUT_ReshapeFunc(w, h);
    // La caméra ne s'occupe que de la zone 3D (Largeur totale - Panneau)
    // On s'assure que la largeur n'est pas négative
    int gl3dWidth = std::max(1, w - sidePanelWidth);

    camera.resize(gl3dWidth, h);
}

int main(int argc, char **argv)
{
    if (argc > 2)
    {
        printUsage();
        exit(EXIT_FAILURE);
    }
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(windowWidth, windowHeight);
    window = glutCreateWindow("Raytracer");

    init();
    glutIdleFunc(idle);
    glutDisplayFunc(display);
    glutKeyboardFunc(key);
    glutSpecialFunc(special);
    glutKeyboardUpFunc(keyUp);
    glutSpecialUpFunc(specialUp);
    glutReshapeFunc(reshape);
    glutMotionFunc(motion);
    glutMouseFunc(mouse);
    key('?', 0, 0);

    camera.move(0., 0., -3.1);
    selected_scene = 0;
    scenes.resize(4);
    scenes[0].setup_single_sphere();
    scenes[1].setup_single_square();
    scenes[2].setup_cornell_box();
    scenes[3].setup_underwater_scene(); // scène sous l'eau pour les tests de caustiques

    scenes[selected_scene].emitPhotons(100'000); // petit chiffre pour le debug rapide
    glutMainLoop();
    clear();
    return EXIT_SUCCESS;
}
