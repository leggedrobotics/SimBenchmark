//
// Created by jhwangbo on 17. 4. 28.
//

#ifndef PROJECT_RAI_GRAPHICS_HPP
#define PROJECT_RAI_GRAPHICS_HPP
#include "raiGraphics/RAI_keyboard.hpp"
#include "raiGraphics/obj/Background.hpp"
#include "raiGraphics/obj/SingleBodyObject.hpp"
#include "raiGraphics/obj/MultiBodyObject.hpp"
#include "raiGraphics/obj/Sphere.hpp"
#include "raiGraphics/obj/Rectangle.hpp"
#include "raiGraphics/obj/Box.hpp"
#include "raiGraphics/obj/Cone.hpp"
#include "raiGraphics/obj/Cylinder.hpp"
#include "raiGraphics/obj/Mesh.hpp"
#include "raiGraphics/obj/Capsule.hpp"

#include "raiGraphics/imp/display.h"
#include "raiGraphics/imp/shader_basic.h"
#include "raiGraphics/imp/shader_flat.h"
#include "raiGraphics/imp/shader_line.h"
#include "raiGraphics/imp/shader_mouseClick.h"
#include "raiGraphics/imp/shader_background.hpp"
#include "raiGraphics/imp/shader_menu.h"
#include "raiGraphics/imp/shader_checkerboard.h"
#include "raiCommon/utils/StopWatch.hpp"
#include <mutex>
#include <raiGraphics/obj/CheckerBoard.hpp>
#include <raiGraphics/imp/shader_mouseClick.h>
#include <raiGraphics/obj/Arrow.hpp>
#include <raiGraphics/obj/Lines.hpp>
#include "SDL2/SDL_ttf.h"
#include "SDL2/SDL.h"
#include <stdio.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>

namespace rai_graphics {

struct LightProp {
  std::vector<float>  pos_light = {-1000.0,0.0,100.0},
      amb_light = {0.2, 0.2, 0.2},
      diff_light = {0.7, 0.7, 0.7},
      spec_light = {0.7, 0.7, 0.7};
};

struct MouseInput {
  int x, y;
  bool leftB, rightB, middleB;
  SDL_MouseWheelEvent wheel;
};

struct CameraProp {
  object::SingleBodyObject *toFollow = nullptr;
  Eigen::Vector3d relativeDist = Eigen::Vector3d::Constant(1);
};

enum KeyboardEvent {
  U = 0,
  I = 1,
  O = 2,
  P = 3,
  J = 4,
  K = 5,
  L = 6,
  DEL = 7
};

class RAI_graphics {
 public:

  RAI_graphics(int windowWidth, int windowHeight);
  ~RAI_graphics();
  void start();
  void end();

  void addObject(object::SingleBodyObject *obj, object::ShaderType type = object::RAI_SHADER_OBJECT_DEFAULT);
  object::Lines* addLineSet();
  void removeLineSet(object::Lines* lineset);
  void addSuperObject(object::MultiBodyObject *obj);
  void addBackground(object::Background *back);
  void addCheckerBoard(object::CheckerBoard *back);
  bool getKeyboardEvent(KeyboardEvent ke);

  void setFPS(double FPS) { FPS_ = FPS; }
  void removeAndDeleteObject(object::SingleBodyObject *obj);
  void removeObject(object::SingleBodyObject *obj);
  void removeSuperObject(object::MultiBodyObject *obj);
  void setBackgroundColor(float r, float g, float b, float a);
  void setLightProp(LightProp &prop);
  void setAntiAliasing(int aa);
  void setCameraProp(CameraProp &prop);

  void savingSnapshots(std::string logDirectory, std::string fileName);
  void saveVideo();
  void images2Video();


  const Uint8* keyboard();
  const MouseInput* mouse();

  bool isInteracting();
  Eigen::Vector3d& getInteractionMagnitude();
  int getInteractingObjectID();
  bool getCustomToggleState(int id);
  float getRealTimeFactor();
  void changeMenuText(int menuId, bool isOnText, std::string mt);
  void changeMenuPosition(int menuId, int x, int y);
  void setMenuPositionNextToCursor(int menuId);
  void changeMenuWordWrap(int menuId, int wr);
  void changeMenuFontSize(int menuId, int size);

  bool isReady(){ return isGraphicsReady; }
  bool isQuitting(){ return isQuiting; }
  void pauseVisualization() {visRunning=false;}
  void resumeVisualization() {visRunning=true;}

  void hideWindow();
  void showWindow();


 private:
  void *loop(void *obj);
  void init();
  void draw();
  void *images2Video_inThread(void *obj);
  int readObjIdx();
  void drawObj(bool isReflection);
  void computeMousePull();
  void savingSnapshots_private(std::string logDirectory, std::string fileName);

  object::Background *background = nullptr;
  object::CheckerBoard *checkerboard = nullptr;
  object::Arrow *interactionArrow = nullptr;
  object::Sphere *interactionBall = nullptr;
  std::vector<object::Rectangle *> textBoard;
  std::vector<std::vector<std::string>> menuText;
  std::vector<bool> menuTextToggle;
  std::vector<bool> customToggle;
  float interactionMagnitude = 1.0;
  float realtimeFactor = 1.0;

  bool backgroundChanged = false, checkerboardChanged = false;

  std::vector<object::SingleBodyObject *> objs_;
  std::vector<object::MultiBodyObject *> supObjs_;
  std::vector<object::SingleBodyObject *> added_objs_;
  std::vector<object::MultiBodyObject *> added_supObjs_;
  std::vector<object::SingleBodyObject *> tobeRemoved_objs_;
  std::vector<object::MultiBodyObject *> tobeRemoved_supObjs_;
  std::vector<object::SingleBodyObject *> tobeRemovedAndDeleted_objs_;
  std::vector<object::Lines*> lines_;


  Display *display = nullptr;
  Shader_basic *shader_basic = nullptr;
  Shader_flat *shader_flat = nullptr;
  Shader_background *shader_background = nullptr;
  Shader_mouseClick *shader_mouseClick = nullptr;
  Shader_line *shader_line = nullptr;
  Shader_menu *shader_menu = nullptr;
  Shader_checkerboard *shader_checkerboard = nullptr;

  std::vector<Shader *> shaders_;
  std::vector<object::ShaderType> added_shaders_;
  std::vector<object::SingleBodyObject *> objectsInOrder_;

  unsigned imageCounter = 0;
  bool areThereimagesTosave = false;
  bool saveSnapShot = false;
  bool saveVideo_ = false;
  Camera *camera = nullptr;
  Light *light = nullptr;
  int windowWidth_, windowHeight_;
  double cameraDepth_;

  SDL_Event e;
  bool freeCamMode;
  float clearColor[4] = {1.0f, 1.0f, 1.0f, 0.0f};
  double FPS_ = 60.0;
  std::string image_dir, videoFileName;
  StopWatch watch;
  bool terminate = false;
  bool visRunning = true;
  std::mutex mtx;           // mutex for critical section
  std::mutex mtxLoop;           // mutex for critical section
  std::mutex mtxinit;           // mutex for critical section
  std::mutex mtxLight;
  std::mutex mtxCamera;

  pthread_t mainloopThread;
  MouseInput mouseInput;
  LightProp lightProp;
  CameraProp cameraProp;
  bool cameraPropChanged;
  bool lightPropChanged;
  int selectableIndexToBeAssigned = 0;
  int highlightedObjId = 16646655;
  int interactingObjSelectableId = -1;
  bool isInteracting_ = false;
  int interStartingX, interStartingY;
  Eigen::Vector3d interactionForce;
  int autoVideoRecordingNumber=0;
  std::vector<TTF_Font *> font;
  std::vector<bool> keyboardEvent;
  std::vector<bool> menuOn_;
  double actualFPS_=0;
  uint loopcounter=0;
  bool isGraphicsReady=false;
  bool isQuiting=false;
  enum {
    RAI_MAIN_MENU = 0,
    RAI_KEY_BOARD_HELP,
  };
  FILE* ffmpeg;

};

} // rai_graphics

#endif //PROJECT_RAI_GRAPHICS_HPP
