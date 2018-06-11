#ifndef DISPLAY_INCLUDED_H
#define DISPLAY_INCLUDED_H

#include <string>
#include <SDL2/SDL.h>

namespace rai_graphics {

class Display {
 public:
  Display(int width, int height, const std::string &title);

  void Clear(float r, float g, float b, float a);
  void SwapBuffers();
  void HideWindow();
  void ShowWindow();

  virtual ~Display();
  SDL_Window *m_window;
  SDL_Renderer *m_renderer;

 protected:
 private:
  void operator=(const Display &display) {}
  Display(const Display &display) {}

  SDL_GLContext m_glContext;
};

} // rai_graphics

#endif
