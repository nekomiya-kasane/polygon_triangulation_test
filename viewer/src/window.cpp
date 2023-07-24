#include "easyx.h"
#include "graphics.h"

int main()
{
  initgraph(1024, 768);

  BeginBatchDraw();

  ExMessage msg;

  bool exit = false;
  while (!exit)
  {
    msg = getmessage(EX_KEY);
    cleardevice();

    if (msg.message == WM_KEYDOWN && msg.vkcode == VK_ESCAPE)
      exit = true;
  }

  closegraph();
  return 0;
}