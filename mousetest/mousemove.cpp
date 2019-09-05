#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

void mouseMove(float x, float y)
{
    Display *displayMain = XOpenDisplay(NULL);

    if (displayMain == NULL)
    {
        fprintf(stderr, "Could not open main display\n");
        exit(EXIT_FAILURE);
    }
    Screen *screen = DefaultScreenOfDisplay(displayMain);
    int width = screen->width;
    int height = screen->height;

    int screenX = round(x*width);
    int screenY = round(y*height);

    Window root = DefaultRootWindow(displayMain);
    XWarpPointer(displayMain, None, root, 0, 0, 0, 0, screenX, screenY);

    XCloseDisplay(displayMain);
}

int main(int arc, char** argv) {
    mouseMove(0.5, 0.5);
    return 0;
}
