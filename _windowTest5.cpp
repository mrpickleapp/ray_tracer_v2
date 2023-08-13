#ifndef UNICODE
#define UNICODE
#endif 

#include <windows.h>
#include <wingdi.h>
#include <windowsx.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"
#include "data.h"

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

typedef uint8_t u8;
typedef uint32_t u32;

#define FILE_MENU_NEW 1
#define FILE_MENU_OPEN 2
#define FILE_MENU_EXIT 99
#define GENERATE_BUTTON 20
#define RERENDER 30

LRESULT CALLBACK WindowProcedure(HWND, UINT, WPARAM, LPARAM);
void AddMenus(HWND);
void AddControls(HWND);
void InitBitmap(HWND);

void ClearSreen(u32);
void DrawPixelRect(int, int, int, int, u32);
void BufferToBitmap();

void RenderScene(HWND);
void ThreadFunction(int, int, Vector, Vector, Vector, Vector);

void MSAAHorizontal();
void MSAAHorizThreadFunction(int, int, Vector, Vector, Vector, Vector, float);
Colour SuperSample(Vector, Vector, Vector, Vector);

void MSAAHorizontalDynamic();
void MSAAHoriDynamThreadFn(int, int, bool, int, int, Vector, Vector, Vector, Vector, float);
void SuperSampleFill(int, int, bool, int, int, Vector, Vector, Vector, Vector);

void SelectObject(int, int);

void ExportRender();
void ExportThreadFunction(uint8_t*, int, int, float*, float*, int, int, Vector, Vector, Vector, Vector);

HMENU hMenu;
HWND hName, hAge, hOut;

HDC DeviceContext;
BITMAPINFO BitmapInfo;
void* BitmapMemory;
int BitmapWidth;
int BitmapHeight;
int ClientWidth;
int ClientHeight;


Scene::Data scene = Scene::Data();
Sim::Data sim = Sim::Data(&scene);


const int N_THREADS = 8;


void SetBufferPixel(int X, int Y, u32 Colour) {
    u32 *Pixel = (u32 *)scene.PIXEL_BUFFER;
    Pixel += Y * BitmapWidth + X;
    *Pixel = Colour;
}

// Draws a pixel at X, Y (from top left corner)
void DrawPixel(int X, int Y, u32 Colour) {
    u32 *Pixel = (u32 *)BitmapMemory;
    Pixel += Y * BitmapWidth + X;
    *Pixel = Colour;
}

u32 GetBufferPixel(int X, int Y) {
    u32 *Pixel = (u32 *)scene.PIXEL_BUFFER + (Y * BitmapWidth + X);
    return *Pixel;
}

u32 GetPixel(int X, int Y) {
    u32 *Pixel = (u32 *)BitmapMemory + (Y * BitmapWidth + X);
    return *Pixel;
}

u8 GetPixelLum(int X, int Y) {
    u32 Pixel = GetPixel(X, Y);
    return (u8)(Pixel >> 24) & 0xff;
}

u8 GetPixelLum(u32 pixel) {
    return (u8)(pixel >> 24) & 0xff;
}

void ClearScreen(u32 Colour) {
    u32 *Pixel = (u32 *)BitmapMemory;
    for(int Index = 0; Index < BitmapWidth * BitmapHeight; ++Index) { 
        *Pixel++ = Colour; 
    }
}
 
void DrawPixelRect(int _X, int _Y, int _x_size, int _y_size, u32 colour) {

    for (int y = 0; y < _y_size & y + _Y < BitmapHeight; ++y) {
        for (int x = 0; x < _x_size & x + _X < BitmapWidth; ++x) {
            DrawPixel(_X + x, _Y + y, colour);
        }
    }
}

u32 RGBtoHex(u8 r, u8 g, u8 b)
{   
    return (((u8)((r + g + b)/3) & 0xff) << 24) + ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
    // return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}


void MSAAHorizontalDynamic() {

    // NEED TO MULTITHREAD THIS

    Vector ray_origin = scene.camera_position;
    Vector ray_direction = scene.camera_direction;
    Vector X_RAY_STEP = Vector(1, 0, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);
    Vector Y_RAY_STEP = Vector(0, 1, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);
    float MSAA_STEP = scene.RAY_STEP * sim.SAMPLE_RATE / sim.MSAA;

    int depth_room = sim.SAMPLE_RATE - sim.MSAA;
    bool floor_room = depth_room >= 0; 
    int n_colours = sim.MSAA * sim.MSAA;
    int f = sim.SAMPLE_RATE / sim.MSAA;

    if (sim.MULTITHREADING) {
        std::thread threads[N_THREADS];

        for (int t = 0; t < N_THREADS; ++t) {
            threads[t] = std::thread(&MSAAHoriDynamThreadFn, t, N_THREADS, floor_room, n_colours, f, ray_origin, ray_direction, X_RAY_STEP, Y_RAY_STEP, MSAA_STEP);
        }

        for (int t = 0; t < N_THREADS; ++t) {
            threads[t].join();
        }
    }

    std::thread thread = std::thread(&MSAAHoriDynamThreadFn, 0, 1, floor_room, n_colours, f, ray_origin, ray_direction, X_RAY_STEP, Y_RAY_STEP, MSAA_STEP);
    thread.join();

}

void MSAAHoriDynamThreadFn(int t, int _threads, bool floor_room, int n_colours, int f, Vector ray_origin, Vector ray_direction, Vector X_RAY_STEP, Vector Y_RAY_STEP, float MSAA_STEP) {
    
    for (int YS = sim.SAMPLE_RATE * t; YS < BitmapHeight; YS += sim.SAMPLE_RATE * _threads) {

        bool left_pixel_sampled = false;

        for (int XS = sim.SAMPLE_RATE; XS < BitmapWidth; XS += sim.SAMPLE_RATE) {

            u32 pixel = GetBufferPixel(XS, YS);
            u8 pixel_lum = GetPixelLum(pixel);

            // check left pixel
            u32 left_pixel = GetBufferPixel(XS-sim.SAMPLE_RATE, YS);
            u8 left_pixel_lum = GetPixelLum(left_pixel);

            bool left_thresh = abs(left_pixel_lum - pixel_lum) > sim.MSAA_CONTRAST_THRESHOLD;
            bool above_thresh = YS - sim.SAMPLE_RATE >= 0 ? abs(GetPixelLum(GetBufferPixel(XS, YS-sim.SAMPLE_RATE)) - pixel_lum) > sim.MSAA_CONTRAST_THRESHOLD : false;
            bool below_thresh = YS + sim.SAMPLE_RATE < BitmapHeight ? abs(GetPixelLum(GetBufferPixel(XS, YS+sim.SAMPLE_RATE)) - pixel_lum) > sim.MSAA_CONTRAST_THRESHOLD : false;

            if (left_thresh | above_thresh | below_thresh) {

                float x = scene.X_RAYS[XS];
                float y = scene.Y_RAYS[YS];
                Vector ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y));

                SuperSampleFill(XS, YS, floor_room, n_colours, f, ray_origin, ray_D, X_RAY_STEP.scaleByLength(MSAA_STEP), Y_RAY_STEP.scaleByLength(MSAA_STEP));

                if (!left_pixel_sampled) {
                    x = scene.X_RAYS[int(XS - sim.SAMPLE_RATE)];
                    ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y));
                    
                    SuperSampleFill(XS-sim.SAMPLE_RATE, YS, floor_room, n_colours, f, ray_origin, ray_D, X_RAY_STEP.scaleByLength(MSAA_STEP), Y_RAY_STEP.scaleByLength(MSAA_STEP));
                }
                
                left_pixel_sampled = true;
            } 
            else {
                left_pixel_sampled = false;
            }  
        }
    }
}


void MSAAHorizontal() {

    Vector ray_origin = scene.camera_position;
    Vector ray_direction = scene.camera_direction;
    Vector X_RAY_STEP = Vector(1, 0, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);
    Vector Y_RAY_STEP = Vector(0, 1, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);
    float MSAA_STEP = scene.RAY_STEP * sim.SAMPLE_RATE / sim.MSAA;

    if (sim.MULTITHREADING) {
        std::thread threads[N_THREADS];

        for (int t = 0; t < N_THREADS; ++t) {
            threads[t] = std::thread(&MSAAHorizThreadFunction, t, N_THREADS, ray_origin, ray_direction, X_RAY_STEP, Y_RAY_STEP, MSAA_STEP);
        }

        for (int t = 0; t < N_THREADS; ++t) {
            threads[t].join();
        }
    }

    else {
        std::thread thread = std::thread(&MSAAHorizThreadFunction, 0, 1, ray_origin, ray_direction, X_RAY_STEP, Y_RAY_STEP, MSAA_STEP);
        thread.join();
    }
}

void MSAAHorizThreadFunction(int t, int _threads, Vector ray_origin, Vector ray_direction, Vector X_RAY_STEP, Vector Y_RAY_STEP, float MSAA_STEP) {
    
    for (int YS = sim.SAMPLE_RATE * t; YS < BitmapHeight; YS += sim.SAMPLE_RATE * _threads) {

        bool left_pixel_sampled = false;

        for (int XS = sim.SAMPLE_RATE; XS < BitmapWidth; XS += sim.SAMPLE_RATE) {

            u32 pixel = GetBufferPixel(XS, YS);
            u8 pixel_lum = GetPixelLum(pixel);

            // check left pixel
            u8 left_lum = GetPixelLum(GetBufferPixel(XS-sim.SAMPLE_RATE, YS));
            u8 above_lum = GetPixelLum(GetBufferPixel(XS, YS-sim.SAMPLE_RATE));

            bool left_thresh = abs(left_lum - pixel_lum) > sim.MSAA_CONTRAST_THRESHOLD;
            bool above_thresh = YS - sim.SAMPLE_RATE >= 0 ? abs(GetPixelLum(GetBufferPixel(XS, YS-sim.SAMPLE_RATE)) - pixel_lum) > sim.MSAA_CONTRAST_THRESHOLD : false;
            bool below_thresh = YS + sim.SAMPLE_RATE < BitmapHeight ? abs(GetPixelLum(GetBufferPixel(XS, YS+sim.SAMPLE_RATE)) - pixel_lum) > sim.MSAA_CONTRAST_THRESHOLD : false;

            if (left_thresh | above_thresh | below_thresh) {

                float x = scene.X_RAYS[XS];
                float y = scene.Y_RAYS[YS];
                Vector ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y));

                Colour col = SuperSample(ray_origin, ray_D, X_RAY_STEP.scaleByLength(MSAA_STEP), Y_RAY_STEP.scaleByLength(MSAA_STEP));
                
                // Fill all pixels in square
                DrawPixelRect(XS, YS, sim.SAMPLE_RATE, sim.SAMPLE_RATE, RGBtoHex(col.r, col.g, col.b));

                if (!left_pixel_sampled) {

                    // do subsampling on previous pixel here
                    x = scene.X_RAYS[int(XS - sim.SAMPLE_RATE)];
                    ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y));

                    // Re-calculating the original pixel value here - could optimise
                    Colour col = SuperSample(ray_origin, ray_D, X_RAY_STEP.scaleByLength(MSAA_STEP), Y_RAY_STEP.scaleByLength(MSAA_STEP));

                    DrawPixelRect(XS-sim.SAMPLE_RATE, YS, sim.SAMPLE_RATE, sim.SAMPLE_RATE, RGBtoHex(col.r, col.g, col.b));
                }
                
                left_pixel_sampled = true;

            } 
            else {
                left_pixel_sampled = false;
            }  
        }
    }
}

void BufferToBitmap() {
    u32 *BitmapPixel = (u32 *)BitmapMemory;
    u32 *BufferPixel = (u32 *)scene.PIXEL_BUFFER;
    for(int Index = 0; Index < BitmapWidth * BitmapHeight; ++Index) { 
        *BitmapPixel++ = *BufferPixel++; 
    }
}


int WINAPI WinMain(HINSTANCE hInst, HINSTANCE hPrevInst, LPSTR args, int ncmdshow) {

    const wchar_t CLASS_NAME[]  = L"myWindowClass";

    WNDCLASSW wc_main = {0};

    wc_main.hbrBackground        = (HBRUSH)COLOR_WINDOW;
    wc_main.hCursor              = LoadCursor(NULL, IDC_CROSS);
    wc_main.hInstance            = hInst;
    wc_main.lpszClassName        = CLASS_NAME;
    wc_main.lpfnWndProc          = WindowProcedure;

    if (!RegisterClassW(&wc_main)) {
        return -1;
    }

    HWND hWnd = CreateWindowW(L"myWindowClass", L"Main", WS_OVERLAPPEDWINDOW | WS_VISIBLE, 
        10, 10, 800, 600,
        NULL, NULL, NULL, NULL
    );

    MSG msg = {0};

    bool RUNNING = true;

    while (RUNNING) {
        RUNNING = (GetMessage(&msg, NULL, 0, 0) > 0);

        TranslateMessage(&msg);
        DispatchMessage(&msg);

        if (!sim.RENDER_PAUSED) {

            // SleepEx((DWORD)sim.TICK_MS, true);      // should only sleep difference from last tick if needed

            // TEMP SLOWING DOWN SIM
            // SleepEx((DWORD)1000, true); 

            scene.camera_direction = Vector(0, 0, scene.Z).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);

            sim.PhysicsIteration();

            scene.updateFromPhysObjects(sim);

            RenderScene(hWnd);  

            StretchDIBits(DeviceContext, 0, 0, BitmapWidth, BitmapHeight, 0, 0, ClientWidth, ClientHeight, BitmapMemory, &BitmapInfo, DIB_RGB_COLORS, SRCCOPY);  
        }
    }

    return 0;
}

LRESULT CALLBACK WindowProcedure(HWND hWnd, UINT msg, WPARAM wp, LPARAM lp) {

    switch(msg) {

        case WM_COMMAND:
            switch(wp) {
                case FILE_MENU_EXIT:
                    DestroyWindow(hWnd);
                    break;

                case GENERATE_BUTTON:
                    RenderScene(hWnd);
                    StretchDIBits(DeviceContext, 0, 0, BitmapWidth, BitmapHeight, 0, 0, ClientWidth, ClientHeight, BitmapMemory, &BitmapInfo, DIB_RGB_COLORS, SRCCOPY);
                    break;
            }
            break;

        case WM_KEYDOWN:
            // std::cout << wp << std::endl;
            if (wp == 0x50) {
                sim.RENDER_PAUSED = !sim.RENDER_PAUSED;
            }
            else if (wp == VK_F1) {
                sim.MODE = RENDER_COLOUR_MODE;
            }
            else if (wp == VK_F2) {
                sim.MODE = RENDER_SHADED_MODE;
            }
            else if (wp == VK_F3) {
                sim.MODE = RENDER_FULL_MODE;
            }
            else if (wp == VK_F5) {
                sim.MULTITHREADING = !sim.MULTITHREADING;
            }
            else if (wp == VK_F6) {
                if (sim.MSAA_TYPE == MSAA_NORMAL) {
                    sim.MSAA_TYPE = MSAA_SUB_SAMPLE;
                }
                else if (sim.MSAA_TYPE == MSAA_SUB_SAMPLE) {
                    sim.MSAA_TYPE = MSAA_NORMAL;
                }
                std::cout << "MSAA TYPE: " << sim.MSAA_TYPE << std::endl; 
            }
            else if (wp == VK_F7) {
                std::cout << "Export button" << std::endl;
                ExportRender();
            }
            else if (wp == 0x31) {
                sim.SAMPLE_RATE = 1;
            }
            else if (wp == 0x32) {
                sim.SAMPLE_RATE = 2;
            }
            else if (wp == 0x33) {
                sim.SAMPLE_RATE = 3;
            }
            else if (wp == 0x34) {
                sim.SAMPLE_RATE = 4;
            }
            else if (wp == 0x35) {
                sim.SAMPLE_RATE = 5;
            }
            else if (wp == 0x36) {
                sim.SAMPLE_RATE = 6;
            }
            else if (wp == 0x37) {
                sim.SAMPLE_RATE = 7;
            }
            else if (wp == 0x38) {
                sim.SAMPLE_RATE = 8;
            }
            else if (wp == VK_LEFT) {
                scene.CAMERA_ROT_Y += sim.CAMERA_ROT_INCR * scene.FOV;
            }
            else if (wp == VK_RIGHT) {
                scene.CAMERA_ROT_Y -= sim.CAMERA_ROT_INCR * scene.FOV;
            }
            else if (wp == VK_UP) {
                scene.CAMERA_ROT_X -= sim.CAMERA_ROT_INCR * scene.FOV;
            }
            else if (wp == VK_DOWN) {
                scene.CAMERA_ROT_X += sim.CAMERA_ROT_INCR * scene.FOV;
            }
            else if (wp == 0x41) {      // A key - LEFT
                scene.camera_position = scene.camera_position.addVector(Vector(-1 * scene.FOV, 0, 0).scaleByLength(sim.CAMERA_TRANS_INC).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X));
            }
            else if (wp == 0x44) {      // D key - RIGHT
                scene.camera_position = scene.camera_position.addVector(Vector(1 * scene.FOV, 0, 0).scaleByLength(sim.CAMERA_TRANS_INC).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X));
            }
            else if (wp == 0x57) {      // W key - FORWARDS
                scene.camera_position = scene.camera_position.addVector(Vector(0, 0, -1 * scene.FOV).scaleByLength(sim.CAMERA_TRANS_INC).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X));
            }
            else if (wp == 0x53) {      // S key - BACKWARDS
                scene.camera_position = scene.camera_position.addVector(Vector(0, 0, 1 * scene.FOV).scaleByLength(sim.CAMERA_TRANS_INC).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X));
            }
            else if (wp == 0x52) {      // R key - UP
                scene.camera_position = scene.camera_position.addVector(Vector(0, 1 * scene.FOV, 0).scaleByLength(sim.CAMERA_TRANS_INC).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X));
            }
            else if (wp == 0x46) {      // F key - DOWN
                scene.camera_position = scene.camera_position.addVector(Vector(0, -1 * scene.FOV, 0).scaleByLength(sim.CAMERA_TRANS_INC).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X));
            }
            else if (wp == 0x48) {      // H key - HALT
            }
            else if (wp == VK_OEM_6) {  // ] - increase MSAA
                sim.MSAA += 1;
                std::cout << "MSAA: " << sim.MSAA << std::endl;
            }
            else if (wp == VK_OEM_4) {  // ] - decrease MSAA
                if (sim.MSAA > 1) {
                    sim.MSAA -= 1;
                }
                std::cout << "MSAA: " << sim.MSAA << std::endl;
            }
            else if (wp == VK_PRIOR) {
                scene.FOV *= sim.FOV_INCR;
                scene.InitRays();
            } 
            else if (wp == VK_NEXT) {
                scene.FOV /= sim.FOV_INCR;
                scene.InitRays();
            }
            else if (wp == VK_SPACE) {
                scene.AddSphere();
            }
            break;

        case WM_LBUTTONDOWN:
            SelectObject(GET_X_LPARAM(lp), GET_Y_LPARAM(lp));
            break;

        case WM_RBUTTONDOWN:
            scene.AddSphere(GET_X_LPARAM(lp), GET_Y_LPARAM(lp), Vector(0, 0, -1));
            break;

        case WM_CREATE:
            InitBitmap(hWnd);
            RenderScene(hWnd);
            StretchDIBits(DeviceContext, 0, 0, BitmapWidth, BitmapHeight, 0, 0, ClientWidth, ClientHeight, BitmapMemory, &BitmapInfo, DIB_RGB_COLORS, SRCCOPY);
            // AddMenus(hWnd);
            // AddControls(hWnd);
            break;
        
        case WM_DESTROY:
            PostQuitMessage(0);
            break;

        case WM_SIZE:
            InitBitmap(hWnd);
            RenderScene(hWnd);
            StretchDIBits(DeviceContext, 0, 0, BitmapWidth, BitmapHeight, 0, 0, ClientWidth, ClientHeight, BitmapMemory, &BitmapInfo, DIB_RGB_COLORS, SRCCOPY);
            // RenderScene(hWnd);
            break;

        case WM_PAINT:
            // StretchDIBits(DeviceContext, 0, 0, BitmapWidth, BitmapHeight, 0, 0, ClientWidth, ClientHeight, BitmapMemory, &BitmapInfo, DIB_RGB_COLORS, SRCCOPY);
            break;

        default:
            return DefWindowProcW(hWnd, msg, wp, lp);
    }

    return (LRESULT)1;
}


void InitBitmap(HWND hWnd) {

    // Get client area dimensions 
    RECT ClientRect;
    GetClientRect(hWnd, &ClientRect);
    ClientWidth = ClientRect.right - ClientRect.left;
    ClientHeight = ClientRect.bottom - ClientRect.top;
    
    BitmapWidth = ClientWidth;
    BitmapHeight = ClientHeight;

    // Allocate memory for the bitmap
    int BytesPerPixel = sizeof(u32);
    BitmapMemory = VirtualAlloc(0, BitmapWidth * BitmapHeight * BytesPerPixel, MEM_RESERVE|MEM_COMMIT, PAGE_READWRITE);

    // BitmapInfo struct for StretchDIBits
    BitmapInfo.bmiHeader.biSize = sizeof(BitmapInfo.bmiHeader);
    BitmapInfo.bmiHeader.biWidth = BitmapWidth;

    // Negative height makes top left as the coordinate system origin for the DrawPixel function, otherwise its bottom left
    BitmapInfo.bmiHeader.biHeight = -BitmapHeight;
    BitmapInfo.bmiHeader.biPlanes = 1;
    BitmapInfo.bmiHeader.biBitCount = 32;
    BitmapInfo.bmiHeader.biCompression = BI_RGB;

    DeviceContext = GetDC(hWnd);

    scene.SetDims(BitmapWidth, BitmapHeight);
}


void RenderScene(HWND hWnd) {

    Vector ray_origin = scene.camera_position;
    Vector ray_direction = scene.camera_direction;
    Ray ray = Ray(ray_origin, ray_direction, scene.SPACE_RI);

    Vector X_RAY_STEP = Vector(1, 0, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);
    Vector Y_RAY_STEP = Vector(0, 1, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);

    if (sim.MULTITHREADING) {
        std::thread threads[N_THREADS];

        for (int t = 0; t < N_THREADS; ++t) {
            threads[t] = std::thread(&ThreadFunction, t, N_THREADS, ray_origin, ray_direction, X_RAY_STEP, Y_RAY_STEP);
        }

        for (int t = 0; t < N_THREADS; ++t) {
            threads[t].join();
        }
    } else {
        std::thread thread = std::thread(&ThreadFunction, 0, 1, ray_origin, ray_direction, X_RAY_STEP, Y_RAY_STEP);
        thread.join();
    }

    BufferToBitmap();

    if (sim.MSAA > 1) {
        if (sim.MSAA_TYPE == MSAA_NORMAL) {
            MSAAHorizontal();
        }
        else if (sim.MSAA_TYPE == MSAA_SUB_SAMPLE) {
            MSAAHorizontalDynamic();
        }
    }
}

void ThreadFunction(int t, int _threads, Vector ray_origin, Vector ray_direction, Vector X_RAY_STEP, Vector Y_RAY_STEP) {
    
    // RAY-TRACING LOOP
    for (int Y = t*sim.SAMPLE_RATE; Y < BitmapHeight; Y+=_threads*sim.SAMPLE_RATE) {        // threads take alternate rows
        for (int X = 0; X < BitmapWidth; X+=sim.SAMPLE_RATE) {

            float x = scene.X_RAYS[X];
            float y = scene.Y_RAYS[Y];
            
            // New ray
            Vector ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y)).normalise();
            Ray ray = Ray(ray_origin, ray_D, scene.SPACE_RI);

            Colour col;

            switch(sim.MODE) {
                case RENDER_COLOUR_MODE:
                    col = ray.objectColour(scene.objects, scene.N_OBJECTS, scene.background_colour);
                    break;
                case RENDER_SHADED_MODE:
                    col = ray.shadedColour(scene.objects, scene.N_OBJECTS, scene.background_colour);
                    break;
                case RENDER_FULL_MODE:
                    col = ray.aggColour(scene.objects, scene.N_OBJECTS, scene.background_colour, scene.global_lights, scene.N_GLOBAL_LIGHTS, scene.point_lights, scene.N_POINT_LIGHTS, 0, scene.MAX_BOUNCES);
            }

            if (sim.SAMPLE_RATE == 1) {
                SetBufferPixel(X, Y, RGBtoHex(col.r, col.g, col.b));
            }

            // Fill all pixels
            else {
                for (int XS = 0; XS < sim.SAMPLE_RATE & X + XS < BitmapWidth; ++XS) {
                    for (int YS = 0; YS < sim.SAMPLE_RATE & Y + YS < BitmapHeight; ++YS) {
                        SetBufferPixel(X + XS, Y + YS, RGBtoHex(col.r, col.g, col.b));
                    }
                }
            }
        }
    }
}

Colour SuperSample(Vector ray_origin, Vector ray_direction, Vector X_RAY_STEP, Vector Y_RAY_STEP) {

    float r = 0;
    float g = 0;
    float b = 0;

    int denom = 0;

    for (int y = 0; y < sim.MSAA; ++y) {
        for (int x = 0; x < sim.MSAA; ++x) {
            
            Vector ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y));
            Ray ray = Ray(ray_origin, ray_D, scene.SPACE_RI);

            Colour col;

            switch(sim.MODE) {
                case RENDER_COLOUR_MODE:
                    col = ray.objectColour(scene.objects, scene.N_OBJECTS, scene.background_colour);
                    break;
                case RENDER_SHADED_MODE:
                    col = ray.shadedColour(scene.objects, scene.N_OBJECTS, scene.background_colour);
                    break;
                case RENDER_FULL_MODE:
                    col = ray.aggColour(scene.objects, scene.N_OBJECTS, scene.background_colour, scene.global_lights, scene.N_GLOBAL_LIGHTS, scene.point_lights, scene.N_POINT_LIGHTS, 0, scene.MAX_BOUNCES);
            }

            r += col.r;
            g += col.g;
            b += col.b;

            ++denom;
        }
    }

    return Colour(r/denom, g/denom, b/denom).ceil();
}


void SuperSampleFill(int _X, int _Y, bool floor_room, int n_colours, int f, Vector ray_origin, Vector ray_direction, Vector X_RAY_STEP, Vector Y_RAY_STEP) {

    Colour* colours = new Colour[n_colours];

    for (int y = 0; y < sim.MSAA; ++y) {
        for (int x = 0; x < sim.MSAA; ++x) {

            Vector ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).subtractVector(Y_RAY_STEP.scaleByLength(y));
            Ray ray = Ray(ray_origin, ray_D, scene.SPACE_RI);

            Colour col;

            switch(sim.MODE) {
                case RENDER_COLOUR_MODE:
                    col = ray.objectColour(scene.objects, scene.N_OBJECTS, scene.background_colour);
                    break;
                case RENDER_SHADED_MODE:
                    col = ray.shadedColour(scene.objects, scene.N_OBJECTS, scene.background_colour);
                    break;
                case RENDER_FULL_MODE:
                    col = ray.aggColour(scene.objects, scene.N_OBJECTS, scene.background_colour, scene.global_lights, scene.N_GLOBAL_LIGHTS, scene.point_lights, scene.N_POINT_LIGHTS, 0, scene.MAX_BOUNCES);
            }

            if (!floor_room) {
                colours[int(y * sim.MSAA + x)] = col;
            }

            else {
                DrawPixelRect(_X + x*f, _Y + y*f, f, f, RGBtoHex(col.r, col.g, col.b));
            }
        }
    }

    // if no floor room, fill average
    if (!floor_room) {
        
        float r = 0;
        float g = 0;
        float b = 0;

        for (int i = 0; i < n_colours; ++i) {
            Colour col = colours[i];
            r += col.r;
            g += col.g;
            b += col.b;
        }
        
        u32 average_col = RGBtoHex(r/n_colours, g/n_colours, b/n_colours);

        DrawPixelRect(_X, _Y, sim.SAMPLE_RATE, sim.SAMPLE_RATE, average_col);
    }
    

    delete [] colours;
}


void SelectObject(int X, int Y) {

    float x = scene.X_RAYS[X];
    float y = scene.Y_RAYS[Y];
    
    Vector X_RAY_STEP = Vector(1, 0, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);
    Vector Y_RAY_STEP = Vector(0, 1, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);

    Vector ray_origin = scene.camera_position;
    Vector ray_D = scene.camera_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y)).normalise();
    Ray ray = Ray(ray_origin, ray_D);

    Intersection terminus = ray.nearestIntersect(scene.objects, scene.N_OBJECTS);

    if (terminus.intersects) {
        sim.selected_object = terminus.object_ptr;
        std::cout << "Object " << sim.selected_object->id << " selected" << std::endl;
    } else {
        sim.selected_object = NULL;
    }
}


void AddMenus(HWND hWnd) {

    hMenu = CreateMenu();
    HMENU hFileMenu = CreateMenu();
    HMENU hSubMenu = CreateMenu();

    AppendMenu(hSubMenu, MF_STRING, 0, L"Nothing");

    AppendMenu(hFileMenu, MF_STRING, FILE_MENU_NEW, L"New");
    AppendMenu(hFileMenu, MF_POPUP, (UINT_PTR)hSubMenu, L"Open");
    AppendMenu(hFileMenu, MF_SEPARATOR, 0, 0);
    AppendMenu(hFileMenu, MF_STRING, FILE_MENU_EXIT, L"Exit");

    AppendMenu(hMenu, MF_POPUP, (UINT_PTR)hFileMenu, L"File");
    AppendMenu(hMenu, MF_STRING, 10, L"Help");

    SetMenu(hWnd, hMenu);

}

void AddControls(HWND hWnd) {

    CreateWindowW(L"static", L"Name:", WS_VISIBLE | WS_CHILD, 100, 50, 98, 48, hWnd, NULL, NULL, NULL);
    hName = CreateWindowW(L"edit", L"", WS_VISIBLE | WS_CHILD | WS_BORDER, 200, 50, 98, 48, hWnd, NULL, NULL, NULL);

    CreateWindowW(L"static", L"Age:", WS_VISIBLE | WS_CHILD, 100, 100, 98, 48, hWnd, NULL, NULL, NULL);
    hAge = CreateWindowW(L"edit", L"", WS_VISIBLE | WS_CHILD | WS_BORDER, 200, 100, 98, 48, hWnd, NULL, NULL, NULL);

    CreateWindowW(L"button", L"Generate", WS_VISIBLE | WS_CHILD, 200, 150, 98, 48, hWnd, (HMENU)GENERATE_BUTTON, NULL, NULL);
    hOut = CreateWindowW(L"edit", L"", WS_VISIBLE | WS_CHILD | WS_BORDER, 100, 200, 200, 100, hWnd, NULL, NULL, NULL);

}


void ExportRender() {

    std::cout << "Exporting..." << std::endl;

    int X_DIM = 2000;
    int Y_DIM = 1400;

    int TOTAL_RAYS = X_DIM * Y_DIM;

    float RAY_STEP = 0.005 * (200.0 / X_DIM) * scene.FOV; 

    // GENERATE OFFSET GRID
    float* X_RAYS = new float[X_DIM];
    for (int i = 0; i < X_DIM; ++i) {
        int n = -X_DIM/2 + i;
        X_RAYS[i] = RAY_STEP * n;
    }

    float* Y_RAYS = new float[Y_DIM];
    for (int i = 0; i < Y_DIM; ++i) {
        int n = Y_DIM/2 - i;
        Y_RAYS[i] = RAY_STEP * n;
    }

    Vector ray_origin = scene.camera_position;
    Vector ray_direction = scene.camera_direction;
    Ray ray = Ray(ray_origin, ray_direction, scene.SPACE_RI);

    Vector X_RAY_STEP = Vector(1, 0, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);
    Vector Y_RAY_STEP = Vector(0, 1, 0).rotateIntrinsic(scene.CAMERA_ROT_Z, scene.CAMERA_ROT_Y, scene.CAMERA_ROT_X);

    uint8_t* pixels = new uint8_t[TOTAL_RAYS * 3];

    std::thread threads[N_THREADS];

    for (int t = 0; t < N_THREADS; ++t) {
        threads[t] = std::thread(&ExportThreadFunction, pixels, X_DIM, Y_DIM, X_RAYS, Y_RAYS, t, N_THREADS, ray_origin, ray_direction, X_RAY_STEP, Y_RAY_STEP);
    }

    for (int t = 0; t < N_THREADS; ++t) {
        threads[t].join();
    }


    // anti-alias here


    const char* filename = "RENDER_EXPORT.png";

    int comp = 3; // RGB
    int w = X_DIM;
    int h = Y_DIM;
    stbi_write_png_compression_level = 0;
    stbi_write_png(filename, w, h, comp, pixels, w*3);

    std::cout << "Export Complete" << std::endl;

    delete [] pixels;
    delete [] X_RAYS;
    delete [] Y_RAYS;
}

void ExportThreadFunction(uint8_t* pixels, int X_DIM, int Y_DIM, float* X_RAYS, float* Y_RAYS, int t, int _threads, Vector ray_origin, Vector ray_direction, Vector X_RAY_STEP, Vector Y_RAY_STEP) {
    
    // RAY-TRACING LOOP
    for (int Y = t; Y < Y_DIM; Y+=_threads) {        // threads take alternate rows
        for (int X = 0; X < X_DIM; ++X) {

            float x = X_RAYS[X];
            float y = Y_RAYS[Y];
            
            // New ray
            Vector ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y)).normalise();
            Ray ray = Ray(ray_origin, ray_D, scene.SPACE_RI);

            Colour col;

            switch(sim.MODE) {
                case RENDER_COLOUR_MODE:
                    col = ray.objectColour(scene.objects, scene.N_OBJECTS, scene.background_colour);
                    break;
                case RENDER_SHADED_MODE:
                    col = ray.shadedColour(scene.objects, scene.N_OBJECTS, scene.background_colour);
                    break;
                case RENDER_FULL_MODE:
                    col = ray.aggColour(scene.objects, scene.N_OBJECTS, scene.background_colour, scene.global_lights, scene.N_GLOBAL_LIGHTS, scene.point_lights, scene.N_POINT_LIGHTS, 0, scene.MAX_BOUNCES);
            }

            int i1 = (Y*X_DIM + X) * 3;

            pixels[i1] = col.r;
            pixels[i1 + 1] = col.g;
            pixels[i1 + 2] = col.b;
        }
    }
}

/*

void MSAAHorizontalExport(uint8_t* pixels, int X_DIM, int Y_DIM, float* X_RAYS, float* Y_RAYS, Vector ray_origin, Vector ray_direction, Vector X_RAY_STEP, Vector Y_RAY_STEP) {

    float MSAA_STEP = scene.RAY_STEP / sim.MSAA;

    if (sim.MULTITHREADING) {
        std::thread threads[N_THREADS];

        for (int t = 0; t < N_THREADS; ++t) {
            threads[t] = std::thread(&MSAAHorizExportThreadFunction, t, N_THREADS, ray_origin, ray_direction, X_RAY_STEP, Y_RAY_STEP, MSAA_STEP);
        }

        for (int t = 0; t < N_THREADS; ++t) {
            threads[t].join();
        }
    }

    else {
        std::thread thread = std::thread(&MSAAHorizExportThreadFunction, 0, 1, ray_origin, ray_direction, X_RAY_STEP, Y_RAY_STEP, MSAA_STEP);
        thread.join();
    }
}

void MSAAHorizExportThreadFunction(uint8_t* pixels, int X_DIM, int Y_DIM, float* X_RAYS, float* Y_RAYS, int t, int _threads, Vector ray_origin, Vector ray_direction, Vector X_RAY_STEP, Vector Y_RAY_STEP, float MSAA_STEP) {
    
    for (int YS = t; YS < Y_DIM; YS += _threads) {

        bool left_pixel_sampled = false;

        for (int XS = 1; XS < X_DIM; ++XS) {

            int i1 = (YS*X_DIM + XS) * 3;

            u8 pixel_r = pixels[i1];
            u8 pixel_g = pixels[i1 + 1];
            u8 pixel_b = pixels[i1 + 2];
            u8 pixel_lum = (u8)((pixel_r + pixel_g + pixel_b) / 3);

            u8 left_pixel_r = pixels[i1 - 3];
            u8 left_pixel_g = pixels[i1 - 3 + 1];
            u8 left_pixel_b = pixels[i1 - 3 + 2];
            u8 left_pixel_lum = (u8)((left_pixel_r + left_pixel_g + left_pixel_b) / 3);

            bool left_thresh = abs(left_pixel_lum - pixel_lum) > sim.MSAA_CONTRAST_THRESHOLD;

            bool above_thresh = false;

            if (YS > 0) {
                u8 above_pixel_r = pixels[i1 - X_DIM];
                u8 above_pixel_g = pixels[i1 - X_DIM + 1];
                u8 above_pixel_b = pixels[i1 - X_DIM + 2];
                u8 above_pixel_lum = (u8)((above_pixel_r + above_pixel_g + above_pixel_b) / 3);

                above_thresh = abs(above_pixel_lum - pixel_lum) > sim.MSAA_CONTRAST_THRESHOLD;
            }

            bool below_thresh = false;

            if (YS + 1 < Y_DIM) {
                u8 below_pixel_r = pixels[i1 - X_DIM];
                u8 below_pixel_g = pixels[i1 - X_DIM + 1];
                u8 below_pixel_b = pixels[i1 - X_DIM + 2];
                u8 below_pixel_lum = (u8)((below_pixel_r + below_pixel_g + below_pixel_b) / 3);

                below_thresh = abs(below_pixel_lum - pixel_lum) > sim.MSAA_CONTRAST_THRESHOLD;
            }

            if (left_thresh | above_thresh | below_thresh) {

                float x = X_RAYS[XS];
                float y = Y_RAYS[YS];
                Vector ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y));

                Colour col = SuperSample(ray_origin, ray_D, X_RAY_STEP.scaleByLength(MSAA_STEP), Y_RAY_STEP.scaleByLength(MSAA_STEP));
                
                

                if (!left_pixel_sampled) {

                    // do subsampling on previous pixel here
                    x = scene.X_RAYS[int(XS - sim.SAMPLE_RATE)];
                    ray_D = ray_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y));

                    // Re-calculating the original pixel value here - could optimise
                    Colour col = SuperSample(ray_origin, ray_D, X_RAY_STEP.scaleByLength(MSAA_STEP), Y_RAY_STEP.scaleByLength(MSAA_STEP));

                    DrawPixelRect(XS-sim.SAMPLE_RATE, YS, sim.SAMPLE_RATE, sim.SAMPLE_RATE, RGBtoHex(col.r, col.g, col.b));
                }
                
                left_pixel_sampled = true;

            } 
            else {
                left_pixel_sampled = false;
            }  
        }
    }
}

*/