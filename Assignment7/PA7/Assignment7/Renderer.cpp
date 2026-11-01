//
// Created by goksu on 2/25/20.
//

#include <algorithm>
#include <atomic>
#include <fstream>
#include <mutex>
#include <thread>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";

    // 按行把画面分给多个线程，每个线程只写自己负责的行，互不重叠
    unsigned int threadCount = std::max(1u, std::thread::hardware_concurrency());
    threadCount = std::min(threadCount, static_cast<unsigned int>(scene.height));
    std::cout << "Threads: " << threadCount << "\n";

    std::atomic<int> finishedRows{0};
    std::mutex progressMutex;

    auto renderRows = [&](int startRow, int endRow) {
        for (int j = startRow; j < endRow; ++j) {
            for (int i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                int pixelIndex = j * scene.width + i;
                for (int k = 0; k < spp; k++){
                    framebuffer[pixelIndex] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
            }

            // 进度条由主线程风格的互斥锁保护，避免多个线程同时刷新
            int done = ++finishedRows;
            if (done % 4 == 0 || done == scene.height) {
                std::lock_guard<std::mutex> lock(progressMutex);
                UpdateProgress(done / (float)scene.height);
            }
        }
    };

    std::vector<std::thread> threads;
    int rowsPerThread = (scene.height + threadCount - 1) / threadCount;
    for (unsigned int t = 0; t < threadCount; ++t) {
        int startRow = static_cast<int>(t) * rowsPerThread;
        int endRow = std::min(scene.height, startRow + rowsPerThread);
        if (startRow < endRow) {
            threads.emplace_back(renderRows, startRow, endRow);
        }
    }
    for (auto &thread : threads) {
        thread.join();
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
