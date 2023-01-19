
#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <string>
#include <cstdio>
#include <cstdint>

using std::uint8_t;

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image/stb_image_write.h"

// simple wrapper for stbi image class
// support basic IO operations
class Image {
public:
    // H x W x C
    int w, h, c;
    uint8_t* buf;
    Image() {
        // std::cout << "contruct"<<std::endl;
        buf = nullptr;
        w = h = c = 0;
    }

    Image (const std::string& path, int channel = 3) {
        channel = std::min(channel, 3);
        uint8_t* stbi_buf = stbi_load(path.c_str(), &w, &h, &c, 0);
        std::cout << "load image from " << path << " (H, W, C)=" << h << " " << w << " " << c << std::endl;
        if (channel != c) {
            buf = new uint8_t[w * h * channel];
            for (int i = 0; i < h; ++i) {
                for (int j = 0; j < w; ++j) {
                    uint8_t* ptr = buf + (w*i+j)*channel;
                    uint8_t* src_ptr = stbi_buf + (w*i+j)*c;
                    memcpy(ptr, src_ptr, channel);
                }
            }
            c = channel;
        } else {
            buf = new uint8_t[w * h * c];
            memcpy(buf, stbi_buf, w*h*c);
        }
        stbi_image_free(stbi_buf);
    }

    Image (int w, int h, int c = 3): w(w), h(h), c(c) {
        buf = new uint8_t[w*h*c];
        memset(buf, 0, w*h*c);
    }

    ~Image() {
        delete[] buf;
    }

    void save(const std::string& path) const {
        stbi_write_png(path.c_str(), w, h, c, buf, 0);
    }

    // image bitwise operations
    uint8_t get_color(int x, int y, int ch) const {
        x = std::max(0, std::min(x, h-1));
        y = std::max(0, std::min(y, w-1));
        ch = std::max(0, std::min(ch, c-1));
        return buf[(w*x+y)*c + ch];
    }

    uint8_t* get_pixel(int x, int y) {
        x = std::max(0, std::min(x, h-1));
        y = std::max(0, std::min(y, w-1));
        return buf + (w*x+y)*c;
    }
};

#endif
