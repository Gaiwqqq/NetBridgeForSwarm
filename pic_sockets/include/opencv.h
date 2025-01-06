/*
 * Copyright (C) 2024 Coder.AN
 * Email: an.hongjun@foxmail.com
 * Page: www.anhongjun.top
 */
#ifndef __OPENCV_H__
#define __OPENCV_H__


#include <stdint.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgcodecs/legacy/constants_c.h>

#include <jpeglib.h>

void MatToJpgBuffer(const cv::Mat mat, std::vector<uint8_t>& buff, int quality=95);
void JpgBufferToMat(uint8_t* buff, uint32_t size, cv::Mat &mat);

struct RgbImg
{
    size_t width;
    size_t height;
    std::vector<uint8_t> buffer;
};

RgbImg JpgBufferToRgbBuffer(uint8_t* jpg_buffer, size_t jpg_size);
std::vector<uint8_t> RgbBufferToJpgBuffer(uint8_t* rgb_buffer, size_t width, size_t height, int quality);

#endif // __OPENCV_H__