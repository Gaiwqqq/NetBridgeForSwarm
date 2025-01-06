/*
 * Copyright (C) 2024 Coder.AN
 * Email: an.hongjun@foxmail.com
 * Page: www.anhongjun.top
 */
#include "opencv.h"


void MatToJpgBuffer(const cv::Mat mat, std::vector<uint8_t>& buff, int quality)
{
    if (mat.empty())
    {
        return;
    }
    std::vector<int> param = std::vector<int>(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = quality;
    cv::imencode(".jpg", mat, buff, param);
    return;
}

void JpgBufferToMat(uint8_t* buff, uint32_t size, cv::Mat &mat)
{
    std::vector<uint8_t> buffer(buff, buff + size);
    mat = cv::imdecode(buffer, CV_LOAD_IMAGE_COLOR);
    return;
}

RgbImg JpgBufferToRgbBuffer(uint8_t* jpg_buffer, size_t jpg_size)
{
    RgbImg rgb;
    
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *buffer;
    int row_stride;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    jpeg_mem_src(&cinfo, jpg_buffer, jpg_size);

    (void) jpeg_read_header(&cinfo, TRUE);
    (void) jpeg_start_decompress(&cinfo);

    rgb.width = cinfo.output_width;
    rgb.height = cinfo.output_height;

    row_stride = cinfo.output_width * cinfo.output_components;

    rgb.buffer.resize((rgb.width) * (rgb.height) * 3);

    buffer = (unsigned char*)malloc(row_stride);

    while (cinfo.output_scanline < cinfo.output_height) {
        row_pointer[0] = &(rgb.buffer[cinfo.output_scanline * (rgb.width) * 3]);
        (void) jpeg_read_scanlines(&cinfo, row_pointer, 1);
    }

    (void) jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    free(buffer);

    return rgb;
}

std::vector<uint8_t> RgbBufferToJpgBuffer(uint8_t* rgb_buffer, size_t width, size_t height, int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char* jpg_buffer = NULL;
    size_t jpg_size = 0;

    // Allocate and initialize a JPEG compression object
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    // Set the image dimensions and color space
    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    // Set the JPEG compression parameters
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    // Allocate memory for the output buffer
    jpeg_mem_dest(&cinfo, &jpg_buffer, &jpg_size);

    // Start the JPEG compression
    jpeg_start_compress(&cinfo, TRUE);

    // Loop through each scanline of the image
    while (cinfo.next_scanline < cinfo.image_height)
    {
        // Set the row pointer to point to the next scanline
        row_pointer[0] = (JSAMPROW)&rgb_buffer[cinfo.next_scanline * cinfo.image_width * 3];

        // Write the scanline to the JPEG file
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    // Finish the JPEG compression
    jpeg_finish_compress(&cinfo);

    // Clean up the compression object and return the encoded JPEG data
    jpeg_destroy_compress(&cinfo);

    std::vector<uint8_t> res(jpg_buffer, jpg_buffer + jpg_size);
    free(jpg_buffer);
    return res;
}
