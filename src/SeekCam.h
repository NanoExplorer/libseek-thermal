/*
 *  Abstract seek camera class
 *  Author: Maarten Vandersteegen
 */

#ifndef SEEK_CAM_H
#define SEEK_CAM_H

#include <opencv2/opencv.hpp>
#include "SeekDevice.h"

namespace LibSeek {

class SeekCam
{
public:
    /*
     *  Initialize the camera
     *  Returns true on success
     */
    bool open();

    /*
     *  Returns true when camera is initialized
     */
    bool isOpened();

    /*
     *  Close the camera
     */
    void close();

    /*
     *  Grab a frame
     *  Returns true on success
     */
    bool grab();
    bool grab2();

    /*
     *  Retrieve the last grabbed 14-bit frame
     *  Returns true on success
     */
    void retrieve(cv::Mat& dst);

    void rawRetrieve(cv::Mat& dst);
    /*
     *  Convert a 14-bit thermal measurement to an
     *  enhanced 8-bit greyscale image for visual inspection
     */
    void convertToGreyScale(cv::Mat& src, cv::Mat& dst);

    /*
     *  Read grabs and retrieves a frame
     *  Returns true on success
     */
    bool read(cv::Mat& dst);

    /*
     *  Read grabs and retrieves a frame
     *  Returns true on success
     *  designed for use in for loop. Does not request a new frame 
     *  beforehand. Use read2_starter beforehand to request the frame.
     */
    bool read2(cv::Mat& dst);
    bool read2_starter();

    void add_dead_pixel(std::vector<cv::Point> new_dead_pixels);
    bool set_additional_ffc(cv::Mat& newframe);
    /*
     *  Get the frame counter value
     */
    virtual int frame_counter() = 0;

protected:

    SeekCam(int vendor_id, int product_id, uint16_t* buffer, size_t raw_height, size_t raw_width, size_t request_size, cv::Rect roi, std::string ffc_filename);
    ~SeekCam();

    virtual bool init_cam() = 0;
    virtual int frame_id() = 0;
    bool open_cam();
    bool get_frame();
    bool request_frame();
    bool cam_fetch_frame();
    void print_usb_data(std::vector<uint8_t>& data);
    void create_dead_pixel_list(cv::Mat frame, cv::Mat& dead_pixel_mask,
                                            std::vector<cv::Point>& dead_pixel_list);
    void apply_dead_pixel_filter(cv::Mat& src, cv::Mat& dst);
    uint16_t calc_mean_value(cv::Mat& img, cv::Point p, uint32_t dead_pixel_marker);

    /*
     *  Variables
     */
    const int m_offset;

    std::string m_ffc_filename;
    bool m_is_opened;
    SeekDevice m_dev;
    uint16_t* m_raw_data;
    size_t m_raw_data_size;
    size_t m_request_size;
    cv::Mat m_raw_frame;
    //cv::Mat m_initial_frame;
    cv::Mat m_full_raw_frame;
    cv::Mat m_flat_field_calibration_frame;
    cv::Mat m_additional_ffc;
    cv::Mat m_dead_pixel_mask;
    std::vector<cv::Point> m_dead_pixel_list;
};

} /* LibSeek */

#endif /* SEEK_CAM_H */
