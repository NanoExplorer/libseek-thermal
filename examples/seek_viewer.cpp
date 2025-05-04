// Seek Thermal Viewer/Streamer
// http://github.com/fnoop/maverick

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "seek.h"
#include "SeekCam.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
#include "args.h"
#include <sstream>
#include <fcntl.h>

#ifdef __linux__
#include <linux/videodev2.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

using namespace cv;
using namespace LibSeek;

const std::string WINDOW_NAME = "SeekThermal";

// Setup sig handling
static volatile sig_atomic_t sigflag = 0;


bool auto_exposure_lock = false;

void handle_sig(int sig) {
    (void)sig;
    sigflag = 1;
}
// cv::Mat openFile(std::string fname, cv::Mat &m_raw_frame)
// {
//     cv::Mat m_additional_ffc;
//     if (fname != std::string()) {
//         m_additional_ffc = cv::imread(fname, -1);
//         if (m_additional_ffc.type() != m_raw_frame.type()) {
//             std::cerr<<"Error: "<<fname.c_str()<<"not found or it has the wrong type: "<<m_additional_ffc.type()<<std::endl;
//             return cv::Mat::zeros(0,0,1);
//         }

//         if (m_additional_ffc.size() != m_raw_frame.size()) {
//             std::cerr<<"Error: expected "<<fname.c_str()<<" to have size ["<<m_raw_frame.cols<<","<<m_raw_frame.rows<<"], got ["<<m_additional_ffc.cols<<","<<m_additional_ffc.rows<<"]"<<std::endl;
//             return cv::Mat::zeros(0,0,1);
//         }
//     }

//     return m_additional_ffc;
// }
// Function to process a raw (corrected) seek frame
void process_frame(Mat &inframe, Mat &outframe, float scale, int colormap, int rotate, int last[], int staticMin, int staticMax) {
    
    Mat frame_g8, frame_g16,temp,tempmask; // Transient Mat containers for processing
    int cols=inframe.cols,rows=inframe.rows;

    ushort min,max,low,high;
    cv::Scalar imMean,imStd;
    cv::meanStdDev(inframe,imMean,imStd);
    frame_g16=cv::Mat::zeros(cols,rows,CV_16UC1);

    double raw_maxvalue;
    cv::minMaxIdx(inframe,0,&raw_maxvalue);
    //std::cout << raw_maxvalue << std::endl;

    low=imMean[0]-2*imStd[0];
    high=imMean[0]+4*imStd[0];

    //slower min/max handling. In the old code, bad pixels would cause the colorbar scale
    //to jump all over the place. The averaging I do here slows that down so that colorbar scale varys slower.



    if (staticMin > -1) {
        min=staticMin;
    } else {
        last[0]=(2*last[0]+low)/3;
        min=last[0];
    } 

    if (staticMax > -1) {
        max=staticMax;
    } else {
        last[1]=(2*last[1]+high)/3;
        max=last[1];
    }
    frame_g16=(inframe-min)*(65535.0/(max-min));
    //normalize(inframe, frame_g16, 0, 65535, NORM_MINMAX,-1,mask);
    // Convert seek CV_16UC1 to CV_8UC1

    frame_g16.convertTo(frame_g8, CV_8UC1, 1.0/256.0 );

    // Rotate image
    if (rotate == 90) {
        transpose(frame_g8, frame_g8);
        flip(frame_g8, frame_g8, 1);
    } else if (rotate == 180) {
        flip(frame_g8, frame_g8, -1);
    } else if (rotate == 270) {
        transpose(frame_g8, frame_g8);
        flip(frame_g8, frame_g8, 0);
    }

    // Resize image: http://docs.opencv.org/3.2.0/da/d54/group__imgproc__transform.html#ga5bb5a1fea74ea38e1a5445ca803ff121
    // Note this is expensive computationally, only do if option set != 1
    if (scale != 1.0) {
        resize(frame_g8, frame_g8, Size(), scale, scale, INTER_LINEAR);
    }

    // Apply colormap: http://docs.opencv.org/3.2.0/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
    if (colormap != -1) {
        applyColorMap(frame_g8, outframe, colormap);
    } else {
        cv::cvtColor(frame_g8, outframe, cv::COLOR_GRAY2BGR);
    }
}

void key_handler(char scancode) {
    switch (scancode) {
        case 'f': {
            int windowFlag = getWindowProperty(WINDOW_NAME, WindowPropertyFlags::WND_PROP_FULLSCREEN) == cv::WINDOW_FULLSCREEN ? cv::WINDOW_NORMAL : cv::WINDOW_FULLSCREEN;
            setWindowProperty(WINDOW_NAME, WindowPropertyFlags::WND_PROP_FULLSCREEN, windowFlag);
            break;
        }
        case 's': {
            waitKey(0);
            break;
        }
        case 'a': {
            auto_exposure_lock = !auto_exposure_lock;
            std::cout << "AEL:" << auto_exposure_lock << std::endl;
            break;
        }
        case 'q': {
            sigflag = SIGINT;
        }
    }
}

#ifdef __linux__
// Function to setup output to a v4l2 device
int setup_v4l2(std::string output, int width,int height) {
    int v4l2 = open(output.c_str(), O_RDWR); 
    if(v4l2 < 0) {
        std::cout << "Error opening v4l2 device: " << strerror(errno) << std::endl;
        exit(1);
    }

    struct v4l2_format v;
    int t;
    v.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    t = ioctl(v4l2, VIDIOC_G_FMT, &v);
    if( t < 0 ) {
        std::cout << "VIDIOC_G_FMT error: " << strerror(errno) << std::endl;
        exit(t);
    }
    
    v.fmt.pix.width = width;
    v.fmt.pix.height = height;
    // BGR is not widely supported in v4l2 clients
    v.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    v.fmt.pix.sizeimage = width * height * 3;
    t = ioctl(v4l2, VIDIOC_S_FMT, &v);
    if( t < 0 ) {
        std::cout << "VIDIOC_S_FMT error: " << strerror(errno) << std::endl;
        exit(t);
    }

    std::cout << "Opened v4l2 device" << std::endl;
    return v4l2;
}

void v4l2_out(int v4l2, Mat& outframe) {
    // Second colorspace conversion done here as applyColorMap only produces BGR. Do it in place.
    cvtColor(outframe, outframe, COLOR_BGR2RGB);
    int framesize = outframe.total() * outframe.elemSize();
    int written = write(v4l2, outframe.data, framesize);
    if (written < 0) {
        std::cout << "Error writing v4l2 device" << std::endl;
        close(v4l2);
        exit(1);
    }
}
#else
int setup_v4l2(std::string output, int width, int height) {
    std::cout << "v4l2 is not supported on this platform" << std::endl;
    exit(1);
    return -1; 
}
void v4l2_out(int v4l2, Mat& outframe) {}
#endif // __linux__


int main(int argc, char** argv) {
    // Setup arguments for parser

    args::ArgumentParser parser("Seek Thermal Viewer");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag<std::string> _mode(parser, "mode", "The mode to use - v4l2, window, file", {'m', "mode"});
    args::ValueFlag<std::string> _output(parser, "output", "Name of the file or video device to write to", {'o', "output"});
    args::ValueFlag<std::string> _ffc(parser, "FFC", "Additional Flat Field calibration - provide ffc file", {'F', "FFC"});
    args::ValueFlag<int> _fps(parser, "fps", "Video Output FPS - Kludge factor", {'f', "fps"});
    args::ValueFlag<float> _scale(parser, "scaling", "Output Scaling - multiple of original image", {'s', "scale"});
    args::ValueFlag<int> _colormap(parser, "colormap", "Color Map - number between 0 and 21 (see: cv::ColormapTypes for maps available in your version of OpenCV)", { 'c', "colormap" });
    args::ValueFlag<int> _rotate(parser, "rotate", "Rotation - 0, 90, 180 or 270 (default) degrees", {'r', "rotate"});
    args::ValueFlag<std::string> _camtype(parser, "camtype", "Seek Thermal Camera Model - seek or seekpro", {'t', "camtype"});
    args::ValueFlag<int> _scalemin(parser, "scalemin", "Minimum value of color scale", {'m', "scalemin"});
    args::ValueFlag<int> _scalemax(parser, "scalemax", "Maximum value of color scale", {'M', "scalemax"});
    args::Flag _doLogScale(parser, "logscale", "Whether to use a log scale", {'l',"logscale"});
    // Parse arguments
    try {
        parser.ParseCLI(argc, argv);
    }
    catch (args::Help) {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }
    catch (args::ValidationError e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }
    
    float scale = 1.0;
    int staticMin = -1;
    int staticMax = -1;

    if (_scale)
        scale = args::get(_scale);

    std::string mode = "window";
    if (_mode)
        mode = args::get(_mode);

    std::string camtype = "seek";
    if (_camtype)
        camtype = args::get(_camtype);

    // 7fps seems to be about what you get from a seek thermal compact
    // Note: fps doesn't influence how often frames are processed, just the VideoWriter interpolation
    int fps = 7;
    if (_fps)
        fps = args::get(_fps);

    // Colormap int corresponding to enum: https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
    int colormap = -1;
    if (_colormap)
        colormap = args::get(_colormap);

    // Rotate default is landscape view to match camera logo/markings
    int rotate = 270;
    if (_rotate)
        rotate = args::get(_rotate);

    std::string output = "";
    if (_output)
        output = args::get(_output);

    if (output.empty()) {
        if (mode == "v4l2") {
            std::cout << "Please specify a video device to output to eg: /dev/video0" << std::endl;
            return 1;
        } else if (mode == "file") {
            std::cout << "Please specify a file to save the output to eg: seek.mp4" << std::endl;
            return 1;
        }
    }
    if (_scalemin)
        staticMin = args::get(_scalemin);

    if (_scalemax)
        staticMax = args::get(_scalemax);
    bool logScale=false;
    if (_doLogScale)
        logScale = args::get(_doLogScale);


    // Register signals
    signal(SIGINT, handle_sig);
    signal(SIGTERM, handle_sig);

    // Setup seek camera
    LibSeek::SeekCam* seek;
    LibSeek::SeekThermalPro seekpro(args::get(_ffc));
    LibSeek::SeekThermal seekclassic(args::get(_ffc));
    if (camtype == "seekpro") {
        seek = &seekpro;
    } else {
        seek = &seekclassic;
    }

    if (!seek->open()) {
        std::cout << "Error accessing camera" << std::endl;
        return 1;
    }

    // Mat containers for seek frames
    Mat seekframe, outframe;

    // Retrieve a single frame, resize to requested scaling value and then determine size of matrix
    //  so we can size the VideoWriter stream correctly
    if (!seek->read(seekframe)) {
        std::cout << "Failed to read initial frame from camera, exiting" << std::endl;
        return 1;
    }
    int last [2];
    last[0]=0;
    last[1]=65535;

    process_frame(seekframe, outframe, scale, colormap, rotate,last,staticMin,staticMax);

    // Setup video for linux if that output is chosen
    int v4l2 = -1;
    if (mode == "v4l2") {
        v4l2 = setup_v4l2(output, outframe.size().width,outframe.size().height);
    }

    // Create an output object, if mode specified then setup the pipeline unless mode is set to 'window'
    VideoWriter writer;
    if (mode == "file") {
#if CV_MAJOR_VERSION > 2
        writer.open(output, VideoWriter::fourcc('F', 'M', 'P', '4'), fps, Size(outframe.cols, outframe.rows));
#else  
        writer.open(output, CV_FOURCC('F', 'M', 'P', '4'), fps, Size(outframe.cols, outframe.rows));
#endif

        if (!writer.isOpened()) {
            std::cerr << "Error can't create video writer" << std::endl;
            return 1;
        }

        std::cout << "Video stream created, dimension: " << outframe.cols << "x" << outframe.rows << ", fps:" << fps << std::endl;
    } else if (mode == "window") {
        startWindowThread();
        namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
        setWindowProperty(WINDOW_NAME, WindowPropertyFlags::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
        resizeWindow(WINDOW_NAME, outframe.cols, outframe.rows);
    }


    // Main loop to retrieve frames from camera and write them out
    if (!seek->read2_starter()) {
        std::cout << "Failed to request frame from camera, exiting" << std::endl;
        return 1;
    }
    while (!sigflag) {

        // If signal for interrupt/termination was received, break out of main loop and exit
        if (!seek->read2(seekframe)) {
            std::cout << "Failed to read frame from camera, exiting" << std::endl;
            return 1;
        }
        if (!seek->read2_starter()) {
            std::cout << "Failed to request frame from camera, exiting" << std::endl;
            return 1;
        }
        std::cout << seek->frame_counter() << std::endl;
        // Retrieve frame from seek and process

        if(logScale){ 
            seekframe.convertTo(seekframe,CV_64F);
            cv::log(seekframe,seekframe);
        }
        if (auto_exposure_lock) {
            process_frame(seekframe, outframe, scale, colormap, rotate,last,last[0],last[1]);
        } else {
            process_frame(seekframe, outframe, scale, colormap, rotate,last,staticMin,staticMax);
        }   
        //std::cout << outframe.type() <<std::endl;
        if (output == "window") {
            imshow("SeekThermal", outframe);
        } else if (mode == "v4l2") {
            v4l2_out(v4l2, outframe);
        } else if (mode == "window") {
            imshow(WINDOW_NAME, outframe);
            char c = waitKey(10);
            key_handler(c);

            // If the window is closed by the user all window properties will return -1 and we should terminate
            if (getWindowProperty(WINDOW_NAME, WindowPropertyFlags::WND_PROP_FULLSCREEN) == -1) {
                std::cout << "Window closed, exiting" << std::endl;
                return 0;
            }
        } else {
            writer << outframe;
            imshow("SeekThermal", outframe);
            char c = waitKey(10);
            if (c == 's') {
                waitKey(0);
            }
        }

    }
    //cv::Mat outfile;
    //outframe.convertTo(outfile, CV_16UC3);
    cv::imwrite("lastimg_raw.png", seekframe);
    std::cout<< outframe.depth()<<outframe.channels();
    std::cout << "Break signal detected, exiting" << std::endl;
    return 0;
}
