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
//for handling framebuffer
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

using namespace cv;
using namespace LibSeek;

// Setup sig handling
static volatile sig_atomic_t sigflag = 0;
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

    ushort min,max;
    cv::Scalar imMean,imStd;
    cv::meanStdDev(inframe,imMean,imStd);
    frame_g16=cv::Mat::zeros(cols,rows,CV_16UC1);

    double raw_maxvalue;
    cv::minMaxIdx(inframe,0,&raw_maxvalue);
    //std::cout << raw_maxvalue << std::endl;

    min=imMean[0]-2*imStd[0];
    max=imMean[0]+4*imStd[0];

    //slower min/max handling. In the old code, bad pixels would cause the colorbar scale
    //to jump all over the place. The averaging I do here slows that down so that colorbar scale varys slower.
    //Still this is not the best solution because a big thing of constant temperature looks hotter near edges of screen.
    last[0]=(2*last[0]+min)/3;
    last[1]=(2*last[1]+max)/3;

    if (staticMin > -1)
        last[0]=staticMin;
    
    if (staticMax > -1)
        last[1]=staticMax;
    
    //std::cout<<min<<" "<<max<<std::endl;
    frame_g16=(inframe-last[0])*(65535.0/(last[1]-last[0]));
    //normalize(inframe, frame_g16, 0, 65535, NORM_MINMAX,-1,mask);
    // Convert seek CV_16UC1 to CV_8UC1
    frame_g16.convertTo(frame_g8, CV_8UC1, 1.0/256.0 );
    //inframe.convertTo(frame_g8,CV_8UC1,1.0/256.0);
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
    if (scale != 1.0)
        resize(frame_g8, frame_g8, Size(), scale, scale, INTER_LINEAR);
    
    // Apply colormap: http://docs.opencv.org/3.2.0/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
    if (colormap != -1) {
        applyColorMap(frame_g8, outframe, colormap);
    } else {
        cv::cvtColor(frame_g8, outframe, cv::COLOR_GRAY2BGR);
    }
}

int main(int argc, char** argv)
{
    // Setup arguments for parser

    args::ArgumentParser parser("Seek Thermal Viewer");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag<std::string> _output(parser, "output", "Output Stream - name of the video file to write", {'o', "output"});
    args::ValueFlag<std::string> _ffc(parser, "FFC", "Additional Flat Field calibration - provide ffc file", {'F', "FFC"});
    args::ValueFlag<int> _fps(parser, "fps", "Video Output FPS - Kludge factor", {'f', "fps"});
    args::ValueFlag<float> _scale(parser, "scaling", "Output Scaling - multiple of original image", {'s', "scale"});
    args::ValueFlag<int> _colormap(parser, "colormap", "Color Map - number between 0 and 12", {'c', "colormap"});
    args::ValueFlag<int> _rotate(parser, "rotate", "Rotation - 0, 90, 180 or 270 (default) degrees", {'r', "rotate"});
    args::ValueFlag<std::string> _camtype(parser, "camtype", "Seek Thermal Camera Model - seek or seekpro", {'t', "camtype"});
    args::ValueFlag<int> _scalemin(parser, "scalemin", "Minimum value of color scale", {'m', "scalemin"});
    args::ValueFlag<int> _scalemax(parser, "scalemax", "Maximum value of color scale", {'M', "scalemax"});
    args::Flag _doLogScale(parser, "logscale", "Whether to use a log scale", {'l',"logscale"});
    // Parse arguments
    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch (args::Help)
    {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }
    catch (args::ValidationError e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }
    float scale = 1.0;
    int staticMin = -1;
    int staticMax = -1;

    if (_scale)
        scale = args::get(_scale);
    std::string output = "window";
    if (_output)
        output = args::get(_output);
    std::string camtype = "seek";
    if (_camtype)
        camtype = args::get(_camtype);
    // 7fps seems to be about what you get from a seek thermal compact
    // Note: fps doesn't influence how often frames are processed, just the VideoWriter interpolation
    int fps = 7;
    if (_fps)
        fps = args::get(_fps);
    // Colormap int corresponding to enum: http://docs.opencv.org/3.2.0/d3/d50/group__imgproc__colormap.html
    int colormap = -1;
    if (_colormap)
        colormap = args::get(_colormap);
    // Rotate default is landscape view to match camera logo/markings
    int rotate = 270;
    if (_rotate)
        rotate = args::get(_rotate);

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

    //setup framebuffer
    int fbfd = 0;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    long int screensize = 0;
    char *fbp = 0;

    // Open the file for reading and writing
    fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        exit(1);
    }
    printf("The framebuffer device was opened successfully.\n");
    // Get fixed screen information
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        exit(2);
    }
    // Get variable screen information
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        exit(3);
    }
    printf("%dx%d, %dbpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
    // Figure out the size of the screen in bytes
    screensize = finfo.smem_len;//vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;
    // Map the device to memory
    std::cout << finfo.line_length << " " << vinfo.xres*vinfo.bits_per_pixel/8 << " " <<std::endl;
                    // location = (vinfo.xres+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
                    //    (vinfo.yres+vinfo.yoffset) * finfo.line_length;
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED,
                       fbfd, 0);
    if ((int64_t)fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        exit(4);
    }
    printf("The framebuffer device was mapped to memory successfully.\n");




    // Setup seek camera
    LibSeek::SeekCam* seek;
    LibSeek::SeekThermalPro seekpro(args::get(_ffc));
    LibSeek::SeekThermal seekclassic(args::get(_ffc));
    if (camtype == "seekpro")
        seek = &seekpro;
    else
        seek = &seekclassic;

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
        return -1;
    }
    int last [2];
    last[0]=0;
    last[1]=65535;
    float xscale,yscale;
    xscale = (float)vinfo.xres/seekframe.cols;
    yscale = (float)vinfo.yres/seekframe.rows;
    if (xscale < yscale){
        scale = xscale;
    }else{
        scale = yscale;
    }
    //std::cout << xscale << yscale << scale << std::endl;
    process_frame(seekframe, outframe, scale, colormap, rotate,last,staticMin,staticMax);
    //std::vector<cv::Point> dp = {cv::Point(212,34),cv::Point(212,35)};
    //std::cout<<outframe.rows<<" "<<outframe.cols<<std::endl;
    //seek->add_dead_pixel(dp);
    // Create an output object, if output specified then setup the pipeline unless output is set to 'window'
    VideoWriter writer;
    if (output != "window") {
        writer.open(output, VideoWriter::fourcc('F', 'M', 'P', '4'), fps, Size(outframe.cols, outframe.rows));
        if (!writer.isOpened()) {
            std::cerr << "Error can't create video writer" << std::endl;
            return 1;
        }

        std::cout << "Video stream created, dimension: " << outframe.cols << "x" << outframe.rows << ", fps:" << fps << std::endl;
    }
    int row, col, location;
    // Main loop to retrieve frames from camera and output
    while (!sigflag) {
        // If signal for interrupt/termination was received, break out of main loop and exit
        if (!seek->read(seekframe)) {
            std::cout << "Failed to read frame from camera, exiting" << std::endl;
            return -1;
        }
        // Retrieve frame from seek and process

        if(logScale){ 
            seekframe.convertTo(seekframe,CV_64F);
            cv::log(seekframe,seekframe);
        }

        process_frame(seekframe, outframe, scale, colormap, rotate,last,staticMin,staticMax);
        if (output != "window") {
            writer << outframe;
        }
        //write image to framebuffer
        //bool flag = true;
        for (row = 0; row < outframe.rows; row++) {
            uint8_t* rowptr = outframe.ptr(row);
            //flag = true;
            for (col=0; col<outframe.cols; col++) {
                location = (col+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
                       (row+vinfo.yoffset) * finfo.line_length;
                if(vinfo.bits_per_pixel==32){
                    // if (row > 760){
                    // std::cout << screensize<< " "<<location << std::endl;
                    // }   
                    *(fbp + location) = rowptr[3*col];
                    *(fbp + location+1) = rowptr[3*col+1];
                    *(fbp + location+2) = rowptr[3*col+2];
                    //note neither of these are RGB, they're both BGR
                    *(fbp + location+3) = 0;
                    // if(flag){
                    //     printf("%d %d %d\n",red,green,blue);
                    //     flag = false;
                    // }
                }else if(vinfo.bits_per_pixel==16){
                    //red 5bit, green 6bit, blue 5bit
                    uint8_t blue=rowptr[3*col]>>3;
                    uint8_t green=rowptr[3*col+1]>>2;
                    uint8_t red=rowptr[3*col+2]>>3;
                    uint8_t byte1=(red<<3)+(green>>3);
                    uint8_t byte2=((green%8)<<5)+blue;
                    *(fbp+location)=byte2;
                    *(fbp+location+1)=byte1;
                    //if(flag){
                    //    printf("%d %d %d\n",red,green,blue);
                    //    flag = false;
                    //}

                }else{
                    //panic
                    printf("bits per pixel == %d NYI",vinfo.bits_per_pixel);
                    return (1);
                }
            }
        }
        
    }
    munmap(fbp,screensize);
    close(fbfd);
    //cv::Mat outfile;
    //outframe.convertTo(outfile, CV_16UC3);
    cv::imwrite("lastimg_raw.png", seekframe);
    std::cout<< outframe.depth()<<outframe.channels();
    std::cout << "Break signal detected, exiting" << std::endl;
    return 0;
}