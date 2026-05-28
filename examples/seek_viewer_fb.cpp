// Seek Thermal Viewer/Streamer
// http://github.com/fnoop/maverick

#ifdef __APPLE__
int main(int argc, char** argv) {}
#else

//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "seek.h"
#include "SeekCam.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
#include "args.h"
#include <fstream>
#include "framebuffer.h"
#include <wiringPi.h>

using namespace cv;
using namespace LibSeek;

struct ButtonState
{
    int b17=0;
    int b22=0;
    int b23=0;
    int b27=0;
    int battery=0;
};

// Setup sig handling
static volatile sig_atomic_t sigflag = 0;
void handle_sig(int sig)
{
    (void)sig;
    sigflag = 1;
}

void process_frame(Mat &inframe, Mat &outframe, float scale, int colormap, int rotate, int last[], int staticMin, int staticMax)
{
    
    Mat frame_g8, frame_g16,temp,tempmask; // Transient Mat containers for processing
    int cols=inframe.cols,rows=inframe.rows;

    ushort min,max,low,high;
    cv::Scalar imMean,imStd;
    cv::meanStdDev(inframe,imMean,imStd);
    frame_g16=cv::Mat::zeros(cols,rows,CV_16UC1);

    double raw_maxvalue;
    cv::minMaxIdx(inframe,0,&raw_maxvalue);

    low=imMean[0]-2*imStd[0];
    high=imMean[0]+4*imStd[0];

    //slower min/max handling. In the old code, bad pixels would cause the colorbar scale
    //to jump all over the place. The averaging I do here slows that down so that colorbar scale varys slower.
    last[0]=(2*last[0]+low)/3;
    last[1]=(2*last[1]+high)/3;

    if (staticMin > -1)
        min=staticMin;
    else
        min=last[0];
    
    if (staticMax > -1)
        max=staticMax;
    else
        max=last[1];
    
    frame_g16=(inframe-min)*(65535.0/(max-min));
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

void new_flat(LibSeek::SeekCam* seek) 
{
    bool new_flat = true;
    cv::Mat frame, avg_frame,frame_u16;
    int smoothing=100;
    std::cout<<"Flat field started"<<std::flush;
    for (int i=0; i<smoothing; i++) {

        if (!seek->grab()) {
            std::cout << "no more LWIR img" << std::endl;
            return;
        }

        seek->rawRetrieve(frame_u16);
        frame_u16.convertTo(frame, CV_64FC1);

        if (new_flat) {
            new_flat=false;
            frame.copyTo(avg_frame);
        } else {
            avg_frame += frame;
        }

        cv::waitKey(10);
    }
    avg_frame/=smoothing;
    cv::Scalar mean = cv::mean(avg_frame);
    avg_frame/=mean[0];
    seek->set_additional_ffc(avg_frame);
    std::cout<<"Flat field finished"<<std::flush;
}

std::string get_next_file_num_and_increment(){
    std::ifstream currentnumfile;
    std::ofstream nextnumfile;
    currentnumfile.open("./imgs/.nextnum");
    int currentnum;
    currentnumfile >> currentnum;
    currentnumfile.close();
    std::string outnum = std::to_string(currentnum);

    while (outnum.size()<4){
        outnum="0"+outnum;
    }

    nextnumfile.open("./imgs/.nextnum");
    nextnumfile << currentnum+1;
    nextnumfile.close();
    std::cout <<std::endl<< "saved image ./imgs/img_"+outnum+".png"<<std::flush;
    return outnum;
}

void save_image(cv::Mat &seekframe, cv::Mat &outframe)
{
    std::string outnum = get_next_file_num_and_increment();
    imwrite("./imgs/img_"+outnum+"_raw.png",seekframe);
    imwrite("./imgs/img_"+outnum+".png",outframe);
}

void setupButtons(){
    //setup gpio buttons
    wiringPiSetup();
    //4 buttons on screen
    pinMode(0,INPUT);
    pullUpDnControl(0,PUD_UP);
    pinMode(3,INPUT);
    pullUpDnControl(3,PUD_UP);
    pinMode(4,INPUT);
    pullUpDnControl(4,PUD_UP);
    pinMode(2,INPUT);
    pullUpDnControl(2,PUD_UP);
    //battery low
    pinMode(7,INPUT);
}

void updatepin(int pinNum, int& priorState){
    if (!digitalRead(pinNum))
        priorState++;
    else
        priorState=0;
}

void readButtons(ButtonState& priorState){
        updatepin(0,priorState.b17);
        updatepin(3,priorState.b22);
        updatepin(4,priorState.b23);
        updatepin(2,priorState.b27);
        updatepin(7,priorState.battery);
}


int main(int argc, char** argv)
{
    // Setup arguments for parser

    args::ArgumentParser parser("Seek Thermal Viewer");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag<std::string> _output(parser, "output", "Output Stream - name of the video file to write", {'o', "output"});
    args::ValueFlag<std::string> _ffc(parser, "FFC", "Additional Flat Field calibration - provide ffc file", {'F', "FFC"});
    args::ValueFlag<int> _fps(parser, "fps", "Video Output FPS - Kludge factor", {'f', "fps"});
    args::ValueFlag<int> _colormap(parser, "colormap", "Color Map - number between 0 and 12", {'c', "colormap"});
    args::ValueFlag<int> _rotate(parser, "rotate", "Rotation - 0, 90, 180 or 270 (default) degrees", {'r', "rotate"});
    args::ValueFlag<std::string> _camtype(parser, "camtype", "Seek Thermal Camera Model - seek or seekpro", {'t', "camtype"});
    args::ValueFlag<std::string> _fbpath(parser, "fbdevice", "Framebuffer Device", {'d',"dev"});
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

    // Register signals
    signal(SIGINT, handle_sig);
    signal(SIGTERM, handle_sig);

    //setup framebuffer
    std::string fbpath = "/dev/fb0";
    if (_fbpath)
        fbpath = args::get(_fbpath);
    Framebuffer fbwriter(fbpath.c_str());

    setupButtons();
    ButtonState bs;
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
    scale=fbwriter.getScale(seekframe);
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
    int textcounter;
    textcounter=0;
    // Main loop to retrieve frames from camera and output
    if (!seek->read2_starter()) {
        std::cout << "Failed to request frame from camera, exiting" << std::endl;
        return 1;
    }
    while (!sigflag) {
        // If signal for interrupt/termination was received, break out of main loop and exit
        if (!seek->read2(seekframe)) {
            std::cout << "Failed to read frame from camera, exiting" << std::endl;
            return -1;
        }
        if (!seek->read2_starter()) {
            std::cout << "Failed to request frame from camera, exiting" << std::endl;
            return 1;
        }
        // Retrieve frame from seek and process

        process_frame(seekframe, outframe, scale, colormap, rotate,last,staticMin,staticMax);
        if (output != "window") {
            writer << outframe;
        }

        readButtons(bs);
        //bs numbers increase as long as they're held
        //down. So if you want someting to trigger exactly
        //once, use bs.b##==1

        //Check for low battery state


        //Check if user wants a flat frame
        if (bs.b27>20){
            // grab queued image from camera
            seek->read2(seekframe);
            //do flat field
            new_flat(seek);
            //request another image
            if (!seek->read2_starter()) {
                std::cout << "Failed to request frame from camera, exiting" << std::endl;
                return 1;
            }
        }

        //Check if user wants to toggle colorbar mode
        if (bs.b23==1 && staticMin >=0 && staticMax >=0){
            //change colorbar mode to variable
            staticMin = -1;
            staticMax = -1;
            std::cout<<std::endl<<"set cbar to variable"<<std::flush;
            textcounter=20;
        } else if (bs.b23==1 && staticMin ==-1 && staticMax ==-1){
            staticMin = last[0];
            staticMax = last[1];
            std::cout<<std::endl<<"set static colorbar"<<std::flush;
            textcounter=20;
        } 

        //Check if user wants to initiate shutdown.
        //Give hint in case I forget the new shutdown logic
        if (bs.b23 > 20 && bs.b17 > 20){
            system("sudo shutdown now");
        } else if (bs.b23 >20){
            std::cout << std::endl<<"Hold buttons 27 and 17 to shutdown"<<std::flush;
            textcounter=20;
        }
        //save snapshot
        if (bs.b17 == 1){
            save_image(seekframe,outframe);
            textcounter=20;
        } 

        //start videotaping?
        if (bs.b22 ==10 && output=="window"){
            output = "imgs/img_"+ get_next_file_num_and_increment()+"_vid.mp4";
            writer.open(output, VideoWriter::fourcc('m', 'p', '4', 'v'), fps, Size(outframe.cols, outframe.rows));
            if (!writer.isOpened()) {
                std::cerr << "Error can't create video writer" << std::endl;
                return 1;
            }
        } else if (bs.b22 ==1 && output !="window"){
            writer.release();
            output="window";
        }

        //render high priority text
        if (output != "window"){
            std::cout<<std::endl<< "RECORDING"<<std::flush;
            textcounter=1;
        }
        //render higher priority text
        if (bs.battery%20==1){
            std::cout<<std::endl<<"LOW BATTERY"<<std::flush;
            textcounter=20;
        } //else if(bs.battery > 500){
        //     system("sudo shutdown now");
        // }
        if (textcounter>0) {
            textcounter --;
            fbwriter.setUncopiedRows(12);
        } else {
            fbwriter.setUncopiedRows(0);
        }
        //render image after everything to cover up extraneous text
        //keep the image onscreen as long as they hold button 17
        if (bs.b17<=1){
            fbwriter.writeFrame(outframe);
        }


    }
    fbwriter.closefb();
    //cv::Mat outfile;
    //outframe.convertTo(outfile, CV_16UC3);
    cv::imwrite("lastimg_raw.png", seekframe);
    std::cout<< outframe.depth()<<outframe.channels();
    seek->read2(seekframe);
    std::cout << "Break signal detected, exiting" << std::endl;
    return 0;
}
#endif //APPLE