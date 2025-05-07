/*
 * Logic for writing opencv matrices to a framebuffer 
 * Author: Christopher Rooney
 * Heavily copied from someone's framebuffer test code. Maybe QT? I don't remember. 
 */

#include "framebuffer.h"

using namespace LibSeek;

Framebuffer::Framebuffer(const char* filename):
    m_fbfd(0),
    m_vinfo(),
    m_finfo(),
    m_screensize(0),
    m_fbp(),
    m_uncopiedrows(0)

{
    // Open the file for reading and writing
    m_fbfd = open(filename, O_RDWR);
    if (m_fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        exit(1);
    }
    printf("The framebuffer device was opened successfully.\n");
    // Get fixed screen information
    if (ioctl(m_fbfd, FBIOGET_FSCREENINFO, &m_finfo) == -1) {
        perror("Error reading fixed information");
        exit(2);
    }
    // Get variable screen information
    if (ioctl(m_fbfd, FBIOGET_VSCREENINFO, &m_vinfo) == -1) {
        perror("Error reading variable information");
        exit(3);
    }
    printf("%dx%d, %dbpp\n", m_vinfo.xres, m_vinfo.yres, m_vinfo.bits_per_pixel);
    // Figure out the size of the screen in bytes
    m_screensize = m_finfo.smem_len;//m_vinfo.xres * m_vinfo.yres * m_vinfo.bits_per_pixel / 8;
    // Map the device to memory
    std::cout << m_finfo.line_length << " " << m_vinfo.xres*m_vinfo.bits_per_pixel/8 << " " <<std::endl;
                    // location = (m_vinfo.xres+m_vinfo.xoffset) * (m_vinfo.bits_per_pixel/8) +
                    //    (m_vinfo.yres+m_vinfo.yoffset) * m_finfo.line_length;
    m_fbp = (char *)mmap(0, m_screensize, PROT_READ | PROT_WRITE, MAP_SHARED,
                       m_fbfd, 0);
    if ((int64_t)m_fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        exit(4);
    }
    printf("The framebuffer device was mapped to memory successfully.\n");



}

Framebuffer::~Framebuffer()
{
    closefb();
}
void Framebuffer::closefb(){
    munmap(m_fbp,m_screensize);
    close(m_fbfd);
}
void Framebuffer::setUncopiedRows(uint numrows){
    if (numrows<m_vinfo.yres)
        m_uncopiedrows = numrows;
}

float Framebuffer::getScale(cv::Mat& frame){
    float xscale,yscale,scale;
    xscale = static_cast<float>(m_vinfo.xres)/frame.cols;
    yscale = static_cast<float>(m_vinfo.yres)/frame.rows;
    if (xscale < yscale){
        scale = xscale;
    }else{
        scale = yscale;
    }
    return scale;
}

void Framebuffer::writeFrame(cv::Mat& frame){
    cv::Mat img_fb;
    if (m_vinfo.bits_per_pixel==32)
        img_fb = frame;
    else if(m_vinfo.bits_per_pixel==16)
        cv::cvtColor(frame, img_fb, cv::COLOR_BGR2BGR565);
    else
        printf("bits per pixel == %d NYI",m_vinfo.bits_per_pixel);

    // for (uint row = 0; row < m_vinfo.yres-m_uncopiedrows; row++) {
    //     uint8_t* rowptr = frame.ptr(row);
    //     //flag = true;
    //     // apparently I fell for one of the classic blunders. 
    //     // The first is to never get involved in a land war in Asia,
    //     // but the second, only slightly less well-known, is 
    //     // never to write directly to an mmaped framebuffer pixel by pixel.
    //     // https://stackoverflow.com/questions/64789056/writing-into-framebuffer-with-a-c-program-is-very-slow-raspberry-pi
    //     for (uint col=0; col<m_vinfo.xres; col++) {
    //         uint location = (col+m_vinfo.xoffset) * (m_vinfo.bits_per_pixel/8) +
    //                (row+m_vinfo.yoffset) * m_finfo.line_length;
    //         if(m_vinfo.bits_per_pixel==32){
    //             *(m_imagedata + location) = rowptr[3*col];
    //             *(m_imagedata + location+1) = rowptr[3*col+1];
    //             *(m_imagedata + location+2) = rowptr[3*col+2];
    //             //note both fbp and rowptr have BGR color order
    //             *(m_imagedata + location+3) = 0;
    //         }else if(m_vinfo.bits_per_pixel==16){
    //             //red 5bit, green 6bit, blue 5bit
    //             //Green 6 bits are split over the two bytes
    //             //(MSB most significant bit / LSB least significant bit)
    //             //First byte has 3 bits of green in 3 MSBs and its 5 LSBs are blue
    //             //Second byte has 3 bits of green in 3 LSBs and its 5 MSBs are red
    //             //something about Little-Endian computers.
    //             uint8_t blue=rowptr[3*col]>>3;
    //             uint8_t green=rowptr[3*col+1]>>2;
    //             uint8_t red=rowptr[3*col+2]>>3;

    //             unsigned short int px = red<<11 | green <<5 | blue;
    //             //I don't know whose method is more readable. I'm leaving both here.
    //             //commented one is mine.
    //             //uint8_t byte1=((green%8)<<5)+blue;
    //             //uint8_t byte2=(red<<3)+(green>>3);
    //             //*(fbp+location)=byte1;
    //             //*(fbp+location+1)=byte2;
    //             *((unsigned short int*)(m_imagedata+location))=px;

    //         }else{
                
    //         }
    //     }
    // }
    memcpy(m_fbp, img_fb.data, m_screensize-m_uncopiedrows*m_finfo.line_length);
}