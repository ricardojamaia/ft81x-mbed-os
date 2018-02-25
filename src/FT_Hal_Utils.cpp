#include "FT_Platform.h"
#include "mbed.h"
//#include "SDFileSystem.h"

/* function to load jpg file from filesystem */
/* return 0 if jpg is ok                     */
/* return x_size and y_size of jpg           */

int FT800::Load_jpg(char* filename, ft_int16_t* x_size, ft_int16_t* y_size, ft_uint32_t address)
{
    unsigned char pbuff[8291];
    unsigned short marker;
    unsigned short length;
    unsigned char data[4];

    ft_uint16_t blocklen;
//    sd.mount();
    FILE *fp = fopen(filename, "r");
    if(fp == NULL) return (-1);         // connot open file

    // search for 0xFFC0 marker
    fseek(fp, 0, SEEK_END);
    unsigned int Fsize = ftell(fp);
    fseek(fp, 2, SEEK_SET);
    fread(data,4,1,fp);
    marker = data[0] << 8 | data[1];
    length = data[2] << 8 | data[3];
    do {
        if(marker == 0xFFC0) break;
        if(marker & 0xFF00 != 0xFF00) break;
        if (fseek(fp, length - 2,SEEK_CUR) != 0) break;
        fread(data,4,1,fp);
        marker = data[0] << 8 | data[1];
        length = data[2] << 8 | data[3];
    } while(1);
    if(marker != 0xFFC0) return (-2);  // no FFC0 Marker, wrong format no baseline DCT-based JPEG
    fseek(fp, 1,SEEK_CUR);
    fread(data,4,1,fp);
    *y_size = (data[0] << 8 | data[1]);
    *x_size = (data[2] << 8 | data[3]);

    //if(*x_size > DispWidth || *y_size > DispHeight) return (-3);  // to big to fit on screen

    fseek(fp, 0, SEEK_SET);
    WrCmd32(CMD_LOADIMAGE);  // load a JPEG image
    WrCmd32(address);              //destination address of jpg decode
    WrCmd32(0);              //output format of the bitmap - default is rgb565
    while(Fsize > 0) {
        /* download the data into the command buffer by 8kb one shot */
        blocklen = Fsize>8192?8192:Fsize;
        /* copy the data into pbuff and then transfter it to command buffer */
        fread(pbuff,1,blocklen,fp);
        Fsize -= blocklen;
        /* copy data continuously into command memory */
        WrCmdBuf(pbuff, blocklen); //alignment is already taken care by this api
    }
    fclose(fp);
//    sd.unmount();

    return(0);
}

//int FT800::Load_raw(char* filename)
//{
//    ft_uint8_t imbuff[8192];
//    ft_uint16_t filesize;
//    ft_uint16_t blocklen;
////    ft_uint16_t ram_start = 0x00;
//    
////    sd.mount();  
//    FILE *fp = fopen(filename, "rb");   //  open file
////    if(fp == NULL) return (-1);       //  connot open file         
//    fseek(fp, 0, SEEK_END);             //  set file position to end of file
//    filesize= ftell(fp);                //  determine file size
//    fseek(fp, 2, SEEK_SET);             //  return to beginning of file       
//
//    while(filesize > 0)
//    {
//        //copy the .raw file data to imbuff[8192] in 8k block
//        blocklen = filesize>8192?8192:filesize;
//        fread(imbuff,1,blocklen,fp);
//        filesize-= blocklen;
//        //write imbuff contents to graphics RAM at address ram_start = 0x00
//        WrCmdBuf(imbuff, blocklen); //alignment is already taken care by this api
////        ram_start += 8192;   
//    }
//    fclose(fp);
////    sd.unmount();
//    
//    return 0;
//}






/* calibrate touch */
ft_void_t FT800::Calibrate()
{
    /*************************************************************************/
    /* Below code demonstrates the usage of calibrate function. Calibrate    */
    /* function will wait untill user presses all the three dots. Only way to*/
    /* come out of this api is to reset the coprocessor bit.                 */
    /*************************************************************************/
    {

        DLstart();                                       // start a new display command list
        DL(CLEAR_COLOR_RGB(64,64,64));       // set the clear color R, G, B
        DL(CLEAR(1,1,1));                    // clear buffers -> color buffer,stencil buffer, tag buffer
        DL(COLOR_RGB(0xff,0xff,0xff));       // set the current color R, G, B
        Text((DispWidth/2), (DispHeight/2), 27, OPT_CENTER, "Please Tap on the dot");  // draw Text at x,y, font 27, centered
        Calibrate(0);                                    // start the calibration of touch screen
        Flush_Co_Buffer();                               // download the commands into FT800 FIFO
        WaitCmdfifo_empty();                             // Wait till coprocessor completes the operation
    }
}


/* API to give fadeout effect by changing the display PWM from 100 till 0 */
ft_void_t FT800::fadeout()
{
   ft_int32_t i;
    
    for (i = 100; i >= 0; i -= 3) 
    {
        Wr8(REG_PWM_DUTY,i);
        Sleep(2);//sleep for 2 ms
    }
}

/* API to perform display fadein effect by changing the display PWM from 0 till 100 and finally 128 */
ft_void_t FT800::fadein()
{
    ft_int32_t i;
    
    for (i = 0; i <=100 ; i += 3) 
    {
        Wr8(REG_PWM_DUTY,i);
        Sleep(2);//sleep for 2 ms
    }
    /* Finally make the PWM 100% */
    i = 128;
    Wr8(REG_PWM_DUTY,i);
}

ft_void_t FT800::read_calibrate(ft_uint8_t data[24]){
    unsigned int i;
    for(i=0;i<24;i++){
        data[i] = Rd8(REG_TOUCH_TRANSFORM_A + i);
        }
}

ft_void_t FT800::write_calibrate(ft_uint8_t data[24]){
    unsigned int i;
    for(i=0;i<24;i++) {
        Wr8(REG_TOUCH_TRANSFORM_A + i,data[i]);
        }
}

ft_uint32_t FT800::color_rgb(ft_uint8_t red,ft_uint8_t green, ft_uint8_t blue){
    return ((4UL<<24)|(((red)&255UL)<<16)|(((green)&255UL)<<8)|(((blue)&255UL)<<0));
    }
    
ft_uint32_t FT800::clear_color_rgb(ft_uint8_t red,ft_uint8_t green, ft_uint8_t blue){
    return ((2UL<<24)|(((red)&255UL)<<16)|(((green)&255UL)<<8)|(((blue)&255UL)<<0));
    }    
    