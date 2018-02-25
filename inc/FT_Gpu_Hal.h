/*!
 * \file FT_GPU_HAL.h
 *
 
 Curt added Load_raw 7/22/16
 
 
 
 * \author FTDI
 * \date 2013.04.24
 *
 * Copyright 2013 Future Technology Devices International Limited
 *
 * Project: FT800 or EVE compatible silicon
 * File Description:
 *    This file defines the generic APIs of host access layer for the FT800 or EVE compatible silicon.
 *    Application shall access FT800 or EVE resources over these APIs. In addition, there are
 *    some helper functions defined for FT800 coprocessor engine as well as host commands.
 * Rivision History:
 * ported to mbed by Peter Drescher, DC2PD 2014
 *
 */
 
#ifndef FT_GPU_HAL_H
#define FT_GPU_HAL_H
 
#include "mbed.h"
#include "FT_DataTypes.h"
 
typedef enum {
    FT_GPU_I2C_MODE = 0,
    FT_GPU_SPI_MODE,
 
    FT_GPU_MODE_COUNT,
    FT_GPU_MODE_UNKNOWN = FT_GPU_MODE_COUNT
} FT_GPU_HAL_MODE_E;
 
typedef enum {
    OPENED,
    READING,
    WRITING,
    CLOSED,
    STATUS_COUNT,
    STATUS_ERROR = STATUS_COUNT
} FT_GPU_HAL_STATUS_E;
 
typedef struct {
    ft_uint8_t reserved;
} Ft_Gpu_App_Context_t;
 
typedef struct {
    /* Total number channels for libmpsse */
    ft_uint32_t TotalChannelNum;
} Ft_Gpu_HalInit_t;
 
typedef enum {
    FT_GPU_READ = 0,
    FT_GPU_WRITE,
} FT_GPU_TRANSFERDIR_T;
 
 
typedef struct {
    ft_uint32_t length; //IN and OUT
    ft_uint32_t address;
    ft_uint8_t  *buffer;
} Ft_Gpu_App_Transfer_t;
 
class FT800
{
public:
    FT800(PinName mosi,
          PinName miso,
          PinName sck,
          PinName ss,
          PinName intr,
          PinName pd);
 
private:
    SPI _spi;
    DigitalOut  _ss;
    DigitalOut  _pd;
    //InterruptIn _f800_isr;
public:
    /* Global used for buffer optimization */
    //Ft_Gpu_Hal_Context_t host,*phost;
    Ft_Gpu_App_Context_t        app_header;
    ft_uint16_t cmd_fifo_wp; //coprocessor fifo write pointer
    ft_uint16_t dl_buff_wp;  //display command memory write pointer
    FT_GPU_HAL_STATUS_E        status;            //OUT
    ft_void_t*                 hal_handle;        //IN/OUT
    ft_uint32_t CmdBuffer_Index;
    ft_uint32_t DlBuffer_Index;
    ft_int16_t DispWidth;
    ft_int16_t DispHeight;
    ft_int16_t DispHCycle;
    ft_int16_t DispHOffset;
    ft_int16_t DispHSync0;
    ft_int16_t DispHSync1;
    ft_int16_t DispVCycle;
    ft_int16_t DispVOffset;
    ft_int16_t DispVSync0;
    ft_int16_t DispVSync1;
    ft_uint8_t DispPCLK;
    ft_char8_t DispSwizzle;
    ft_char8_t DispPCLKPol;
 
 
    ft_void_t BootupConfig(void);
    ft_bool_t Bootup(void);
 
 
    /*The basic APIs Level 1*/
    ft_bool_t              Init( );
    ft_bool_t              Open( );
 
    /*The APIs for reading/writing transfer continuously only with small buffer system*/
    ft_void_t              StartTransfer(FT_GPU_TRANSFERDIR_T rw,ft_uint32_t addr);
    ft_uint8_t             Transfer8(ft_uint8_t value);
    ft_uint16_t            Transfer16(ft_uint16_t value);
    ft_uint32_t            Transfer32(ft_uint32_t value);
    ft_void_t              EndTransfer( );
 
    /*Read & Write APIs for both burst and single transfer,depending on buffer size*/
    ft_void_t              Read(Ft_Gpu_App_Transfer_t *transfer);
    ft_void_t              Write(Ft_Gpu_App_Transfer_t *transfer);
 
    ft_void_t              Close();
    ft_void_t              DeInit();
 
    /*Helper function APIs Read*/
    ft_uint8_t  Rd8(ft_uint32_t addr);
    ft_uint16_t Rd16(ft_uint32_t addr);
    ft_uint32_t Rd32(ft_uint32_t addr);
 
    /*Helper function APIs Write*/
    ft_void_t Wr8(ft_uint32_t addr, ft_uint8_t v);
    ft_void_t Wr16(ft_uint32_t addr, ft_uint16_t v);
    ft_void_t Wr32(ft_uint32_t addr, ft_uint32_t v);
 
    /*******************************************************************************/
    /*******************************************************************************/
    /*APIs for coprocessor Fifo read/write and space management*/
    ft_void_t Updatecmdfifo(ft_uint16_t count);
    ft_void_t WrCmd32(ft_uint32_t cmd);
    ft_void_t WrCmdBuf(ft_uint8_t *buffer,ft_uint16_t count);
    ft_void_t WaitCmdfifo_empty();
    ft_void_t ResetCmdFifo();
    ft_void_t CheckCmdBuffer(ft_uint16_t count);
    ft_void_t ResetDLBuffer();
 
    ft_void_t StartCmdTransfer(FT_GPU_TRANSFERDIR_T rw, ft_uint16_t count);
    ft_void_t Powercycle(ft_bool_t up);
 
 
    /*******************************************************************************/
    /*******************************************************************************/
    /*APIs for Host Commands*/
    typedef enum {
        FT_GPU_INTERNAL_OSC = 0x48, //default
        FT_GPU_EXTERNAL_OSC = 0x44,
    } FT_GPU_PLL_SOURCE_T;
    typedef enum {
        FT_GPU_PLL_48M = 0x62,  //default
        FT_GPU_PLL_36M = 0x61,
        FT_GPU_PLL_24M = 0x64,
    } FT_GPU_PLL_FREQ_T;
 
    typedef enum {
        FT_GPU_ACTIVE_M =  0x00,
        FT_GPU_STANDBY_M = 0x41,//default
        FT_GPU_SLEEP_M =   0x42,
        FT_GPU_POWERDOWN_M = 0x50,
    } FT_GPU_POWER_MODE_T;
 
#define FT_GPU_CORE_RESET  (0x68)
 
    ft_int32_t hal_strlen(const ft_char8_t *s);
    ft_void_t Sleep(ft_uint16_t ms);
    ft_void_t ClockSelect(FT_GPU_PLL_SOURCE_T pllsource);
    ft_void_t PLL_FreqSelect(FT_GPU_PLL_FREQ_T freq);
    ft_void_t PowerModeSwitch(FT_GPU_POWER_MODE_T pwrmode);
    ft_void_t CoreReset();
//ft_void_t Ft_Gpu_Hal_StartTransfer( ,FT_GPU_TRANSFERDIR_T rw,ft_uint32_t addr);
    ft_void_t WrMem(ft_uint32_t addr, const ft_uint8_t *buffer, ft_uint32_t length);
    ft_void_t WrMemFromFlash(ft_uint32_t addr,const ft_prog_uchar8_t *buffer, ft_uint32_t length);
    ft_void_t WrCmdBufFromFlash(FT_PROGMEM ft_prog_uchar8_t *buffer,ft_uint16_t count);
    ft_void_t RdMem(ft_uint32_t addr, ft_uint8_t *buffer, ft_uint32_t length);
    ft_void_t WaitLogo_Finish();
    ft_uint8_t TransferString(const ft_char8_t *string);
    ft_void_t HostCommand(ft_uint8_t cmd);
    ft_int32_t Dec2Ascii(ft_char8_t *pSrc,ft_int32_t value);
 
    ft_void_t Text(ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s);
    ft_void_t Number(ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, ft_int32_t n);
    ft_void_t LoadIdentity();
    ft_void_t Toggle(ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t font, ft_uint16_t options, ft_uint16_t state, const ft_char8_t* s);
    ft_void_t Gauge(ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t major, ft_uint16_t minor, ft_uint16_t val, ft_uint16_t range);
    ft_void_t RegRead(ft_uint32_t ptr, ft_uint32_t result);
    ft_void_t GetProps(ft_uint32_t ptr, ft_uint32_t w, ft_uint32_t h);
    ft_void_t Memcpy(ft_uint32_t dest, ft_uint32_t src, ft_uint32_t num);
    ft_void_t Spinner(ft_int16_t x, ft_int16_t y, ft_uint16_t style, ft_uint16_t scale);
    ft_void_t BgColor(ft_uint32_t c);
    ft_void_t Swap();
    ft_void_t Inflate(ft_uint32_t ptr);
    ft_void_t Translate(ft_int32_t tx, ft_int32_t ty);
    ft_void_t Stop();
    ft_void_t Slider(ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t range);
    ft_void_t Interrupt(ft_uint32_t ms);
    ft_void_t FgColor(ft_uint32_t c);
    ft_void_t Rotate(ft_int32_t a);
    ft_void_t Button(ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s);
    ft_void_t MemWrite(ft_uint32_t ptr, ft_uint32_t num);
    ft_void_t Scrollbar(ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t size, ft_uint16_t range);
    ft_void_t GetMatrix(ft_int32_t a, ft_int32_t b, ft_int32_t c, ft_int32_t d, ft_int32_t e, ft_int32_t f);
    ft_void_t Sketch(ft_int16_t x, ft_int16_t y, ft_uint16_t w, ft_uint16_t h, ft_uint32_t ptr, ft_uint16_t format);
    ft_void_t MemSet(ft_uint32_t ptr, ft_uint32_t value, ft_uint32_t num);
    ft_void_t Calibrate(ft_uint32_t result);
    ft_void_t SetFont(ft_uint32_t font, ft_uint32_t ptr);
    ft_void_t RomFont( ft_uint32_t font, ft_uint32_t rom_slot);
    ft_void_t Bitmap_Transform(ft_int32_t x0, ft_int32_t y0, ft_int32_t x1, ft_int32_t y1, ft_int32_t x2, ft_int32_t y2, ft_int32_t tx0, ft_int32_t ty0, ft_int32_t tx1, ft_int32_t ty1, ft_int32_t tx2, ft_int32_t ty2, ft_uint16_t result);
    ft_void_t GradColor(ft_uint32_t c);
    ft_void_t Append(ft_uint32_t ptr, ft_uint32_t num);
    ft_void_t MemZero(ft_uint32_t ptr, ft_uint32_t num);
    ft_void_t Scale(ft_int32_t sx, ft_int32_t sy);
    ft_void_t Clock(ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t h, ft_uint16_t m, ft_uint16_t s, ft_uint16_t ms);
    ft_void_t Gradient(ft_int16_t x0, ft_int16_t y0, ft_uint32_t rgb0, ft_int16_t x1, ft_int16_t y1, ft_uint32_t rgb1);
    ft_void_t SetMatrix();
    ft_void_t Track(ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t tag);
    ft_void_t GetPtr(ft_uint32_t result);
    ft_void_t Progress(ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t range);
    ft_void_t ColdStart();
    ft_void_t Keys(ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s);
    ft_void_t Dial(ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t val);
    ft_void_t LoadImage(ft_uint32_t ptr, ft_uint32_t options);
    ft_void_t DLstart();
    ft_void_t Snapshot(ft_uint32_t ptr);
    ft_void_t ScreenSaver();
    ft_void_t Memcrc(ft_uint32_t ptr, ft_uint32_t num, ft_uint32_t result);
 
    ft_void_t Logo();
 
    ft_void_t SendCmd( ft_uint32_t cmd);
    ft_void_t SendStr( const ft_char8_t *s);
    ft_void_t StartFunc( ft_uint16_t count);
    ft_void_t EndFunc( ft_uint16_t count);
    ft_void_t TouchTransform( ft_int32_t x0, ft_int32_t y0, ft_int32_t x1, ft_int32_t y1, ft_int32_t x2, ft_int32_t y2, ft_int32_t tx0, ft_int32_t ty0, ft_int32_t tx1, ft_int32_t ty1, ft_int32_t tx2, ft_int32_t ty2, ft_uint16_t result);
    ft_void_t BitmapTransform( ft_int32_t x0, ft_int32_t y0, ft_int32_t x1, ft_int32_t y1, ft_int32_t x2, ft_int32_t y2, ft_int32_t tx0, ft_int32_t ty0, ft_int32_t tx1, ft_int32_t ty1, ft_int32_t tx2, ft_int32_t ty2, ft_uint16_t result);
    ft_void_t MemCrc( ft_uint32_t ptr, ft_uint32_t num, ft_uint32_t result);
 
    ft_uint16_t fifo_Freespace( );
 
    ft_void_t DL(ft_uint32_t cmd);
    ft_void_t WrDlCmd_Buffer(ft_uint32_t cmd);
    ft_void_t Flush_DL_Buffer();
    ft_void_t Flush_Co_Buffer();
    ft_void_t fadeout();
    ft_void_t fadein();
    ft_void_t DLSwap(ft_uint8_t DL_Swap_Type);
 
    ft_void_t Sound_ON();
    ft_void_t Sound_OFF();
    
    int Load_jpg(char* filename, ft_int16_t* x_size, ft_int16_t* y_size,ft_uint32_t address);
    
    //Curt added Load_raw 7/22/16
    int Load_raw(char* filename);
    
    ft_void_t Calibrate();
    ft_void_t read_calibrate(ft_uint8_t data[24]);
    ft_void_t write_calibrate(ft_uint8_t data[24]);
    
    ft_uint32_t color_rgb(ft_uint8_t red,ft_uint8_t green, ft_uint8_t blue);
    ft_uint32_t clear_color_rgb(ft_uint8_t red,ft_uint8_t green, ft_uint8_t blue);
 
};  // end of class
 
#endif  /*FT_GPU_HAL_H*/
