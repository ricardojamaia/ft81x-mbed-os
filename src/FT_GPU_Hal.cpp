/* mbed Library for FTDI FT800  Enbedded Video Engine "EVE"
 * based on Original Code Sample from FTDI
 * ported to mbed by Peter Drescher, DC2PD 2014
 * Released under the MIT License: http://mbed.org/license/mit 
 * 19.09.14 changed to shorter function names  
 * FTDI was using very long names. 
 * Ft_App_Flush_Co_Buffer -> Flush_Co_Buffer ...  */

#include "FT_Platform.h"
#include "mbed.h"
#include "FT_LCD_Type.h"
//Serial pc(USBTX, USBRX);

FT800::FT800(PinName mosi,
            PinName miso,
            PinName sck,
            PinName ss,
            PinName intr,
            PinName pd)
    :

     _spi(mosi, miso, sck),
     _ss(ss),
     _pd(pd)//,
//     _f800_isr(intr)
     {
         _spi.format(8,0);                  // 8 bit spi mode 0
         _spi.frequency(1000000);          // start with 10 Mhz SPI clock
         _ss = 1;                           // cs high
         _pd = 1;                           // PD high
//         Bootup();

         DispHCycle = my_DispHCycle;
         DispHOffset = my_DispHOffset;
         DispHSync0 = my_DispHSync0;
         DispHSync1 = my_DispHSync1;
         DispVCycle = my_DispVCycle;
         DispVOffset = my_DispVOffset;
         DispVSync0 = my_DispVSync0;
         DispVSync1 = my_DispVSync1;
         DispSwizzle = my_DispSwizzle;
         DispPCLKPol = my_DispPCLKPol;
         DispWidth = my_DispWidth;
     }


ft_bool_t FT800::Bootup(void){
//    terminal.printf("Bootup() entered\r\n");
    Open();

    BootupConfig();

    return(1);
    }


ft_void_t FT800::BootupConfig(void){
    ft_uint8_t chipid;
    /* Do a power cycle for safer side */
    Powercycle( FT_TRUE);
    /*
    7/8/16: Curt added the sleep delay below...
    */
//  Sleep(30);
    
    
    /* Access address 0 to wake up the FT800 */
    HostCommand( FT_GPU_ACTIVE_M);
    Sleep(500);

    /* Set the clk to external clock */
    HostCommand( FT_GPU_EXTERNAL_OSC);
    Sleep(10);


    /* Switch PLL output to 48MHz */
//  HostCommand( FT_GPU_PLL_48M);
    //Sleep(10);

    /* Do a core reset for safer side */
    //HostCommand( FT_GPU_CORE_RESET);
    //Sleep(500);

    //Read Register ID to check if FT800 is ready.
    chipid = Rd8(  REG_ID);
//  chipid = Rd8(0x0C0000);
//    pc.printf("ID%08X\n", chipid);
    while(chipid != 0x7C){
        chipid = Rd8(  REG_ID);
    }


    // Speed up
    _spi.frequency(30000000);           // 30 Mhz SPI clock DC
//    _spi.frequency(20000000);           // 20 Mhz SPI clock DC
//    _spi.frequency(12000000);           // 12 Mhz SPI clock
    /* Configuration of LCD display */
    DispHCycle = my_DispHCycle;
    Wr16(  REG_HCYCLE, DispHCycle);
    DispHOffset = my_DispHOffset;
    Wr16(  REG_HOFFSET, DispHOffset);
    DispHSync0 = my_DispHSync0;
    Wr16(  REG_HSYNC0, DispHSync0);
    DispHSync1 = my_DispHSync1;
    Wr16(  REG_HSYNC1, DispHSync1);
    DispVCycle = my_DispVCycle;
    Wr16(  REG_VCYCLE, DispVCycle);
    DispVOffset = my_DispVOffset;
    Wr16(  REG_VOFFSET, DispVOffset);
    DispVSync0 = my_DispVSync0;
    Wr16(  REG_VSYNC0, DispVSync0);
    DispVSync1 = my_DispVSync1;
    Wr16(  REG_VSYNC1, DispVSync1);
    DispSwizzle = my_DispSwizzle;
    Wr8(  REG_SWIZZLE, DispSwizzle);
    DispPCLKPol = my_DispPCLKPol;
    Wr8(  REG_PCLK_POL, DispPCLKPol);
    Wr8(  REG_CSPREAD, 1);
    Wr16( REG_DITHER, 1);

    DispWidth = my_DispWidth;
    Wr16(  REG_HSIZE, DispWidth);
    DispHeight = my_DispHeight;
    Wr16(  REG_VSIZE, DispHeight);

    /* Touch configuration - configure the resistance value to 1200 - this value is specific to customer requirement and derived by experiment */
    Wr16(  REG_TOUCH_RZTHRESH,1200);
//    Wr16(  REG_TOUCH_RZTHRESH,0xFFFF);

    DispPCLK = my_DispPCLK;
    Wr8(  REG_PCLK, DispPCLK);//after this display is visible on the LCD



    Wr16(  REG_PWM_HZ, 10000);

//#ifdef Inv_Backlite               // turn on backlite
//    Wr16(  REG_PWM_DUTY, 0);
//#else
    Wr16(  REG_PWM_DUTY, 32);
//#endif
    Wr8(  REG_GPIO_DIR,0x82);  //| Rd8( REG_GPIO_DIR));
    Wr8(  REG_GPIO,0x080);     //| Rd8( REG_GPIO));

    Wr32(  RAM_DL, CLEAR(1,1,1));
    Wr32(  RAM_DL+4, DISPLAY());
    Wr32(  REG_DLSWAP,1);

    Wr16(  REG_PCLK, DispPCLK);



}



/* API to initialize the SPI interface */
ft_bool_t  FT800::Init()
{
    // is done in constructor
    return 1;
}


ft_bool_t  FT800::Open()
{
    cmd_fifo_wp = dl_buff_wp = 0;
    status = OPENED;
    return 1;
}

ft_void_t  FT800::Close( )
{
    status = CLOSED;
}

ft_void_t FT800::DeInit()
{

}

/*The APIs for reading/writing transfer continuously only with small buffer system*/
ft_void_t  FT800::StartTransfer( FT_GPU_TRANSFERDIR_T rw,ft_uint32_t addr)
{
    if (FT_GPU_READ == rw){
        _ss = 0;       // cs low
        _spi.write(addr >> 16);
        _spi.write(addr >> 8);
        _spi.write(addr & 0xff);
        _spi.write(0); //Dummy Read Byte
        status = READING;
    }else{
        _ss = 0;       // cs low
        _spi.write(0x80 | (addr >> 16));
        _spi.write(addr >> 8);
        _spi.write(addr & 0xff);
        status = WRITING;
    }
}


/*The APIs for writing transfer continuously only*/
ft_void_t  FT800::StartCmdTransfer( FT_GPU_TRANSFERDIR_T rw, ft_uint16_t count)
{
    StartTransfer( rw, cmd_fifo_wp + RAM_CMD);
}

ft_uint8_t  FT800::TransferString( const ft_char8_t *string)
{
    ft_uint16_t length = strlen(string);
    while(length --){
       Transfer8( *string);
       string ++;
    }
    //Append one null as ending flag
    Transfer8( 0);
    return(1);
}


ft_uint8_t  FT800::Transfer8( ft_uint8_t value)
{
        return _spi.write(value);
}


ft_uint16_t  FT800::Transfer16( ft_uint16_t value)
{
    ft_uint16_t retVal = 0;

        if (status == WRITING){
        Transfer8( value & 0xFF);//LSB first
        Transfer8( (value >> 8) & 0xFF);
    }else{
        retVal = Transfer8( 0);
        retVal |= (ft_uint16_t)Transfer8( 0) << 8;
    }

    return retVal;
}

ft_uint32_t  FT800::Transfer32( ft_uint32_t value)
{
    ft_uint32_t retVal = 0;
    if (status == WRITING){
        Transfer16( value & 0xFFFF);//LSB first
        Transfer16( (value >> 16) & 0xFFFF);
    }else{
        retVal = Transfer16( 0);
        retVal |= (ft_uint32_t)Transfer16( 0) << 16;
    }
    return retVal;
}

ft_void_t   FT800::EndTransfer( )
{
    _ss = 1;
    status = OPENED;
}


ft_uint8_t  FT800::Rd8( ft_uint32_t addr)
{
    ft_uint8_t value;
    StartTransfer( FT_GPU_READ,addr);
    value = Transfer8( 0);
    EndTransfer( );
    return value;
}
ft_uint16_t FT800::Rd16( ft_uint32_t addr)
{
    ft_uint16_t value;
    StartTransfer( FT_GPU_READ,addr);
    value = Transfer16( 0);
    EndTransfer( );
    return value;
}
ft_uint32_t FT800::Rd32( ft_uint32_t addr)
{
    ft_uint32_t value;
    StartTransfer( FT_GPU_READ,addr);
    value = Transfer32( 0);
    EndTransfer( );
    return value;
}

ft_void_t FT800::Wr8( ft_uint32_t addr, ft_uint8_t v)
{
    StartTransfer( FT_GPU_WRITE,addr);
    Transfer8( v);
    EndTransfer( );
}
ft_void_t FT800::Wr16( ft_uint32_t addr, ft_uint16_t v)
{
    StartTransfer( FT_GPU_WRITE,addr);
    Transfer16( v);
    EndTransfer( );
}
ft_void_t FT800::Wr32( ft_uint32_t addr, ft_uint32_t v)
{
    StartTransfer( FT_GPU_WRITE,addr);
    Transfer32( v);
    EndTransfer( );
}

ft_void_t FT800::HostCommand( ft_uint8_t cmd)
{
  _ss = 0;
  _spi.write(cmd);
  _spi.write(0);
  _spi.write(0);
  _ss = 1;
}

ft_void_t FT800::ClockSelect( FT_GPU_PLL_SOURCE_T pllsource)
{
   HostCommand( pllsource);
}

ft_void_t FT800::PLL_FreqSelect( FT_GPU_PLL_FREQ_T freq)
{
   HostCommand( freq);
}

ft_void_t FT800::PowerModeSwitch( FT_GPU_POWER_MODE_T pwrmode)
{
   HostCommand( pwrmode);
}

ft_void_t FT800::CoreReset( )
{
   HostCommand( 0x68);
}


ft_void_t FT800::Updatecmdfifo( ft_uint16_t count)
{
     cmd_fifo_wp  = ( cmd_fifo_wp + count) & 4095;
    //4 byte alignment
     cmd_fifo_wp = ( cmd_fifo_wp + 3) & 0xffc;
     
    do
    {
        Wr16( REG_CMD_WRITE, cmd_fifo_wp);
    } while (Rd16(REG_CMD_WRITE) != cmd_fifo_wp);
}


ft_uint16_t FT800::fifo_Freespace( )
{
    ft_uint16_t fullness,retval;

    fullness = ( cmd_fifo_wp - Rd16( REG_CMD_READ)) & 4095;
    retval = (FT_CMD_FIFO_SIZE - 4) - fullness;
    return (retval);
}

ft_void_t FT800::WrCmdBuf( ft_uint8_t *buffer,ft_uint16_t count)
{
    ft_uint32_t length =0, SizeTransfered = 0;

#define MAX_CMD_FIFO_TRANSFER   fifo_Freespace( )
    do {
        length = count;
        if (length > MAX_CMD_FIFO_TRANSFER){
            length = MAX_CMD_FIFO_TRANSFER;
        }
                CheckCmdBuffer( length);

                StartCmdTransfer( FT_GPU_WRITE,length);

                SizeTransfered = 0;
        while (length--) {
                    Transfer8( *buffer);
                    buffer++;
                    SizeTransfered ++;
        }
                length = SizeTransfered;

        EndTransfer( );
        Updatecmdfifo( length);

        WaitCmdfifo_empty( );

        count -= length;
    }while (count > 0);
}


ft_void_t FT800::WrCmdBufFromFlash( FT_PROGMEM ft_prog_uchar8_t *buffer,ft_uint16_t count)
{
    ft_uint32_t length =0, SizeTransfered = 0;

#define MAX_CMD_FIFO_TRANSFER   fifo_Freespace( )
    do {
        length = count;
        if (length > MAX_CMD_FIFO_TRANSFER){
            length = MAX_CMD_FIFO_TRANSFER;
        }
                CheckCmdBuffer( length);

                StartCmdTransfer( FT_GPU_WRITE,length);


                SizeTransfered = 0;
        while (length--) {
                    Transfer8( ft_pgm_read_byte_near(buffer));
            buffer++;
                    SizeTransfered ++;
        }
                length = SizeTransfered;

                EndTransfer( );
        Updatecmdfifo( length);

        WaitCmdfifo_empty( );

        count -= length;
    }while (count > 0);
}


ft_void_t FT800::CheckCmdBuffer( ft_uint16_t count)
{
   ft_uint16_t getfreespace;
   do{
        getfreespace = fifo_Freespace( );
   }while(getfreespace < count);
}

ft_void_t FT800::WaitCmdfifo_empty( )
{
   while(Rd16( REG_CMD_READ) != Rd16( REG_CMD_WRITE));
//    Thread::wait(5);
//    Thread::wait(10);
    cmd_fifo_wp = Rd16( REG_CMD_WRITE);
}

ft_void_t FT800::WaitLogo_Finish( )
{
    ft_int16_t cmdrdptr,cmdwrptr;

    do{
         cmdrdptr = Rd16( REG_CMD_READ);
         cmdwrptr = Rd16( REG_CMD_WRITE);
    }while ((cmdwrptr != cmdrdptr) || (cmdrdptr != 0));
     cmd_fifo_wp = 0;
}


ft_void_t FT800::ResetCmdFifo( )
{
    cmd_fifo_wp = 0;
}


ft_void_t FT800::WrCmd32( ft_uint32_t cmd)
{
         CheckCmdBuffer( sizeof(cmd));

         Wr32( RAM_CMD +  cmd_fifo_wp,cmd);

         Updatecmdfifo( sizeof(cmd));
}


ft_void_t FT800::ResetDLBuffer( )
{
            dl_buff_wp = 0;
}

/* Toggle PD_N pin of FT800 board for a power cycle*/
ft_void_t FT800::Powercycle(  ft_bool_t up)
{
    if (up)
    {
             //Toggle PD_N from low to high for power up switch
            _pd = 0;
            Sleep(20);

            _pd = 1;
            Sleep(20);
    }else
    {
             //Toggle PD_N from high to low for power down switch
            _pd = 1;
            Sleep(20);

            _pd = 0;
            Sleep(20);
    }
}

ft_void_t FT800::WrMemFromFlash( ft_uint32_t addr,const ft_prog_uchar8_t *buffer, ft_uint32_t length)
{
    //ft_uint32_t SizeTransfered = 0;

    StartTransfer( FT_GPU_WRITE,addr);

    while (length--) {
            Transfer8( ft_pgm_read_byte_near(buffer));
        buffer++;
    }

    EndTransfer( );
}

ft_void_t FT800::WrMem( ft_uint32_t addr,const ft_uint8_t *buffer, ft_uint32_t length)
{
    //ft_uint32_t SizeTransfered = 0;

    StartTransfer( FT_GPU_WRITE,addr);

    while (length--) {
            Transfer8( *buffer);
        buffer++;
    }

    EndTransfer( );
}


ft_void_t FT800::RdMem( ft_uint32_t addr, ft_uint8_t *buffer, ft_uint32_t length)
{
    //ft_uint32_t SizeTransfered = 0;

    StartTransfer( FT_GPU_READ,addr);

    while (length--) {
       *buffer = Transfer8( 0);
       buffer++;
    }

    EndTransfer( );
}

ft_int32_t FT800::Dec2Ascii(ft_char8_t *pSrc,ft_int32_t value)
{
    ft_int16_t Length;
    ft_char8_t *pdst,charval;
    ft_int32_t CurrVal = value,tmpval,i;
    ft_char8_t tmparray[16],idx = 0;

    Length = strlen(pSrc);
    pdst = pSrc + Length;

    if(0 == value)
    {
        *pdst++ = '0';
        *pdst++ = '\0';
        return 0;
    }

    if(CurrVal < 0)
    {
        *pdst++ = '-';
        CurrVal = - CurrVal;
    }
    /* insert the value */
    while(CurrVal > 0){
        tmpval = CurrVal;
        CurrVal /= 10;
        tmpval = tmpval - CurrVal*10;
        charval = '0' + tmpval;
        tmparray[idx++] = charval;
    }

    for(i=0;i<idx;i++)
    {
        *pdst++ = tmparray[idx - i - 1];
    }
    *pdst++ = '\0';

    return 0;
}


ft_void_t FT800::Sleep(ft_uint16_t ms)
{
    wait_ms(ms);
}

ft_void_t FT800::Sound_ON(){
     Wr8(  REG_GPIO, 0x02 | Rd8( REG_GPIO));
}

ft_void_t FT800::Sound_OFF(){
     Wr8(  REG_GPIO, 0xFD & Rd8( REG_GPIO));
}




