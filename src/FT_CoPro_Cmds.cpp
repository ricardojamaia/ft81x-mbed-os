/* mbed Library for FTDI FT800  Enbedded Video Engine "EVE"
 * based on Original Code Sample from FTDI
 * ported to mbed by Peter Drescher, DC2PD 2014
 * Released under the MIT License: http://mbed.org/license/mit */

#include "FT_Platform.h"


ft_void_t FT800::SendCmd( ft_uint32_t cmd)
{
   Transfer32( cmd);
}

ft_void_t FT800::SendStr( const ft_char8_t *s)
{
  TransferString( s);
}


ft_void_t FT800::StartFunc( ft_uint16_t count)
{
  CheckCmdBuffer( count);
  StartCmdTransfer( FT_GPU_WRITE,count);
}

ft_void_t FT800::EndFunc( ft_uint16_t count)
{
  EndTransfer( );
  Updatecmdfifo( count);
}

ft_void_t FT800::Text( ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s)
{
  StartFunc( FT_CMD_SIZE*3 + strlen(s) + 1);
  SendCmd(  CMD_TEXT);
  //Copro_SendCmd(  (((ft_uint32_t)y<<16)|(ft_uint32_t)x));
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)options<<16)|(ft_uint32_t)font));
  SendStr(  s);
  EndFunc( (FT_CMD_SIZE*3 + strlen(s) + 1));
}

ft_void_t FT800::Number( ft_int16_t x, ft_int16_t y, ft_int16_t font, ft_uint16_t options, ft_int32_t n)
{
  StartFunc( FT_CMD_SIZE*4);
  SendCmd(  CMD_NUMBER);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)options<<16)|font));
  SendCmd(  n);
  EndFunc( (FT_CMD_SIZE*4));
}

ft_void_t FT800::LoadIdentity( )
{
  StartFunc( FT_CMD_SIZE*1);
  SendCmd(  CMD_LOADIDENTITY);
  EndFunc( (FT_CMD_SIZE*1));
}

ft_void_t FT800::Toggle( ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t font, ft_uint16_t options, ft_uint16_t state, const ft_char8_t* s)
{
  StartFunc( FT_CMD_SIZE*4 + strlen(s) + 1);
  SendCmd(  CMD_TOGGLE);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)font<<16)|w));
  SendCmd(  (((ft_uint32_t)state<<16)|options));
  SendStr(  s);
  EndFunc( (FT_CMD_SIZE*4 + strlen(s) + 1));
}

/* Error handling for val is not done, so better to always use range of 65535 in order that needle is drawn within display region */
ft_void_t FT800::Gauge( ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t major, ft_uint16_t minor, ft_uint16_t val, ft_uint16_t range)
{
  StartFunc( FT_CMD_SIZE*5);
  SendCmd(  CMD_GAUGE);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)options<<16)|r));
  SendCmd(  (((ft_uint32_t)minor<<16)|major));
  SendCmd(  (((ft_uint32_t)range<<16)|val));
  EndFunc( (FT_CMD_SIZE*5));
}

ft_void_t FT800::RegRead( ft_uint32_t ptr, ft_uint32_t result)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_REGREAD);
  SendCmd(  ptr);
  SendCmd(  0);
  EndFunc( (FT_CMD_SIZE*3));

}

ft_void_t FT800::GetProps( ft_uint32_t ptr, ft_uint32_t w, ft_uint32_t h)
{
  StartFunc( FT_CMD_SIZE*4);
  SendCmd(  CMD_GETPROPS);
  SendCmd(  ptr);
  SendCmd(  w);
  SendCmd(  h);
  EndFunc( (FT_CMD_SIZE*4));
}

ft_void_t FT800::Memcpy( ft_uint32_t dest, ft_uint32_t src, ft_uint32_t num)
{
  StartFunc( FT_CMD_SIZE*4);
  SendCmd(  CMD_MEMCPY);
  SendCmd(  dest);
  SendCmd(  src);
  SendCmd(  num);
  EndFunc( (FT_CMD_SIZE*4));
}

ft_void_t FT800::Spinner( ft_int16_t x, ft_int16_t y, ft_uint16_t style, ft_uint16_t scale)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_SPINNER);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)scale<<16)|style));
  EndFunc( (FT_CMD_SIZE*3));
}

ft_void_t FT800::BgColor( ft_uint32_t c)
{
  StartFunc( FT_CMD_SIZE*2);
  SendCmd(  CMD_BGCOLOR);
  SendCmd(  c);
  EndFunc( (FT_CMD_SIZE*2));
}

ft_void_t FT800::Swap()
{
  StartFunc( FT_CMD_SIZE*1);
  SendCmd(  CMD_SWAP);
  EndFunc( (FT_CMD_SIZE*1));
}

ft_void_t FT800::Inflate( ft_uint32_t ptr)
{
  StartFunc( FT_CMD_SIZE*2);
  SendCmd(  CMD_INFLATE);
  SendCmd(  ptr);
  EndFunc( (FT_CMD_SIZE*2));
}

ft_void_t FT800::Translate( ft_int32_t tx, ft_int32_t ty)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_TRANSLATE);
  SendCmd(  tx);
  SendCmd(  ty);
  EndFunc( (FT_CMD_SIZE*3));
}

ft_void_t FT800::Stop()
{
  StartFunc( FT_CMD_SIZE*1);
  SendCmd(  CMD_STOP);
  EndFunc( (FT_CMD_SIZE*1));
}

ft_void_t FT800::Slider( ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t range)
{
  StartFunc( FT_CMD_SIZE*5);
  SendCmd(  CMD_SLIDER);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)h<<16)|w));
  SendCmd(  (((ft_uint32_t)val<<16)|options));
  SendCmd(  range);
  EndFunc( (FT_CMD_SIZE*5));
}

ft_void_t FT800::TouchTransform( ft_int32_t x0, ft_int32_t y0, ft_int32_t x1, ft_int32_t y1, ft_int32_t x2, ft_int32_t y2, ft_int32_t tx0, ft_int32_t ty0, ft_int32_t tx1, ft_int32_t ty1, ft_int32_t tx2, ft_int32_t ty2, ft_uint16_t result)
{
  StartFunc( FT_CMD_SIZE*6*2+FT_CMD_SIZE*2);
  SendCmd(  CMD_TOUCH_TRANSFORM);
  SendCmd(  x0);
  SendCmd(  y0);
  SendCmd(  x1);
  SendCmd(  y1);
  SendCmd(  x2);
  SendCmd(  y2);
  SendCmd(  tx0);
  SendCmd(  ty0);
  SendCmd(  tx1);
  SendCmd(  ty1);
  SendCmd(  tx2);
  SendCmd(  ty2);
  SendCmd(  result);
  EndFunc( (FT_CMD_SIZE*6*2+FT_CMD_SIZE*2));
}

ft_void_t FT800::Interrupt( ft_uint32_t ms)
{
  StartFunc( FT_CMD_SIZE*2);
  SendCmd(  CMD_INTERRUPT);
  SendCmd(  ms);
  EndFunc( (FT_CMD_SIZE*2));
}

ft_void_t FT800::FgColor( ft_uint32_t c)
{
  StartFunc( FT_CMD_SIZE*2);
  SendCmd(  CMD_FGCOLOR);
  SendCmd(  c);
  EndFunc( (FT_CMD_SIZE*2));
}

ft_void_t FT800::Rotate( ft_int32_t a)
{
  StartFunc( FT_CMD_SIZE*2);
  SendCmd(  CMD_ROTATE);
  SendCmd(  a);
  EndFunc( (FT_CMD_SIZE*2));
}

ft_void_t FT800::Button( ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s)
{
  StartFunc( FT_CMD_SIZE*4 + strlen(s) + 1);
  SendCmd(  CMD_BUTTON);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)h<<16)|w));
  SendCmd( (((ft_uint32_t)options<<16)|font));       // patch from Ivano Pelicella to draw flat buttons
  SendStr(  s);
  EndFunc( (FT_CMD_SIZE*4 + strlen(s) + 1));
}

ft_void_t FT800::MemWrite( ft_uint32_t ptr, ft_uint32_t num)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_MEMWRITE);
  SendCmd(  ptr);
  SendCmd(  num);
  EndFunc( (FT_CMD_SIZE*3));
}

ft_void_t FT800::Scrollbar( ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t size, ft_uint16_t range)
{
  StartFunc( FT_CMD_SIZE*5);
  SendCmd(  CMD_SCROLLBAR);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)h<<16)|w));
  SendCmd(  (((ft_uint32_t)val<<16)|options));
  SendCmd(  (((ft_uint32_t)range<<16)|size));
  EndFunc( (FT_CMD_SIZE*5));
}

ft_void_t FT800::GetMatrix( ft_int32_t a, ft_int32_t b, ft_int32_t c, ft_int32_t d, ft_int32_t e, ft_int32_t f)
{
  StartFunc( FT_CMD_SIZE*7);
  SendCmd(  CMD_GETMATRIX);
  SendCmd(  a);
  SendCmd(  b);
  SendCmd(  c);
  SendCmd(  d);
  SendCmd(  e);
  SendCmd(  f);
  EndFunc( (FT_CMD_SIZE*7));
}

ft_void_t FT800::Sketch( ft_int16_t x, ft_int16_t y, ft_uint16_t w, ft_uint16_t h, ft_uint32_t ptr, ft_uint16_t format)
{
  StartFunc( FT_CMD_SIZE*5);
  SendCmd(  CMD_SKETCH);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)h<<16)|w));
  SendCmd(  ptr);
  SendCmd(  format);
  EndFunc( (FT_CMD_SIZE*5));
}
ft_void_t FT800::MemSet( ft_uint32_t ptr, ft_uint32_t value, ft_uint32_t num)
{
  StartFunc( FT_CMD_SIZE*4);
  SendCmd(  CMD_MEMSET);
  SendCmd(  ptr);
  SendCmd(  value);
  SendCmd(  num);
  EndFunc( (FT_CMD_SIZE*4));
}
ft_void_t FT800::GradColor( ft_uint32_t c)
{
  StartFunc( FT_CMD_SIZE*2);
  SendCmd(  CMD_GRADCOLOR);
  SendCmd(  c);
  EndFunc( (FT_CMD_SIZE*2));
}
ft_void_t FT800::BitmapTransform( ft_int32_t x0, ft_int32_t y0, ft_int32_t x1, ft_int32_t y1, ft_int32_t x2, ft_int32_t y2, ft_int32_t tx0, ft_int32_t ty0, ft_int32_t tx1, ft_int32_t ty1, ft_int32_t tx2, ft_int32_t ty2, ft_uint16_t result)
{
  StartFunc( FT_CMD_SIZE*6*2+FT_CMD_SIZE*2);
  SendCmd(  CMD_BITMAP_TRANSFORM);
  SendCmd(  x0);
  SendCmd(  y0);
  SendCmd(  x1);
  SendCmd(  y1);
  SendCmd(  x2);
  SendCmd(  y2);
  SendCmd(  tx0);
  SendCmd(  ty0);
  SendCmd(  tx1);
  SendCmd(  ty1);
  SendCmd(  tx2);
  SendCmd(  ty2);
  SendCmd(  result);
  EndFunc( (FT_CMD_SIZE*6*2+FT_CMD_SIZE*2));
}
ft_void_t FT800::Calibrate( ft_uint32_t result)
{
  StartFunc( FT_CMD_SIZE*2);
  SendCmd(  CMD_CALIBRATE);
  SendCmd(  result);
  EndFunc( (FT_CMD_SIZE*2));
  WaitCmdfifo_empty( );

}
ft_void_t FT800::SetFont( ft_uint32_t font, ft_uint32_t ptr)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_SETFONT);
  SendCmd(  font);
  SendCmd(  ptr);
  EndFunc( (FT_CMD_SIZE*3));
}
//Curtis Mattull added this function on 11/14/16, copied from above...
ft_void_t FT800::RomFont( ft_uint32_t rom_slot, ft_uint32_t font)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_ROMFONT);
  SendCmd(  rom_slot);
  SendCmd(  font);
  EndFunc( (FT_CMD_SIZE*3));
}


ft_void_t FT800::Logo(  )
{
  StartFunc( FT_CMD_SIZE*1);
  SendCmd(  CMD_LOGO);
  EndFunc( FT_CMD_SIZE*1);
}
ft_void_t FT800::Append( ft_uint32_t ptr, ft_uint32_t num)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_APPEND);
  SendCmd(  ptr);
  SendCmd(  num);
  EndFunc( (FT_CMD_SIZE*3));
}
ft_void_t FT800::MemZero( ft_uint32_t ptr, ft_uint32_t num)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_MEMZERO);
  SendCmd(  ptr);
  SendCmd(  num);
  EndFunc( (FT_CMD_SIZE*3));
}
ft_void_t FT800::Scale( ft_int32_t sx, ft_int32_t sy)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_SCALE);
  SendCmd(  sx);
  SendCmd(  sy);
  EndFunc( (FT_CMD_SIZE*3));
}
ft_void_t FT800::Clock( ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t h, ft_uint16_t m, ft_uint16_t s, ft_uint16_t ms)
{
  StartFunc( FT_CMD_SIZE*5);
  SendCmd(  CMD_CLOCK);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)options<<16)|r));
  SendCmd(  (((ft_uint32_t)m<<16)|h));
  SendCmd(  (((ft_uint32_t)ms<<16)|s));
  EndFunc( (FT_CMD_SIZE*5));
}

ft_void_t FT800::Gradient( ft_int16_t x0, ft_int16_t y0, ft_uint32_t rgb0, ft_int16_t x1, ft_int16_t y1, ft_uint32_t rgb1)
{
  StartFunc( FT_CMD_SIZE*5);
  SendCmd(  CMD_GRADIENT);
  SendCmd(  (((ft_uint32_t)y0<<16)|(x0 & 0xffff)));
  SendCmd(  rgb0);
  SendCmd(  (((ft_uint32_t)y1<<16)|(x1 & 0xffff)));
  SendCmd(  rgb1);
  EndFunc( (FT_CMD_SIZE*5));
}

ft_void_t FT800::SetMatrix(  )
{
  StartFunc( FT_CMD_SIZE*1);
  SendCmd(  CMD_SETMATRIX);
  EndFunc( (FT_CMD_SIZE*1));
}

ft_void_t FT800::Track( ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t tag)
{
  StartFunc( FT_CMD_SIZE*4);
  SendCmd(  CMD_TRACK);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)h<<16)|w));
  SendCmd(  tag);
  EndFunc( (FT_CMD_SIZE*4));
}

ft_void_t FT800::GetPtr( ft_uint32_t result)
{
  StartFunc( FT_CMD_SIZE*2);
  SendCmd(  CMD_GETPTR);
  SendCmd(  result);
  EndFunc( (FT_CMD_SIZE*2));
}

ft_void_t FT800::Progress( ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_uint16_t options, ft_uint16_t val, ft_uint16_t range)
{
  StartFunc( FT_CMD_SIZE*5);
  SendCmd(  CMD_PROGRESS);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)h<<16)|w));
  SendCmd(  (((ft_uint32_t)val<<16)|options));
  SendCmd(  range);
  EndFunc( (FT_CMD_SIZE*5));
}

ft_void_t FT800::ColdStart(  )
{
  StartFunc( FT_CMD_SIZE*1);
  SendCmd(  CMD_COLDSTART);
  EndFunc( (FT_CMD_SIZE*1));
}

ft_void_t FT800::Keys( ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t h, ft_int16_t font, ft_uint16_t options, const ft_char8_t* s)
{
  StartFunc( FT_CMD_SIZE*4 + strlen(s) + 1);
  SendCmd(  CMD_KEYS);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)h<<16)|w));
  SendCmd(  (((ft_uint32_t)options<<16)|font));
  SendStr(  s);
  EndFunc( (FT_CMD_SIZE*4 + strlen(s) + 1));
}

ft_void_t FT800::Dial( ft_int16_t x, ft_int16_t y, ft_int16_t r, ft_uint16_t options, ft_uint16_t val)
{
  StartFunc( FT_CMD_SIZE*4);
  SendCmd(  CMD_DIAL);
  SendCmd(  (((ft_uint32_t)y<<16)|(x & 0xffff)));
  SendCmd(  (((ft_uint32_t)options<<16)|r));
  SendCmd(  val);
  EndFunc( (FT_CMD_SIZE*4));
}

ft_void_t FT800::LoadImage( ft_uint32_t ptr, ft_uint32_t options)
{
  StartFunc( FT_CMD_SIZE*3);
  SendCmd(  CMD_LOADIMAGE);
  SendCmd(  ptr);
  SendCmd(  options);
  EndFunc( (FT_CMD_SIZE*3));
}

ft_void_t FT800::DLstart(  )
{
  StartFunc( FT_CMD_SIZE*1);
  SendCmd(  CMD_DLSTART);
  EndFunc( (FT_CMD_SIZE*1));
}

ft_void_t FT800::Snapshot( ft_uint32_t ptr)
{
  StartFunc( FT_CMD_SIZE*2);
  SendCmd(  CMD_SNAPSHOT);
  SendCmd(  ptr);
  EndFunc( (FT_CMD_SIZE*2));
}

ft_void_t FT800::ScreenSaver(  )
{
  StartFunc( FT_CMD_SIZE*1);
  SendCmd(  CMD_SCREENSAVER);
  EndFunc( (FT_CMD_SIZE*1));
}

ft_void_t FT800::MemCrc( ft_uint32_t ptr, ft_uint32_t num, ft_uint32_t result)
{
  StartFunc( FT_CMD_SIZE*4);
  SendCmd(  CMD_MEMCRC);
  SendCmd(  ptr);
  SendCmd(  num);
  SendCmd(  result);
  EndFunc( (FT_CMD_SIZE*4));
}


ft_void_t FT800::DL(ft_uint32_t cmd)
{
   WrCmd32(cmd);
   /* Increment the command index */
   CmdBuffer_Index += FT_CMD_SIZE;
}

ft_void_t FT800::WrDlCmd_Buffer(ft_uint32_t cmd)
{
   Wr32((RAM_DL+DlBuffer_Index),cmd);
   /* Increment the command index */
   DlBuffer_Index += FT_CMD_SIZE;
}

ft_void_t FT800::Flush_DL_Buffer()
{
   DlBuffer_Index = 0;

}

ft_void_t FT800::Flush_Co_Buffer()
{
   CmdBuffer_Index = 0;
}


/* API to check the status of previous DLSWAP and perform DLSWAP of new DL */
/* Check for the status of previous DLSWAP and if still not done wait for few ms and check again */
ft_void_t FT800::DLSwap(ft_uint8_t DL_Swap_Type)
{
    ft_uint8_t Swap_Type = DLSWAP_FRAME,Swap_Done = DLSWAP_FRAME;

    if(DL_Swap_Type == DLSWAP_LINE)
    {
        Swap_Type = DLSWAP_LINE;
    }

    /* Perform a new DL swap */
    Wr8(REG_DLSWAP,Swap_Type);

    /* Wait till the swap is done */
    while(Swap_Done)
    {
        Swap_Done = Rd8(REG_DLSWAP);

        if(DLSWAP_DONE != Swap_Done)
        {
            Sleep(10);//wait for 10ms
        }
    }
}



/* Nothing beyond this */




