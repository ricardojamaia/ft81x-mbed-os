//this file modified by Curtis Mattull 7/21/16
// to add support for beyondtek lcd screen

//comment out line 12 unless using beyondtek

/* mbed Library for FTDI FT810  Enbedded Video Engine "EVE"
 * ported to mbed by David Childs, based on the great work of Peter Drescher (DC2PD 2014) 
 * Released under the MIT License: http://mbed.org/license/mit */
// This is for the 800x480 screen that comes with the FTDI FT810 demo board

#ifndef FT_LCD_TYPE_H
#define FT_LCD_TYPE_H
/* Global variables for display resolution to support various display panels */

/*
    WVGA setup from 
    https://github.com/NewhavenDisplay/EVE2-TFT-Modules/blob/master/5.0in/Resistive/FT_App_Keyboard/Src/FT_App_Keyboard.c
    
    FT_DispWidth = 800;
    FT_DispHeight = 480;
    FT_DispHCycle =  928;
    FT_DispHOffset = 88;
    FT_DispHSync0 = 0;
    FT_DispHSync1 = 48;
    FT_DispVCycle = 525;
    FT_DispVOffset = 32;
    FT_DispVSync0 = 0;
    FT_DispVSync1 = 3;
    FT_DispPCLK = 2;
    FT_DispSwizzle = 0;
    FT_DispPCLKPol = 1;
    FT_DispCSpread = 0;
    FT_DispDither = 1;
    */

#define my_DispWidth    480
#define my_DispHeight   272
#define my_DispHCycle   548
#define my_DispHOffset  43
#define my_DispHSync0   0
#define my_DispHSync1   41
#define my_DispVCycle   292
#define my_DispVOffset  12
#define my_DispVSync0   0
#define my_DispVSync1   10
#define my_DispPCLK     5
#define my_DispSwizzle  1
#define my_DispPCLKPol  1
#define my_DispCSpread  1
#define my_DispDither   1

#endif
