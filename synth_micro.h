// ATmega32u4
// Modification to support arduino micro
// signals on arduino micro pin 13(CHA) and pin 10(CHB)
// Pin out diagram: https://content.arduino.cc/assets/arduino_micro_schematic.pdf
#ifndef _SYNTH
#define _SYNTH
//*************************************************************************************
//  Arduino synth V4.1
//  Optimized audio driver, modulation engine, envelope engine.
//
//  Dzl/Illutron 2014
//
//*************************************************************************************
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "tables.h"

#define DIFF 1
#define CHA 2
#define CHB 3

#define SINE 0
#define TRIANGLE 1
#define SQUARE 2
#define SAW 3
#define RAMP 4
#define NOISE 5

#define ENVELOPE0 0
#define ENVELOPE1 1
#define ENVELOPE2 2
#define ENVELOPE3 3

#define FS 20000.0 //-Sample rate (NOTE: must match tables.h)

#define SET(x, y) (x |= (1 << y))    //-Bit set/clear macros
#define CLR(x, y) (x &= (~(1 << y))) // |
#define CHK(x, y) (x & (1 << y))     // |
#define TOG(x, y) (x ^= (1 << y))    //-+

volatile unsigned int PCW[4] = {
    0, 0, 0, 0}; //-Wave phase accumolators
volatile unsigned int FTW[4] = {
    1000, 200, 300, 400}; //-Wave frequency tuning words
volatile unsigned char AMP[4] = {
    255, 255, 255, 255}; //-Wave amplitudes [0-255]
volatile unsigned int PITCH[4] = {
    500, 500, 500, 500}; //-Voice pitch
volatile int MOD[4] = {
    20, 0, 64, 127};           //-Voice envelope modulation [0-1023 512=no mod. <512 pitch down >512 pitch up]
volatile unsigned int wavs[4]; //-Wave table selector [address of wave in memory]
volatile unsigned int envs[4]; //-Envelopte selector [address of envelope in memory]
volatile unsigned int EPCW[4] = {
    0x8000, 0x8000, 0x8000, 0x8000}; //-Envelope phase accumolator
volatile unsigned int EFTW[4] = {
    10, 10, 10, 10};                //-Envelope speed tuning word
volatile unsigned char divider = 4; //-Sample rate decimator for envelope
volatile unsigned int tim = 0;
volatile unsigned char tik = 0;
volatile unsigned char output_mode;

//*********************************************************************************************
//  Audio driver interrupt
//*********************************************************************************************

SIGNAL(TIMER3_COMPA_vect)
{
  //-------------------------------
  // Time division
  //-------------------------------
  divider++;
  if (!(divider &= 0x03))
    tik = 1;

  //-------------------------------
  // Volume envelope generator
  //-------------------------------

  if (!(((unsigned char *)&EPCW[divider])[1] & 0x80))
    AMP[divider] = pgm_read_byte(envs[divider] + (((unsigned char *)&(EPCW[divider] += EFTW[divider]))[1]));
  else
    AMP[divider] = 0;

  //-------------------------------
  //  Synthesizer/audio mixer
  //-------------------------------

  OCR4A = OCR4B = 127 +
                  ((
                       (((signed char)pgm_read_byte(wavs[0] + ((unsigned char *)&(PCW[0] += FTW[0]))[1]) * AMP[0]) >> 8) +
                       (((signed char)pgm_read_byte(wavs[1] + ((unsigned char *)&(PCW[1] += FTW[1]))[1]) * AMP[1]) >> 8) +
                       (((signed char)pgm_read_byte(wavs[2] + ((unsigned char *)&(PCW[2] += FTW[2]))[1]) * AMP[2]) >> 8) +
                       (((signed char)pgm_read_byte(wavs[3] + ((unsigned char *)&(PCW[3] += FTW[3]))[1]) * AMP[3]) >> 8)) >>
                   2);

  //************************************************
  //  Modulation engine
  //************************************************
  //  FTW[divider] = PITCH[divider] + (int)   (((PITCH[divider]/64)*(EPCW[divider]/64)) /128)*MOD[divider];
  FTW[divider] = PITCH[divider] + (int)(((PITCH[divider] >> 6) * (EPCW[divider] >> 6)) / 128) * MOD[divider];
  tim++;
}

class synth_micro
{
private:
public:
  synth_micro()
  {
  }

  void begin()
  {
    output_mode = CHA;
    TCCR3A = 0x00; //-Start audio interrupt
    TCCR3B = 0x09;
    TCCR3C = 0x00;
    OCR3A = 16000000.0 / FS; //-Auto sample rate
    SET(TIMSK3, OCIE3A);     //-Start audio interrupt
    sei();                   //-+  //-+

    TCCR4A = 0x83; //-8 bit audio PWM
    TCCR4B = 0x01; // |
    OCR4A = 127;   //-+
    SET(DDRC, 7);  //-PWM pin // arduino pin 13
  }

  //*********************************************************************
  //  Startup fancy selecting varoius output modes
  //*********************************************************************

  void begin(unsigned char d)
  {
    TCCR3A = 0x00; //-Start audio interrupt
    TCCR3B = 0x09;
    TCCR3C = 0x00;
    OCR3A = 16000000.0 / FS; //-Auto sample rate
    SET(TIMSK3, OCIE3A);     //-Start audio interrupt
    sei();                   //-+  //-+

    output_mode = d;

    switch (d)
    {
    case DIFF:             //-Differntial signal on CHA and CHB pins (13, 10)
      TCCR4A = 0xB3;       //-8 bit audio PWM
      TCCR4B = 0x01;       // |
      OCR4A = OCR4B = 127; //-+

      SET(DDRB, 6); //-PWM pin - arduino micro pin 10
      SET(DDRC, 7); //-PWM pin - arduino micro 13
      break;

    case CHB:        //-Single ended signal on CHB pin (10)
      TCCR4A = 0x23; //-8 bit audio PWM
      TCCR4B = 0x01; // |
      OCR4B = 127;   //-+
      SET(DDRB, 6);  //-PWM pin - arduino micro pin 10
      break;

    case CHA:
    default:
      output_mode = CHA;   //-Single ended signal in CHA pin (13)
      TCCR4A = 0x83;       //-8 bit audio PWM
      TCCR4B = 0x01;       // |
      OCR4A = OCR4B = 127; //-+
      SET(DDRC, 7);        //-PWM pin - arduino micro pin 13
      break;
    }
  }

  //*********************************************************************
  //  Timing/sequencing functions
  //*********************************************************************

  unsigned char synthTick(void)
  {
    if (tik)
    {
      tik = 0;
      return 1; //-True every 4 samples
    }
    return 0;
  }

  unsigned char voiceFree(unsigned char voice)
  {
    if (!(((unsigned char *)&EPCW[voice])[1] & 0x80))
      return 0;
    return 1;
  }

  //*********************************************************************
  //  Setup all voice parameters in MIDI range
  //  voice[0-3],wave[0-6],pitch[0-127],envelope[0-4],length[0-127],mod[0-127:64=no mod]
  //*********************************************************************

  void setupVoice(unsigned char voice, unsigned char wave, unsigned char pitch, unsigned char env, unsigned char length, unsigned int mod)
  {
    setWave(voice, wave);
    setPitch(voice, pitch);
    setEnvelope(voice, env);
    setLength(voice, length);
    setMod(voice, mod);
  }

  //*********************************************************************
  //  Setup wave [0-6]
  //*********************************************************************

  void setWave(unsigned char voice, unsigned char wave)
  {
    switch (wave)
    {
    case TRIANGLE:
      wavs[voice] = (unsigned int)TriangleTable;
      break;
    case SQUARE:
      wavs[voice] = (unsigned int)SquareTable;
      break;
    case SAW:
      wavs[voice] = (unsigned int)SawTable;
      break;
    case RAMP:
      wavs[voice] = (unsigned int)RampTable;
      break;
    case NOISE:
      wavs[voice] = (unsigned int)NoiseTable;
      break;
    default:
      wavs[voice] = (unsigned int)SinTable;
      break;
    }
  }
  //*********************************************************************
  //  Setup Pitch [0-127]
  //*********************************************************************

  void setPitch(unsigned char voice, unsigned char MIDInote)
  {
    PITCH[voice] = pgm_read_word(&PITCHS[MIDInote]);
  }

  //*********************************************************************
  //  Setup Envelope [0-4]
  //*********************************************************************

  void setEnvelope(unsigned char voice, unsigned char env)
  {
    switch (env)
    {
    case 1:
      envs[voice] = (unsigned int)Env0;
      break;
    case 2:
      envs[voice] = (unsigned int)Env1;
      break;
    case 3:
      envs[voice] = (unsigned int)Env2;
      break;
    case 4:
      envs[voice] = (unsigned int)Env3;
      break;
    default:
      envs[voice] = (unsigned int)Env0;
      break;
    }
  }

  //*********************************************************************
  //  Setup Length [0-128]
  //*********************************************************************

  void setLength(unsigned char voice, unsigned char length)
  {
    EFTW[voice] = pgm_read_word(&EFTWS[length]);
  }

  //*********************************************************************
  //  Setup mod
  //*********************************************************************

  void setMod(unsigned char voice, unsigned char mod)
  {
    //    MOD[voice]=(unsigned int)mod*8;//0-1023 512=no mod
    MOD[voice] = (int)mod - 64; // 0-1023 512=no mod
  }

  //*********************************************************************
  //  Midi trigger
  //*********************************************************************

  void mTrigger(unsigned char voice, unsigned char MIDInote)
  {
    PITCH[voice] = pgm_read_word(&PITCHS[MIDInote]);
    EPCW[voice] = 0;
    FTW[divider] = PITCH[voice] + (int)(((PITCH[voice] >> 6) * (EPCW[voice] >> 6)) / 128) * MOD[voice];
  }

  //*********************************************************************
  //  Set frequency direct
  //*********************************************************************

  void setFrequency(unsigned char voice, float f)
  {
    PITCH[voice] = f / (FS / 65535.0);
  }

  //*********************************************************************
  //  Set time
  //*********************************************************************

  void setTime(unsigned char voice, float t)
  {
    EFTW[voice] = (1.0 / t) / (FS / (32767.5 * 10.0)); //[s];
  }

  //*********************************************************************
  //  Simple trigger
  //*********************************************************************

  void trigger(unsigned char voice)
  {
    EPCW[voice] = 0;
    FTW[voice] = PITCH[voice];
    //    FTW[voice]=PITCH[voice]+(PITCH[voice]*(EPCW[voice]/(32767.5*128.0  ))*((int)MOD[voice]-512));
  }

  //*********************************************************************
  //  Suspend/resume synth
  //*********************************************************************

  void suspend()
  {
    CLR(TIMSK3, OCIE3A); //-Stop audio interrupt
  }
  void resume()
  {
    SET(TIMSK3, OCIE3A); //-Start audio interrupt
  }
};

#endif
