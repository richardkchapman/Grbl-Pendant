/*
  Inspired by GRBL universal DRO (grudr11) by jpbbricole

  Use an Arduino Mega, for multiple serial ports and sufficient program memory

  Mega is also simple for prototyping with the TFT shield as it means there is still access to TX1/RX1 and other pins
  
  Can operate in one of two hardware configurations:
  1. Inline between the computer running UGS or similar and the second Arduino running GRBL 1.1g or later
     Commands received on the USB port (connected to UGS) are echoed throgh to the second serial port
     (connected to GRBL) and vice versa. Additional commands can be injected as needed - we can guarantee
     no clashes on the serial wire and can tell from sniffing the traffic when UGS is idle. We can also
     prevent some output from GRBL ever reaching UGS, such as responses to jog requests we initiate.
  2. Sniffing the traffic on the existing serial connection between UGS and GRBL.
     We can see (but not modify) all traffic from GRBL to UGS. We cannot see traffic from UGS to GRBL
     (to do so we would need another serial port on the DRO's Arduino that was not the USB port - might be
     possible on a Mega or using a standalone Atmel chip like an ATMega324)
     It is possible to inject commands to the UGS but they may clash with commands from UGS (resulting in
     corruption). Injecting jog commands will result in UGS receiving unecpected "ok" responses from GRBL
     causing it to pop up a dialog. I have modified my copy of UGS to suppress this message but unless/until
     that change is accepted in the upstream UGS repository (or you modify your own copy of UGS in the same
     way) this means you can't really jog while connected to UGS via this scheme

  The program logic and the hardware layout is the same for either technique - the difference is whether you
  connect UGS to the pendant's USB port (mode 1) or to the GRBL controller's USB port (mode 2). Also if using
  mode 1 you may want to add a FTDI USB converter to the pendant's output so that you can use a standard USB
  connection to the GRBL controller rather than connecting to the tx/rx pins at the GRBL end.

  We can use a diode to allow us to safely inject onto the tx line, though a SN74LVC1G07 is better (and probably needed
  if voltage levels don't match, e.g. if using a bluetooth link).
  
  Other solutions that have been tried (and work okish include using a 4066 chip to gate our output onto tx only when 
  e are using it, and disabling the tx part of Serial1 using UCSR0B &= ~bit (TXEN0) or similar (depends on board) 
  when not using t (and thus letting the pin go high impedance).

  TODO:
    - Use state==idle to decide whether to allow jogging (we see that even if we don't see traffic from ugs
    - Add some jogging support (hardware and software)!
    - Add rotary encoder reading code

  The jogging I have in mind has three use-cases:
  1. Simple jogging of my Shapeoko so that I can have my computer further away from my (dusty) mill.
     I will want to be able to jog and zero all three axes. Maybe even support a probe
  2. Manual control (fine positioning and power feed) of one or two axes of a mini-mill, to save winding
     handles all the time.
  3. Auto-stepover when using manual feed on mill

  I foresee the following UI:
  - use rotary knob for fine positioning (coarse/fine via rotary button)
  - a couple of (on)-off-(on) switches (one for each powered axis) - or perhaps on-off-on will work better?
  - an option in menu to set soft travel limits that the powerfeed will go to
    - if you release before it gets there we stop
  - an option to automatically step a second axis at one or both ends of current axis travel

  Soft buttons - I have room for 6. What should I use them for?
    - inches/mm
    - set soft limits
    - coarse/fine indication ?
    - RPM lookup table?
    - calculator ?
    - Auto-stepover at one or both limits
    - spindle on/off?

  https://github.com/gnea/grbl/wiki/Grbl-v1.1-Jogging describes ways to jog using a joystick, with a focus on how to 
  make the speed reflect the joystick position. I am not planning to put a joystick on this UI (maybe I should!) so
  a lot of it does not apply - I will have to try my scheme out in real use cases to see how it needs tweaking.

  Getting to the point where I should give up on idea of using a Leonardo - code is nearly full and feature set isn't,
  plus it's not easy to wire with the LCD shield in place. Would need to create a stacked shield but then the size is not so
  good. Other than being slightly cheaper there's no real plus side
*/

#include <Arduino.h>
#include <Wire.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "Adafruit_FT6206.h"
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
Adafruit_FT6206 ts = Adafruit_FT6206();

bool forceRepaint = false;
uint8_t currentAxis = 0;

volatile int rotary_position = 0;
static int last_rotary_position = 0;

unsigned long lastPolled = millis();
static unsigned long pollPeriod = 200;
static constexpr int activityTimeout = 5000;
static bool backlashCompensation = true;
static bool debugMode = false;

static constexpr const char *jogStepsMM[] = { "0.01", "0.1", };    
static constexpr const char *jogStepsInch[] = { "0.001", "0.01" };    
static constexpr unsigned feedLimitsMM[] = { 500, 1000 };  
static constexpr unsigned zfeedLimitsMM[] = { 300, 400 };  
static unsigned feedSpeedsMM[] = { 200, 1000 };
static unsigned zfeedSpeedsMM[] = { 200, 400 };
static constexpr unsigned feedSpeedsInch[] = { 8, 40 };
static constexpr unsigned zfeedSpeedsInch[] = { 8, 16 };

Stream &ugs = Serial;

bool switchActive = false;  // XY axes jog switches active
bool zswitchActive = false;  // Z axes jog switch active
uint8_t jogsActive = 0;   // Could probably be a boolean - we should not send another jog until we see the OK
// Some way to clear jogsActive in the event that something gets "lost" might be good - e.g. on receipt of an idle status
// or after a timeout
// It's really "commands active" - number of things we have sent and not yet seen ok for

void grblStateMachine();

class DStream
{
public:
  bool sendToGrbl = true;
  bool sendToUsb = true;
  template<class C> size_t print(C c) 
  {
    size_t ret = Serial2.print(c);
    if (sendToUsb)
       ret = Serial.print(c);
    if (sendToGrbl)
       ret = Serial1.print(c);
    return ret;
  }
  template<class C> size_t println(C c)
  {
    size_t ret = Serial2.println(c);
    if (sendToUsb)
       ret = Serial.println(c);
    if (sendToGrbl)
    {
       ret = Serial1.println(c);
       waitForOk();
    }
    return ret;
  }
  size_t write(int c) 
  {
    size_t ret = Serial2.write(c);
    if (sendToUsb)
       ret = Serial.write(c);
    if (sendToGrbl)
       ret = Serial1.write(c);
    return ret;
  }
  void waitForOk()
  {
    auto start = millis();
    jogsActive = 1;
    while (jogsActive && millis()-start < 1000)   // An OK can take a while to come back, in the case of a jog just after a cancel jog. The OK does not come back until the machine has stopped and is ready to restart.
    {
      if (available())
        grblStateMachine();
      else
        delay(1);
    }
    if (jogsActive)
    {
      if (debugMode)
        Serial.print("Lost OK");
      write(0x18);
    }
    else if (debugMode)
      Serial.println(String("OK after ") + (millis()-start) + "ms");
    jogsActive = 0;
  }
  int read() { return Serial1.read(); }
  int available() { return Serial1.available(); }
} grbl;

static void strcatc(char *dest, char c)
{
  auto len = strlen(dest);
  dest[len++] = c;
  dest[len] = '\0';
}

//-----------
/* encoder routine. Expects encoder with four state changes between detents */
/* This code will need changing for other boards and other pins */
/* This is for a Mega using pin-change interrupts on pk0/pk1 aka A8/A9 */
/* Based on ideas at https://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros/ */

// Use port K and use pin change interrupt using pk0/pk1, shift 0 places after read
#define ROTA  A8 
#define ROTB  A9
#define ROTSW A10
#define ENC_RD  (PINK & 0x03)             //encoder port read - reads whole port for speed
void setEncoderPCI()
{
  PCMSK2 |= (( 1 << PCINT16 ) | ( 1 << PCINT17 ) | ( 1 << PCINT18 )); //enable encoder pins interrupt sources
  PCICR |= ( 1 << PCIE2 );
}
#define ENCODER_ISR PCINT2_vect

const int xupPin = A15;
const int xdownPin = A13;
const int yupPin = A14;
const int ydownPin = A12;
const int zupPin = A11;
const int zdownPin = A7;

void setupEncoder()
{
  pinMode(ROTA, INPUT_PULLUP);
  pinMode(ROTB, INPUT_PULLUP);
  pinMode(ROTSW, INPUT_PULLUP);
  setEncoderPCI();
}

ISR(ENCODER_ISR)
{
  static int8_t stepsSeen = 0;
  static uint8_t old_AB = 3; //lookup table index and initial state
  uint8_t encport;           //encoder port copy
  static const int8_t enc_states[] PROGMEM =
  { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 }; //encoder lookup table
  old_AB <<= 2;     //remember previous state
  encport = ENC_RD;  //read encoder (direct port read for speed and atomicity)
  old_AB |= encport;   //create index
  stepsSeen += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));
  if (stepsSeen > 3)
  {
    // four steps forward
    rotary_position += 1;
    stepsSeen = 0;
  }
  else if (stepsSeen < -3)
  {
    //four steps backwards
    rotary_position -= 1;
    stepsSeen = 0;
  }
}

void enterDROMode();
void processLine(char *line);

// State machines used by loop() to decide how to process incoming chars

enum UGSstate { ugsSOL, ugsNormal, ugsDollar } ugsState = ugsSOL;    // State machine for chars seen from ugs end
unsigned long ugsCmdSeen = millis();                        // Last time we saw a command from ugs (not a jog, status request etc).

void ugsStateMachine()
{
  char c = (char) ugs.read();
  if (c=='\x18')
    digitalWrite(LED_BUILTIN, HIGH);

  grbl.write(c);
  if (c == '\n' || c == '\r')
    ugsState = ugsSOL;
  else
  {
    switch (ugsState)
    {
      case ugsDollar:
        // May nor may not be interesting to spot $ commands
        ugsState = ugsNormal;
        break;
      case ugsSOL:
        if (c == '$')
          ugsState = ugsDollar;
        else if (c != 0x18 && c != '?') // ctrl-x is soft reset, and should not be taken to mean that we are running a program
        {
          ugsState = ugsNormal;
          ugsCmdSeen = millis();
        }
        break;
      case ugsNormal:
        break;
    }
  }
}

enum { grblSOL, grblNormal, grblReadStatus,
       grblSeenO, grblSeenK,
       grblSeenDollar,
     } grblState = grblSOL;                                 // State machine for chars seen from grbl end

static constexpr uint8_t maxLineLength = 128;
char line[maxLineLength + 1];
uint8_t pos = 0;

bool readMM = true;     // MORE - read it from $13 somehow
bool showMM = true;     // MORE - persist it in EEPROM


void grblStateMachine()
{
  char c = (char) grbl.read();
  if (c == '\n' || c == '\r')
  {
    if (grblState == grblSeenK)
    {
      // Work our whether we want to echo this ok
      if (jogsActive)
      {
        jogsActive--;
        c = 0;
      }
      else
        ugs.print("ok");    // the eol char is still in c and will be echoed
    }
    else if (grblState==grblSeenDollar)
    {
      // NOTE - don't send to ugs if we initiated the $$ request (signaled by jogsActive=1)
      // If we are sniffing, we will just have to hope ugs doesn't mind
      line[pos] = 0;
      if (line[1]=='1' && line[2]=='3' && line[3]=='=')
        readMM = line[4]=='0';
      if (jogsActive)  // Note - don't mark inactive until we see the final 'ok'
        c = 0;
      else
        ugs.print(line);
    }
    grblState = grblSOL;
  }
  else if (c == '<')
  {
    grblState = grblReadStatus;
    pos = 0;
  }
  else
  {
    switch (grblState)
    {
      case grblSeenK: // Can't happen but shuts the compiler up
      case grblNormal:
        break;
      case grblReadStatus:
        if (c == '>')
        {
          ugs.print(c);
          line[pos] = 0;
          processLine(line);
          lastPolled = millis();
          grblState = grblNormal;
          c = 0;
        }
        else if (pos < maxLineLength)
          line[pos++] = c;
        break;
      case grblSeenDollar:
        line[pos++] = c;
        c = 0;
        break;
      case grblSOL:
        if (c == 'o')
        {
          c = 0;
          grblState = grblSeenO;
        }
        else if (c == '$')
        {
          pos = 0;
          line[pos++] = c;
          c = 0;
          grblState = grblSeenDollar;
        }
        else
          grblState = grblNormal;
        break;
      case grblSeenO:
        if (c == 'k')
        {
          c = 0;
          grblState = grblSeenK;
        }
        else
          grblState = grblNormal;
        break;
    }
  }
  if (c)
    ugs.write(c);
}

// Screen stuff

// Display is made up of "zones" each capable of displaying its contents. We take care to avoid 
// redrawing unchanged text for flicker-reduction reasons

const GFXfont *fonts[2] = { &FreeSans9pt7b, &FreeSans18pt7b };

class TextZone
{
  protected:
    bool largeFont = false;
    int x = 0;
    int y = 0;
    bool dirty = true;
  public:
    constexpr TextZone(bool _largeFont, int _x, int _y)
    : largeFont(_largeFont), x(_x), y(_y)
    {
    }
};

class InfoText : public TextZone
{
    static const int8_t maxLen = 12;
    char buff[maxLen+1] = "";
    mutable char lbuff[maxLen+1] = "";
    uint8_t prefixLen = 0;
  public:
    InfoText(const char *_text, bool _largeFont, int _x, int _y, const char *_prefix = nullptr) 
    : TextZone(_largeFont, _x, _y)
    {
      if (_prefix)
      {
        prefixLen = strlen(_prefix);
        strcpy(buff, _prefix);
      }
      if (_text)
      {
        strcat(buff, _text);
      }
    }
    void set(const char *text)
    {
      uint8_t i;
      for (i = prefixLen; i < maxLen; i++)
      {
        char c = *text++;
        if (!c)
          break;
        if (c != lbuff[i])
          dirty = true;
        buff[i] = c;
      }
      if (lbuff[i])
        dirty = true;
      buff[i] = '\0';
    }
    void print() const
    {
      if (!dirty && !forceRepaint)
        return;
      tft.setFont(fonts[largeFont]);
      const char *b = buff;
      char *l = lbuff;
      int16_t  oldx, oldy;
      uint16_t oldw, oldh;
      bool printed = false;
      tft.getTextBounds(lbuff, x, y+40, &oldx, &oldy, &oldw, &oldh);  // Whole of prior string
      for (;;)
      {
        char c = *b;
        if (!c)
        {
          *l = c;
          break;
        }
        if (!printed && (*l != c || forceRepaint))
        {
          *l = 0;
          int16_t  x1, y1;
          uint16_t w, h;
          tft.getTextBounds(lbuff, x, y+40, &x1, &y1, &w, &h); // initial matching part of prior string
          tft.fillRect(oldx+w, oldy, oldw-w, oldh, ILI9341_BLACK);  // clear tail of prior string
          tft.setCursor(x, y+40);  // MORE - We could print just the tail, but working out where to start is tricky, and not really necessary
          tft.print(buff);
          printed = true;;
        }
        *l = c;
        l++;
        b++;
      }
    }
};

class Button : public TextZone
{
  protected:
    bool selected = false;
    static constexpr uint8_t w = 50;
    static constexpr uint8_t h = 50;
    const char *text = nullptr;
    unsigned bgColor = ILI9341_BLACK;
    static constexpr unsigned fgColor = ILI9341_WHITE;
  public:
    constexpr Button(const char *_text, bool _largeFont, int _x, int _y, unsigned _bgColor = ILI9341_BLACK)
    : TextZone(_largeFont, _x, _y), text(_text), bgColor(_bgColor)
    {
    }
    Button & set(const char *_text)
    {
      text = _text;
      dirty = true;
      return *this;
    }
    Button &select(bool on)
    {
      return on ? select() : unselect();
    }
    Button &select()
    {
      if (!selected)
        dirty = true;
      selected = true;
      return *this;
    }
    Button &unselect()
    {
      if (dirty)
        selected = true;
      selected = false;
      return *this;
    }
    bool pressed(const TS_Point &p)
    {
      return (p.x >=x && p.x <= x+w && p.y >= y && p.y <= y+h);
    }
    void print() const
    {
      if (!dirty)
        return;
      tft.setFont(fonts[largeFont]);
      int16_t  tx, ty;
      uint16_t tw, th;
      tft.getTextBounds(text, x, y, &tx, &ty, &tw, &th);
      tft.fillRect(x, y, w, h, selected ? ILI9341_DARKGREEN : bgColor);
      tft.drawRoundRect(x, y, w, h, 3, fgColor);
      tft.setCursor(x+(w-tw)/2, y+h-(h-th)/2);
      tft.print(text);
    }
    const char *str() const
    {
      return text;
    }
};

// Fixed point numbers - I work in thousands of a XX where XX is mm or inches depending on $13 setting

class FixedPoint
{
  protected:
    long int val = 0;
    static constexpr uint8_t decDigits = 3;  // How many we store - may not display all. Additional digits are discarded (not rounded)
    static constexpr unsigned maxFrac = 1000;
  public:
    constexpr FixedPoint()
    {
    }
    constexpr FixedPoint(float v) : val(v*maxFrac)
    {
    }
    double gval()
    {
      return atof(queryStr());
    }
    FixedPoint &set(const FixedPoint &s)
    {
      val = s.val;
      return *this;
    }
    FixedPoint &set(const char *v)
    {
      val = 0;
      uint8_t decSeen = 0;
      bool stopSeen = false;
      while (isspace(*v))
        v++;
      bool negative = false;
      if (*v=='-')
      {
        negative = true;
        v++;
      }
      while (decSeen < decDigits)
      {
        if (isdigit(*v))
        {
          val = 10 * val + *v++ - '0';
          if (stopSeen)
            decSeen++;
        }
        else if (*v=='.' && !stopSeen)
        {
          v++;
          stopSeen = true;
        }
        else
        {
          while (decSeen++ < decDigits)
            val = 10 * val;
          break;
        }
      }
      if (negative)
        val = -val;
      return *this;
    }
    FixedPoint &half()
    {
      val /= 2;
      return *this;
    }
    FixedPoint &add(const FixedPoint &r)
    {
      val += r.val;
      return *this;
    }
    FixedPoint &sub(const FixedPoint &r)
    {
      val -= r.val;
      return *this;
    }
    bool gtequal(const FixedPoint &r) const
    {
      return val >= r.val;
    }
    bool gt(const FixedPoint &r) const
    {
      return val > r.val;
    }
    bool positive() const
    {
      return val >= 0;
    }
    const char *queryStr() const
    {
      char *ptr = buf;
      long int fval = val;
      if (showMM != readMM)
      {
        if (showMM)
          fval = (fval * 254) / 10;
        else
          fval = (fval * 10) / 254;
      }
      if (fval < 0)
      {
        fval = -fval;
        *ptr++='-';
      }
      ltoa(fval/1000, ptr, 10);
      ptr += strlen(ptr);
      *ptr++='.';
      fval = fval%1000;
      *ptr++='0'+(fval/100);
      fval = fval%100;
      *ptr++='0'+(fval/10);
      fval = fval%10;
      *ptr++='0'+fval;
      return buf;
    }
private:
    static char buf[15];
};

char FixedPoint::buf[15];

class FixedPointZone : public FixedPoint
{
  private:
    bool largeFont = false;
    int16_t x = 0;
    int16_t y = 0;
    uint8_t cellw = 18;
    uint8_t cellh = 40;
    mutable long int lval = 1;
    mutable unsigned lfrac = 1;
    mutable bool lmm = true;
    mutable char lbuff[10] = "";
    mutable unsigned lColor = ILI9341_WHITE;
    mutable unsigned lbgColor = ILI9341_BLACK;
    unsigned color = ILI9341_WHITE;
    unsigned bgColor = ILI9341_BLACK;
  public:
    constexpr FixedPointZone(bool _largeFont, int _cellw, int _cellh, int _x, int _y)
    : largeFont(_largeFont), x(_x), y(_y), cellw(_cellw), cellh(_cellh)
    {
    }
    void setColor(unsigned _color, unsigned _bgColor = ILI9341_BLACK)
    {
      color = _color;
      bgColor = _bgColor;
    }
    bool checkClick(const TS_Point &pt) const
    {
      return (pt.x >= x && pt.y >= y && pt.y <= y+cellh);      
    }
    void negate()
    {
      val = -val;
    }
    void print() const
    {
      bool force = forceRepaint || (lmm != showMM) || (bgColor != lbgColor) || (color != lColor);
      if (lval==val && !force)
        return;
      lval = val;      
      lmm = showMM;
      lbgColor = bgColor;
      lColor = color;
      constexpr uint8_t dp = 5;  // position of decimal point
      constexpr uint8_t width = dp+decDigits+1;
      char buff[width+1];
      buff[width]='\0';
      long int fval = val;
      if (showMM != readMM)
      {
        if (showMM)
          fval = (fval * 254) / 10;
        else
          fval = (fval * 10) / 254;
      }
      bool minus = false;
      if (fval < 0)
      {
        fval = -fval;
        minus = true;
      }
      for (int i = width-1; i >= 0; i--)
      {
        if (i==dp)
          buff[i] = '.';
        else if (fval)
        {
          buff[i] = '0' + (fval % 10);
          fval = fval / 10;
        }
        else if (i>=dp-1)
          buff[i] = '0';
        else if (minus)
        {
          buff[i] = '-';
          minus = false;
        }
        else
          buff[i] = ' ';
      }
      tft.setFont(fonts[largeFont]);
      const char *b = buff;
      char *l = lbuff;
      if (showMM)
        buff[width-1] ='\0';
      else
      {
        b++;
        l++;
      }
      int xx = x;
      for (;;)
      {
        char c = *b;
        if (!c)
        {
          *l = c;
          break;
        }
        else if (*l != c || force)
        {
          tft.fillRect(xx, y, cellw, cellh+2, bgColor);   // We use fixed size character cells for numeric output
          tft.drawChar(xx, y+cellh, c, color, bgColor, 1);
          *l = c;
        }
        xx += (c=='.')?cellw/3:cellw;   // NOTE - this might mess things up if decimal point changes position
        l++;
        b++;
      }
    }
};

const FixedPoint backLashOvershoot(2.0);
const FixedPoint backLashOvershootInches(0.1);

FixedPoint WCO[3];
FixedPoint endpointHigh[3] = { {1000.0}, {1000.0}, {1000.0}};
FixedPoint endpointLow[3] = { {-1000.0}, {-1000.0}, {-1000.0}};
FixedPointZone MPos[3] = {
  { false, 10, 20, 120, 90 },
  { false, 10, 20, 120, 148 },
  { false, 10, 20, 120, 206 }
};
FixedPointZone WPos[3] = {
  { true, 18, 28, 60, 60 },
  { true, 18, 28, 60, 118 },
  { true, 18, 28, 60, 176 }
};

void doMoveXY(FixedPoint &x, FixedPoint &y, bool rapid, bool antiBacklash)
{
  if (antiBacklash && (WPos[0].gt(x) || WPos[1].gt(y)))
  {
    FixedPoint x1 = x;
    if (WPos[0].gt(x))
      x1.sub(showMM ? backLashOvershoot : backLashOvershootInches);
    FixedPoint y1 = y;
    if (WPos[1].gt(y))
      y1.sub(showMM ? backLashOvershoot : backLashOvershootInches);
    doMoveXY(x1, y1, rapid, false);
  }
  grbl.print("$J=G90"); // jog absolute
  if (showMM)
    grbl.print("G21");  // mm
  else
    grbl.print("G20");  // inches
  grbl.print("X");
  grbl.print(x.queryStr());
  grbl.print("Y");
  grbl.print(y.queryStr());
  grbl.print("F");
  grbl.println(showMM ? feedSpeedsMM[rapid] : feedSpeedsInch[rapid]); // MORE - should make this rate configurable!
}

void gotoZero(bool axes[3], bool antiBacklash)
{
  bool xHigh = axes[0] && WPos[0].positive();
  bool yHigh = axes[1] && WPos[1].positive();
  bool zHigh = WPos[2].positive();
  if (axes[2] && !zHigh)
  {
    if (antiBacklash)
    {
      grbl.print("$J=G90G21Z1F");
      zHigh = true;
    }
    else
      grbl.print("$J=G90G21Z0F");
    grbl.println(zfeedSpeedsMM[true]); 
  }
  if (axes[0] || axes[1])  
  {
    if (antiBacklash && (xHigh || yHigh))
    {
      grbl.print("$J=G90G21"); // jog absolute
      if (xHigh) grbl.print("X-2");
      if (yHigh) grbl.print("Y-2");
      grbl.print("F");
      grbl.println(feedSpeedsMM[true]);
    }
    grbl.print("$J=G90G21"); // jog absolute
    if (axes[0]) grbl.print("X0");
    if (axes[1]) grbl.print("Y0");
    grbl.print("F");
    grbl.println(feedSpeedsMM[true]);
  }
  if (axes[2] && zHigh)
  {
    grbl.print("$J=G90G21Z0F"); // jog absolute
    grbl.println(zfeedSpeedsMM[true]); 
  }
}

void doMove(FixedPoint &to, int axis, bool rapid)
{
  // MORE - should we consider antibacklash here too?
  grbl.print("$J=G90"); // jog absolute
  if (showMM)
    grbl.print("G21");  // mm
  else
    grbl.print("G20");  // inches
  grbl.print("XYZ"[axis]);
  grbl.print(to.queryStr()); // NOTE - queryStr converts to showMM units 
  grbl.print("F");
  if (axis==2)
    grbl.println(showMM ? zfeedSpeedsMM[rapid] : zfeedSpeedsInch[rapid]); 
  else
    grbl.println(showMM ? feedSpeedsMM[rapid] : feedSpeedsInch[rapid]); // MORE - should make this rate configurable!
}

void zeroAxis(int currentAxis)
{
  grbl.print("G10 P0 L20 ");
  grbl.print("XYZ"[currentAxis]);
  grbl.println("0.0");
}

class Screen
{
protected:
  const uint8_t numButtons = 0;
  Button *buttons = nullptr;
  unsigned long touchSeen = 0;
  unsigned long touchStart = 0;
  unsigned long longTouchTime = 0;

  int8_t touchButton = 0;
  bool touching = false;
  static constexpr unsigned long touchDebounce = 10;

  constexpr Screen(unsigned _numButtons, Button *_buttons)
  : numButtons(_numButtons), buttons(_buttons)
  {
  }
  virtual int8_t findButton(const TS_Point &touchPoint)
  {
    for (unsigned i = 0; i < numButtons; i++)
    {
      if (buttons[i].pressed(touchPoint))
        return i;
    }
    return -1;
  }
  static TS_Point rotateTouch(const TS_Point &p)
  {
    auto y = p.x;
    auto x = tft.width()-p.y;
    return TS_Point(x, y, 0);
  }
  virtual void onPress(uint8_t pressed) = 0;
  virtual void onLongPress(uint8_t pressed) {}
  void start()
  {
    currentScreen = this;
    tft.fillScreen(ILI9341_BLACK);
    forceRepaint = true;
    for (uint8_t i = 0; i < numButtons; i++)
    {
      buttons[i].print();
    }
  }
public:
  virtual void screenStateMachine(unsigned long now)
  {
    TS_Point pt;  // Seems to be a codegen bug if it's block local - no proper stack frame created?
    if (ts.touched())
    {
      touchSeen = now;
      if (!touching)
      {
        touching = true;
        touchStart = now;
        pt = rotateTouch(ts.getPoint());
        touchButton = findButton(pt);
        if (touchButton >= 0)
          buttons[touchButton].select().print();
      }
    }
    else if (touching)
    { 
      if (now - touchSeen > touchDebounce)
      {
        touching = false;
        if (touchButton >= 0)
        {
          buttons[touchButton].unselect().print();
          if (longTouchTime && (now - touchStart >= longTouchTime))
            onLongPress(touchButton);
          else
            onPress(touchButton);
        }
      }
    }
  }
  static Screen *currentScreen;
};

Screen *Screen::currentScreen = nullptr;

class CalcScreen : public Screen
{
  // MORE - I think this code goes wrong if readMM is not equal to showMM
  
  enum CalcButtonNames { zeroButton, oneButton, twoButton, threeButton, fourButton, fiveButton, sixButton, 
                        sevenButton, eightButton, nineButton, plusMinusButton, dotButton,
                        cancelButton,
                        okButton };
  static constexpr unsigned numCalcButtons = 14;

  Button calcButtons[numCalcButtons] = {
    {"0", true, 65, 178},
    {"1", true, 10, 4},
    {"2", true, 65, 4},
    {"3", true, 120, 4},
    {"4", true, 10, 62},
    {"5", true, 65, 62},
    {"6", true, 120, 62},
    {"7", true, 10, 120},
    {"8", true, 65, 120},
    {"9", true, 120, 120},
    {"+/-", false, 10, 178},
    {".", true, 120, 178},
    {"X", true, 200, 178, ILI9341_RED},
    {"O", true, 265, 178, ILI9341_GREEN}
  };

  InfoText calcPrompt = {"", true, 200, 0};

  FixedPointZone calcVal = {true, 18, 28, 180, 70};
  FixedPointZone calcVal2 = {true, 18, 28, 180, 120};
  FixedPointZone *activeCalcVal = &calcVal;
  FixedPoint lastPolarR = 0;
  FixedPoint lastPolarTheta = 0;
  bool dualCalc = false;
  char calcBuf[10] = "";
  bool calcSeenDot = false;
  uint8_t calcBufPos = 1;

  void exitCalcMode(bool cancelled)
  {
    if (!cancelled)
    {
      switch (calcModeReason)
      {
        case calcModeLimitLow:
          endpointLow[currentAxis].set(calcVal);
          // MORE - some error checking eg low is less than high might be nice
          break;
        case calcModeLimitHigh:
          endpointHigh[currentAxis].set(calcVal);
          break;
        case calcModeMove:
          if (dualCalc)
          {
            calcVal.add(WPos[0]);
            calcVal2.add(WPos[1]);
          }
          else
            calcVal.add(WPos[currentAxis]);
          // Fall through
        case calcModeGoto:
          if (dualCalc)
            doMoveXY(calcVal, calcVal2, true, backlashCompensation);
          else
            doMove(calcVal, currentAxis, true);
          break;
        case calcModePolar:
        {
          lastPolarR.set(calcVal);
          lastPolarTheta.set(calcVal2);
          double theta = (calcVal2.gval() * PI) / 180.0;
          FixedPoint x = cos(theta) * calcVal.gval();
          FixedPoint y = sin(theta) * calcVal.gval();
          doMoveXY(x, y, true, backlashCompensation);
          break;
        }
        case calcModeNone:
          // Should never happen, only here to shut the compiler warning up
          break;
      }
    }
    enterDROMode();
  }

  void setActiveCalc(FixedPointZone &newActive)
  {
    activeCalcVal = &newActive;
    calcVal.setColor(ILI9341_WHITE);
    calcVal2.setColor(ILI9341_WHITE);
    activeCalcVal->setColor(ILI9341_YELLOW);
    calcBuf[0] = ' ';
    calcBuf[1] = 0;
    calcBufPos = 1;
    calcSeenDot = false;
    calcVal.print();
    if (dualCalc)
      calcVal2.print();
  }

  virtual int8_t findButton(const TS_Point &touchPoint)
  {
    int8_t pressed = Screen::findButton(touchPoint);
    if (pressed != -1)
      return pressed;
    if (calcVal.checkClick(touchPoint))
      setActiveCalc(calcVal);
    else if (calcVal2.checkClick(touchPoint) && dualCalc)
      setActiveCalc(calcVal2);
    return -1;
  }

public:
  CalcScreen() : Screen(numCalcButtons, calcButtons) 
  {
  };
  enum CalcModeReason { calcModeNone, calcModeMove, calcModeGoto, calcModeLimitHigh, calcModeLimitLow, calcModePolar };
  void enterCalcMode(CalcModeReason reason, const char *prompt, bool startHere, bool dual)
  {
    Screen::start();
    calcModeReason = reason;
    char usePrompt[10];
    strcpy(usePrompt, prompt);
    if (reason != calcModePolar)
    {
      strcat(usePrompt, " ");
      if (dual)
        strcat(usePrompt, "XY");
      else
        strcatc(usePrompt, "XYZ"[currentAxis]);
    }
    strcat(usePrompt, ":");
    dualCalc = dual;
    if (reason == calcModePolar)
    {
      calcVal.set(lastPolarR);
      calcVal2.set(lastPolarTheta);
      setActiveCalc(calcVal2);
    }
    else if (dualCalc)
    {
      if (startHere)
      {
        calcVal.set(WPos[0]);
        calcVal2.set(WPos[1]);
      }
      else
      {
        calcVal.set("0");
        calcVal2.set("0");
      }      
      setActiveCalc(currentAxis==0 ? calcVal : calcVal2);
    }
    else
    {
      calcVal.set("0");
      calcVal2.set("0");
      if (startHere)
      {
        calcVal.set(WPos[currentAxis]);
        calcVal.setColor(0xc5e0);
      }
      setActiveCalc(calcVal);
    }
    calcPrompt.set(usePrompt);
    calcPrompt.print();
    forceRepaint = false;
  }
  virtual void onPress(uint8_t pressed) override
  {
    switch (pressed)
    {
      case plusMinusButton:
        if (calcBuf[0]=='-')
          calcBuf[0] = ' ';
        else
          calcBuf[0] = '-';
        activeCalcVal->negate();
        activeCalcVal->print();
        return;
      case dotButton:
        if (calcSeenDot)
          return;
        calcSeenDot = true;
        // Fall through
      case oneButton:
      case twoButton:
      case threeButton:
      case fourButton:
      case fiveButton:
      case sixButton:
      case sevenButton:
      case eightButton:
      case nineButton:
      case zeroButton:
        if (calcBufPos < 9 && (calcSeenDot || calcBufPos < 5))
        {
          calcBuf[calcBufPos++] = *calcButtons[touchButton].str();
          calcBuf[calcBufPos] = 0;
        }
        break;
      case okButton:
        exitCalcMode(false);
        return;
      case cancelButton:
        exitCalcMode(true);
        return;
    }
    activeCalcVal->setColor(ILI9341_YELLOW, ILI9341_BLACK);
    activeCalcVal->set(calcBuf);
    activeCalcVal->print();
  }
private:
  CalcModeReason calcModeReason = calcModeNone;
} calcScreen;

class ZeroScreen : public Screen
{
  enum buttonNames { xButton, yButton, zButton, 
                     cancelButton,
                     okButton
                   };
  static constexpr unsigned numZeroButtons = 5;

  Button zeroButtons[numZeroButtons] = {
    {"X", true, 10, 62},
    {"Y", true, 10, 120},
    {"Z", true, 10, 178},
    {"X", true, 200, 178, ILI9341_RED},
    {"O", true, 265, 178, ILI9341_GREEN}
  };

  InfoText zeroPrompt = {"Goto 0:", true, 200, 0};

  void exitZeroMode(bool cancelled)
  {
    if (cancelled)
      memcpy(zero, saved, sizeof(zero));
    else
      gotoZero(zero, backlashCompensation);
    enterDROMode();
  }
  bool zero[3] = { true, true, false };
  bool saved[3] = { false, false, false };
public:
  ZeroScreen() : Screen(numZeroButtons, zeroButtons) 
  {
  };
  void enterZeroMode()
  {
    for (uint8_t i = 0; i<3; i++) 
      buttons[i].select(zero[i]);
    Screen::start();
    memcpy(saved, zero, sizeof(zero));
    zeroPrompt.print();
    forceRepaint = false;
  }
  virtual void onPress(uint8_t pressed) override
  {
    switch (pressed)
    {
      case xButton:
      case yButton:
      case zButton:
        zero[pressed] = !zero[pressed];
        buttons[pressed].select(zero[pressed]).print();
        break;
      case cancelButton:
        exitZeroMode(true);
        return;
      case okButton:
        exitZeroMode(false);
        return;
    }
  }
} zeroScreen;

class HalfScreen : public Screen
{
  enum buttonNames { xButton, yButton, 
                     cancelButton,
                     setButton,
                     gotoButton
                   };
  static constexpr unsigned numHalfButtons = 5;

  FixedPointZone halfX = {true, 18, 28, 60, 60};
  FixedPointZone halfY = {true, 18, 28, 60, 118};

  Button halfButtons[numHalfButtons] = {
    {"X", true, 10, 62},
    {"Y", true, 10, 120},
    {"X", true, 200, 178, ILI9341_RED},
    {"set", false, 265, 178, ILI9341_GREEN},
    {"goto", false, 265, 120, ILI9341_GREEN}
  };

  InfoText halfPrompt = {"Half:", true, 200, 0};

  void showValues()
  {
    halfX = WPos[0];
    halfY = WPos[1];
    if (half[0])
      halfX.half();
    if (half[1])
      halfY.half();
    halfX.print();
    halfY.print();
  }

  void exitHalfMode(uint8_t pressed)
  {
    if (pressed == cancelButton)
      memcpy(half, saved, sizeof(half));
    else if (pressed==setButton)
    {
      if (half[0])
      {
        grbl.print("G10 P0 L20 X");
        grbl.println(halfX.queryStr());
      }
      if (half[1])
      {
        grbl.print("G10 P0 L20 Y");
        grbl.println(halfY.queryStr());
      }
    }
    else // must be goto button
    {
      if (half[0]&&half[1])
        doMoveXY(halfX, halfY, true, backlashCompensation);
      else if (half[0])
        doMove(halfX, 0, true);
      else if (half[1])
        doMove(halfY, 1, true);
    }
    enterDROMode();
  }
  bool half[2] = { true, true };
  bool saved[2] = { false, false };
public:
  HalfScreen() : Screen(numHalfButtons, halfButtons) 
  {
  };
  void enterHalfMode()
  {
    for (uint8_t i = 0; i<2; i++) 
      buttons[i].select(half[i]);
    Screen::start();
    memcpy(saved, half, sizeof(half));
    showValues();
    forceRepaint = false;
  }
  virtual void onPress(uint8_t pressed) override
  {
    switch (pressed)
    {
      case xButton:
      case yButton:
        half[pressed] = !half[pressed];
        forceRepaint = true;
        showValues();
        buttons[pressed].select(half[pressed]).print();
        break;
      case setButton:
      case cancelButton:
      case gotoButton:
        exitHalfMode(pressed);
        return;
    }
  }
} halfScreen;

static const char **setupString = nullptr;
static const char *grblSettings [] = {
  "$0 = 10",      // (Step pulse time, microseconds)
  "$1 = 25",      // (Step idle delay, milliseconds)
  "$2 = 0",       // (Step pulse invert, mask)
  "$3 = 5",       // (Step direction invert, mask)
  "$4 = 0",       // (Invert step enable pin, boolean)
  "$5 = 0",       // (Invert limit pins, boolean)
  "$6 = 0",       // (Invert probe pin, boolean)
  "$10 = 1",      // (Status report options, mask)
  "$11 = 0.010",  // (Junction deviation, millimeters)
  "$12 = 0.002",  // (Arc tolerance, millimeters)
  "$13 = 0",      //    (Report in inches, boolean)
  "$20 = 0",      //    (Soft limits enable, boolean)
  "$21 = 0",      //    (Hard limits enable, boolean)
  "$22 = 0",      //    (Homing cycle enable, boolean)
  "$23 = 0",      //    (Homing direction invert, mask)
  "$24 = 25.0",   //    (Homing locate feed rate, mm/min)
  "$25 = 500.0",  //    (Homing search seek rate, mm/min)
  "$26 = 250",    //    (Homing switch debounce delay, milliseconds)
  "$27 = 1.000",  //    (Homing switch pull-off distance, millimeters)
  "$30 = 1000",   //   (Maximum spindle speed, RPM)
  "$31 = 0",      //    (Minimum spindle speed, RPM)
  "$32 = 0",      //    (Laser-mode enable, boolean)
  "$100 = 80.0",  //    (X-axis travel resolution, step/mm)
  "$101 = 80.0",  //    (Y-axis travel resolution, step/mm)
  "$102 = 160.0", //    (Z-axis travel resolution, step/mm)
  "$110 = 1000.0", //    (X-axis maximum rate, mm/min)
  "$111 = 1000.0", //    (Y-axis maximum rate, mm/min)
  "$112 = 1000.0", //    (Z-axis maximum rate, mm/min)
  "$120 = 40.0",  //    (X-axis acceleration, mm/sec^2)
  "$121 = 40.0",  //    (Y-axis acceleration, mm/sec^2)
  "$122 = 40.0",  //    (Z-axis acceleration, mm/sec^2)
  "$130 = 200.0", //    (X-axis maximum travel, millimeters) - probably wrong!
  "$131 = 200.0", //    (Y-axis maximum travel, millimeters). - probably wrong!
  "$132 = 200.0", //    (Z-axis maximum travel, millimeters). - probably wrong!
  nullptr
};

class OptScreen : public Screen
{
  enum OptButtonNames { optMmButton, optInchButton,
                        optLashOffButton, optLashOnButton,
                        optLimitHighButton, optLimitLowButton,
                        optDebugOffButton, optDebugOnButton,
                        optDollarButton,
                        optOkButton };
  static const unsigned numOptButtons = 10;
  Button optButtons[numOptButtons] = {
    { "mm", false, 10, 4},
    { "inch", false, 65, 4},
    { "off", false, 10, 62},
    { "on", false, 65, 62},
    { "high", false, 10, 120},
    { "low", false, 65, 120},
    { "off", false, 10, 178},
    { "on", false, 65, 178},
    { "$$", false, 265, 4, ILI9341_YELLOW},
    { "O", true, 265, 178, ILI9341_GREEN}
  };

  InfoText unitsPrompt = {"units", false, 130, 0};
  InfoText lashPrompt = {"antiBackLash", false, 130, 58};
  InfoText limitsPrompt = {"limits", false, 130, 58*2};
  InfoText debugPrompt = {"debug", false, 130, 58*3};
  virtual void onPress(uint8_t pressed) override
  {
    switch (pressed)
    {
      case optMmButton:
        showMM = true;
        optButtons[optMmButton].select().print();
        optButtons[optInchButton].unselect().print();
        break;
      case optInchButton:
        showMM = false;
        optButtons[optMmButton].unselect().print();
        optButtons[optInchButton].select().print();
        break;
      case optLashOffButton:
        backlashCompensation = false;
        optButtons[optLashOffButton].select().print();
        optButtons[optLashOnButton].unselect().print();
        break;
      case optLashOnButton:
        backlashCompensation = true;
        optButtons[optLashOffButton].unselect().print();
        optButtons[optLashOnButton].select().print();
        break;
      case optLimitHighButton:
        calcScreen.enterCalcMode(CalcScreen::calcModeLimitHigh, "Max", true, false);
        break;
      case optLimitLowButton:
        calcScreen.enterCalcMode(CalcScreen::calcModeLimitLow, "Min", true, false);
        break;
      case optDebugOffButton:
        debugMode = false;
        optButtons[optDebugOffButton].select().print();
        optButtons[optDebugOnButton].unselect().print();
        break;
      case optDebugOnButton:
        debugMode = true;
        optButtons[optDebugOnButton].select().print();
        optButtons[optDebugOffButton].unselect().print();
        break;
      case optDollarButton:
        setupString = grblSettings;
        break;
      case optOkButton:
        enterDROMode();
        break;
    }
  }
public:
  OptScreen() : Screen(numOptButtons, optButtons) 
  {
    optButtons[optMmButton].select(showMM);
    optButtons[optInchButton].select(!showMM);
    optButtons[optLashOffButton].select(!backlashCompensation);
    optButtons[optLashOnButton].select(backlashCompensation);
    optButtons[optDebugOffButton].select(!debugMode);
    optButtons[optDebugOnButton].select(debugMode);
  };
  void enterOptMode()
  {
    Screen::start();
    unitsPrompt.print();
    lashPrompt.print();
    limitsPrompt.print();
    debugPrompt.print();
    forceRepaint = false;
  }
} optScreen;

class DROScreen : public Screen
{
  bool zeroed = false;

  InfoText grblStatus = { "Unknown", true, 9, 0, "S:" };
  InfoText feedRate = { "", true, 200, 0, "F:" };
  InfoText scaleZone = { "Fine", false, 200, 18 };

  static constexpr unsigned numDROButtons = 9;

  enum DROButtonNames { xButton, yButton, zButton, halfButton, polarButton, moveButton, gotoButton, zeroButton, moreButton };
  Button DRObuttons[numDROButtons] = {
    {"X", true, 10, 62},
    {"Y", true, 10, 120},
    {"Z", true, 10, 178},
    {"half", false, 205, 62},
    {"polar", false, 265, 62},
    {"move", false, 205, 120},
    {"goto", false, 265, 120},
    {"zero", false, 205, 178},
    {"...", false, 265, 178}
  };

  unsigned long rotswSeen = 0;
  static constexpr unsigned long rotswDebounce = 10;
  bool rotswDown = false;

  class DirectionSwitch
  {
    int8_t lastState = 0;           // As read off the pins
    int8_t lastDebouncedState = 0;
    unsigned long lastPinChange = 0;
    uint8_t axis;
    uint8_t upPin;
    uint8_t downPin;
    static constexpr unsigned long switchDebounce = 10;
    static constexpr int jogDistance = 1;

  public:
    DirectionSwitch(uint8_t _axis, uint8_t up, uint8_t down) : axis(_axis), upPin(up), downPin(down)
    {
      pinMode(upPin, INPUT_PULLUP);
      pinMode(downPin, INPUT_PULLUP);
    }
    inline bool active() { return lastDebouncedState != 0; }

    String checkSwitch(bool rapid)
    {
      unsigned long now = millis();
      int8_t downNow = 0;
      if (digitalRead(downPin)==LOW)
        downNow = -1;
      else if (digitalRead(upPin)==LOW)
        downNow = 1;
      if (downNow != lastState)
      {
        lastPinChange = now;
        lastState = downNow;
      }
      if (now - lastPinChange > switchDebounce)
        lastDebouncedState = downNow;
      if (!lastDebouncedState)
        return "";
      // Check if we are already beyond endstop, and don't move if we are
      // Endstops are expressed in local coords which could be confusing. It means if you zero an axis you move the endpoints
      // If we wanted to change that we would need to convert the jog destination.
      FixedPoint *destPoint;
      if (lastDebouncedState<0)
      {
        destPoint = &endpointLow[axis];
        if (destPoint->gtequal(WPos[axis]))
          return "";
      }
      else // lastDebouncedState>0
      {
        destPoint = &endpointHigh[axis];
        if (WPos[axis].gtequal(*destPoint))
          return "";
      }
      return String("XYZ"[axis]) + destPoint->queryStr();
    }
};

  DirectionSwitch x_sw, y_sw, z_sw;
  static constexpr unsigned jogInterval = 500;
  unsigned long lastJogSent = 0;
  String lastJog;

  void encoderStateMachine(unsigned long now)
  {
    if (digitalRead(ROTSW)==LOW)
    {
      rotswSeen = now;
      rotswDown = true;
    }
    else if (rotswDown && (now - rotswSeen > rotswDebounce))
    {
      rotswDown = false;
      onRotSwPress();
    }
    // Check XY switch states if Z switches inactive
    if (!zswitchActive)
    {
      String switchState = x_sw.checkSwitch(rotswDown) + y_sw.checkSwitch(rotswDown);   // Z jogging separate because of differing feed rate
      if (switchState.length()==0)
      {
        if (switchActive)
        {
          // Stop any active jogs
          switchActive = 0;
          rotary_position = last_rotary_position;
          grbl.write(0x85);    // Cancel the move. 
          lastJog = "";
        }
      }
      else 
      {
        if (!switchActive || now - lastJogSent > jogInterval)
        {
          unsigned speed = feedSpeedsMM[rotswDown];
          if (x_sw.active() && y_sw.active())
            speed = speed * 1.414;
          String thisJog = "$J=G90G21" + switchState + "F" + speed;
          if (!thisJog.equals(lastJog))
          {
            grbl.write(0x85);    // Cancel the previous command. 
            grbl.println(thisJog);
            lastJog = thisJog;
          }
          lastJogSent = now ;//- (switchActive ? 0 : jogInterval);
          switchActive = true;
        }
      }
    }
    // Check Z switch state if XY switches inactive
    if (!switchActive)
    {
      String zswitchState = z_sw.checkSwitch(rotswDown);      
      if (zswitchState.length()==0)
      {
        if (zswitchActive)
        {
          // Stop any active jogs
          zswitchActive = false;
          rotary_position = last_rotary_position;
          grbl.write(0x85);    // Cancel the move. 
          lastJog = "";
        }
      }
      else 
      { 
        if (!zswitchActive || now - lastJogSent > jogInterval-10)  // -10 is a hack to try to keep speed up without overrun
        {
          String thisJog = "$J=G90G21" + zswitchState + "F" + zfeedSpeedsMM[rotswDown];
          if (!thisJog.equals(lastJog))
          {
            grbl.write(0x85);    // Cancel the previous command. 
            grbl.println(thisJog);
            lastJog = thisJog;
          }
          lastJogSent = now ;//- (switchActive ? 0 : jogInterval);
          zswitchActive = true;
        }
      }
    }
  }

public:
  DROScreen() : Screen(numDROButtons, DRObuttons), x_sw(0, xupPin, xdownPin), y_sw(1, yupPin, ydownPin), z_sw(2, zupPin, zdownPin)
  {
    longTouchTime = 500;
  }

  void enterDROMode()
  {
    buttons[currentAxis].select();
    Screen::start();
    for (uint8_t i = 0; i < 3; i++)
    {
      MPos[i].print();
      WPos[i].print();
    }
    grblStatus.print();
    scaleZone.print();
    forceRepaint = false;
    last_rotary_position = rotary_position;  // Discard anything that happened in calc mode
  }

  virtual void onLongPress(uint8_t pressed) override
  {
    switch (pressed)
    {
      case xButton:
      case yButton:
      case zButton:
        if (pressed!=currentAxis)
          onPress(pressed);
        zeroAxis(currentAxis);
        zeroed = true;
    }
  }
  virtual void onPress(uint8_t pressed) override
  {
    switch (pressed)
    {
      case xButton:
      case yButton:
      case zButton:
        buttons[currentAxis].unselect().print();
        buttons[pressed].select().print();
        currentAxis = pressed;
        zeroed = false;
        return;
      case polarButton:
        calcScreen.enterCalcMode(CalcScreen::calcModePolar, "Polar", false, true);
        break;
      case moveButton:
        calcScreen.enterCalcMode(CalcScreen::calcModeMove, "Jog", false, currentAxis < 2);
        break;
      case gotoButton:
        calcScreen.enterCalcMode(CalcScreen::calcModeGoto, "Go", true, currentAxis < 2);
        break;
      case zeroButton:
        zeroScreen.enterZeroMode();
        break;
      case halfButton:
        halfScreen.enterHalfMode();
        break;
      case moreButton:
        optScreen.enterOptMode();
        break;
    }
    buttons[pressed].unselect();
  }

  void adjustFeedSpeed(bool z, bool up)
  {
    unsigned &speed = (z ? zfeedSpeedsMM : feedSpeedsMM)[false];
    unsigned limit = (z ? zfeedLimitsMM : feedLimitsMM)[false];
    if (up)
    {
      if (speed + 10 <= limit)
        speed += 10;
    }
    else
    {
      if (speed > 10)
        speed -= 10;
    }
  }

  virtual void screenStateMachine(unsigned long now) override
  {
    Screen::screenStateMachine(now);
    encoderStateMachine(now);

    if (grblState == grblSOL && ugsState == ugsSOL)
    {
      // if not polled for a while, poll.
      if (pollPeriod && now - lastPolled > pollPeriod)
      {
        lastPolled = now;
        grbl.print('?');
      }
      // if we seem to be idle, enable jogging.
      // Probably need to rethink the "idle" bit - desire is to avoid jogging while running a program
      // but also to avoid clashing on serial line when in "sniff" mode.
      //if (now - ugsCmdSeen > activityTimeout)
      int jogDistance = rotary_position - last_rotary_position;
      if (jogDistance && !jogsActive)
      {
        last_rotary_position = rotary_position;  // note - we only use direction not distance. We could change that if we wanted.
        if (switchActive)
        {
          if (!rotswDown)
            adjustFeedSpeed(false, jogDistance>0);
        }
        else if (zswitchActive)
        {
          if (!rotswDown)
            adjustFeedSpeed(true, jogDistance>0);
        }
        else
        {
          grbl.print("$J=G91"); // jog relative
          if (showMM)
            grbl.print("G21");  // mm
          else
            grbl.print("G20");  // inches
          grbl.print("XYZ"[currentAxis]);
          if (jogDistance<0)
            grbl.print('-');
          grbl.print((showMM ? jogStepsMM : jogStepsInch)[(rotswDown ? 0 : 1)]);
          grbl.println(showMM ? "F1000" : "F40"); // MORE - could make this rate configurable. Affects speed if you spin the wheel fast enough to give it a way to go
        }
      }
    }
  }
  
  void processLine(char *line)
  {
    if (debugMode)
    {
      ugs.print("R:"); ugs.println(line);
    }
    char *r;
    char *f;
    f = strtok_r(line, "|>", &r);
    grblStatus.set(f);
    if (currentScreen==this)
      grblStatus.print();
    char coordType = 'M';
    while (f)
    {
      f = strtok_r(nullptr, "|>", &r);
      if (f)
      {
        char *wr;
        if (f[0] == 'W' && f[1] == 'C' && f[2] == 'O' && f[3] == ':')
        {
          WCO[0].set(strtok_r(f + 4, ",|>", &wr));
          WCO[1].set(strtok_r(nullptr, ",|>", &wr));
          WCO[2].set(strtok_r(nullptr, ",|>", &wr));
        }
        else if ((f[0] == 'M' || f[0] == 'W') && f[1] == 'P' && f[2] == 'o' && f[3] == 's' && f[4] == ':')
        {
          coordType = f[0];
          FixedPointZone *dest;
          if (coordType == 'W')
            dest = WPos;
          else
            dest = MPos;
          dest[0].set(strtok_r(f + 5, ",|>", &wr));
          dest[1].set(strtok_r(nullptr, ",|>", &wr));
          dest[2].set(strtok_r(nullptr, ",|>", &wr));
        }
        else if (f[0] == 'F' && f[1] == ':')
        {
          // Feed rate
          feedRate.set(strtok_r(f + 2, ",|>", &wr));
          if (currentScreen==this)
            feedRate.print();
        }
        else if (f[0] == 'F' && f[1] == 'S' && f[2] == ':')
        {
          // Feed rate and spindle
          feedRate.set(strtok_r(f + 3, ",|>", &wr));
          if (currentScreen==this)
            feedRate.print();
        }
        // Probably don't really care about anything else
      }
    }
    for (uint8_t i = 0; i < 3; i++)
    {
      if (coordType == 'W')
      {
        MPos[i].set(WPos[i]).add(WCO[i]);
      }
      else
      {
        WPos[i].set(MPos[i]).sub(WCO[i]);
      }
      if (currentScreen==this)
      {
        MPos[i].print();
        WPos[i].print();
      }
    }
  }

  void onRotSwPress()
  {
    // Nothing - it's used as an override while pressed
  }
} droScreen;

void enterDROMode()
{
  droScreen.enterDROMode();
}

void processLine(char *line)
{
  droScreen.processLine(line);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(zupPin, INPUT_PULLUP);
  pinMode(zdownPin, INPUT_PULLUP);
  delay(100);
  if (digitalRead(zupPin)==LOW)
  {
    // Debug mode
    grbl.sendToGrbl = false;
    grbl.sendToUsb = true;
    pollPeriod = 0;
    debugMode = true;
  }
  else if (digitalRead(zdownPin)==LOW)
  {
    // Observe mode
    grbl.sendToGrbl = false;
    grbl.sendToUsb = false;
    pollPeriod = 0;   // Assumes the PC is observing
  }
  setupEncoder();
  Serial.begin(115200);
  Serial1.begin(115200);
  // Note:  ugs sends a soft reset immediately after opening the serial port. But opening the serial port resets the arduino
  // and this means the reset is not actually seen.
  // In "proxy" mode this is a problem, solvable in various ways:
  // 1. Modify ugs to sleep before sending the soft reset (which is probably correct)
  // 2. Modify the pendant hardware to not reset on USB connect
  // 3. Send a soft reset on startup to make up for the one that we know ugs sent while we were still restarting
  //    but this is not correct for non-proxy mode and may cause some confusion at ugs end
  // I have gone for option 1 but if you want to go for option 3, uncomment the line below
  // Serial1.write('\x18');
  ts.begin();
  tft.begin();
  tft.setRotation(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  enterDROMode();
}

void loop()
{
  if (ugs.available())
    ugsStateMachine();
  else if (grbl.available())
  {
    grblStateMachine();
  }
  else
  {
    if (setupString && *setupString)
    {
      grbl.println(*setupString);
      setupString++;
      if (!*setupString)
        setupString = nullptr;
    }
    else
      Screen::currentScreen->screenStateMachine(millis());
  }
}
