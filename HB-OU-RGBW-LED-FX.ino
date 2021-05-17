//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2019-08-26 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=644p aes=no

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER
#define HIDE_IGNORE_MSG
#define EI_NOTEXTERNAL

#ifndef ARDUINO_ARCH_ESP32
  #include <EnableInterrupt.h>
  #include <LowPower.h>
#endif

#include <SPI.h>
#include <AskSinPP.h>
#include <Register.h>
#include <MultiChannelDevice.h>
#include <Switch.h>
#include <WS2812FX.h>

#ifdef __AVR_ATmega328P__
  #define CONFIG_BUTTON_PIN   8
  #define WSLED_PIN           9
  #define WSLED_ACTIVATE_PIN  6
  #define ONBOARD_LED_PIN1    4
  #define CC1101_CS          10
  #define CC1101_GDO0         2
#endif

#ifdef __AVR_ATmega644P__        //Pin Definitionen (when using 644P(A): use Standard Pinout)
  #define CONFIG_BUTTON_PIN   1  //PB1
  #define WSLED_PIN          18  //PC2
  #define WSLED_ACTIVATE_PIN 20  //PC4
  #define ONBOARD_LED_PIN1    0  //PB0
  #define CC1101_CS           4  //PB4
  #define CC1101_GDO0        10  //PD2
#endif

#ifdef ARDUINO_ARCH_ESP32
  #define CONFIG_BUTTON_PIN   16
  #define WSLED_PIN           21
  #define WSLED_ACTIVATE_PIN  22
  #define ONBOARD_LED_PIN1    2
  #define CC1101_CS           5
  #define CC1101_GDO0         4
#endif


WS2812FX ws2812fx(30, WSLED_PIN, NEO_GRBW + NEO_KHZ800);

#define NUM_CHANNELS            3

#define PEERS_PER_LED_CHANNEL  10

using namespace as;

const struct DeviceInfo PROGMEM devinfo = {
  {0xF3, 0x52, 0x01},     // Device ID
  "JPLEDFX001",           // Device Serial
  {0xF3, 0x52},           // Device Model
  0x10,                   // Firmware Version
  as::DeviceType::Dimmer, // Device Type
  {0x01, 0x01}            // Info Bytes
};

typedef AskSin<StatusLed<ONBOARD_LED_PIN1>, NoBattery, Radio<LibSPI<CC1101_CS>, CC1101_GDO0>> Hal;
Hal hal;

DEFREGISTER(OUReg0, MASTERID_REGS, 0x01)
class OUList0 : public RegList0<OUReg0> {
  public:
    OUList0(uint16_t addr) : RegList0<OUReg0>(addr) {}
    void defaults () {
      clear();
    }
};

DEFREGISTER(LEDReg1, CREG_AES_ACTIVE, 0x01, 0x02)
class LEDList1 : public RegList1<LEDReg1> {
  public:
    LEDList1(uint16_t addr) : RegList1<LEDReg1>(addr) {}

    bool ledCount (uint16_t value) const { return this->writeRegister(0x01, (value >> 8) & 0xff) && this->writeRegister(0x02, value & 0xff); }
    uint16_t ledCount () const { return (this->readRegister(0x01, 0) << 8) + this->readRegister(0x02, 0); }

    void defaults () {
      clear();
      ledCount(60);
    }
};

DEFREGISTER(OUReg3, SWITCH_LIST3_STANDARD_REGISTER, PREG_ACTTYPE, PREG_ACTNUM, PREG_ACTCOLOR_R, PREG_ACTCOLOR_G, PREG_ACTCOLOR_B, PREG_ACTCOLOR_W, PREG_ACTINTENS);
typedef RegList3<OUReg3> SwPeerListEx;
class OUList3 : public SwitchList3Tmpl<SwPeerListEx> {
  public:
    OUList3(uint16_t addr) : SwitchList3Tmpl<SwPeerListEx>(addr) {}

    void defaults() {
      SwitchList3Tmpl<SwPeerListEx>::defaults();
    }
    void even () {
      SwitchList3Tmpl<SwPeerListEx>::even();
    }
    void odd () {
      SwitchList3Tmpl<SwPeerListEx>::odd();
    }
    void single () {
      SwitchList3Tmpl<SwPeerListEx>::single();
    }
};

uint8_t  channel2segnum[NUM_CHANNELS];
uint8_t  segmentStart[NUM_CHANNELS];
uint8_t  segmentEnd[NUM_CHANNELS];
uint8_t  segmentCount = 0;

void __attribute__((weak)) calculateLedCount();

uint16_t triwave16(uint16_t in) {
  if ((uint16_t)in < 0x8000) return in *2UL;
  return 0xFFFF - ((uint16_t)in - 0x8000)*2UL;
}


uint16_t spots_base(void) {
  uint16_t threshold = (255 - (ws2812fx.getSpeed() / 255)) << 8;
  ws2812fx.fill(0);
  uint16_t maxZones = ws2812fx.getLength() >> 2;
  uint16_t zones = 1UL + (((uint16_t)ws2812fx.getBrightness() * maxZones) >> 8);
  uint16_t zoneLen = ws2812fx.getLength() / zones;
  uint16_t offset = (ws2812fx.getLength() - zones * zoneLen) >> 1;

  for (uint16_t z = 0; z < zones; z++)
  {
    uint16_t pos = offset + z * zoneLen;
    for (uint16_t i = 0; i < zoneLen; i++)
    {
      uint16_t wave = triwave16(((uint32_t)i * 0xFFFF) / (uint32_t)zoneLen);
      if (wave > threshold) {
        uint16_t index = 0 + pos + i;
        uint8_t s = ((uint16_t)wave - (uint16_t)threshold)*255UL / (0xFFFF - (uint16_t)threshold);
        ws2812fx.setPixelColor(index, ws2812fx.color_blend(ws2812fx.getColor(),0x00000000,255-s));
      }
    }
  }

  return ws2812fx.getSpeed() / 255;
}

void setSegment(uint8_t ch, uint8_t brightness, uint8_t speed, uint8_t fx, uint32_t color) {

    uint8_t segnum = channel2segnum[ch - 1];

    uint16_t start = segmentStart[segnum];
    uint16_t end   = segmentEnd[segnum];

    //there is just one brightness value for the whole stripe
    DPRINT("setSegment setBrightness: ");DDECLN(brightness);
    ws2812fx.setBrightness(brightness);

    //WS2812FX speed has a range from 0 ... 65535 (16bit), but we will only get 8 bit for the speed value
    uint16_t s = (uint16_t)speed * 255;

    if (fx > 0) fx = fx - 1;

    //FX_MODE_CUSTOM_0 are the spots over the whole stripe length
    //all single segments must be removed
    if (fx == FX_MODE_CUSTOM_0) ws2812fx.resetSegments();

    DPRINT("ws2812fx.setSegment(");DDEC(segnum);DPRINT(", ");DDEC(start);DPRINT(", ");DDEC(end);DPRINT(", ");DDEC(fx);DPRINT(", ");DHEX(color);DPRINT(", ");DDEC(s);DPRINTLN(", NO_OPTIONS)");
    ws2812fx.removeActiveSegment(segnum);
    ws2812fx.setSegment(segnum, start, end, fx, color, s, NO_OPTIONS);
    ws2812fx.addActiveSegment(segnum);
}

class LEDChannel : public ActorChannel<Hal, LEDList1, OUList3, PEERS_PER_LED_CHANNEL, OUList0, SwitchStateMachine>  {
  public:
  class LEDOffDelayAlarm : public Alarm {
      LEDChannel& chan;
    public:
      LEDOffDelayAlarm (LEDChannel& c) : Alarm(0), chan(c) {}
      virtual ~LEDOffDelayAlarm () {}

      void trigger (__attribute__ ((unused)) AlarmClock& clock)  {
        chan.ledOff(true);
      }
  };

  private:
    bool first, boot, on;
    uint8_t  r,g ,b,w;
    uint8_t  brightness;
    uint8_t  speed;
    uint8_t  fx;
  protected:
    LEDOffDelayAlarm ledOffDelayAlarm;
    typedef ActorChannel<Hal, LEDList1, OUList3, PEERS_PER_LED_CHANNEL, OUList0, SwitchStateMachine> BaseChannel;
  public:
    LEDChannel () : BaseChannel(), first(true), boot(true), on(false), r(0), g(0), b(0), w(0), brightness(0), speed(0), fx(0), ledOffDelayAlarm(*this) {}
    virtual ~LEDChannel() {}

    void setLedOffDelay(uint16_t dly) {
      DPRINT("set off delay ");DDECLN(dly);
      sysclock.cancel(ledOffDelayAlarm);
      ledOffDelayAlarm.set(dly);
      sysclock.add(ledOffDelayAlarm);
    }

    void ledOn(bool setCh) {
      on = true;
      uint32_t c = (((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | ((uint32_t)b));
      setSegment(number(), brightness, speed, fx, c);
      if (setCh) BaseChannel::set( 0xc8, 0x00, 0xffff );
    }

    void ledOff(bool setCh) {
      on = false;
      sysclock.cancel(ledOffDelayAlarm);
      setSegment(number(), brightness,0,1,(uint32_t)0);
      if (setCh) BaseChannel::set( 0x00, 0x00, 0xffff );
    }

    bool isOn() {
      return on;
    }

    bool process (const ActionSetMsg& msg) {
      BaseChannel::set( msg.value(), msg.ramp(), msg.delay() );
      return true;
    }

    bool process (const ActionCommandMsg& msg) {
      static uint8_t lastmsgcnt = 0;
      if (msg.count() != lastmsgcnt) {
        lastmsgcnt = msg.count();
 
        //1 = Helligkeit
        //2 = Speed
        //3 = FX Nr
        //4 - 7 = R/G/B/W
 
        brightness  = msg.value(1);
        speed       = msg.value(2);
        fx          = msg.value(3);
        r           = msg.value(4);
        g           = msg.value(5);
        b           = msg.value(6);
        w           = msg.value(7);
 
        if (fx == 0) {
          ledOff(true);
        } else {
          uint16_t t = ((msg.value(msg.len() - 2)) << 8) + (msg.value(msg.len() - 1));
          if (t > 0 && t != 0x83CA) {
            setLedOffDelay(AskSinBase::intTimeCvt(t));
          }
          ledOn(true);
        }
      }
      return true;
    }

    uint8_t getConditionForStatePl(uint8_t stat,const SwPeerListEx& lst) const {
      switch( stat ) {
        case AS_CM_JT_ONDELAY:  return lst.ctDlyOn();
        case AS_CM_JT_ON:       return lst.ctOn();
        case AS_CM_JT_OFFDELAY: return lst.ctDlyOff();
        case AS_CM_JT_OFF:      return lst.ctOff();
      }
      return AS_CM_CT_X_GE_COND_VALUE_LO;
    }

    void runPl (const SwPeerListEx& pl,uint8_t cnt) {
      if (cnt != lastmsgcnt) {
       lastmsgcnt = cnt;
       DPRINT("runPl ");DDECLN(number());
 
       /*
       DPRINT(F("ACT_NUM     ")); DDECLN(pl.actNum());   // Speed
       DPRINT(F("ACT_INTENS  ")); DDECLN(pl.actIntens());// Helligkeit
       DPRINT(F("ACT_TYPE    ")); DDECLN(pl.actType());  // FX
       DPRINT(F("ACT_COLOR_R ")); DDECLN(pl.actColorR());// R
       DPRINT(F("ACT_COLOR_G ")); DDECLN(pl.actColorG());// G
       DPRINT(F("ACT_COLOR_B ")); DDECLN(pl.actColorB());// B
       DPRINT(F("ACT_COLOR_W ")); DDECLN(pl.actColorW());// W
       DPRINT(F("OFFDELAY   "));  DDECLN(pl.offDly());   // Ausschaltverzögerung
       */

       brightness  = pl.actIntens();
       speed       = pl.actNum();
       fx          = pl.actType();
       r           = pl.actColorR();
       g           = pl.actColorG();
       b           = pl.actColorB();
       w           = pl.actColorW();
 
       if (pl.actType() == 0) {
         ledOff(true);
       } else {
         if (pl.offDly() > 0)
           setLedOffDelay(AskSinBase::byteTimeCvt(pl.offDly()));
         ledOn(true);
       }
      }
    }

    void sensorPl (const SwPeerListEx& lst,uint8_t counter,uint8_t value) {
      uint8_t cond = getConditionForStatePl(state,lst);
      bool doit = false;
      switch( cond ) {
      case AS_CM_CT_X_GE_COND_VALUE_LO:
        doit = (value >= lst.ctValLo());
        break;
      case AS_CM_CT_X_GE_COND_VALUE_HI:
        doit = (value >= lst.ctValHi());
        break;
      case AS_CM_CT_X_LT_COND_VALUE_LO:
        doit = (value < lst.ctValLo());
        break;
      case AS_CM_CT_X_LT_COND_VALUE_HI:
        doit = (value < lst.ctValHi());
        break;
      case AS_CM_CT_COND_VALUE_LO_LE_X_LT_COND_VALUE_HI:
        doit = ((lst.ctValLo() <= value) && (value < lst.ctValHi()));
        break;
      case AS_CM_CT_X_LT_COND_VALUE_LO_OR_X_GE_COND_VALUE_HI:
        doit =((value < lst.ctValLo()) || (value >= lst.ctValHi()));
        break;
      }
      if( doit == true ) {
        runPl(lst,counter);
      }
    }

    bool process (const SensorEventMsg& msg) {
      bool lg = msg.isLong();
      Peer p(msg.peer());
      uint8_t cnt = msg.counter();
      uint8_t value = msg.value();
      OUList3 l3 = BaseChannel::getList3(p);
      if( l3.valid() == true ) {
        typename OUList3::PeerList pl = lg ? l3.lg() : l3.sh();
        if (lg == false || pl.multiExec() == false ) {
          sensorPl(pl,cnt,value);
        }
        return true;
      }
      return false;
    }

    bool process (const RemoteEventMsg& msg) {
      bool lg = msg.isLong();
      Peer p(msg.peer());
      uint8_t cnt = msg.counter();
      OUList3 l3 = BaseChannel::getList3(p);
      if ( l3.valid() == true ) {
        typename OUList3::PeerList pl = lg ? l3.lg() : l3.sh();
        if (lg == false || pl.multiExec() == false ) {
          runPl(pl, cnt);
        }
        return true;
      }
      return false;
    }

    void init() {
      ledOff(true);
      //this->changed(true);
      first = false;
    }

    uint8_t flags () const {
      return 0;
    }

    void configChanged() {
      uint16_t cnt = this->getList1().ledCount();
      DPRINT("ledCount for CH#");DDEC(number());DPRINT(": ");DDECLN(cnt);
      if (boot == false) calculateLedCount();
      boot = false;
    }

    virtual void switchState(__attribute__((unused)) uint8_t oldstate, __attribute__((unused)) uint8_t newstate, __attribute__((unused)) uint32_t delay) {
      if ( newstate == AS_CM_JT_OFF ) {
        if (first == false ) {
          this->ledOff(false);
        }
      }
      if ( newstate == AS_CM_JT_ON ) {
        this->ledOn(false);
      }
      this->changed(true);
    }
};

class OUDevice : public ChannelDevice<Hal, VirtBaseChannel<Hal, OUList0>, NUM_CHANNELS, OUList0> {
  public:
    VirtChannel<Hal, LEDChannel, OUList0> ledc[6];
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, OUList0>, NUM_CHANNELS, OUList0> DeviceType;
    OUDevice (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr) {
      for (uint8_t i = 0; i < NUM_CHANNELS; i++) DeviceType::registerChannel(ledc[i], i+1);
    }
    virtual ~OUDevice () {}

    LEDChannel& ledChannel(uint8_t num)  {
      return ledc[num];
    }

    virtual void configChanged () {}
};

OUDevice sdev(devinfo, 0x20);
ConfigButton<OUDevice> cfgBtn(sdev);

void dumpLedStripe() {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    DPRINT("Channel ");DDEC(i+1); DPRINT(" belongs to segment ");DDECLN(channel2segnum[i]);
  }

  for (uint8_t i = 0; i < segmentCount; i++) {
    if (channel2segnum[i] == 0xFF) break;
    DPRINT("Segment ");DDEC(i);DPRINT(" goes from LED ");DDEC(segmentStart[i]);DPRINT(" to ");DDECLN(segmentEnd[i]);
  }
  DPRINT("Total LED Count = ");DDECLN(ws2812fx.getLength());
}

void calculateLedCount() {
  uint8_t  segmentNumber = 0;
  uint16_t totalLeds     = 0;

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {

    uint16_t ledCount     = sdev.ledChannel(i).getList1().ledCount();
    uint16_t nextLedCount = (i < NUM_CHANNELS - 1) ? sdev.ledChannel(i + 1).getList1().ledCount() : 1024;

    channel2segnum[i] = segmentNumber;

    if (ledCount < 1024) {
      segmentStart[segmentNumber] = (segmentNumber == 0) ? 0 :  segmentEnd[segmentNumber - 1];
      segmentEnd  [segmentNumber] = segmentStart[segmentNumber] + ledCount - 1;
      totalLeds += ledCount;
      if (nextLedCount < 1024) segmentNumber++;
    }
  }
  segmentCount = segmentNumber;
  ws2812fx.updateLength(totalLeds);
  dumpLedStripe();
}

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);

  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);

  ws2812fx.init();
  ws2812fx.start();

  calculateLedCount();

  for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    sdev.ledChannel(i).init();

  sdev.initDone();

  pinMode(WSLED_ACTIVATE_PIN, OUTPUT);

  ws2812fx.setBrightness(128);
  ws2812fx.setCustomMode(0, spots_base);
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();

  bool channelActive = false;
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    if (sdev.ledChannel(i).isOn() == true) {
      channelActive = true;
      break;
    }
  }

  if (channelActive == true) {
    digitalWrite(WSLED_ACTIVATE_PIN, HIGH);
  } else {
    digitalWrite(WSLED_ACTIVATE_PIN, LOW);
    if (worked == false && poll == false ) {
#ifndef ARDUINO_ARCH_ESP32
      hal.activity.savePower<Idle<>>(hal);
#endif
    }
  }

  ws2812fx.service();
}
