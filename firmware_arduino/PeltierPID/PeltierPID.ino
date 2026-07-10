// ============================================================
//  PID Peltier Controller v13 — Relay Autotuning + Feed-Forward
//  Adafruit ItsyBitsy M0 (ATSAMD21G18)
//  Wersja "PC MODE" — bez OLED, bez przyciskow/potencjometrow.
//  Sterowanie wylacznie z aplikacji PC przez USB Serial.
// ============================================================
//  PIN MAPPING (Cytron MDD10A REV2.0, tryb DIR+PWM Sign-Magnitude):
//    DIR1  -> pin 3   (kierunek Peltiera)
//    PWM1  -> pin 4   (moc Peltiera)
//    DIR2  -> pin 5   (kierunek wentylatora)
//    PWM2  -> pin 11  (moc wentylatora)
//    GND   -> G
//
//  MAX31856 (SPI):
//    SCK->SCK | SDO->MISO | SDI->MOSI
//    CS1 (czujnik glowny, PID)    -> pin 9
//    CS2 (czujnik dodatkowy)      -> pin 10
//    GND->G | VCC->3V
//
//  BIBLIOTEKI: Adafruit MAX31856, Adafruit BusIO, FlashStorage_SAMD
//
//  STEROWANIE: PID + Feed-Forward (symetryczny: grzanie i chlodzenie)
//    Rampa:   jednostronne (tylko grzanie LUB tylko chlodzenie)
//  SELF-TUNE: co 2s, 60 cykli = 2 minuty (heurystyczny, na zywo)
//  RELAY AUTOTUNING: Astrom-Hagglund + Tyreus-Luyben (kalibracja per-temperatura)
//  36 PROFILI: 9 temp x 8 ramp (relay: jeden test wypelnia wszystkie rampy danej temp)
// ============================================================

#include <SPI.h>
#include <Adafruit_MAX31856.h>
#include <FlashStorage_SAMD.h>

// ── PINY STEROWNIKA (Cytron MDD10A, DIR+PWM) ────────────────────
#define PIN_DIR1   3    // Peltier kierunek
#define PIN_PWM1   4    // Peltier moc
#define PIN_DIR2   5    // Wentylator kierunek
#define PIN_PWM2   11   // Wentylator moc

// ── PINY TERMOPAR (MAX31856, SPI) ───────────────────────────────
#define PIN_CS_TC   9   // czujnik glowny (PID)
#define PIN_CS_TC2  10  // czujnik dodatkowy (pomiar, nie wplywa na PID)

#define PWM_MAX       255
#define TEMP_MIN_C    -15.0f
#define TEMP_MAX_DEF   110.0f
#define PID_DT_MS      100
#define INTEGRAL_MAX   1000.0f

// ── FEED-FORWARD (symetryczny: grzanie i chlodzenie) ────
#define T_AMBIENT      25.0f
#define KFF_HOLD_DEF   2.5f
#define KFF_RAMP_DEF   1.0f
#define KFF_HOLD_COOL_DEF   2.5f
#define KFF_RAMP_COOL_DEF   1.0f
#define FF_MAX         210.0f
#define FF_GAIN        3.0f

#define SP_MIN    -15.0f
#define SP_MAX     100.0f
#define KP_MIN       1.0f
#define KP_MAX      30.0f
#define KI_MIN       0.0f
#define KI_MAX       1.5f
#define KD_MIN       0.0f
#define KD_MAX      80.0f
#define KP_BASE      10.0f
#define KI_BASE      0.3f
#define KD_BASE_H    0.8f
#define KD_BASE_C    0.3f
#define RAMP_MIN     0.5f
#define RAMP_MAX    80.0f
#define TMAX_MIN    50.0f
#define TMAX_MAX   115.0f

#define ST_INT_MS   2000
#define ST_CYC_MAX    60
#define ST_HIST        6
#define ST_ADJ      0.04f
#define ST_DEAD     0.3f

#define FAN_RUNON_MS 120000UL
#define FREEZE_TARGET   20.0f
#define FREEZE_RAMP     6.0f
#define FREEZE_TOL      0.8f
#define FREEZE_STABLE_MS 8000

#define SS_STEP     5
#define SS_INT     50
#define SS_INIT    20

#define PT_N  9
#define PR_N  8
#define P_TOT (PT_N*PR_N)
const float PT[PT_N]={20,30,40,50,60,70,80,90,100};
const float PR[PR_N]={2,5,10,20,30,40,60,80};

#define CAL_RAMP_MAX 20
float calRamps[CAL_RAMP_MAX]={2,5,10,20,30,40,60,80};
int   calRampN=8;

struct Prof {
  float Kp_h,Ki_h,Kd_h;
  float Kp_c,Ki_c,Kd_c;
  bool  valid;
};
struct FD { bool cal; Prof p[P_TOT]; float ru,rd,tm; bool polSet; bool polSw; float calMin,calMax; };
FlashStorage(pidFlash,FD);

Adafruit_MAX31856 tc=Adafruit_MAX31856(PIN_CS_TC);
Adafruit_MAX31856 tc2=Adafruit_MAX31856(PIN_CS_TC2);
float temp2=NAN;
bool tc2OK=false;

Prof prof[P_TOT];
bool calDone=false;

float spT=25,spA=25;
float Kp_h=10,Ki_h=0.3f,Kd_h=0.8f;
float Kp_c=10,Ki_c=0.3f,Kd_c=0.3f;
float Kp=10,Ki=0.3f,Kd=0.8f;
float kffHold=KFF_HOLD_DEF, kffRamp=KFF_RAMP_DEF;
float kffHoldCool=KFF_HOLD_COOL_DEF, kffRampCool=KFF_RAMP_COOL_DEF;
float rU=2,rD=2,tMax=TEMP_MAX_DEF;
bool  htg=true;
bool  oppositeDirBrake=false;  // ON: PID moze chwilowo uzyc przeciwnego kierunku
                                // (np. grzanie przy koncu chlodzenia) zeby skorygowac
                                // przeregulowanie blisko celu - inaczej system moze tylko
                                // zejsc do PWM=0 i czekac az temperatura sama sie wyrowna,
                                // co objawia sie jako powolne "uciekanie" od setpointu.
                                // OFF (domyslnie): sztywna blokada jednokierunkowa - mniej
                                // przelaczen kierunku = mniej szumu na czulym pomiarze,
                                // ale wolniejsza korekta przy przeregulowaniu.
bool  wasAtT=false;
float dFilt=0;
float pwmFilt=0;
float ffFilt=0;

float ig=0,pe=0,lT=25;
int   lPwm=0;
bool  tcE=false;

String cmdBuf="";
float calOffset=0.0f;

bool  stOn=false,stDone=false;
int   stC=0;
float stEH[ST_HIST]={},stPH[ST_HIST]={};
int   stI=0;
float stBH=999,stBKpH,stBKiH,stBKdH;
float stBC=999,stBKpC,stBKiC,stBKdC;
unsigned long stLt=0;
String stSt="";

bool ssA=false; int ssPwm=0,ssTgt=0;
unsigned long ssTm=0;

unsigned long fanRunonT=0;
bool fanRunonActive=false;

#define SB 20
float slTb[SB]={};unsigned long slTm[SB]={};
int slI=0;bool slF=false;unsigned long slT=0;
String slSt="";

bool polSw=false;
bool polSet=false;

enum St{MAN,AUTO,COOL,RTEST,CAL,FREEZE};
unsigned long frzStableT=0;
bool frzReady=false;

int  fanSpeed=100;
bool fanOn=false;
St sys=MAN;

int rtP=0;float rtU=0,rtD=0,rtT0=0;
unsigned long rtTm=0;String rtSt="";

int cTi=0,cRi=0,cPh=0,cIt=0;
unsigned long cPT=0;
float cTmn=20,cTmx=90;
#define CPM 10
float cTP[CPM];int cTN=0;
float cBH=999,cBC=999;
float cKpH,cKiH,cKdH,cKpC,cKiC,cKdC;
#define CH 10
float cEH[CH]={},cPwH[CH]={};int cHI=0;
unsigned long cLI=0;
String cSt="";
#define CA 300000
#define CS 15000
#define CT 60000
#define CI  2000

#define RELAY_AMP    60
#define RELAY_HYST   1.0f
#define RELAY_CYCLES 6
#define RELAY_WARMUP 2
#define RELAY_MIN_PER 5000UL
#define RELAY_MAX_MS 180000
float relayPeakHi=-999,relayPeakLo=999;
float relayAmps[RELAY_CYCLES]={};
float relayPers[RELAY_CYCLES]={};
unsigned long relayTcross=0;
int relayCycN=0;
bool relayState=false;
bool relayWasAbove=false;

unsigned long tP=0,tD=0,tR=0;
unsigned long rampT0=0;
#define DT_D 200
#define DT_R 200

String fts(float v,int d){return String(v,d);}
int pi_(int ti,int ri){return ti*PR_N+ri;}
int nTi(float t){int b=0;float bd=9999;for(int i=0;i<PT_N;i++){float d=abs(PT[i]-t);if(d<bd){bd=d;b=i;}}return b;}
int nRi(float r){int b=0;float bd=9999;for(int i=0;i<PR_N;i++){float d=abs(PR[i]-r);if(d<bd){bd=d;b=i;}}return b;}

void ldProf(float temp,float ramp){
  int ti0=0,ti1=0,ri0=0,ri1=0;
  for(int i=0;i<PT_N-1;i++) if(temp>=PT[i]&&temp<=PT[i+1]){ti0=i;ti1=i+1;break;}
  if(temp<PT[0]){ti0=ti1=0;}if(temp>PT[PT_N-1]){ti0=ti1=PT_N-1;}
  for(int i=0;i<PR_N-1;i++) if(ramp>=PR[i]&&ramp<=PR[i+1]){ri0=i;ri1=i+1;break;}
  if(ramp<PR[0]){ri0=ri1=0;}if(ramp>PR[PR_N-1]){ri0=ri1=PR_N-1;}
  float wt=(ti1!=ti0)?(temp-PT[ti0])/(PT[ti1]-PT[ti0]):0.5f;
  float wr=(ri1!=ri0)?(ramp-PR[ri0])/(PR[ri1]-PR[ri0]):0.5f;
  float kph=0,kih=0,kdh=0,kpc=0,kic=0,kdc=0;float wsum=0;int cnt=0;
  auto add=[&](int ti,int ri,float w){
    Prof&p=prof[pi_(ti,ri)];
    if(p.valid){kph+=p.Kp_h*w;kih+=p.Ki_h*w;kdh+=p.Kd_h*w;
                kpc+=p.Kp_c*w;kic+=p.Ki_c*w;kdc+=p.Kd_c*w;wsum+=w;cnt++;}
  };
  add(ti0,ri0,(1-wt)*(1-wr));add(ti0,ri1,(1-wt)*wr);
  add(ti1,ri0,wt*(1-wr));add(ti1,ri1,wt*wr);
  if(cnt>0 && wsum>0.001f){
    kph/=wsum;kih/=wsum;kdh/=wsum;kpc/=wsum;kic/=wsum;kdc/=wsum;
    Kp_h=constrain(kph,KP_MIN,KP_MAX);Ki_h=constrain(kih,KI_MIN,KI_MAX);Kd_h=constrain(kdh,KD_MIN,KD_MAX);
    Kp_c=constrain(kpc,KP_MIN,KP_MAX);Ki_c=constrain(kic,KI_MIN,KI_MAX);Kd_c=constrain(kdc,KD_MIN,KD_MAX);
    if(htg){Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;}else{Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;}
    Serial.print("Prof: Kp=");Serial.print(Kp,1);Serial.print(" Ki=");Serial.print(Ki,2);Serial.print(" Kd=");Serial.println(Kd,2);
  }
}

void savF(){FD fd;fd.cal=calDone;for(int i=0;i<P_TOT;i++) fd.p[i]=prof[i];fd.ru=rU;fd.rd=rD;fd.tm=tMax;fd.polSet=polSet;fd.polSw=polSw;fd.calMin=cTmn;fd.calMax=cTmx;pidFlash.write(fd);Serial.println("Flash: zapisano.");}
void ldF(){FD fd;pidFlash.read(fd);if(fd.cal){calDone=true;for(int i=0;i<P_TOT;i++) prof[i]=fd.p[i];rU=fd.ru;rD=fd.rd;tMax=fd.tm;Serial.println("Flash: wczytano.");}if(fd.polSet){polSet=true;polSw=fd.polSw;}if(fd.calMin>=0&&fd.calMin<fd.calMax&&fd.calMax<=115){cTmn=fd.calMin;cTmx=fd.calMax;}}
void savePol(){FD fd;pidFlash.read(fd);fd.polSet=true;fd.polSw=polSw;pidFlash.write(fd);}
void rst(){calDone=false;for(int i=0;i<P_TOT;i++) prof[i]={10,0.3f,0.8f,10,0.3f,0.3f,false};Kp_h=Kp_c=Kp=10;Ki_h=Ki_c=Ki=0.3f;Kd_h=0.8f;Kd_c=Kd=0.3f;rU=rD=2;tMax=TEMP_MAX_DEF;ig=0;pe=0;Serial.println("Reset.");}

// ── STEROWANIE WYJSCIAMI: Cytron MDD10A (DIR+PWM Sign-Magnitude) ────────────
// wPwm: o>0 = grzanie (DIR1=HIGH), o<0 = chlodzenie (DIR1=LOW), |o| = PWM 0-255.
// polSw odwraca DIR jesli polaryzacja zostala wykryta jako odwrotna.
void wPwm(int o){
  lPwm=o;
  bool heat = (o>=0);
  if(polSw) heat = !heat;           // odwrocenie polaryzacji jesli wykryto
  digitalWrite(PIN_DIR1, heat ? HIGH : LOW);
  analogWrite(PIN_PWM1, abs(o));
}

// Wentylator: jeden kierunek obrotow (DIR2 zawsze LOW - napiecie na M2A, nie M2B), PWM2 = predkosc
void fanApply(){
  int pwm = fanOn ? (int)(fanSpeed*2.55f) : 0;
  pwm = constrain(pwm,0,255);
  digitalWrite(PIN_DIR2, LOW);
  analogWrite(PIN_PWM2, pwm);
}
void stpPel(){analogWrite(PIN_PWM1,0);digitalWrite(PIN_DIR1,LOW);lPwm=0;ssA=false;ssPwm=0;}
void setPwr(int o){
  o=constrain(o,-PWM_MAX,PWM_MAX);
  bool dir=(lPwm>0&&o<0)||(lPwm<0&&o>0),zero=(lPwm==0&&o!=0);
  if(dir||zero){if(dir){wPwm(0);delay(50);}ssA=true;ssTgt=o;ssPwm=(o>0)?SS_INIT:-SS_INIT;ssTm=millis();wPwm(ssPwm);}
  else if(ssA){ssTgt=o;}else{wPwm(o);}
}
void updSS(){
  if(!ssA) return;if(millis()-ssTm<SS_INT) return;ssTm=millis();
  if(ssTgt>0){ssPwm+=SS_STEP;if(ssPwm>=ssTgt){ssPwm=ssTgt;ssA=false;}}
  else if(ssTgt<0){ssPwm-=SS_STEP;if(ssPwm<=ssTgt){ssPwm=ssTgt;ssA=false;}}
  else{ssPwm=0;ssA=false;}wPwm(ssPwm);
}

float rdT(){
  uint8_t f=tc.readFault();if(f){tcE=true;return lT;}
  tcE=false;float raw=tc.readThermocoupleTemperature();
  if(isnan(raw)||raw<-50.0f||raw>200.0f){tcE=true;return lT;}
  lT=raw+calOffset;return lT;
}

void updRamp(){
  float stepU=rU/300.0f, stepD=rD/300.0f;
  float d=spT-spA;
  if(abs(d)<0.02f){spA=spT;return;}
  if(d>0) spA=min(spA+stepU,spT);
  else    spA=max(spA-stepD,spT);
}

void updSlope(float temp){
  unsigned long now=millis();
  if(slT==0){slT=now;for(int i=0;i<SB;i++){slTb[i]=temp;slTm[i]=now;}return;}
  if(now-slT<1000) return;slT=now;
  slTb[slI]=temp;slTm[slI]=now;slI=(slI+1)%SB;if(slI==0) slF=true;
  int oi=slF?slI:0;float dt=(now-slTm[oi])/60000.0f;if(dt<0.05f) return;
  float act=(temp-slTb[oi])/dt,tgt=htg?rU:-rD;
  if(abs(spA-spT)<0.5f){slSt="";return;}
  float err=tgt-act;
  if(abs(err)<0.5f) slSt="OK";else if(err>0) slSt="+"+fts(err,1);else slSt=fts(err,1);
}

// ── Self-tune ─────────────────────────────────────────────────
void stStart(){
  stOn=true;stDone=false;stC=0;stLt=millis();stSt="Starting...";
  stBH=stBC=999;
  stBKpH=Kp_h;stBKiH=Ki_h;stBKdH=Kd_h;
  stBKpC=Kp_c;stBKiC=Ki_c;stBKdC=Kd_c;
  for(int i=0;i<ST_HIST;i++){stEH[i]=0;stPH[i]=0;}stI=0;
  ig=0;pe=0;
  Serial.print("ST START SP=");Serial.print(spT,1);Serial.print(" R=");Serial.println(rU,1);
}
void stStop(){
  stOn=false;stDone=true;
  Kp_h=stBKpH;Ki_h=stBKiH;Kd_h=stBKdH;
  Kp_c=stBKpC;Ki_c=stBKiC;Kd_c=stBKdC;
  if(htg){Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;}else{Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;}
  int idx=pi_(nTi(spT),nRi(htg?rU:rD));
  prof[idx]={Kp_h,Ki_h,Kd_h,Kp_c,Ki_c,Kd_c,true};
  calDone=true;ig=0;pe=0;
  stSt="OK zapisano";
  Serial.print("ST KONIEC Kp=");Serial.print(Kp,2);Serial.print(" Ki=");Serial.print(Ki,3);Serial.print(" Kd=");Serial.println(Kd,2);
  savF();
  Serial.println("Profil zapisany do Flash automatycznie");
}
void runST(float temp){
  if(!stOn||sys!=AUTO) return;
  unsigned long now=millis();if(now-stLt<ST_INT_MS) return;
  stLt=now;stC++;
  float err=spA-temp,ae=abs(err);
  stEH[stI]=err;stPH[stI]=(float)lPwm;stI=(stI+1)%ST_HIST;
  int sc=0;for(int i=0;i<ST_HIST-1;i++){int a=i,b=(i+1)%ST_HIST;if(stEH[a]*stEH[b]<0)sc++;}
  bool osc=(sc>=2);
  int sat=0;for(int i=0;i<ST_HIST;i++) if(abs(stPH[i])>=PWM_MAX-5) sat++;
  bool satd=(sat>=ST_HIST-1);
  int pi2=(stI-2+ST_HIST)%ST_HIST,ci2=(stI-1+ST_HIST)%ST_HIST;
  float tr=abs(stEH[ci2])-abs(stEH[pi2]);
  bool im=(tr<-0.1f),wo=(tr>0.3f);
  if(htg){
    if(osc){Kp_h=constrain(Kp_h*(1-ST_ADJ*1.5f),KP_MIN,KP_MAX);Kd_h=constrain(Kd_h*(1-ST_ADJ),KD_MIN,KD_MAX);Ki_h=constrain(Ki_h*(1-ST_ADJ*0.5f),KI_MIN,KI_MAX);ig*=0.5f;stSt="OSC-";}
    else if(satd&&ae>2){stSt="SAT";}
    else if(ae>8&&!im){Kp_h=constrain(Kp_h*(1+ST_ADJ*2),KP_MIN,KP_MAX);stSt="SLOW++";}
    else if(ae>3&&wo){Kp_h=constrain(Kp_h*(1+ST_ADJ),KP_MIN,KP_MAX);stSt="WORSE";}
    else if(ae>ST_DEAD&&im){Ki_h=constrain(Ki_h*(1+ST_ADJ*0.5f),KI_MIN,KI_MAX);stSt="Ki+";}
    else{stSt="OK";}
    Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;
    if(ae<stBH){stBH=ae;stBKpH=Kp_h;stBKiH=Ki_h;stBKdH=Kd_h;}
  } else {
    if(osc){Kp_c=constrain(Kp_c*(1-ST_ADJ*1.5f),KP_MIN,KP_MAX);Kd_c=constrain(Kd_c*(1-ST_ADJ),KD_MIN,KD_MAX);Ki_c=constrain(Ki_c*(1-ST_ADJ*0.5f),KI_MIN,KI_MAX);ig*=0.5f;stSt="OSC-";}
    else if(satd&&ae>2){stSt="SAT";}
    else if(ae>8&&!im){Kp_c=constrain(Kp_c*(1+ST_ADJ*2),KP_MIN,KP_MAX);stSt="SLOW++";}
    else if(ae>3&&wo){Kp_c=constrain(Kp_c*(1+ST_ADJ),KP_MIN,KP_MAX);stSt="WORSE";}
    else if(ae>ST_DEAD&&im){Ki_c=constrain(Ki_c*(1+ST_ADJ*0.5f),KI_MIN,KI_MAX);stSt="Ki+";}
    else{stSt="OK";}
    Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;
    if(ae<stBC){stBC=ae;stBKpC=Kp_c;stBKiC=Ki_c;stBKdC=Kd_c;}
  }
  Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
  Serial.print(spA,2);Serial.print(",");Serial.print(spT,2);Serial.print(",");
  Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
  Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);Serial.println(",ST-"+stSt);
  if(stC>=ST_CYC_MAX) stStop();
}

// ── PID + Feed-Forward (symetryczny: grzanie i chlodzenie) ─────
int compPID(float temp){
  float dt=PID_DT_MS/1000.0f,err=spA-temp;

  // Kierunek (grzanie/chlodzenie): TWARDA blokada wg celu (spT) wzgledem
  // RZECZYWISTEJ temperatury (temp), a NIE wzgledem pozycji rampy (spA).
  // Zasada: cel wyzej niz aktualna temperatura -> caly cykl TYLKO grzanie
  // (PWM nigdy nie schodzi ponizej 0, nawet chwilowo). Cel nizej -> TYLKO
  // chlodzenie. Histereza DIR_HYST wokol granicy zapobiega drganiu kierunku
  // przy szumie termopary dokladnie w momencie gdy temp~=spT.
  //
  // Wczesniej kierunek zalezal od porownania spT vs spA (pozycja rampy), co
  // dawalo dwa problemy: (1) gdy rampa dojdzie do celu (spA==spT), warunek
  // stawal sie niejednoznaczny i wymagal dodatkowego override'u na >3C bledu,
  // (2) mimo override'u nadal zdarzaly sie niepotrzebne przelaczenia HEAT/COOL
  // generujace szum (kazde przelaczenie = reset integratora ig=0). Teraz
  // kierunek zalezy tylko od jednej, stabilnej wielkosci (spT-temp), wiec
  // przez caly czas trwania jednego cyklu grzania/chlodzenia kierunek jest
  // ustalony raz i sie nie zmienia (chyba ze uzytkownik sam zmieni SP).
  const float DIR_HYST = 0.5f;   // C - zwieksz np. do 1.0 jesli nadal drga
  bool rH;
  if      (spT - temp >  DIR_HYST) rH = true;   // cel wyzej -> grzanie
  else if (temp - spT >  DIR_HYST) rH = false;  // cel nizej -> chlodzenie
  else                              rH = htg;    // w strefie histerezy - bez zmian

  if(rH!=htg){
    ig=0;htg=rH;
    if(htg){Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;}
    else   {Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;}
  }

  // Wentylator zawsze na 100% podczas chlodzenia (niezaleznie od recznego
  // ustawienia uzytkownika) - chlodzenie potrzebuje maksymalnego przeplywu
  // powietrza zeby skutecznie odprowadzac cieplo z goracej strony Peltiera.
  // Sprawdzane co cykl PID (nie tylko przy zmianie kierunku), zeby dzialalo
  // tez np. zaraz po starcie w trybie COOL. Fan NIE jest dotykany podczas
  // grzania - wtedy dziala tak jak uzytkownik go ustawil recznie.
  if(!htg && (!fanOn || fanSpeed!=100)){
    fanOn=true; fanSpeed=100; fanApply();
  }

  float dRaw=(err-pe)/dt; pe=err;
  dFilt = dFilt + 0.3f*(dRaw - dFilt);

  // Zakres dozwolonego PWM: domyslnie (oppositeDirBrake=OFF) sztywno tylko
  // jeden kierunek wg htg - to eliminuje szum z przelaczen kierunku blisko
  // celu. Gdy oppositeDirBrake=ON, dopuszczamy pelny zakres [-PWM_MAX,PWM_MAX]
  // niezaleznie od htg - jesli PID "zobaczy" przeregulowanie (err przeciwnego
  // znaku), moze aktywnie zahamowac zamiast tylko zejsc do PWM=0 i czekac.
  // Dobor wzmocnien (Kp/Ki/Kd, ff) nadal podaza za htg - to sie NIE zmienia,
  // zeby nie wracal problem czestego przelaczania calego profilu regulacji.
  float lo = oppositeDirBrake ? -(float)PWM_MAX : (htg ? 0.0f : -(float)PWM_MAX);
  float hi = oppositeDirBrake ? (float)PWM_MAX : (htg ? (float)PWM_MAX : 0.0f);

  float ff=0;
  if(htg){
    float hold = kffHold*(spA - T_AMBIENT);
    float ramp = 0.0f;
    if(spT > spA+0.2f){
      ramp = kffRamp*rU;
      ramp *= constrain((spT-spA)/3.0f, 0.0f, 1.0f);
    }
    ff = constrain(hold + ramp, 0.0f, FF_MAX);
  } else {
    // Analogiczny feed-forward dla CHLODZENIA - wczesniej go nie bylo (stad
    // komentarz w naglowku pliku "tylko grzanie"), przez co chlodzenie
    // polegalo wylacznie na reaktywnym PID bez zadnego przewidywania i
    // gorzej nadazalo za zadana rampa niz grzanie. Struktura identyczna jak
    // dla grzania, tylko lustrzana wzgledem T_AMBIENT i ze znakiem ujemnym:
    // hold - moc potrzebna zeby TYLKO utrzymac temperature ponizej otoczenia
    // (im nizej ponizej ambient, tym wiecej trzeba), ramp - dodatkowy zastrzyk
    // mocy proporcjonalny do zadanego tempa chlodzenia (rD) w trakcie aktywnej
    // rampy w dol, wygaszany w miare zblizania sie do celu.
    float hold = kffHoldCool*(T_AMBIENT - spA);
    float ramp = 0.0f;
    if(spT < spA-0.2f){
      ramp = kffRampCool*rD;
      ramp *= constrain((spA-spT)/3.0f, 0.0f, 1.0f);
    }
    ff = -constrain(hold + ramp, 0.0f, FF_MAX);
  }
  ffFilt += 0.08f*(ff - ffFilt);

  float igTry = constrain(ig + err*dt, -INTEGRAL_MAX, INTEGRAL_MAX);
  float outT  = ffFilt + Kp*err + Ki*igTry + Kd*dFilt;
  float outTc = constrain(outT, lo, hi);
  bool inSat  = (outT != outTc);
  bool deeper = (htg && err>0) || (!htg && err<0);
  if(!(inSat && deeper)) ig = igTry;

  float out = ffFilt + Kp*err + Ki*ig + Kd*dFilt;
  out = constrain(out, lo, hi);

  pwmFilt = pwmFilt + 0.4f*(out - pwmFilt);

  return (int)pwmFilt;
}

void detPol(){
  Serial.println("Polarity check... Do not touch! 4s");
  delay(300);float t0=tc.readThermocoupleTemperature();
  polSw=false;
  digitalWrite(PIN_DIR1,HIGH);analogWrite(PIN_PWM1,80);delay(4000);
  float t1=tc.readThermocoupleTemperature();analogWrite(PIN_PWM1,0);
  float d=t1-t0;if(d>=0.3f) polSw=false;else if(d<=-0.3f) polSw=true;
  polSet=true; savePol();
  Serial.print(polSw?"Pol:SWAPPED":"Pol:NORMAL");Serial.print(" dT=");Serial.println(d,2);
}

void startRT(){sys=RTEST;rtP=0;rtU=rtD=0;rtT0=lT;rtTm=millis();rtSt="HEAT 0/60s";wPwm(PWM_MAX);}
void runRT(float t){
  if(sys!=RTEST) return;unsigned long el=millis()-rtTm;int s=(int)(el/1000);
  if(rtP==0){rtSt="HEAT "+String(s)+"/60s";if(t>=tMax-5||s>=60){wPwm(0);float dT=t-rtT0,dM=el/60000.0f;rtU=(dM>0)?dT/dM:0;rtP=1;rtT0=t;rtTm=millis();delay(300);wPwm(-PWM_MAX);}}
  else if(rtP==1){rtSt="COOL "+String(s)+"/60s";if(t<=TEMP_MIN_C+2||s>=60){wPwm(0);float dT=rtT0-t,dM=el/60000.0f;rtD=(dM>0)?dT/dM:0;if(rtU>0) rU=constrain(rtU*0.8f,RAMP_MIN,RAMP_MAX);if(rtD>0) rD=constrain(rtD*0.8f,RAMP_MIN,RAMP_MAX);rtP=2;sys=MAN;stpPel();rtSt="G:"+fts(rtU,1)+" C:"+fts(rtD,1);}}
}

void bldCP(){cTN=0;float t=cTmn;while(t<=cTmx+0.1f&&cTN<CPM){cTP[cTN++]=t;t+=10;}}
void stCalS(){sys=CAL;cPh=-1;cSt="Ustaw zakres";}
void stCalR(){
  bldCP();cTi=cRi=cPh=cIt=0;cPT=millis();cBH=cBC=999;
  Kp_h=KP_BASE;Ki_h=KI_BASE;Kd_h=KD_BASE_H;Kp_c=KP_BASE;Ki_c=KI_BASE;Kd_c=KD_BASE_C;
  cKpH=Kp_h;cKiH=Ki_h;cKdH=Kd_h;cKpC=Kp_c;cKiC=Ki_c;cKdC=Kd_c;
  for(int i=0;i<CH;i++){cEH[i]=cPwH[i]=0;}cHI=0;cLI=0;ig=0;pe=0;
  spT=cTP[0];spA=lT;rU=rD=RAMP_MAX;
  int tot=cTN;
  char b[24];sprintf(b,"Start 1/%d",tot);cSt=String(b);
  Serial.println("=== KAL. RELAY START ===");
  Serial.print("CALPLAN:");Serial.print(tot);
  Serial.print(",temps=");
  for(int i=0;i<cTN;i++){Serial.print(cTP[i],0);if(i<cTN-1)Serial.print("/");}
  Serial.print(",ramps=relay");
  Serial.println();
}
void savCP(){
  int ti=nTi(cTP[cTi]);
  for(int ri=0;ri<PR_N;ri++){
    int idx=pi_(ti,ri);
    prof[idx]={cKpH,cKiH,cKdH,cKpC,cKiC,cKdC,true};
  }
  Serial.print("Prof T=");Serial.print(cTP[cTi],0);Serial.print(" (wszystkie rampy) Kp=");Serial.println(cKpH,1);
}
void nxtCS(){
  savCP();cTi++;
  int tot=cTN,done=cTi;
  if(cTi>=cTN){calDone=true;savF();sys=MAN;stpPel();char b[24];sprintf(b,"DONE %d/%d",tot,tot);cSt=String(b);Serial.println("=== KAL. ZAKONCZONA ===");return;}
  cPh=0;cPT=millis();
  Kp_h=KP_BASE;Ki_h=KI_BASE;Kd_h=KD_BASE_H;Kp_c=KP_BASE;Ki_c=KI_BASE;Kd_c=KD_BASE_C;
  spT=cTP[cTi];rU=rD=RAMP_MAX;spA=lT;ig=0;pe=0;
  char b[24];sprintf(b,"Temp %d/%d",done+1,tot);cSt=String(b);
}

// ── KALIBRACJA: RELAY FEEDBACK AUTOTUNING (Astrom-Hagglund + Tyreus-Luyben) ──
void runCal(float temp){
  if(sys!=CAL||cPh==-1) return;
  unsigned long now=millis(),el=now-cPT;
  float err=spA-temp,ae=abs(err);

  if(cPh==0){
    rU=rD=RAMP_MAX;
    if(now-tR>=DT_R){tR=now;updRamp();}
    setPwr(compPID(temp));
    char b[32];sprintf(b,"->%.0fC T=%.1f",cTP[cTi],temp);
    cSt=String(b);
    if(now-cLI>=500){
      cLI=now;
      int tot=cTN,done=cTi+1;
      Serial.print("CALSTAT:");Serial.print(done);Serial.print("/");
      Serial.print(tot);Serial.print(",T=");Serial.print(cTP[cTi],0);
      Serial.println(",R=heating");
    }
    if(ae<2.0f||el>CA){
      cPh=1;cPT=now;cSt="Stabilizing...";
      ig=0;pe=0;cLI=now;
    }
  }
  else if(cPh==1){
    setPwr(compPID(temp));
    cSt="Stabil "+String((CS-el)/1000)+"s";
    if(now-cLI>=500){
      cLI=now;
      int tot=cTN,done=cTi+1;
      Serial.print("CALSTAT:");Serial.print(done);Serial.print("/");
      Serial.print(tot);Serial.print(",T=");Serial.print(cTP[cTi],0);
      Serial.println(",R=stabil");
    }
    if(el>CS){
      cPh=2;cPT=now;
      spT=cTP[cTi];spA=cTP[cTi];
      relayPeakHi=-999;relayPeakLo=999;
      for(int i=0;i<RELAY_CYCLES;i++){relayAmps[i]=relayPers[i]=0;}
      relayCycN=0;
      relayTcross=now;
      relayWasAbove=(temp>spT);
      relayState=(temp<spT);
      cLI=now;
      ig=0;pe=0;
      cSt="Relay test...";
    }
  }
  else if(cPh==2){
    float sp=cTP[cTi];
    bool prevHeating=relayState;
    if(temp < sp-RELAY_HYST) relayState=true;
    else if(temp > sp+RELAY_HYST) relayState=false;
    setPwr(relayState ? RELAY_AMP : -RELAY_AMP);

    if(temp>relayPeakHi) relayPeakHi=temp;
    if(temp<relayPeakLo) relayPeakLo=temp;

    if(relayState && !prevHeating){
      unsigned long per=now-relayTcross;
      float amp=(relayPeakHi-relayPeakLo)/2.0f;
      if(relayCycN>=RELAY_WARMUP && per>=RELAY_MIN_PER && amp>0.05f){
        int slot=(relayCycN-RELAY_WARMUP)%RELAY_CYCLES;
        relayAmps[slot]=amp;
        relayPers[slot]=(float)per;
      }
      relayTcross=now;
      relayPeakHi=-999;relayPeakLo=999;
      relayCycN++;
    }

    char b[32];sprintf(b,"Relay %d/%d cykli",max(0,relayCycN-RELAY_WARMUP),RELAY_CYCLES);
    cSt=String(b);

    if(now-cLI>=500){
      cLI=now;
      int tot=cTN,done=cTi+1;
      Serial.print("CALSTAT:");Serial.print(done);Serial.print("/");
      Serial.print(tot);Serial.print(",T=");Serial.print(sp,0);
      Serial.println(",R=relay");
      Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
      Serial.print(sp,2);Serial.print(",");Serial.print(sp,2);Serial.print(",");
      Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
      Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);
      Serial.print(",CAL-");Serial.println(done);
    }

    if(relayCycN>=RELAY_WARMUP+RELAY_CYCLES || el>RELAY_MAX_MS){
      cPh=3;cPT=now;
    }
  }
  else if(cPh==3){
    setPwr(0);
    int valid=0; float ampSum=0, perSum=0;
    for(int i=0;i<RELAY_CYCLES;i++){
      if(relayAmps[i]>0.01f && relayPers[i]>0){
        ampSum+=relayAmps[i]; perSum+=relayPers[i]; valid++;
      }
    }
    if(valid>=2 && ampSum>0.01f){
      float aAvg=ampSum/valid;
      float Tu=(perSum/valid)/1000.0f;
      float Ku=(4.0f*RELAY_AMP)/(3.14159f*aAvg);
      float Kp_new=Ku/2.2f;
      float Ti=2.2f*Tu;
      float Td=Tu/6.3f;
      float Ki_new=Kp_new/Ti;
      float Kd_new=Kp_new*Td;
      Kp_new=constrain(Kp_new,KP_MIN,KP_MAX);
      Ki_new=constrain(Ki_new,KI_MIN,KI_MAX);
      Kd_new=constrain(Kd_new,KD_MIN,KD_MAX);
      cKpH=Kp_new;cKiH=Ki_new;cKdH=Kd_new;
      cKpC=Kp_new;cKiC=Ki_new;cKdC=Kd_new;
      Serial.print("RELAY T=");Serial.print(cTP[cTi],0);
      Serial.print(" a=");Serial.print(aAvg,1);Serial.print(" Tu=");Serial.print(Tu,1);
      Serial.print(" Ku=");Serial.print(Ku,1);
      Serial.print(" Kp=");Serial.print(Kp_new,2);Serial.print(" Ki=");Serial.print(Ki_new,3);
      Serial.print(" Kd=");Serial.println(Kd_new,2);
    } else {
      cKpH=KP_BASE;cKiH=KI_BASE;cKdH=KD_BASE_H;
      cKpC=KP_BASE;cKiC=KI_BASE;cKdC=KD_BASE_C;
      Serial.println("RELAY FAIL - bazowe");
    }
    for(int i=0;i<RELAY_CYCLES;i++){relayAmps[i]=relayPers[i]=0;}
    nxtCS();
  }
}

void setup(){
  Serial.begin(115200);

  pinMode(PIN_DIR1,OUTPUT);pinMode(PIN_PWM1,OUTPUT);
  digitalWrite(PIN_DIR1,LOW);analogWrite(PIN_PWM1,0);
  pinMode(PIN_DIR2,OUTPUT);pinMode(PIN_PWM2,OUTPUT);
  digitalWrite(PIN_DIR2,LOW);analogWrite(PIN_PWM2,0);

  if(!tc.begin()) Serial.println("ERROR: MAX31856 (CS9)!");
  tc.setThermocoupleType(MAX31856_TCTYPE_K);
  // Tryb CIAGLEJ konwersji zamiast domyslnego one-shot: w one-shot kazde
  // wywolanie readThermocoupleTemperature() wyzwala nowa konwersje ADC i
  // BLOKUJE az sie skonczy (wg dokumentacji Adafruit ~100ms, przy dwoch
  // czujnikach odczytywanych w tej samej iteracji petli moglo to sumowac sie
  // do wiecej niz PID_DT_MS=100ms zanim PID w ogole zobaczyl nowy odczyt).
  // W trybie ciaglym chip mierzy caly czas w tle, a odczyt to juz tylko
  // pobranie najswiezszej gotowej wartosci z rejestru - praktycznie
  // natychmiastowe. Dzieki temu petla PID faktycznie trzyma sie 100ms
  // zamiast byc ukryto spowalniana przez czas konwersji ADC.
  tc.setConversionMode(MAX31856_CONTINUOUS);

  tc2.begin();
  tc2.setThermocoupleType(MAX31856_TCTYPE_K);
  tc2.setConversionMode(MAX31856_CONTINUOUS);
  tc2OK=true;
  Serial.println("TC2 init done (CS10)");

  for(int i=0;i<P_TOT;i++) prof[i]={10,0.3f,0.8f,10,0.3f,0.3f,false};

  delay(200);float rt=tc.readThermocoupleTemperature();
  if(!isnan(rt)&&rt>-50&&rt<150){lT=rt;}

  ldF();  // wczytaj Flash (kalibracja + polaryzacja) jesli byla zapisana wczesniej
  if(!polSet){
    detPol();
    rt=tc.readThermocoupleTemperature();if(!isnan(rt)&&rt>-50&&rt<150) lT=rt;
  } else {
    Serial.println(polSw?"Pol:swapped (z Flash)":"Pol:normal (z Flash)");
  }
  spA=spT=lT;
  Serial.println("czas_s,temp_C,setpoint_akt,setpoint_cel,PWM,Kp,Ki,Kd,stan");
  Serial.print("Start T=");Serial.println(lT,1);
  Serial.println("PC MODE - sterowanie z aplikacji");
  sendCfg();
}

// ════════════════════════════════════════════════════════
//  PARSER KOMEND z PC (Serial)
//  Format: KOMENDA:wartosc\n  np. SP:25.5
// ════════════════════════════════════════════════════════
void sendCfg(){
  Serial.print("CFG:SP=");Serial.print(spT,2);
  Serial.print(",RU=");Serial.print(rU,2);
  Serial.print(",RD=");Serial.print(rD,2);
  Serial.print(",TMAX=");Serial.print(tMax,1);
  Serial.print(",KP=");Serial.print(Kp,3);
  Serial.print(",KI=");Serial.print(Ki,4);
  Serial.print(",KD=");Serial.print(Kd,3);
  Serial.print(",OFFSET=");Serial.print(calOffset,2);
  Serial.print(",STATE=");
  Serial.print(sys==AUTO?"AUTO":sys==COOL?"COOL":sys==CAL?"CAL":sys==RTEST?"RTEST":sys==FREEZE?"FREEZE":"MAN");
  Serial.print(",CAL=");Serial.print(calDone?1:0);
  Serial.print(",POL=");Serial.print(polSw?1:0);
  Serial.print(",POLSET=");Serial.print(polSet?1:0);
  Serial.print(",CALMIN=");Serial.print(cTmn,0);
  Serial.print(",CALMAX=");Serial.print(cTmx,0);
  Serial.print(",FAN=");Serial.print(fanOn?fanSpeed:0);
  Serial.print(",KFFH=");Serial.print(kffHold,2);
  Serial.print(",KFFR=");Serial.print(kffRamp,2);
  Serial.print(",KFFHC=");Serial.print(kffHoldCool,2);
  Serial.print(",KFFRC=");Serial.print(kffRampCool,2);
  Serial.print(",OPPDIR=");Serial.print(oppositeDirBrake?1:0);
  Serial.println();
}

void procCmd(String c){
  c.trim();
  if(c.length()==0) return;
  int colon=c.indexOf(':');
  String key = (colon>=0)?c.substring(0,colon):c;
  String val = (colon>=0)?c.substring(colon+1):"";
  key.toUpperCase();
  float fv=val.toFloat();

  if(key=="SP"){ spT=constrain(fv,SP_MIN,SP_MAX); }
  else if(key=="RU"){ rU=constrain(fv,RAMP_MIN,RAMP_MAX); }
  else if(key=="RD"){ rD=constrain(fv,RAMP_MIN,RAMP_MAX); }
  else if(key=="TMAX"){ tMax=constrain(fv,TMAX_MIN,TMAX_MAX); }
  else if(key=="KP"){ Kp=constrain(fv,KP_MIN,KP_MAX); }
  else if(key=="KI"){ Ki=constrain(fv,KI_MIN,KI_MAX); }
  else if(key=="KD"){ Kd=constrain(fv,KD_MIN,KD_MAX); }
  else if(key=="KFFH"){ kffHold=constrain(fv,0.0f,20.0f); }
  else if(key=="KFFR"){ kffRamp=constrain(fv,0.0f,20.0f); }
  else if(key=="KFFHC"){ kffHoldCool=constrain(fv,0.0f,20.0f); }
  else if(key=="KFFRC"){ kffRampCool=constrain(fv,0.0f,20.0f); }
  else if(key=="OFFSET"){ calOffset=constrain(fv,-20.0f,20.0f); }
  else if(key=="OPPDIR"){
    oppositeDirBrake=(val.toInt()>0);
    Serial.println(oppositeDirBrake?"OPPDIR ON (hamowanie przeciwnym kierunkiem)":"OPPDIR OFF (sztywna blokada kierunku)");
  }
  else if(key=="START"){
    if(sys==MAN){
      sys=AUTO;spA=lT;ig=0;pe=0;slT=0;tR=millis();
      dFilt=0;pwmFilt=0;ffFilt=0;
      rampT0=millis();
      if(calDone) ldProf(spT,rU);
      Serial.println("ON");
    }
  }
  else if(key=="STOP"){
    stpPel(); sys=MAN; stOn=false;
    fanOn=true; fanSpeed=100; fanApply();
    fanRunonActive=true; fanRunonT=millis();
    Serial.println("STOP");
  }
  else if(key=="ESTOP"){ wPwm(0);stpPel();sys=MAN;stOn=false;Serial.println("E-STOP"); }
  else if(key=="FREEZE"){
    sys=FREEZE; spT=FREEZE_TARGET; spA=lT; ig=0; pe=0; slT=0;
    rU=rD=FREEZE_RAMP; stOn=false; frzReady=false; frzStableT=0;
    Serial.print("FREEZE START -> target ");Serial.print(FREEZE_TARGET,0);
    Serial.println("C (gal solid)");
  }
  else if(key=="FREEZESTOP"){
    if(sys==FREEZE){ stpPel();sys=MAN;frzReady=false;Serial.println("FREEZE stopped"); }
  }
  else if(key=="FAN"){
    fanSpeed=constrain((int)fv,0,100);
    if(fanSpeed>0) fanOn=true;
    if(fanSpeed==0) fanOn=false;
    fanApply();
    Serial.print("FAN ");Serial.print(fanOn?"ON ":"OFF ");Serial.print(fanSpeed);Serial.println("%");
  }
  else if(key=="FANON"){
    fanOn=true; if(fanSpeed==0) fanSpeed=100; fanApply();
    Serial.print("FAN ON ");Serial.print(fanSpeed);Serial.println("%");
  }
  else if(key=="FANOFF"){
    fanOn=false; fanApply();
    Serial.println("FAN OFF");
  }
  else if(key=="SELFTUNE"){ if(sys==AUTO) stStart(); }
  else if(key=="SELFTUNESTOP"){ if(stOn) stStop(); }
  else if(key=="AUTOCAL"){
    sys=CAL; cPh=-1;
    stCalR();
    Serial.println("AUTOCAL START");
  }
  else if(key=="CALRANGE"){
    int cm=val.indexOf(',');
    if(cm>0){
      float lo=val.substring(0,cm).toFloat();
      float hi=val.substring(cm+1).toFloat();
      lo=constrain(lo,(float)TEMP_MIN_C,100.0f);
      hi=constrain(hi,lo+10.0f,115.0f);
      cTmn=lo; cTmx=hi;
      savF();
      Serial.print("CALRANGE set: ");Serial.print(cTmn,0);
      Serial.print("-");Serial.println(cTmx,0);
    }
  }
  else if(key=="SETCALRAMPS"){
    int n=0;
    String rest=val;
    while(n<CAL_RAMP_MAX && rest.length()>0){
      int cm=rest.indexOf(',');
      String tok=(cm>=0)?rest.substring(0,cm):rest;
      float r=tok.toFloat();
      if(r>=RAMP_MIN && r<=RAMP_MAX){ calRamps[n++]=r; }
      if(cm<0) break;
      rest=rest.substring(cm+1);
    }
    if(n>0){ calRampN=n; }
    Serial.print("CALRAMPS set: ");Serial.print(calRampN);Serial.print(" ramps: ");
    for(int i=0;i<calRampN;i++){Serial.print(calRamps[i],0);if(i<calRampN-1)Serial.print(",");}
    Serial.println();
  }
  else if(key=="REPOL"){
    polSet=false;
    detPol();
    Serial.println("Polaryzacja wykryta ponownie");
  }
  else if(key=="SETPOL"){
    polSw=(val.toInt()>0); polSet=true; savePol();
    Serial.println(polSw?"Pol:swapped (reczne)":"Pol:normal (reczne)");
  }
  else if(key=="AUTOCALSTOP"){
    if(sys==CAL){ stpPel(); sys=MAN; Serial.println("AUTOCAL ABORTED"); }
  }
  else if(key=="SAVE"){ savF(); }
  else if(key=="LOAD"){ ldF(); }
  else if(key=="RESET"){ rst(); }
  else if(key=="GET"){ sendCfg(); return; }
  else if(key=="DUMPCAL"){
    Serial.print("CALDUMP:");Serial.print(P_TOT);
    Serial.print(",cal=");Serial.println(calDone?1:0);
    for(int i=0;i<P_TOT;i++){
      Serial.print("PROF:");Serial.print(i);Serial.print(",");
      Serial.print(prof[i].Kp_h,3);Serial.print(",");
      Serial.print(prof[i].Ki_h,4);Serial.print(",");
      Serial.print(prof[i].Kd_h,3);Serial.print(",");
      Serial.print(prof[i].Kp_c,3);Serial.print(",");
      Serial.print(prof[i].Ki_c,4);Serial.print(",");
      Serial.print(prof[i].Kd_c,3);Serial.print(",");
      Serial.println(prof[i].valid?1:0);
    }
    Serial.println("CALDUMPEND");
  }
  else if(key=="SETPROF"){
    int idx=val.toInt();
    int c1=val.indexOf(',');
    if(idx>=0&&idx<P_TOT&&c1>0){
      String rest=val.substring(c1+1);
      float v[7]; int vi=0;
      while(vi<7){
        int cm=rest.indexOf(',');
        String tok=(cm>=0)?rest.substring(0,cm):rest;
        v[vi++]=tok.toFloat();
        if(cm<0) break;
        rest=rest.substring(cm+1);
      }
      if(vi>=6){
        prof[idx].Kp_h=v[0];prof[idx].Ki_h=v[1];prof[idx].Kd_h=v[2];
        prof[idx].Kp_c=v[3];prof[idx].Ki_c=v[4];prof[idx].Kd_c=v[5];
        prof[idx].valid=(vi>=7)?(v[6]>0.5f):true;
      }
    }
    return;
  }
  else if(key=="SETCALDONE"){
    calDone=(val.toInt()>0);
    savF();
    Serial.println("Kalibracja wgrana z PC");
  }
  else if(key=="PROFILE"){
    int p1=val.indexOf(','),p2=val.indexOf(',',p1+1);
    int p3=val.indexOf(',',p2+1),p4=val.indexOf(',',p3+1);
    if(p1>0&&p2>0&&p3>0&&p4>0){
      spT=constrain(val.substring(0,p1).toFloat(),SP_MIN,SP_MAX);
      rU =constrain(val.substring(p1+1,p2).toFloat(),RAMP_MIN,RAMP_MAX);
      Kp =constrain(val.substring(p2+1,p3).toFloat(),KP_MIN,KP_MAX);
      Ki =constrain(val.substring(p3+1,p4).toFloat(),KI_MIN,KI_MAX);
      Kd =constrain(val.substring(p4+1).toFloat(),KD_MIN,KD_MAX);
    }
  }
  sendCfg();
}

void readSerial(){
  while(Serial.available()){
    char ch=Serial.read();
    if(ch=='\n'||ch=='\r'){
      if(cmdBuf.length()>0){ procCmd(cmdBuf); cmdBuf=""; }
    } else {
      cmdBuf+=ch;
      if(cmdBuf.length()>64) cmdBuf="";
    }
  }
}

void loop(){
  uint32_t now=millis();
  readSerial();
  if(fanRunonActive){
    if(now-fanRunonT>=FAN_RUNON_MS){
      fanRunonActive=false; fanOn=false; fanApply();
      Serial.println("FAN runon done - off");
    }
  }
  if(sys==AUTO&&(now-tR>=DT_R)){tR=now;updRamp();}
  if(sys==AUTO) updSlope(lT);else slT=0;
  if(sys==AUTO) runST(lT);
  if(sys==CAL) runCal(lT);
  if(sys==RTEST) runRT(lT);
  updSS();
  if(now-tP>=PID_DT_MS){
    tP=now;float temp=rdT();
    if(tc2OK){
      float r2=tc2.readThermocoupleTemperature();
      if(!isnan(r2) && r2>-50 && r2<200) temp2=r2;
      else temp2=NAN;
    }
    if(temp>tMax&&sys!=MAN){stpPel();sys=MAN;stOn=false;if(fanOn){fanRunonActive=true;fanRunonT=now;}Serial.println("!!! TEMP MAX - STOP !!!");}
    switch(sys){
      case AUTO:
        if(temp<TEMP_MIN_C&&lPwm<0) stpPel();else setPwr(compPID(temp));
        Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
        Serial.print(spA,2);Serial.print(",");Serial.print(spT,2);Serial.print(",");
        Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
        Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);Serial.print(",AUTO,");
        Serial.println(isnan(temp2)?0.0f:temp2,2);
        break;
      case FREEZE: {
        updRamp();
        setPwr(compPID(temp));
        if(fabs(temp-FREEZE_TARGET)<=FREEZE_TOL){
          if(frzStableT==0) frzStableT=now;
          else if(!frzReady && (now-frzStableT)>=FREEZE_STABLE_MS){
            frzReady=true;
            Serial.println("FREEZE READY - gal solid, mozna wymienic probke");
          }
        } else {
          frzStableT=0;
          if(frzReady){ frzReady=false; }
        }
        Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
        Serial.print(spA,2);Serial.print(",");Serial.print(FREEZE_TARGET,1);Serial.print(",");
        Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
        Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);
        Serial.print(frzReady?",FREEZE_READY,":",FREEZE,");
        Serial.println(isnan(temp2)?0.0f:temp2,2);
        break;
      }
      case MAN:
        stpPel();
        Serial.print(now/1000.0f,1);Serial.print(",");
        Serial.print(temp,2);Serial.print(",");
        Serial.print(lT,2);Serial.print(",");
        Serial.print(spT,2);Serial.print(",");
        Serial.print(0);Serial.print(",");
        Serial.print(Kp,3);Serial.print(",");
        Serial.print(Ki,4);Serial.print(",");
        Serial.print(Kd,3);Serial.print(",MAN,");
        Serial.println(isnan(temp2)?0.0f:temp2,2);
        break;
      default:break;
    }
  }
}
