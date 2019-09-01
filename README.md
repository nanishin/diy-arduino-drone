# diy-arduino-drone
Let's make a DIY Arduino Drone based on MultiWii

# HOWTO

## 1. Order basic drone components by online store
### QAV250 carbon body frame set (clone version) x 1 EA
![qav250 carbon body frame](https://nanishin.github.io/assets/20180509_195858.png)
### BLDC (BrushLess Direct Current) motor set x 2 EA
- CCW (Counter Clock Wise) : Black Cap
- CW (Clock Wise) : Silver Cap
![BLDC motor](https://nanishin.github.io/assets/20180509_194858.png)
### ESC (Electronic Speed Controller) x 4 EA
![ESC simonk 12A](/media/20180511_072039.png)
### Propeller set x 2 EA
- Black for front
- Green for rear
![Propeller set](/media/20190901_152138.png)
### Li-Polymer power battery x 2 EA
![Li-Polymer power battery](/media/20190901_152223.png)
### Power charger x 1EA
![Power charger](https://nanishin.github.io/assets/20180510_204843.png)
### RC (Remote Control) transmitter/receiver set x 1 EA
- RC transmitter
![RC Transmitter](https://nanishin.github.io/assets/20180512_225129.png)
- RC receiver
![RC Receiver](https://nanishin.github.io/assets/20180512_225701.png)
### Reuse arduino set x 1 EA
- Arduino nano (ATmega328P, clone version)
- MPU6050 (Gyro sensor)
- Breadboard
![Arduino set](https://nanishin.github.io/assets/20180512_175004.png)
### Required tool
- Hexagon wrench set
![Hexagon wrench set](https://nanishin.github.io/assets/20180510_194357.png)

## 2. DIY arduino drone, ASSEMBLE!!!
![Drone body top](https://nanishin.github.io/assets/20180510_201456.png)
![Drone body bottom](https://nanishin.github.io/assets/20180510_201734.png)
![Power Dispatcher](https://nanishin.github.io/assets/20180512_001807.png)
![Attach Power Dispatcher](https://nanishin.github.io/assets/20180512_005123.png)
![Attach ESC top](https://nanishin.github.io/assets/20180512_162255.png)
![Attach Power Dispatcher bottom](https://nanishin.github.io/assets/20180512_164918.png)
![Attach arduiono set top](https://nanishin.github.io/assets/20180512_213341.png)
![Atrach arduino set side](https://nanishin.github.io/assets/20180512_213348.png)
![Attach arduino set bottom](https://nanishin.github.io/assets/20180512_213426.png)
![Attach sonar sensor top](https://nanishin.github.io/assets/20180816_025935.png)
![Attach sonar sensor bottom](https://nanishin.github.io/assets/20180816_025945.png)
![Attach sonar sensor side](https://nanishin.github.io/assets/20180816_025952.png)

## 3. Upload patched multiwii firmware to arduino nano
- Refer the patch got from [arduino_drone branch of nanishin/multiwii-firmware](https://github.com/nanishin/multiwii-firmware/commit/3703f92f7f69c481d64289d886bb625819cfb93f)
```shell
diff --git a/IMU.cpp b/IMU.cpp
index ea85d32..5dc2b2a 100644
--- a/IMU.cpp
+++ b/IMU.cpp
@@ -292,7 +292,9 @@ void getEstimatedAttitude(){
     value += deadband;                  \
   }
 
-#if BARO
+#if BARO || SONAR
+static int32_t  BaroHome = 0;
+
 uint8_t getEstimatedAltitude(){
   int32_t  BaroAlt;
   static float baroGroundTemperatureScale,logBaroGroundPressureSum;
@@ -315,7 +317,56 @@ uint8_t getEstimatedAltitude(){
   // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
   BaroAlt = ( logBaroGroundPressureSum - log(baroPressureSum) ) * baroGroundTemperatureScale;
 
-  alt.EstAlt = (alt.EstAlt * 6 + BaroAlt ) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
+  //alt.EstAlt = (alt.EstAlt * 6 + BaroAlt ) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
+  
+#if SONAR
+  if (f.SONAR_MODE) {
+    if (calibratingS > 0) {
+      if (!f.ARMED) { //init offset till motors not armed
+        //alt.EstAlt = alt.EstAlt * SONAR_BARO_LPF_LC + sonarAlt * (1 - SONAR_BARO_LPF_LC); // additional LPF to reduce baro noise (faster by 30 µs)
+
+        BaroHome = (alt.EstAlt * 6 + BaroAlt * 2) >> 3; // play with optimal coef. here
+      }
+
+      calibratingS--;
+    }
+  }
+#endif
+
+#if BARO && !SONAR
+  alt.EstAlt = (alt.EstAlt * 6 + BaroAlt) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
+#elif SONAR && !BARO
+  alt.EstAlt = alt.EstAlt * SONAR_BARO_LPF_LC + sonarAlt * (1 - SONAR_BARO_LPF_LC);
+#elif SONAR && BARO
+  // limit sonar altitude
+  /*if (sonarAlt > SONAR_MAX_HOLD) {
+  sonarAlt = SONAR_MAX_HOLD;
+  }*/
+
+  if (sonarAlt < SONAR_BARO_FUSION_LC) {
+    alt.EstAlt = alt.EstAlt * SONAR_BARO_LPF_LC + (BaroHome + sonarAlt) * (1 - SONAR_BARO_LPF_LC); // additional LPF to reduce baro noise (faster by 30 µs)
+  }
+  else if (sonarAlt < SONAR_BARO_FUSION_HC) {
+    float fade = SONAR_BARO_FUSION_RATIO;
+    if (fade == 0.0) fade = ((float) (SONAR_BARO_FUSION_HC - sonarAlt)) / (SONAR_BARO_FUSION_HC - SONAR_BARO_FUSION_LC);
+    fade = constrain(fade, 0.0f, 1.0f);
+
+    // LOG: will LPF should be faded too ? sonar is less sloppy than baro and will be oversmoothed
+    // LOG: try same as baro alone 6/4 ratio (same as above about smoothing)
+    alt.EstAlt = alt.EstAlt * SONAR_BARO_LPF_HC + ((BaroHome + sonarAlt) * fade + (BaroAlt) * (1 - fade)) * (1 - SONAR_BARO_LPF_HC);
+  }
+  else {
+    alt.EstAlt = (alt.EstAlt * 6 + BaroAlt) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
+  }
+
+
+  //alt.EstAlt = alt.EstAlt * SONAR_BARO_LPF_LC + sonarAlt * (1 - SONAR_BARO_LPF_LC); // SONAR
+#endif
+
+  debug[0] = sonarAlt; // raw sonar altitude
+  debug[1] = BaroAlt; // barometer altitude
+  debug[2] = alt.EstAlt;
+  
   #if (defined(VARIOMETER) && (VARIOMETER != 2)) || !defined(SUPPRESS_BARO_ALTHOLD)
     //P
     int16_t error16 = constrain(AltHold - alt.EstAlt, -300, 300);
@@ -352,4 +403,4 @@ uint8_t getEstimatedAltitude(){
   #endif
   return 1;
 }
-#endif //BARO
+#endif //BARO
diff --git a/MultiWii.cpp b/MultiWii.cpp
index 535bf00..3e4a88b 100644
--- a/MultiWii.cpp
+++ b/MultiWii.cpp
@@ -97,6 +97,9 @@ const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
   "MISSION;"
   "LAND;"
 #endif
+#if SONAR
+"SONAR;"
+#endif 
   ;
 
 const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
@@ -151,6 +154,9 @@ const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way
 #if GPS
   20, //"MISSION;"
   21, //"LAND;"
+#endif
+#if SONAR
+  22, //"SONAR;"
 #endif
   };
 
@@ -161,6 +167,9 @@ uint16_t cycleTime = 0;     // this is the number in micro second to achieve a f
 uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
 uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
 uint16_t calibratingG;
+#if SONAR
+uint16_t calibratingS = 0;
+#endif
 int16_t  magHold,headFreeModeHold; // [-180;+180]
 uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
 uint8_t  rcOptions[CHECKBOXITEMS];
@@ -708,6 +717,9 @@ void setup() {
   #endif
   calibratingG = 512;
   calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
+#if SONAR
+  calibratingS = 200;
+#endif
   #if defined(POWERMETER)
     for(uint8_t j=0; j<=PMOTOR_SUM; j++) pMeter[j]=0;
   #endif
@@ -784,6 +796,9 @@ void go_arm() {
         #if BARO
           calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
         #endif
+    #if SONAR
+      calibratingS = 10;
+    #endif
       #endif
       #ifdef LCD_TELEMETRY // reset some values when arming
         #if BARO
@@ -932,6 +947,9 @@ void loop () {
           #if BARO
             calibratingB=10;  // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
           #endif
+      #if SONAR
+      calibratingS = 10;
+      #endif
         }
         #if defined(INFLIGHT_ACC_CALIBRATION)  
          else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI) {    // Inflight ACC calibration START/STOP
@@ -1082,6 +1100,29 @@ void loop () {
       if (f.ANGLE_MODE || f.HORIZON_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
     #endif
 
+#if SONAR
+    if (rcOptions[BOXSONAR]) {
+      if (f.SONAR_MODE == 0) {
+        f.SONAR_MODE = 1;
+
+        AltHold = alt.EstAlt;
+
+#if defined(ALT_HOLD_THROTTLE_MIDPOINT)
+        initialThrottleHold = ALT_HOLD_THROTTLE_MIDPOINT;
+#else
+        initialThrottleHold = rcCommand[THROTTLE];
+#endif
+
+        errorAltitudeI = 0;
+        BaroPID = 0;
+        f.THROTTLE_IGNORED = 0;
+      }
+    }
+    else {
+      f.SONAR_MODE = 0;
+    }
+#endif
+
     #if BARO
       #if (!defined(SUPPRESS_BARO_ALTHOLD))
         #if GPS 
@@ -1253,10 +1294,16 @@ void loop () {
         #endif
       case 2:
         taskOrder++;
-        #if BARO
-          if (getEstimatedAltitude() != 0) break; // 280 us
-        #endif    
+    #if SONAR
+      Sonar_update(); //debug[2] = sonarAlt;
+      #endif
+    break;
       case 3:
+        taskOrder++;
+        #if BARO || SONAR
+          if (getEstimatedAltitude() != 0) break; // 280 us
+        #endif  
+      case 4:
         taskOrder++;
         #if GPS
           if (GPS_Compute() != 0) break;  // performs computation on new frame only if present
@@ -1264,11 +1311,9 @@ void loop () {
           if (GPS_NewData() != 0) break;  // 160 us with no new data / much more with new data 
           #endif
         #endif
-      case 4:
+      case 5:
         taskOrder=0;
-        #if SONAR
-          Sonar_update(); //debug[2] = sonarAlt;
-        #endif
+    
         #ifdef LANDING_LIGHTS_DDR
           auto_switch_landing_lights();
         #endif
diff --git a/MultiWii.h b/MultiWii.h
index 27d68f4..e72863d 100644
--- a/MultiWii.h
+++ b/MultiWii.h
@@ -21,6 +21,9 @@ extern uint16_t cycleTime;
 extern uint16_t calibratingA;
 extern uint16_t calibratingB;
 extern uint16_t calibratingG;
+#if SONAR
+extern uint16_t calibratingS;
+#endif
 extern int16_t  magHold,headFreeModeHold;
 extern uint8_t  vbatMin;
 extern uint8_t  rcOptions[CHECKBOXITEMS];
@@ -232,4 +235,4 @@ extern uint16_t AccInflightCalibrationActive;
 
 void annexCode();
 void go_disarm();
-#endif /* MULTIWII_H_ */
+#endif /* MULTIWII_H_ */
diff --git a/Protocol.cpp b/Protocol.cpp
index e10ee05..5f9c787 100644
--- a/Protocol.cpp
+++ b/Protocol.cpp
@@ -451,6 +451,9 @@ void evaluateCommand(uint8_t c) {
       #if defined(OSD_SWITCH)
         if(rcOptions[BOXOSD]) tmp |= 1<<BOXOSD;
       #endif
+      #if SONAR
+    if (f.SONAR_MODE) tmp |= 1 << BOXSONAR;
+      #endif
       if(f.ARMED) tmp |= 1<<BOXARM;
       st.flag             = tmp;
       st.set              = global_conf.currentSet;
@@ -842,4 +845,4 @@ static void debugmsg_serialize(uint8_t l) {
 }
 #else
 void debugmsg_append_str(const char *str) {};
-#endif
+#endif
diff --git a/Sensors.cpp b/Sensors.cpp
index 88ea67a..e4ece2c 100644
--- a/Sensors.cpp
+++ b/Sensors.cpp
@@ -1511,9 +1511,47 @@ void Sonar_update() {
   sonarAlt = srf08_ctx.range[0]; // only one sensor considered for the moment
 }
 #else
+#if defined(SONAR_GENERIC_ECHOPULSE)
+// ************************************************************************************************************
+// Generic Sonar Support
+// ************************************************************************************************************
+volatile unsigned long SONAR_GEP_startTime = 0;
+volatile unsigned long SONAR_GEP_echoTime = 0;
+volatile static int32_t  tempSonarAlt = 0;
+
+void Sonar_init() {
+  SONAR_GEP_EchoPin_PCICR;
+  SONAR_GEP_EchoPin_PCMSK;
+  SONAR_GEP_EchoPin_PINMODE_IN;
+  SONAR_GEP_TriggerPin_PINMODE_OUT;
+}
+
+void Sonar_update() {
+  sonarAlt = 1 + tempSonarAlt;
+  SONAR_GEP_TriggerPin_PIN_LOW;
+  delayMicroseconds(2);
+  SONAR_GEP_TriggerPin_PIN_HIGH;
+  delayMicroseconds(10);
+  SONAR_GEP_TriggerPin_PIN_LOW;
+}
+
+ISR(SONAR_GEP_EchoPin_PCINT_vect) {
+  if (SONAR_GEP_EchoPin_PIN & (1 << SONAR_GEP_EchoPin_PCINT)) {
+    SONAR_GEP_startTime = micros();
+  }
+  else {
+    SONAR_GEP_echoTime = micros() - SONAR_GEP_startTime;
+    if (SONAR_GEP_echoTime <= SONAR_GENERIC_MAX_RANGE*SONAR_GENERIC_SCALE)
+      tempSonarAlt = SONAR_GEP_echoTime / SONAR_GENERIC_SCALE;
+    else
+      tempSonarAlt = -1;
+  }
+}
+#else
 inline void Sonar_init() {}
 void Sonar_update() {}
 #endif
+#endif
 
 
 void initS() {
@@ -1536,4 +1574,4 @@ void initSensors() {
     initS();
     if (i2c_errors_count == 0) break; // no error during init => init ok
   }
-}
+}
diff --git a/config.h b/config.h
index dffdfa1..3281b46 100644
--- a/config.h
+++ b/config.h
@@ -36,7 +36,7 @@
     //#define BI
     //#define TRI
     //#define QUADP
-    //#define QUADX
+    #define QUADX
     //#define Y4
     //#define Y6
     //#define HEX6
@@ -72,8 +72,8 @@
     #define MINCOMMAND  1000
 
   /**********************************  I2C speed for old WMP config (useless config for other sensors)  *************/
-    #define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
-    //#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones
+    //#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
+    #define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones
 
   /***************************    Internal i2c Pullups   ********************************/
     /* enable internal I2C pull ups (in most cases it is better to use external pullups) */
@@ -127,6 +127,7 @@
       //#define CRIUS_LITE      // Crius MultiWii Lite
       //#define CRIUS_SE        // Crius MultiWii SE
       //#define CRIUS_SE_v2_0   // Crius MultiWii SE 2.0 with MPU6050, HMC5883 and BMP085
+      //#define RCTIMER_CRIUS_SE_v2_0 // RCTimer CRIUS v2 AIOP with MPU6050, HMC5883 and MS561101BA
       //#define OPENLRSv2MULTI  // OpenLRS v2 Multi Rc Receiver board including ITG3205 and ADXL345
       //#define BOARD_PROTO_1   // with MPU6050 + HMC5883L + MS baro
       //#define BOARD_PROTO_2   // with MPU6050 + slave  MAG3110 + MS baro
@@ -169,7 +170,7 @@
       //#define ITG3200
       //#define MPU3050
       //#define L3G4200D
-      //#define MPU6050       //combo + ACC
+      #define MPU6050       //combo + ACC
       //#define LSM330        //combo + ACC
       
       /* I2C accelerometer */
@@ -202,8 +203,8 @@
       //#define ADCACC
 
       /* enforce your individual sensor orientation - even overrides board specific defaults */
-      //#define FORCE_ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  =  Y; imu.accADC[PITCH]  = -X; imu.accADC[YAW]  = Z;}
-      //#define FORCE_GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = -Y; imu.gyroADC[PITCH] =  X; imu.gyroADC[YAW] = Z;}
+      #define FORCE_ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  =  Y; imu.accADC[PITCH]  = -X; imu.accADC[YAW]  = Z;}
+      #define FORCE_GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X; imu.gyroADC[PITCH] =  Y; imu.gyroADC[YAW] = -Z;}
       //#define FORCE_MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = Z;}
 
       /* Board orientation shift */
@@ -522,7 +523,7 @@ At this moment you can use this function only with WinGUI 2.3 release. MultiWiiC
       //#define GYRO_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
       //#define GYRO_LPF_188HZ
       //#define GYRO_LPF_98HZ
-      //#define GYRO_LPF_42HZ
+      #define GYRO_LPF_42HZ
       //#define GYRO_LPF_20HZ
       //#define GYRO_LPF_10HZ
       //#define GYRO_LPF_5HZ       // Use this only in extreme cases, rather change motors and/or props -- setting not available on ITG3200
@@ -552,13 +553,13 @@ At this moment you can use this function only with WinGUI 2.3 release. MultiWiiC
 
   /************************        Reset Baro altitude on arm         ********************/
   /* When unchecked a calibration of the baro altitude is preformed every time arming is activated */
-  //#define ALTITUDE_RESET_ON_ARM
+  #define ALTITUDE_RESET_ON_ARM
 
   /************************        Angele throttle correction         ********************/
   /* Automatically increase throttle based on the angle of the copter
      Original idea by Kraut Rob, first implementation HAdrian */
 
-  //#define THROTTLE_ANGLE_CORRECTION 40
+  #define THROTTLE_ANGLE_CORRECTION 40
   
   /*** HEADFREE : the copter can be controled by an absolute stick orientation, whatever the yaw orientation ***/
   //#define HEADFREE
@@ -597,7 +598,7 @@ At this moment you can use this function only with WinGUI 2.3 release. MultiWiiC
        PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THROTTLE value. You must set this value to descending about 1m/s or so
        for best results. This value is depended from your configuration, AUW and some other params.  Next, after FAILSAFE_OFF_DELAY the copter is disarmed, 
        and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
-    //#define FAILSAFE                                // uncomment  to activate the failsafe function
+    #define FAILSAFE                                // uncomment  to activate the failsafe function
     #define FAILSAFE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
     #define FAILSAFE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
     #define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case
@@ -1227,5 +1228,26 @@ Also note, that maqgnetic declination changes with time, so recheck your value e
 /****           END OF CONFIGURABLE PARAMETERS                                                ****/
 /*************************************************************************************************/
 
+// SONAR!! http://www.multiwii.com/forum/viewtopic.php?f=7&t=1033&start=170#p36603
+/* Generic sonar: hc-sr04, srf04, dyp-me007, all generic sonar with echo/pulse pin
+default pulse is PH6/8, echo is PB4/7
+*/
+#define SONAR_GENERIC_ECHOPULSE 
+#define SONAR_GENERIC_SCALE 58        //scale for ranging conversion (hcsr04 is 58)
+#define SONAR_GENERIC_MAX_RANGE 500     //cm (could be more)
+#define SONAR_GENERIC_TRIGGER_PIN 8    // Arduino Nano D8
+#define SONAR_GENERIC_ECHO_PIN 7     // Arduino Nano D7
+
+/************************* Sonar alt hold / precision / ground collision keeper *******/
+#define SONAR_MAX_HOLD 400          //cm, kind of error delimiter, for now to avoid rocket climbing, only usefull if no baro
+
+//if using baro + sonar       
+#define SONAR_BARO_FUSION_LC 100      //cm, baro/sonar readings fusion, low cut, below = full sonar
+#define SONAR_BARO_FUSION_HC SONAR_MAX_HOLD //cm, baro/sonar readings fusion, high cut, above = full baro
+#define SONAR_BARO_FUSION_RATIO 0.0     //0.0-1.0,  baro/sonar readings fusion, amount of each sensor value, 0 = proportionnel between LC and HC
+#define SONAR_BARO_LPF_LC 0.9f 
+#define SONAR_BARO_LPF_HC 0.9f
+#pragma endregion
+
 #endif /* CONFIG_H_ */
-
+
diff --git a/def.h b/def.h
index 04a93dc..10ea11b 100644
--- a/def.h
+++ b/def.h
@@ -1268,6 +1268,17 @@
   #define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;}
 #endif
 
+#if defined(RCTIMER_CRIUS_SE_v2_0)
+#define MPU6050
+#define HMC5883
+#define MS561101BA
+#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
+#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
+#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;}
+
+//#define MS561101BA_ADDRESS 0x76
+#endif
+
 #if defined(BOARD_PROTO_1)
   #define MPU6050
   #define HMC5883
@@ -2029,4 +2040,20 @@
   #error "you use one feature that is no longer supported or has undergone a name change"
 #endif
 
-#endif /* DEF_H_ */
+#pragma region GENERIC SONAR SUPPORT
+#if defined(SONAR_GENERIC_ECHOPULSE)
+#define SONAR_GEP_TriggerPin             SONAR_GENERIC_TRIGGER_PIN
+#define SONAR_GEP_TriggerPin_PINMODE_OUT pinMode(SONAR_GEP_TriggerPin,OUTPUT);
+#define SONAR_GEP_TriggerPin_PIN_HIGH    PORTB |= 1<<6;
+#define SONAR_GEP_TriggerPin_PIN_LOW     PORTB &= ~(1<<6);
+#define SONAR_GEP_EchoPin                SONAR_GENERIC_ECHO_PIN
+#define SONAR_GEP_EchoPin_PINMODE_IN     pinMode(SONAR_GEP_EchoPin,INPUT);
+#define SONAR_GEP_EchoPin_PCINT          PCINT5
+#define SONAR_GEP_EchoPin_PCICR          PCICR |= (1<<PCIE0); // PCINT 0-7 belong to PCIE0
+#define SONAR_GEP_EchoPin_PCMSK          PCMSK0 = (1<<SONAR_GEP_EchoPin_PCINT); // Mask Pin PCINT5 - all other PIns PCINT0-7 are not allowed to create interrupts!
+#define SONAR_GEP_EchoPin_PCINT_vect     PCINT0_vect  // PCINT0-7 belog PCINT0_vect
+#define SONAR_GEP_EchoPin_PIN            PINB  // PCINT0-7 belong to PINB
+#endif
+#pragma endregion
+
+#endif /* DEF_H_ */
diff --git a/types.h b/types.h
index b7a5b3d..ff97301 100644
--- a/types.h
+++ b/types.h
@@ -83,6 +83,9 @@ enum box {
     BOXGPSNAV,
     BOXLAND,
   #endif
+  #if SONAR
+  BOXSONAR,
+  #endif
   CHECKBOXITEMS
 };
 
@@ -146,6 +149,9 @@ typedef struct {
   uint8_t LAND_COMPLETED: 1;
   uint8_t LAND_IN_PROGRESS: 1;
 #endif
+#if SONAR
+  uint8_t SONAR_MODE : 1;
+#endif
 } flags_struct_t;
 
 typedef struct {
@@ -333,4 +339,4 @@ typedef struct {
 
 #endif
 
-#endif /* TYPES_H_ */
+#endif /* TYPES_H_ */
```
## 3. Check flight control operation with MultiWiiConf util
![Flight Control Operation](https://nanishin.github.io/assets/20180512_223419.png)
![MultiWiiConf Util](https://nanishin.github.io/assets/20180512_230425.png)

## 4. Tada! Fly diy arduino drone at outdoor :D
<iframe width="560" height="315" src="https://www.youtube.com/embed/v8UkYDHHhgw" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## 5. FAQ (Frequently Asked Question)
### Melted power switch
- Power dispatcher board oftenly got a lot of heat by high current during flying. So it can melt power switch easily.
- In this time, you can fix it by soldering of switch circuit.
![Soldering switch circuit](https://nanishin.github.io/assets/20180805_232816.png)
### Out of balaance at take-off time
- Sometimes you can find totaly out of balance at take-off time. At that time, if a weird operation of specific propeller is found, then it's maybe caused by broken ESC.
- To fix it, you need enough ESC parts for replacement.
![Broken ESC](https://nanishin.github.io/assets/20180513_165724.png)
