#pragma once

#include <Arduino.h>

enum ServoName : uint8_t {
  R1 = 0, 
  R2 = 1,
  L1 = 2,
  L2 = 3,
  R4 = 4,
  R3 = 5,
  L3 = 6,
  L4 = 7
};

const char* const ServoNames[]={"R1","R2","L1","L2","R4","R3","L3","L4"};

inline int servoNameToIndex(const String& servo) {
  if (servo == "L1") return L1;
  if (servo == "L2") return L2;
  if (servo == "L3") return L3;
  if (servo == "L4") return L4;
  if (servo == "R1") return R1;
  if (servo == "R2") return R2;
  if (servo == "R3") return R3;
  if (servo == "R4") return R4;
  return -1;
}

enum FaceAnimMode : uint8_t {
  FACE_ANIM_LOOP = 0,
  FACE_ANIM_ONCE = 1,
  FACE_ANIM_BOOMERANG = 2
};

// External globals and helpers used by movement/pose sequences
extern int frameDelay;
extern int walkCycles;
extern String currentCommand;

extern void setServoAngle(uint8_t channel, int angle);
#if USE_PCA9685
extern void flushServoFrame();
extern void setFrame(int r1, int r2, int l1, int l2, int r4, int r3, int l3, int l4);
#endif
extern void setFace(const String& faceName);
extern void setFaceMode(FaceAnimMode mode);
extern void setFaceWithMode(const String& faceName, FaceAnimMode mode);
extern void delayWithFace(unsigned long ms);
extern void enterIdle();
extern bool pressingCheck(const String& cmd, int ms);

// Pose/animation prototypes
void runRestPose();
void runStandPose(int face = 1);
void runWavePose();
void runDancePose();
void runSwimPose();
void runPointPose();
void runPushupPose();
void runBowPose();
void runCutePose();
void runFreakyPose();
void runWormPose();
void runShakePose();
void runShrugPose();
void runDeadPose();
void runCrabPose();
void runWalkPose();
void runWalkBackward();
void runTurnLeft();
void runTurnRight();

// ====== POSES ======
inline void runRestPose() { 
  DBG_PRINTLN(F("REST")); 
  setFaceWithMode("rest", FACE_ANIM_BOOMERANG); 
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(       90,  90,  90,  90,  90,  90,  90,  90);
#else
  for (int i = 0; i < 8; i++) setServoAngle(i, 90); 
#endif
}

inline void runStandPose(int face) { 
  DBG_PRINTLN(F("STAND")); 
  if (face == 1) setFaceWithMode("stand", FACE_ANIM_ONCE); 
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      135,  45,  45, 135,   0, 180,   0, 180);
#else
  setServoAngle(R1, 135); 
  setServoAngle(R2, 45); 
  setServoAngle(L1, 45); 
  setServoAngle(L2, 135); 
  setServoAngle(R4, 0); 
  setServoAngle(R3, 180); 
  setServoAngle(L3, 0); 
  setServoAngle(L4, 180); 
#endif
  if (face == 1) enterIdle();
}

inline void runWavePose() { 
  DBG_PRINTLN(F("WAVE")); 
  setFaceWithMode("wave", FACE_ANIM_ONCE); 
  runStandPose(0); 
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      100,  45,  45,  60,  80, 180, 180, 180);
  delayWithFace(200);
  setFrame(      100,  45,  45,  60,  80, 180, 180, 180);
  delayWithFace(300); 
  for (int i = 0; i < 4; i++) { 
    setFrame(    100,  45,  45,  60,  80, 180, 180, 180);
    delayWithFace(300); 
    setFrame(    100,  45,  45,  60,  80, 180, 100, 180);
    delayWithFace(300); 
  } 
#else
  setServoAngle(R4, 80); setServoAngle(L3, 180); 
  setServoAngle(L2, 60); setServoAngle(R1, 100); 
  delayWithFace(200);
  setServoAngle(L3, 180); 
  delayWithFace(300); 
  for (int i = 0; i < 4; i++) { 
    setServoAngle(L3, 180); delayWithFace(300); 
    setServoAngle(L3, 100); delayWithFace(300); 
  } 
#endif
  runStandPose(1); 
  if (currentCommand == "wave") currentCommand = "";
}

inline void runDancePose() { 
  DBG_PRINTLN(F("DANCE")); 
  setFaceWithMode("dance", FACE_ANIM_LOOP); 
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(       90,  90,  90,  90, 160, 160,  10,  10);
  delayWithFace(300); 
  for (int i = 0; i < 5; i++) { 
    setFrame(     90,  90,  90,  90, 115, 115,  10,  10);
    delayWithFace(300); 
    setFrame(     90,  90,  90,  90, 160, 160,  65,  65);
    delayWithFace(300); 
  } 
#else
  setServoAngle(R1, 90); setServoAngle(R2, 90); 
  setServoAngle(L1, 90); setServoAngle(L2, 90); 
  setServoAngle(R4, 160); setServoAngle(R3, 160); 
  setServoAngle(L3, 10); setServoAngle(L4, 10); 
  delayWithFace(300); 
  for (int i = 0; i < 5; i++) { 
    setServoAngle(R4, 115); setServoAngle(R3, 115); 
    setServoAngle(L3, 10); setServoAngle(L4, 10); 
    delayWithFace(300); 
    setServoAngle(R4, 160); setServoAngle(R3, 160); 
    setServoAngle(L3, 65); setServoAngle(L4, 65); 
    delayWithFace(300); 
  } 
#endif
  runStandPose(1); 
  if (currentCommand == "dance") currentCommand = "";
}

inline void runSwimPose() { 
  DBG_PRINTLN(F("SWIM")); 
  setFaceWithMode("swim", FACE_ANIM_ONCE); 
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(       90,  90,  90,  90,  90,  90,  90,  90);
  for (int i = 0; i < 4; i++) { 
    setFrame(    135,  45,  45, 135,  90,  90,  90,  90);
    delayWithFace(400); 
    setFrame(     90,  90,  90,  90,  90,  90,  90,  90);
    delayWithFace(400); 
  } 
#else
  for (int i = 0; i < 8; i++) setServoAngle(i, 90); 
  for (int i = 0; i < 4; i++) { 
    setServoAngle(R1, 135); setServoAngle(R2, 45); 
    setServoAngle(L1, 45); setServoAngle(L2, 135); 
    delayWithFace(400); 
    setServoAngle(R1, 90); setServoAngle(R2, 90); 
    setServoAngle(L1, 90); setServoAngle(L2, 90); 
    delayWithFace(400); 
  } 
#endif
  runStandPose(1); 
  if (currentCommand == "swim") currentCommand = "";
}

inline void runPointPose() { 
  DBG_PRINTLN(F("POINT")); 
  setFaceWithMode("point", FACE_ANIM_BOOMERANG); 
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      135, 100,  25,  60,  80, 170, 145, 180);
#else
  setServoAngle(L2, 60); setServoAngle(R1, 135); 
  setServoAngle(R2, 100); setServoAngle(L4, 180); 
  setServoAngle(L1, 25); setServoAngle(L3, 145);
  setServoAngle(R4, 80); setServoAngle(R3, 170); 
#endif
  delayWithFace(2000); 
  runStandPose(1); 
  if (currentCommand == "point") currentCommand = "";
}

inline void runPushupPose() {
  DBG_PRINTLN(F("PUSHUP"));
  setFaceWithMode("pushup", FACE_ANIM_ONCE);
  runStandPose(0); 
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      180,  45,   0, 135,   0,  90,  90, 180);
  delayWithFace(500);
  for (int i = 0; i < 4; i++) {
    setFrame(    180,  45,   0, 135,   0, 180,   0, 180);
    delayWithFace(600);
    setFrame(    180,  45,   0, 135,   0,  90,  90, 180);
    delayWithFace(500);
  }
#else
  setServoAngle(L1, 0);
  setServoAngle(R1, 180);
  setServoAngle(L3, 90);
  setServoAngle(R3, 90);
  delayWithFace(500);
  for (int i = 0; i < 4; i++) {
    setServoAngle(L3, 0);
    setServoAngle(R3, 180);
    delayWithFace(600);
    setServoAngle(L3, 90);
    setServoAngle(R3, 90);
    delayWithFace(500);
  }
#endif
  runStandPose(1);
  if (currentCommand == "pushup") currentCommand = "";
}

inline void runBowPose() {
  DBG_PRINTLN(F("BOW"));
  setFaceWithMode("bow", FACE_ANIM_ONCE);
  runStandPose(0); 
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      180,   0,   0, 180,   0, 180,   0, 180);
  delayWithFace(600);
  setFrame(      180,   0,   0, 180,   0,  90,  90, 180);
  delayWithFace(3000);
#else
  setServoAngle(L1, 0);
  setServoAngle(R1, 180);
  setServoAngle(L3, 0);
  setServoAngle(R3, 180);
  setServoAngle(L2, 180);
  setServoAngle(R2, 0);
  setServoAngle(R4, 0);
  setServoAngle(L4, 180);
  delayWithFace(600);
  setServoAngle(L3, 90);
  setServoAngle(R3, 90);
  delayWithFace(3000);
#endif
  runStandPose(1);
  if (currentCommand == "bow") currentCommand = "";
}

inline void runCutePose() {
  DBG_PRINTLN(F("CUTE"));
  setFaceWithMode("cute", FACE_ANIM_ONCE);
  runStandPose(0); 
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      180,  20,   0, 160, 180,   0, 180,   0);
  delayWithFace(200);
  for (int i = 0; i < 5; i++) {
    setFrame(    180,  20,   0, 160, 180,   0, 180,  45);
    delayWithFace(300);
    setFrame(    180,  20,   0, 160, 135,   0, 180,   0);
    delayWithFace(300);
  }
#else
  setServoAngle(L2, 160);
  setServoAngle(R2, 20);
  setServoAngle(R4, 180);
  setServoAngle(L4, 0);

  setServoAngle(L1, 0);
  setServoAngle(R1, 180);
  setServoAngle(L3, 180);
  setServoAngle(R3, 0);
  delayWithFace(200);
  for (int i = 0; i < 5; i++) {
    setServoAngle(R4, 180);
    setServoAngle(L4, 45);
    delayWithFace(300);
    setServoAngle(R4, 135);
    setServoAngle(L4, 0);
    delayWithFace(300);
  }
#endif
  runStandPose(1);
  if (currentCommand == "cute") currentCommand = "";
}

inline void runFreakyPose() {
  DBG_PRINTLN(F("FREAKY"));
  setFaceWithMode("freaky", FACE_ANIM_ONCE);
  runStandPose(0); 
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      180,   0,   0, 180,  90,   0,   0, 180);
  delayWithFace(200);
  for (int i = 0; i < 3; i++) {
    setFrame(    180,   0,   0, 180,  90,  25,   0, 180);
    delayWithFace(400);
    setFrame(    180,   0,   0, 180,  90,   0,   0, 180);
    delayWithFace(400);
  }
#else
  setServoAngle(L1, 0);
  setServoAngle(R1, 180);
  setServoAngle(L2, 180);
  setServoAngle(R2, 0);
  setServoAngle(R4, 90);
  setServoAngle(R3, 0);
  delayWithFace(200);
  for (int i = 0; i < 3; i++) {
    setServoAngle(R3, 25);
    delayWithFace(400);
    setServoAngle(R3, 0);
    delayWithFace(400);
  }
#endif
  runStandPose(1);
  if (currentCommand == "freaky") currentCommand = "";
}

inline void runWormPose() {
  DBG_PRINTLN(F("WORM"));
  setFaceWithMode("worm", FACE_ANIM_ONCE);
  runStandPose(0);
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      180,   0,   0, 180,  90,  90,  90,  90);
  delayWithFace(200);
  for(int i=0; i<5; i++) {
    setFrame(    180,   0,   0, 180,  45,  45, 135, 135);
    delayWithFace(300);
    setFrame(    180,   0,   0, 180, 135, 135,  45,  45);
    delayWithFace(300);
  }
#else
  setServoAngle(R1, 180); setServoAngle(R2, 0); setServoAngle(L1, 0); setServoAngle(L2, 180);
  setServoAngle(R4, 90); setServoAngle(R3, 90); setServoAngle(L3, 90); setServoAngle(L4, 90);
  delayWithFace(200);
  for(int i=0; i<5; i++) {
    setServoAngle(R3, 45); setServoAngle(L3, 135); setServoAngle(R4, 45); setServoAngle(L4, 135);
    delayWithFace(300);
    setServoAngle(R3, 135); setServoAngle(L3, 45); setServoAngle(R4, 135); setServoAngle(L4, 45);
    delayWithFace(300);
  }
#endif
  runStandPose(1);
  if (currentCommand == "worm") currentCommand = "";
}

inline void runShakePose() {
  DBG_PRINTLN(F("SHAKE"));
  setFaceWithMode("shake", FACE_ANIM_ONCE);
  runStandPose(0);
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      135,  90,  45,  90,   0,  90,  90, 180);
  delayWithFace(200);
  for(int i=0; i<5; i++) {
    setFrame(    135,  90,  45,  90,  45,  90,  90, 135);
    delayWithFace(300);
    setFrame(    135,  90,  45,  90,   0,  90,  90, 180);
    delayWithFace(300);
  }
#else
  setServoAngle(R1, 135); setServoAngle(L1, 45); setServoAngle(L3, 90); setServoAngle(R3, 90);
  setServoAngle(L2, 90); setServoAngle(R2, 90);
  delayWithFace(200);
  for(int i=0; i<5; i++) {
    setServoAngle(R4, 45); setServoAngle(L4, 135);
    delayWithFace(300);
    setServoAngle(R4, 0); setServoAngle(L4, 180);
    delayWithFace(300);
  }
#endif
  runStandPose(1);
  if (currentCommand == "shake") currentCommand = "";
}

inline void runShrugPose() {
  DBG_PRINTLN(F("SHRUG"));
  runStandPose(0);
  setFaceWithMode("dead", FACE_ANIM_ONCE);
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      135,  45,  45, 135,  90,  90,  90,  90);
  delayWithFace(1000);
  setFaceWithMode("shrug", FACE_ANIM_ONCE);
  setFrame(      135,  45,  45, 135, 180,   0, 180,   0);
  delayWithFace(1500);
#else
  setServoAngle(R3, 90); setServoAngle(R4, 90); setServoAngle(L3, 90); setServoAngle(L4, 90);
  delayWithFace(1000);
  setFaceWithMode("shrug", FACE_ANIM_ONCE);
  setServoAngle(R3, 0); setServoAngle(R4, 180); setServoAngle(L3, 180); setServoAngle(L4, 0);
  delayWithFace(1500);
#endif
  runStandPose(1);
  if (currentCommand == "shrug") currentCommand = "";
}

inline void runDeadPose() {
  DBG_PRINTLN(F("DEAD"));
  runStandPose(0);
  setFaceWithMode("dead", FACE_ANIM_BOOMERANG);
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      135,  45,  45, 135,  90,  90,  90,  90);
#else
  setServoAngle(R3, 90); setServoAngle(R4, 90); setServoAngle(L3, 90); setServoAngle(L4, 90);
#endif
  if (currentCommand == "dead") currentCommand = "";
}

inline void runCrabPose() {
  DBG_PRINTLN(F("CRAB"));
  setFaceWithMode("crab", FACE_ANIM_ONCE);
  runStandPose(0);
  delayWithFace(200);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(       90,  90,  90,  90,   0, 180,  45, 135);
  for(int i=0; i<5; i++) {
    setFrame(     90,  90,  90,  90,  45, 135,   0, 180);
    delayWithFace(300);
    setFrame(     90,  90,  90,  90,   0, 180,  45, 135);
    delayWithFace(300);
  }
#else
  setServoAngle(R1, 90); setServoAngle(R2, 90); setServoAngle(L1, 90); setServoAngle(L2, 90);
  setServoAngle(R4, 0); setServoAngle(R3, 180); setServoAngle(L3, 45); setServoAngle(L4, 135);
  for(int i=0; i<5; i++) {
    setServoAngle(R4, 45); setServoAngle(R3, 135); setServoAngle(L3, 0); setServoAngle(L4, 180);
    delayWithFace(300);
    setServoAngle(R4, 0); setServoAngle(R3, 180); setServoAngle(L3, 45); setServoAngle(L4, 135);
    delayWithFace(300);
  }
#endif
  runStandPose(1);
  if (currentCommand == "crab") currentCommand = "";
}

// --- MOVEMENT ANIMATIONS ---
inline void runWalkPose() {
  DBG_PRINTLN(F("WALK FWD"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
#if USE_PCA9685
  //              R1   R2   L1   L2   R4   R3   L3   L4
  setFrame(      135, 100,  25, 135,   0, 135,  45, 180);  // Initial step
  if (!pressingCheck("forward", frameDelay)) return;

  for (int i = 0; i < walkCycles; i++) {
    setFrame(    135, 100,  25, 135,   0, 135,   0, 180);
    if (!pressingCheck("forward", frameDelay)) return;
    setFrame(    180, 100,  25,  90,   0, 135,   0, 135);
    if (!pressingCheck("forward", frameDelay)) return;
    setFrame(    180,  45,  90,  90,   0, 135,   0, 135);
    if (!pressingCheck("forward", frameDelay)) return;
    setFrame(    180,  45,  90,  90,  45, 135,   0, 180);
    if (!pressingCheck("forward", frameDelay)) return;
    setFrame(    180,  90,   0,  90,  45, 180,  45, 180);
    if (!pressingCheck("forward", frameDelay)) return;
    setFrame(     90,  90,   0, 135,  45, 180,  45, 180);
    if (!pressingCheck("forward", frameDelay)) return;
  }
#else
  // Initial Step
  setServoAngle(R3, 135); setServoAngle(L3, 45);
  setServoAngle(R2, 100); setServoAngle(L1, 25);
  if (!pressingCheck("forward", frameDelay)) return;
  
  for (int i = 0; i < walkCycles; i++) {
    setServoAngle(R3, 135); setServoAngle(L3, 0);
    if (!pressingCheck("forward", frameDelay)) return;
    setServoAngle(L4, 135); setServoAngle(L2, 90);
    setServoAngle(R4, 0); setServoAngle(R1, 180);
    if (!pressingCheck("forward", frameDelay)) return;    
    setServoAngle(R2, 45); setServoAngle(L1, 90);
    if (!pressingCheck("forward", frameDelay)) return;
    setServoAngle(R4, 45); setServoAngle(L4, 180);
    if (!pressingCheck("forward", frameDelay)) return;
    setServoAngle(R3, 180); setServoAngle(L3, 45);
    setServoAngle(R2, 90); setServoAngle(L1, 0);
    if (!pressingCheck("forward", frameDelay)) return;  
    setServoAngle(L2, 135); setServoAngle(R1, 90);
    if (!pressingCheck("forward", frameDelay)) return;
  }
#endif
  runStandPose(1);
}

// Logic reversed from Walk
inline void runWalkBackward() {
  DBG_PRINTLN(F("WALK BACK"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
#if USE_PCA9685
  if (!pressingCheck("backward", frameDelay)) return;

  for (int i = 0; i < walkCycles; i++) {
    //              R1   R2   L1   L2   R4   R3   L3   L4
    setFrame(    135,  45,  45, 135,   0, 135,   0, 180);
    if (!pressingCheck("backward", frameDelay)) return;
    setFrame(     90,  45,  45, 135,   0, 135,   0, 135);
    if (!pressingCheck("backward", frameDelay)) return;
    setFrame(     90,  90,   0, 135,   0, 135,   0, 135);
    if (!pressingCheck("backward", frameDelay)) return;
    setFrame(     90,  90,   0, 135,  45, 135,   0, 180);
    if (!pressingCheck("backward", frameDelay)) return;
    setFrame(     90,  45,  90, 135,  45, 180,  45, 180);
    if (!pressingCheck("backward", frameDelay)) return;
    setFrame(    180,  45,  90,  90,  45, 180,  45, 180);
    if (!pressingCheck("backward", frameDelay)) return;
  }
#else
  if (!pressingCheck("backward", frameDelay)) return;
  
  for (int i = 0; i < walkCycles; i++) {
    setServoAngle(R3, 135); setServoAngle(L3, 0);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(L4, 135); setServoAngle(L2, 135);
    setServoAngle(R4, 0); setServoAngle(R1, 90);
    if (!pressingCheck("backward", frameDelay)) return;    
    setServoAngle(R2, 90); setServoAngle(L1, 0);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(R4, 45); setServoAngle(L4, 180);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(R3, 180); setServoAngle(L3, 45);
    setServoAngle(R2, 45); setServoAngle(L1, 90);
    if (!pressingCheck("backward", frameDelay)) return;  
    setServoAngle(L2, 90); setServoAngle(R1, 180);
    if (!pressingCheck("backward", frameDelay)) return;
  }
#endif
  runStandPose(1);
}

// Simple turn logic
inline void runTurnLeft() {
  DBG_PRINTLN(F("TURN LEFT"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  for (int i = 0; i < walkCycles; i++) {
#if USE_PCA9685
    //              R1   R2   L1   L2   R4   R3   L3   L4
    // legset 1 (R1 L2) — lift
    setFrame(    135,  45,  45, 135,   0, 135,   0, 135);
    if (!pressingCheck("left", frameDelay)) return;
    // legset 1 — swing
    setFrame(    180,  45,  45, 180,   0, 135,   0, 135);
    if (!pressingCheck("left", frameDelay)) return;
    // legset 1 — plant
    setFrame(    180,  45,  45, 180,   0, 180,   0, 180);
    if (!pressingCheck("left", frameDelay)) return;
    // legset 1 — push
    setFrame(    135,  45,  45, 135,   0, 180,   0, 180);
    if (!pressingCheck("left", frameDelay)) return;
    // legset 2 (R2 L1) — lift
    setFrame(    135,  45,  45, 135,  45, 180,  45, 180);
    if (!pressingCheck("left", frameDelay)) return;
    // legset 2 — swing
    setFrame(    135,  90,  90, 135,  45, 180,  45, 180);
    if (!pressingCheck("left", frameDelay)) return;
    // legset 2 — plant
    setFrame(    135,  90,  90, 135,   0, 180,   0, 180);
    if (!pressingCheck("left", frameDelay)) return;
    // legset 2 — push
    setFrame(    135,  45,  45, 135,   0, 180,   0, 180);
    if (!pressingCheck("left", frameDelay)) return;
#else
    //legset 1 (R1 L2)
    setServoAngle(R3, 135); setServoAngle(L4, 135); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R1, 180); setServoAngle(L2, 180); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R3, 180); setServoAngle(L4, 180); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R1, 135); setServoAngle(L2, 135);
    if (!pressingCheck("left", frameDelay)) return;
      //legset 2 (R2 L1)
    setServoAngle(R4, 45); setServoAngle(L3, 45); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R2, 90); setServoAngle(L1, 90); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R4, 0); setServoAngle(L3, 0); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R2, 45); setServoAngle(L1, 45);
    if (!pressingCheck("left", frameDelay)) return;  
#endif
  }
  runStandPose(1);
}

inline void runTurnRight() {
  DBG_PRINTLN(F("TURN RIGHT"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  for (int i = 0; i < walkCycles; i++) {
#if USE_PCA9685
    //              R1   R2   L1   L2   R4   R3   L3   L4
    // legset 2 (R2 L1) — lift
    setFrame(    135,  45,  45, 135,  45, 180,  45, 180);
    if (!pressingCheck("right", frameDelay)) return;
    // legset 2 — swing
    setFrame(    135,   0,   0, 135,  45, 180,  45, 180);
    if (!pressingCheck("right", frameDelay)) return;
    // legset 2 — plant
    setFrame(    135,   0,   0, 135,   0, 180,   0, 180);
    if (!pressingCheck("right", frameDelay)) return;
    // legset 2 — push
    setFrame(    135,  45,  45, 135,   0, 180,   0, 180);
    if (!pressingCheck("right", frameDelay)) return;
    // legset 1 (R1 L2) — lift
    setFrame(    135,  45,  45, 135,   0, 135,   0, 135);
    if (!pressingCheck("right", frameDelay)) return;
    // legset 1 — swing
    setFrame(     90,  45,  45,  90,   0, 135,   0, 135);
    if (!pressingCheck("right", frameDelay)) return;
    // legset 1 — plant
    setFrame(     90,  45,  45,  90,   0, 180,   0, 180);
    if (!pressingCheck("right", frameDelay)) return;
    // legset 1 — push
    setFrame(    135,  45,  45, 135,   0, 180,   0, 180);
    if (!pressingCheck("right", frameDelay)) return;
#else
    //legset 2 (R2 L1)
    setServoAngle(R4, 45); setServoAngle(L3, 45); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R2, 0); setServoAngle(L1, 0); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R4, 0); setServoAngle(L3, 0); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R2, 45); setServoAngle(L1, 45);
    if (!pressingCheck("right", frameDelay)) return;  
    //legset 1 (R1 L2)
    setServoAngle(R3, 135); setServoAngle(L4, 135); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R1, 90); setServoAngle(L2, 90); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R3, 180); setServoAngle(L4, 180); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R1, 135); setServoAngle(L2, 135);
    if (!pressingCheck("right", frameDelay)) return;
#endif
  }
  runStandPose(1);
}
