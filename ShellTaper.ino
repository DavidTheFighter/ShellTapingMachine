#define SHELLCONFIG_MAX_ANGLES 4
#define NUM_SHELL_CONFIGS 6 // The number of available shell configs as well as the number of positions on the rotary switch used to select the shell config

enum MachineState
{
  MACHINE_STATE_IDLE = 0,
  MACHINE_STATE_TAPING = 1,
  MACHINE_STATE_WAIT_FOR_BURNISHING = 2,
  MACHINE_STATE_BURNISHING = 3,
  MACHINE_STATE_TAPING_PAUSED = 4,
  MACHINE_STATE_BURNISHING_PAUSED = 5
};

struct ShellConfig
{
  uint32_t numAngles; // Number of taping angles/applications, this does NOT include the burnishing angle, so the total number of angles will be numAngles + 1

  float shellArmAngles[SHELLCONFIG_MAX_ANGLES + 1]; // The angle per application, the last angle being the burnishing angle
  float shellStepperSpeed[SHELLCONFIG_MAX_ANGLES + 1]; // Speed of the shell stepper motor as a fraction relative to the rim speed per application, the last being the burnishing speed
  float rimRotationsUntilNextAngle[SHELLCONFIG_MAX_ANGLES + 1]; // Usually (1 / shellStepperSpeed) or a multiple of 2 of that per application, the last being the burnishing rotatiosn until next angle
};

// -- Pin Definitions -- //

const int armStepperEnablePin = 2;
const int armStepperDirPin = 3;
const int armStepperPulsePin = 4;

const int rimMotorPWMPin = 5;
const int rimMotorEnablePin = 6;

const int shellStepperEnablePin = 7;
const int shellStepperDirPin = 8;
const int shellStepperPulsePin = 9;

const int startResumeButtonPin = 10;
const int pauseButtonPin = 11;
const int abortButtonPin = 12;

const int rotarySwitchPin = A0;

// -- Machine constants -- //

const uint32_t shellStepperPulsesPerRotation = 200 * 4;
const uint32_t armStepperPulsesPerRotation = 200 * 8;
const float armStepperPulsesPerDegree = armStepperPulsesPerRotation / 360.0f;

const uint32_t shellArmMoveDelayMs = 20;
const uint32_t startingAngle = 20;
const float rimRPM = 53.0f;
const float rimPWMDutyCycle = 0.3f;

ShellConfig currentShellConfig; // The currently selected shell config, determined by the rotary switch position and the shellConfig[..]

ShellConfig shellConfigs[NUM_SHELL_CONFIGS]; // Each element in the shell config array corresponds to a position on the rotary switch, used to control which shell config is currently selected

void setup() 
{
  initShellConfigs();
  currentShellConfig = shellConfigs[1];

  Serial.begin(9600);

  // Initialize the pins
  pinMode(armStepperEnablePin, OUTPUT);
  pinMode(armStepperDirPin, OUTPUT);
  pinMode(armStepperPulsePin, OUTPUT);
  pinMode(rimMotorPWMPin, OUTPUT);
  pinMode(rimMotorEnablePin, OUTPUT);
  pinMode(shellStepperEnablePin, OUTPUT);
  pinMode(shellStepperDirPin, OUTPUT);
  pinMode(shellStepperPulsePin, OUTPUT);

  pinMode(startResumeButtonPin, INPUT);
  pinMode(pauseButtonPin, INPUT);
  pinMode(abortButtonPin, INPUT);

  digitalWrite(rimMotorEnablePin, HIGH);
  analogWrite(rimMotorPWMPin, 0);

  // Write some default values to pins
  digitalWrite(armStepperEnablePin, LOW);
  digitalWrite(armStepperDirPin, LOW);
  digitalWrite(armStepperPulsePin, LOW);

  digitalWrite(shellStepperEnablePin, LOW);
  digitalWrite(shellStepperDirPin, HIGH);
  digitalWrite(shellStepperPulsePin, LOW);
}

MachineState machineState = MACHINE_STATE_IDLE; // What state the machine is currently in

uint32_t currentAngleIndex = 0; // The current angle the taping session is on
float currentAngleRimRotations = 0.0f; // The number of rim rotations since the last angle change (or start of taping for the 1st angle/application)

float shellStepperPulsesCounter = 0.0f; // The number of pulses the shell stepper motor needs to do (happens all at once)
float shellArmStepperPulsesCounter = 0.0f; // The number of rotations the shell arm stepper motor needs to do (happens every ~shellArmMoveDelayMs milliseconds)

float shellArmMoveTimer = 0.0f; // Times each pulse for the shell arm stepper motor
float delta = 0.0f;

void loop() 
{
  unsigned long startTime = millis();

  // Change the current shell config to the one selected by the rotary switch
  int rotarySwitchValue = analogRead(rotarySwitchPin);
  int rotarySwitchPosition = int(round(float(NUM_SHELL_CONFIGS - 1) * (float(rotarySwitchValue) / 1023.0f))); // Based on the voltage level (0-1023), NUM_SHELL_CONFIGS different positions are possible, this chooses the nearest position

  currentShellConfig = shellConfigs[rotarySwitchPosition];

  // Move the shell arm, using a delay to keep the speed in check as its counter is incremented all at once
  shellArmMoveTimer += delta;
  if (shellArmMoveTimer > shellArmMoveDelayMs / 1000.0f)
  {
    shellArmMoveTimer = 0.0f;
    if (shellArmStepperPulsesCounter != 0.0f)
    {
      if (shellArmStepperPulsesCounter > 0.0f)
        digitalWrite(armStepperDirPin, LOW);
      else
        digitalWrite(armStepperDirPin, HIGH);
  
      digitalWrite(armStepperPulsePin, HIGH);
      delayMicroseconds(10);
      digitalWrite(armStepperPulsePin, LOW);
  
      shellArmStepperPulsesCounter -= shellArmStepperPulsesCounter > 0.0f ? 1.0f : -1.0f;
    }
  }

  // Rotate the shell, no delay used because its counter is incremented in realtime and not at once
  while (shellStepperPulsesCounter >= 1.0f || shellStepperPulsesCounter <= -1.0f)
  {  
    if (shellStepperPulsesCounter > 0)
      digitalWrite(shellStepperDirPin, HIGH);
    else
      digitalWrite(shellStepperDirPin, LOW);
  
    digitalWrite(shellStepperPulsePin, HIGH);
    delayMicroseconds(10);
    digitalWrite(shellStepperPulsePin, LOW);

    shellStepperPulsesCounter -= shellStepperPulsesCounter > 0.0f ? 1.0f : -1.0f;
  }

  // Handle button inputs
  if (digitalRead(startResumeButtonPin) == HIGH)
  {
    if (machineState == MACHINE_STATE_IDLE) // Begin a taping session (start rim motor, and rotate the shell arm to it's first angle)
    {
      machineState = MACHINE_STATE_TAPING;
      analogWrite(rimMotorPWMPin, uint8_t(255 * rimPWMDutyCycle));
      rotateShellArm(currentShellConfig.shellArmAngles[currentAngleIndex] - startingAngle);
    }
    else if (machineState == MACHINE_STATE_WAIT_FOR_BURNISHING) // Begin a burnishing session (restart rim motor)
    {
      machineState = MACHINE_STATE_BURNISHING;
      analogWrite(rimMotorPWMPin, uint8_t(255 * rimPWMDutyCycle));
    }
    else if (machineState == MACHINE_STATE_TAPING_PAUSED) // Resume taping
    {
      machineState = MACHINE_STATE_TAPING;
      analogWrite(rimMotorPWMPin, uint8_t(255 * rimPWMDutyCycle));
    }
    else if (machineState == MACHINE_STATE_BURNISHING_PAUSED) // Resume burnishing
    {
      machineState = MACHINE_STATE_BURNISHING;
      analogWrite(rimMotorPWMPin, uint8_t(255 * rimPWMDutyCycle));
    }
  }
  else if (digitalRead(pauseButtonPin) == HIGH)
  {
    if (machineState == MACHINE_STATE_TAPING) // Pause taping (stop rim motor)
    {
      analogWrite(rimMotorPWMPin, 0);
      machineState = MACHINE_STATE_TAPING_PAUSED;
    }
    else if (machineState == MACHINE_STATE_BURNISHING) // Pause burnishing (stop rim motor)
    {
      analogWrite(rimMotorPWMPin, 0);
      machineState = MACHINE_STATE_BURNISHING_PAUSED;
    }
  }
  else if (digitalRead(abortButtonPin) == HIGH)
  {
    // Abort whatever is currently happening, resets everything to the initial startup state
    machineState = MACHINE_STATE_IDLE;
    analogWrite(rimMotorPWMPin, 0);
    currentAngleIndex = 0;
    shellStepperPulsesCounter = 0;
    currentAngleRimRotations = 0.0f;
  }

  // Handle an active taping or burnishing session
  if (machineState == MACHINE_STATE_TAPING || machineState == MACHINE_STATE_BURNISHING)
  {
    currentAngleRimRotations += (rimRPM / 60.0f) * delta;
    Serial.println(currentAngleRimRotations);

    // Rotate the shell stepper
    shellStepperPulsesCounter += (rimRPM / 60.0f) * delta * currentShellConfig.shellStepperSpeed[currentAngleIndex] * shellStepperPulsesPerRotation;

    // Decide what to do if the current application just finished
    if (currentAngleRimRotations >= currentShellConfig.rimRotationsUntilNextAngle[currentAngleIndex])
    {
      if (machineState == MACHINE_STATE_TAPING)
      {
        // We're done with the taping session, so wait for the user to resume to begin burnishing the shell
        if (currentAngleIndex == currentShellConfig.numAngles - 1)
        {
          machineState = MACHINE_STATE_WAIT_FOR_BURNISHING;
          analogWrite(rimMotorPWMPin, 0);
        }

        rotateShellArm(int32_t(currentShellConfig.shellArmAngles[currentAngleIndex + 1]) - int32_t(currentShellConfig.shellArmAngles[currentAngleIndex]));
  
        currentAngleIndex++;
        currentAngleRimRotations = 0.0f;
      }
      else if (machineState == MACHINE_STATE_BURNISHING)
      {
        machineState = MACHINE_STATE_IDLE;
        analogWrite(rimMotorPWMPin, 0);
        rotateShellArm(startingAngle - currentShellConfig.shellArmAngles[currentAngleIndex]);
        
        currentAngleIndex = 0;
        shellStepperPulsesCounter = 0;
        currentAngleRimRotations = 0.0f;
      }
    }
  }

  unsigned long elapsedTime = millis() - startTime;

  if (elapsedTime >= 10)
  {
    Serial.print("Warning! Running behind, took ");
    Serial.print(elapsedTime);
    Serial.println("ms for a tick! Should take <10ms");    
  }
  
  delta = (millis() - startTime) / 1000.0f;
}

// Initializes the shell configs for each position on the rotary switch
void initShellConfigs()
{
  // 3" shell w/ 1" tape
  shellConfigs[1].numAngles = 2;
  shellConfigs[1].shellArmAngles[1] = 35;
  shellConfigs[1].shellArmAngles[0] = 60;
  shellConfigs[1].shellStepperSpeed[1] = (1 / 39.0f) * 2.0f;
  shellConfigs[1].shellStepperSpeed[0] = (1 / 16.0f) * 2.0f;
  shellConfigs[1].rimRotationsUntilNextAngle[1] = 39.0f / 2.0f;
  shellConfigs[1].rimRotationsUntilNextAngle[0] = 16.0f / 2.0f;
  // Burnishing
  shellConfigs[1].shellArmAngles[2] = 35;
  shellConfigs[1].shellStepperSpeed[2] = 1 / 20.0f;
  shellConfigs[1].rimRotationsUntilNextAngle[2] = 20.0f;

  // 4" shell w/ 1" tape
  shellConfigs[2].numAngles = 3;
  shellConfigs[2].shellArmAngles[2] = 28;
  shellConfigs[2].shellArmAngles[1] = 48;
  shellConfigs[2].shellArmAngles[0] = 65;
  shellConfigs[2].shellStepperSpeed[2] = 1 / 29.0f;
  shellConfigs[2].shellStepperSpeed[1] = 1 / 15.0f;
  shellConfigs[2].shellStepperSpeed[0] = 1 / 10.0f;
  shellConfigs[2].rimRotationsUntilNextAngle[2] = 29.0f;
  shellConfigs[2].rimRotationsUntilNextAngle[1] = 15.0f;
  shellConfigs[2].rimRotationsUntilNextAngle[0] = 10.0f;
  // Burnishing
  shellConfigs[2].shellArmAngles[3] = 28;
  shellConfigs[2].shellStepperSpeed[3] = 1 / 30.0f;
  shellConfigs[2].rimRotationsUntilNextAngle[3] = 30.0f;
}

// Rotates the shell arm to a new angle, will not pause anything while moving (including the rim motor)
void rotateShellArm(float angleDelta)
{
  shellArmStepperPulsesCounter += armStepperPulsesPerDegree * angleDelta; 
}
