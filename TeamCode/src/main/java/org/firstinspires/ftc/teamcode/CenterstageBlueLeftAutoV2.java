package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "CenterstageBlueLeftAutoV2")
public class CenterstageBlueLeftAutoV2 extends LinearOpMode {

  private DcMotor FrontLeft;
  private DcMotor FrontRight;
  private DcMotor RearLeft;
  private DcMotor RearRight;
  private DcMotor intake;
  private BNO055IMU imu;
  private DcMotor LiftMotor2;
  private Servo PushServo;
  private Servo DropArm;
  private VoltageSensor ControlHub_VoltageSensor;
  private Servo scoop;
  private Servo ClawServo;

  int reqID;
  List<AprilTagDetection> myAprilTagDetections;
  Double myAprilTagPoseX;
  double myAprilTagPoseBearing;
  int tickstoDestination;
  double myAprilTagPoseRange;
  AprilTagDetection myAprilTagDetection;
  ElapsedTime myTimer;
  String State;
  double myAprilTagPoseYaw;
  float Z_Rotation;
  TfodProcessor myTfodProcessor;
  AprilTagProcessor myAprilTagProcessor;
  double ticksPerInch;
  Integer myAprilTagIdCode;

  /**
   * Describe this function...
   */
  private void SpikeLeftEncoder() {
    MoveForwardEncoder(28);
    while (opModeIsActive() && Z_Rotation <= 75) {
      FrontLeft.setPower(-0.5);
      FrontRight.setPower(0.5);
      RearLeft.setPower(-0.5);
      RearRight.setPower(0.5);
      IMU_Telemetry();
    }
    MoveForwardEncoder(11);
    while (opModeIsActive() && Z_Rotation >= 15) {
      FrontLeft.setPower(0.5);
      FrontRight.setPower(-0.5);
      RearLeft.setPower(0.5);
      RearRight.setPower(-0.5);
      IMU_Telemetry();
    }
    MoveBackwardEncoder(4);
    while (opModeIsActive() && Z_Rotation <= 75) {
      FrontLeft.setPower(-0.5);
      FrontRight.setPower(0.5);
      RearLeft.setPower(-0.5);
      RearRight.setPower(0.5);
      IMU_Telemetry();
    }
    MoveForwardEncoder(5);
    myTimer.reset();
    while (myTimer.milliseconds() <= 500) {
      intake.setPower(-1);
      myTimerTelemetry();
    }
    intake.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void SpikeMiddleEncoder() {
    MoveForwardEncoder(33);
    IMU_Telemetry();
    while (opModeIsActive() && Z_Rotation >= -160) {
      FrontLeft.setPower(0.5);
      FrontRight.setPower(-0.5);
      RearLeft.setPower(0.5);
      RearRight.setPower(-0.5);
      IMU_Telemetry();
    }
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
    myTimer.reset();
    while (myTimer.milliseconds() <= 500) {
      intake.setPower(-1);
      myTimerTelemetry();
    }
    intake.setPower(0);
    MoveForwardEncoder(6);
    IMU_Telemetry();
    while (opModeIsActive() && (Z_Rotation >= 100 || Z_Rotation <= -160)) {
      FrontLeft.setPower(0.5);
      FrontRight.setPower(-0.5);
      RearLeft.setPower(0.5);
      RearRight.setPower(-0.5);
      IMU_Telemetry();
    }
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void SpikeRightEncoder() {
    MoveForwardEncoder(28);
    while (opModeIsActive() && Z_Rotation <= 80) {
      FrontLeft.setPower(-0.25);
      FrontRight.setPower(0.25);
      RearLeft.setPower(-0.25);
      RearRight.setPower(0.25);
      IMU_Telemetry();
    }
    MoveBackwardEncoder(3);
    myTimer.reset();
    while (myTimer.milliseconds() <= 500) {
      intake.setPower(-1);
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      RearLeft.setPower(0);
      RearRight.setPower(0);
      myTimerTelemetry();
    }
    intake.setPower(0);
    while (opModeIsActive() && Z_Rotation <= 90) {
      FrontLeft.setPower(-0.25);
      FrontRight.setPower(0.25);
      RearLeft.setPower(-0.25);
      RearRight.setPower(0.25);
      IMU_Telemetry();
    }
    MoveForwardEncoder(22);
  }

  /**
   * Describe this function...
   */
  private void IMU_Telemetry() {
    Orientation angles;
    Acceleration gravity;

    // Get absolute orientation
    // Get acceleration due to force of gravity.
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    gravity = imu.getGravity();
    Z_Rotation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    // Display orientation info.
    telemetry.addData("rot about Z", angles.firstAngle);
    telemetry.addData("rot about Y", angles.secondAngle);
    telemetry.addData("rot about X", angles.thirdAngle);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void Init_IMU() {
    BNO055IMU.Parameters imuParameters;

    // Create new IMU Parameters object.
    imuParameters = new BNO055IMU.Parameters();
    // Use degrees as angle unit.
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    // Express acceleration as m/s^2.
    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    // Disable logging.
    imuParameters.loggingEnabled = false;
    // Initialize IMU.
    imu.initialize(imuParameters);
    // Prompt user to press start buton.
    telemetry.addData("IMU ON", "Press start to continue...");
  }

  /**
   * Describe this function...
   */
  private void Init_VisionPortal() {
    TfodProcessor.Builder myTfodProcessorBuilder;
    VisionPortal myVisionPortal;

    // WIP UNTIL WE KNOW MORE
    // Create a new TfodProcessor.Builder object.
    myTfodProcessorBuilder = new TfodProcessor.Builder();
    // Set the name of the file where the model can be found.
    myTfodProcessorBuilder.setModelFileName("CenterstageBlue.tflite");
    // Set the full ordered list of labels the model is trained to recognize.
    myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("Pixel"));
    // Set the aspect ratio for the images used when the model was created.
    myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
    // Build the TensorFlow Object Detection processor and assign it to a variable.
    myTfodProcessor = myTfodProcessorBuilder.build();
    // Create the AprilTag processor and assign it to a variable.
    myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    // Create a VisionPortal, with the specified webcam name, AprilTag processor,
    // and TensorFlow Object Detection processor, and assign it to a variable.
    myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "WebCam"), myAprilTagProcessor, myTfodProcessor);
  }

  /**
   * Describe this function...
   */
  private void SpikeMiddleEncoderMinimal() {
    MoveForwardEncoder(32);
    MoveBackwardEncoder(4);
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
    DropPixel();
    MoveBackwardEncoder(11);
    while (opModeIsActive() && Z_Rotation <= 80) {
      FrontLeft.setPower(-0.25);
      FrontRight.setPower(0.25);
      RearLeft.setPower(-0.25);
      RearRight.setPower(0.25);
      IMU_Telemetry();
    }
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
    MoveForwardEncoder(11);
  }

  /**
   * Describe this function...
   */
  private void SpikeLeftEncoderMinimal() {
    StrafeLeft(0.6);
    MoveForwardEncoder(33);
    MoveBackwardEncoder(6);
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
    DropPixel();
    MoveBackwardEncoder(6);
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
    StrafeLeft(0.75);
    IMU_Telemetry();
    while (opModeIsActive() && Z_Rotation <= 70) {
      FrontLeft.setPower(-0.5);
      FrontRight.setPower(0.5);
      RearLeft.setPower(-0.5);
      RearRight.setPower(0.5);
      IMU_Telemetry();
    }
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
    StrafeLeft(0.25);
  }

  /**
   * Describe this function...
   */
  private void SpikeRightEncoderMinimal() {
    MoveForwardEncoder(28);
    while (opModeIsActive() && Z_Rotation >= -80) {
      FrontLeft.setPower(0.25);
      FrontRight.setPower(-0.25);
      RearLeft.setPower(0.25);
      RearRight.setPower(-0.25);
      IMU_Telemetry();
    }
    MoveForwardEncoder(6);
    MoveBackwardEncoder(3);
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
    DropPixel();
    MoveBackwardEncoder(6);
    while (opModeIsActive() && Z_Rotation <= 90) {
      FrontLeft.setPower(-0.25);
      FrontRight.setPower(0.25);
      RearLeft.setPower(-0.25);
      RearRight.setPower(0.25);
      IMU_Telemetry();
    }
    MoveForwardEncoder(11);
  }

  /**
   * Describe this function...
   */
  private void DetectSpike() {
    List<Recognition> myTfodRecognitions;
    Float SpikeX = null;
    Recognition myTfodRecognition;
    float SpikeY;

    // Get a list containing the latest recognitions, which may be stale.
    myTfodRecognitions = myTfodProcessor.getRecognitions();
    for (Recognition myTfodRecognition_item : myTfodRecognitions) {
      myTfodRecognition = myTfodRecognition_item;
      SpikeX = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
      SpikeY = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
      telemetry.addData("Position", "" + JavaUtil.formatNumber(SpikeX, 2) + JavaUtil.formatNumber(SpikeY, 2));
      telemetry.update();
      if (SpikeX <= 320) {
        State = "SpikeMiddle";
        reqID = 2;
        telemetry.addLine("SpikeMiddle");
        telemetry.update();
      } else {
        State = "SpikeRight";
        reqID = 3;
        telemetry.addLine("SpikeRight");
        telemetry.update();
      }
    }
    if (SpikeX == null) {
      State = "SpikeLeft";
      reqID = 1;
      telemetry.addLine("SpikeLeft");
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void DetectAprilTags() {
    // Get a list containing the latest detections, which may be stale.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      if (myAprilTagDetection.metadata != null) {
        myAprilTagIdCode = myAprilTagDetection.id;
        if (myAprilTagIdCode == reqID) {
          myAprilTagPoseX = myAprilTagDetection.ftcPose.x;
          myAprilTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
          myAprilTagPoseRange = myAprilTagDetection.ftcPose.range;
          myAprilTagPoseYaw = myAprilTagDetection.ftcPose.yaw;
          break;
        }
      }
    }
  }

  /**
   * Describe this function...
   */
  private void MoveTowardAprilTag(int reqID) {
    double rotationStatic;
    DetectAprilTags();
    if (myAprilTagPoseX < -0.2) {
      rotationStatic = Z_Rotation + 75;
      while (Z_Rotation < rotationStatic) {
        FrontRight.setPower(0.3);
        FrontLeft.setPower(-0.3);
        RearRight.setPower(0.3);
        RearLeft.setPower(-0.3);
        IMU_Telemetry();
      }
      MoveForwardEncoder((int)Math.abs(myAprilTagPoseX * 1.4));
      rotationStatic = Z_Rotation - 75;
      while (Z_Rotation > rotationStatic) {
        FrontRight.setPower(-0.3);
        FrontLeft.setPower(0.3);
        RearRight.setPower(-0.3);
        RearLeft.setPower(0.3);
        IMU_Telemetry();
      }
    }
    else if (myAprilTagPoseX > 0.2) {
      rotationStatic = Z_Rotation - 75;
      while (Z_Rotation > rotationStatic) {
        FrontRight.setPower(-0.3);
        FrontLeft.setPower(0.3);
        RearRight.setPower(-0.3);
        RearLeft.setPower(0.3);
        IMU_Telemetry();
      }
      MoveForwardEncoder((int)Math.abs(myAprilTagPoseX * 1.4));
      rotationStatic = Z_Rotation + 75;
      while (Z_Rotation < rotationStatic) {
        FrontRight.setPower(0.3);
        FrontLeft.setPower(-0.3);
        RearRight.setPower(0.3);
        RearLeft.setPower(-0.3);
        IMU_Telemetry();
      }
    }
    FrontRight.setPower(0);
    FrontLeft.setPower(0);
    RearRight.setPower(0);
    RearLeft.setPower(0);

    if (myAprilTagPoseYaw < 0) {
      while (myAprilTagPoseYaw < 0) {
        FrontRight.setPower(0.15);
        FrontLeft.setPower(0.2);
        RearRight.setPower(0.15);
        RearLeft.setPower(0.2);
        DetectAprilTags();
      }
    }
    else {
      FrontRight.setPower(0.2);
      FrontLeft.setPower(0.15);
      RearRight.setPower(0.2);
      RearLeft.setPower(0.15);
      DetectAprilTags();
    }
    FrontRight.setPower(0);
    FrontLeft.setPower(0);
    RearRight.setPower(0);
    RearLeft.setPower(0);
    DetectAprilTags();
    MoveForwardEncoder((int)myAprilTagPoseRange - 3);
  }

  /**
   * Describe this function...
   */
  private void MoveForwardEncoder(int Distance) {
    ResetEncoder();
    tickstoDestination = (int) (Distance * ticksPerInch);
    FrontLeft.setTargetPosition(tickstoDestination);
    FrontRight.setTargetPosition(tickstoDestination);
    RearLeft.setTargetPosition(tickstoDestination);
    RearRight.setTargetPosition(tickstoDestination);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontLeft.setPower(0.5);
    FrontRight.setPower(0.5);
    RearLeft.setPower(0.5);
    RearRight.setPower(0.5);
    while (FrontRight.isBusy()) {
    }
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
    DisableEncoders();
  }

  /**
   * Describe this function...
   */
  private void MoveBackwardEncoder(int Distance) {
    ResetEncoder();
    tickstoDestination = (int) -(Distance * ticksPerInch);
    FrontLeft.setTargetPosition(tickstoDestination);
    FrontRight.setTargetPosition(tickstoDestination);
    RearLeft.setTargetPosition(tickstoDestination);
    RearRight.setTargetPosition(tickstoDestination);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontLeft.setPower(-0.5);
    FrontRight.setPower(-0.5);
    RearLeft.setPower(-0.5);
    RearRight.setPower(-0.5);
    while (RearLeft.isBusy()) {
    }
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    DisableEncoders();
  }

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int ticksperRevolution;
    double wheelCircumference;

    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    RearLeft = hardwareMap.get(DcMotor.class, "RearLeft");
    RearRight = hardwareMap.get(DcMotor.class, "RearRight");
    intake = hardwareMap.get(DcMotor.class, "intake");
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    LiftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");
    PushServo = hardwareMap.get(Servo.class, "PushServo");
    DropArm = hardwareMap.get(Servo.class, "DropArm");
    ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    scoop = hardwareMap.get(Servo.class, "scoop");
    ClawServo = hardwareMap.get(Servo.class, "ClawServo");

    // Put initialization blocks here.
    FrontRight.setDirection(DcMotor.Direction.REVERSE);
    FrontLeft.setDirection(DcMotor.Direction.REVERSE);
    RearLeft.setDirection(DcMotor.Direction.REVERSE);
    RearRight.setDirection(DcMotor.Direction.REVERSE);
    LiftMotor2.setDirection(DcMotor.Direction.REVERSE);
    PushServo.setPosition(0);
    Init_IMU();
    IMU_Telemetry();
    Init_VisionPortal();
    ticksperRevolution = 480;
    wheelCircumference = 12.56;
    ticksPerInch = ticksperRevolution / wheelCircumference;
    // A single square is about 22 inches.
    myTimer = new ElapsedTime();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      State = "DetectSpike";
      while (opModeIsActive()) {
        if (State.equals("DetectSpike")) {
          DetectSpike();
        }
        if (State.equals("SpikeMiddle")) {
          SpikeMiddleEncoderMinimal();
          State = "FindBackboard";
        }
        if (State.equals("SpikeLeft")) {
          SpikeLeftEncoderMinimal();
          State = "FindBackboard";
        }
        if (State.equals("SpikeRight")) {
          SpikeRightEncoderMinimal();
          State = "FindBackboard";
        }
        if (State.equals("FindBackboard")) {
          while (opModeIsActive() && myAprilTagIdCode != reqID) {
            if (myAprilTagIdCode == null) {
              FrontLeft.setPower(0.1);
              FrontRight.setPower(-0.1);
              RearLeft.setPower(0.1);
              RearRight.setPower(-0.1);
              DetectAprilTags();
            } else {
              FrontLeft.setPower(-0.1);
              FrontRight.setPower(0.1);
              RearLeft.setPower(-0.1);
              RearRight.setPower(0.1);
              DetectAprilTags();
            }
          }
          FrontLeft.setPower(0);
          FrontRight.setPower(0);
          RearLeft.setPower(0);
          RearRight.setPower(0);
          telemetry.addLine(Integer.toString(myAprilTagIdCode));
          State = "ScoreOnApriltag";
        }
        if (State.equals("ScoreOnApriltag")) {
          MoveTowardAprilTag(reqID);
          StrafeLeft(0.3 + 0.15 / reqID);
          MoveForwardEncoder(3);
          DropArm.setPosition(1);
          myTimer.reset();
          while (myTimer.seconds() <= 1) {
          }
          DropArm.setPosition(0.95);
          myTimer.reset();
          while (myTimer.seconds() <= 1) {
          }
          State = "Park";
        }
        if (State.equals("Park")) {
          MoveBackwardEncoder(6);
          DropArm.setPosition(-1);
          StrafeLeft(1.5 + 0.35 / reqID);
          MoveForwardEncoder(14);
          PushServo.setPosition(0);
          State = "AAAAAAAAAAAA";
        }
      }
    }
  }

  /**
   * Describe this function...
   */
  private void RaiseLift(
      int LiftHeight) {
    myTimer.reset();
    while (myTimer.seconds() <= LiftHeight) {
      LiftMotor2.setPower(0.5);
      intake.setPower(0);
    }
    LiftMotor2.setPower(0);
    intake.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void LowerLift(int LiftHeight) {
    // NOT WORKING RIGHT NOW
    LiftMotor2.setTargetPosition(LiftHeight * -100);
    LiftMotor2.setTargetPosition(LiftHeight * -100);
    LiftMotor2.setPower(-1);
    LiftMotor2.setPower(-1);
    while (LiftMotor2.isBusy()) {
    }
    LiftMotor2.setPower(0);
    LiftMotor2.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void ResetEncoder() {
    FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  /**
   * Describe this function...
   */
  private void DisableEncoders() {
    FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  /**
   * Describe this function...
   */
  private void ResetLiftEncoder() {
    LiftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LiftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LiftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LiftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LiftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LiftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  /**
   * Describe this function...
   */
  private void RaiseLiftEncoder(double LiftHeight) {
    ResetLiftEncoder();
    LiftMotor2.setTargetPosition((int) (LiftHeight * 1000));
    LiftMotor2.setTargetPosition((int) (LiftHeight * 1000));
    LiftMotor2.setPower(0.5);
    intake.setPower(0);
    while (LiftMotor2.isBusy()) {
    }
    LiftMotor2.setPower(0);
    intake.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void DropPixel() {
    scoop.setPosition(1);
    myTimer.reset();
    while (myTimer.milliseconds() <= 500) {
    }
    scoop.setPosition(0.7);
  }

  /**
   * Describe this function...
   */
  private void ScoopInit() {
    ClawServo.setDirection(Servo.Direction.REVERSE);
    ClawServo.setPosition(1);
  }

  /**
   * Describe this function...
   */
  private void StrafeRight(int StrafeTime) {
    myTimer.reset();
    while (myTimer.seconds() <= StrafeTime * (13 / ControlHub_VoltageSensor.getVoltage())) {
      FrontLeft.setPower(0.5);
      FrontRight.setPower(-0.5);
      RearLeft.setPower(-0.5);
      RearRight.setPower(0.5);
    }
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void ScoopOpen() {
    scoop.setPosition(1);
  }

  /**
   * Describe this function...
   */
  private void ScoopClose() {
    scoop.setPosition(0.7);
  }

  /**
   * Describe this function...
   */
  private void myTimerTelemetry() {
    telemetry.addLine("Time: " + myTimer.time());
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void StrafeLeft(double StrafeTime) {
    myTimer.reset();
    while (myTimer.seconds() <= StrafeTime * (13 / ControlHub_VoltageSensor.getVoltage())) {
      FrontLeft.setPower(-0.5);
      FrontRight.setPower(0.5);
      RearLeft.setPower(0.5);
      RearRight.setPower(-0.5);
    }
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    RearLeft.setPower(0);
    RearRight.setPower(0);
  }
}
