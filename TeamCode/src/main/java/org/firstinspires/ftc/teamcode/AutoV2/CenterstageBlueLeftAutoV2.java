package org.firstinspires.ftc.teamcode.AutoV2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import java.util.Objects;

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

    Integer reqID;
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
    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 0.75;     // Maximum FWD power applied to motor
    static final double MAX_REV = -0.75;     // Maximum REV power applied to motor
    double power = 0.1;
    boolean rampUp = true;

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
        ticksperRevolution = 550;
        wheelCircumference = 12.56;
        ticksPerInch = ticksperRevolution / wheelCircumference;
        // A single square is about 22 inches.
        myTimer = new ElapsedTime();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            State = "DetectSpike";
            while (opModeIsActive()) {
                if (State.equals("Test")) {
                    MoveForwardEncoder(10);
                    StrafeRightEncoder(10);
                    State = "AAAAAAA";
                }
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
                    DetectAprilTags();
                    myTimer.reset();
                    while (opModeIsActive() && !Objects.equals(myAprilTagIdCode, reqID)) {
                        if (myAprilTagIdCode == null) {
                            FrontLeft.setPower(-0.15);
                            FrontRight.setPower(0.15);
                            RearLeft.setPower(-0.15);
                            RearRight.setPower(0.15);
                            DetectAprilTags();
                        } else {
                            FrontLeft.setPower(0.15);
                            FrontRight.setPower(-0.15);
                            RearLeft.setPower(0.15);
                            RearRight.setPower(-0.15);
                            DetectAprilTags();
                        }
                        if (myTimer.seconds() >= 4) {
                            IMU_Telemetry();
                            if (Z_Rotation <= -85) {
                                while (Z_Rotation <= -85) {
                                    FrontLeft.setPower(-0.15);
                                    FrontRight.setPower(0.15);
                                    RearLeft.setPower(-0.15);
                                    RearRight.setPower(0.15);
                                    IMU_Telemetry();
                                }
                                FrontLeft.setPower(0);
                                FrontRight.setPower(0);
                                RearLeft.setPower(0);
                                RearRight.setPower(0);
                            } else {
                                while (Z_Rotation >= -85) {
                                    FrontLeft.setPower(0.15);
                                    FrontRight.setPower(-0.15);
                                    RearLeft.setPower(0.15);
                                    RearRight.setPower(-0.15);
                                    IMU_Telemetry();
                                }
                                FrontLeft.setPower(0);
                                FrontRight.setPower(0);
                                RearLeft.setPower(0);
                                RearRight.setPower(0);
                            }
                            MoveForwardEncoder(11);
                            reqID = -1;
                            break;
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
//          StrafeLeft(0.3 + 0.15 / reqID); These are no longer needed, but if issues
//          MoveForwardEncoder(3);          arise, these may need to be used.
                    FrontLeft.setPower(0);
                    FrontRight.setPower(0);
                    RearLeft.setPower(0);
                    RearRight.setPower(0);
                    DropArm.setPosition(1);
                    while (DropArm.getPosition() < 0.5) {
                        MoveBackwardEncoder(2);
                    }
                    myTimer.reset();
                    while (myTimer.seconds() <= 1) {
                    }
                    DropArm.setPosition(DropArm.getPosition() - 0.05);
                    myTimer.reset();
                    while (myTimer.seconds() <= 1) {
                    }
                    State = "Park";
                }
                if (State.equals("Park")) {
                    MoveBackwardEncoder(6);
                    DropArm.setPosition(-1);
                    StrafeLeft(1 + 0.35 * reqID);
                    MoveForwardEncoder(18);
                    PushServo.setPosition(0);
                    State = "AAAAAAAAAAAA";
                }
            }
        }
    }

    /**
     * Describe this function...
     */
    private void SpikeMiddleEncoderMinimal() {
        MoveForwardEncoder(30);
        MoveBackwardEncoder(3);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
        DropPixel();
        MoveBackwardEncoder(5);
        while (opModeIsActive() && Z_Rotation <= 85) {
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
        MoveForwardEncoder(23);
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
        while (opModeIsActive() && Z_Rotation <= 85) {
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
    }

    /**
     * Describe this function...
     */
    private void SpikeRightEncoderMinimal() {
        MoveForwardEncoder(28);
        while (opModeIsActive() && Z_Rotation >= -85) {
            FrontLeft.setPower(0.25);
            FrontRight.setPower(-0.25);
            RearLeft.setPower(0.25);
            RearRight.setPower(-0.25);
            IMU_Telemetry();
        }
        MoveForwardEncoder(8);
        MoveBackwardEncoder(4);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
        DropPixel();
        MoveBackwardEncoder(6);
        while (opModeIsActive() && Z_Rotation <= 85) {
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
    private void IMU_Telemetry() {
        Orientation angles;

        // Get absolute orientation
        // Get acceleration due to force of gravity.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
        imu.write8(BNO055IMU.Register.OPR_MODE, 0b00000011);
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
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
        if (reqID < 0) {
            return;
        } else {
            DetectAprilTags();
            if (myAprilTagPoseX < -0.2) {
                StrafeLeftEncoder((int) Math.abs(myAprilTagPoseX * 1.4));
            } else if (myAprilTagPoseX > 0.2) {
                StrafeRightEncoder((int) Math.abs(myAprilTagPoseX * 1.4));
            }
            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            RearRight.setPower(0);
            RearLeft.setPower(0);

            DetectAprilTags();
            MoveForwardEncoder((int) Math.round(myAprilTagPoseRange - 8.5));
        }
    }

    private void StrafeLeftEncoder(int Distance) {
        ResetEncoder();
        tickstoDestination = (int) (Distance * ticksPerInch * 1.1);
        FrontLeft.setTargetPosition(-tickstoDestination);
        FrontRight.setTargetPosition(tickstoDestination);
        RearLeft.setTargetPosition(tickstoDestination);
        RearRight.setTargetPosition(-tickstoDestination);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        power = 0.1;
        while (FrontRight.isBusy()) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT;
                if (power <= MAX_REV) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            FrontLeft.setPower(-power);
            FrontRight.setPower(power);
            RearLeft.setPower(power);
            RearRight.setPower(-power);
            myTimer.reset();
            while (myTimer.milliseconds() <= CYCLE_MS) {
                telemetry.update();
            }
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
        DisableEncoders();
    }

    private void StrafeRightEncoder(int Distance) {
        ResetEncoder();
        tickstoDestination = (int) (Distance * ticksPerInch * 1.1);
        FrontLeft.setTargetPosition(tickstoDestination);
        FrontRight.setTargetPosition(-tickstoDestination);
        RearLeft.setTargetPosition(-tickstoDestination);
        RearRight.setTargetPosition(tickstoDestination);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        power = 0.1;
        while (FrontRight.isBusy()) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT;
                if (power <= MAX_REV) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            FrontLeft.setPower(power);
            FrontRight.setPower(-power);
            RearLeft.setPower(-power);
            RearRight.setPower(power);
            myTimer.reset();
            while (myTimer.milliseconds() <= CYCLE_MS) {
                telemetry.update();
            }
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
        FrontLeft.setPower(0.1);
        FrontRight.setPower(0.1);
        RearLeft.setPower(0.1);
        RearRight.setPower(0.1);
        power = 0.1;
        while (FrontRight.isBusy()) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT;
                if (power <= MAX_REV) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            FrontLeft.setPower(power);
            FrontRight.setPower(power);
            RearLeft.setPower(power);
            RearRight.setPower(power);
            myTimer.reset();
            while (myTimer.milliseconds() <= CYCLE_MS) {
                telemetry.update();
            }
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
        FrontLeft.setPower(-0.1);
        FrontRight.setPower(-0.1);
        RearLeft.setPower(-0.1);
        RearRight.setPower(-0.1);
        power = 0.1;
        while (RearLeft.isBusy()) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT;
                if (power <= MAX_REV) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            FrontLeft.setPower(-power);
            FrontRight.setPower(-power);
            RearLeft.setPower(-power);
            RearRight.setPower(-power);
            myTimer.reset();
            while (myTimer.milliseconds() <= CYCLE_MS) {
                telemetry.update();
            }
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        DisableEncoders();
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
