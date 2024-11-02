package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class ItDLeftAutoMinimal extends LinearOpMode {
    DcMotorEx FrontLeft;
    DcMotorEx FrontRight;
    DcMotorEx RearLeft;
    DcMotorEx RearRight;
    DcMotorEx Arm;
    DcMotorEx Lift;
    DcMotorEx Intake;
    Servo Bucket;
    private BNO055IMU imu;
    double power;
    int ticksperRevolution;
    double wheelCircumference;
    double ticksPerInch;
    int tickstoDestination;

    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 0.75;     // Maximum FWD power applied to motor
    static final double MAX_REV = -0.75;     // Maximum REV power applied to motor
    boolean rampUp = true;

    ElapsedTime myTimer;
    ElapsedTime myElapsedTime;

    float Z_Rotation;

    public void runOpMode() {
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        RearLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
        RearRight = hardwareMap.get(DcMotorEx.class, "RearRight");
        Arm = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        Lift = hardwareMap.get(DcMotorEx.class,"LiftMotor");
        Intake = hardwareMap.get(DcMotorEx.class,"IntakeMotor");
        Bucket = hardwareMap.get(Servo.class,"bucket");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        RearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        myTimer = new ElapsedTime();
        myElapsedTime = new ElapsedTime();

        ticksperRevolution = 550;
        wheelCircumference = 12.56;
        ticksPerInch = ticksperRevolution / wheelCircumference;

        Init_IMU();
        IMU_Telemetry();
        waitForStart();
        if(opModeIsActive()) {
            MoveBackwardEncoder(24);
            ScoreOnHighBasket();
            MoveForwardEncoder(24);
            StrafeLeftEncoder(54);
            EncoderMoveToPosition(Arm,1400);

            while (opModeIsActive() && Z_Rotation <= 175) {
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

            MoveForwardEncoder(4);
            PickupSample();

            while (Z_Rotation >= 5) {
                FrontLeft.setPower(0.25);
                FrontRight.setPower(0.25);
                RearLeft.setPower(0.25);
                RearRight.setPower(0.25);
                IMU_Telemetry();
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearLeft.setPower(0);
            RearRight.setPower(0);

            StrafeRightEncoder(54);
            MoveBackwardEncoder(20);

            ScoreOnHighBasket();

            //PARK
            MoveForwardEncoder(24);
            StrafeLeftEncoder(72);
            MoveForwardEncoder(24);
        }
    }

    private void EncoderMoveToPosition(DcMotorEx motor, int ticks) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(ticks > motor.getCurrentPosition()) {
            motor.setPower(0.5);
        }
        else {
            motor.setPower(-0.5);
        }
        while(motor.isBusy()) {
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


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

    private void ScoreOnHighBasket() {
        EncoderMoveToPosition(Lift,3700);
        Bucket.setPosition(1);
        while(myElapsedTime.milliseconds() < 500) {
        }
        Bucket.setPosition(0.4);
        EncoderMoveToPosition(Lift,0);
    }

    private void PickupSample() {
//        EncoderMoveToPosition(Arm,1400);
        Intake.setPower(-1);
        myTimer.reset();
        while(myTimer.milliseconds() < 500) {
        }
        Intake.setPower(0);
        EncoderMoveToPosition(Arm,0);
        Intake.setPower(-1);
        myTimer.reset();
        while(myTimer.milliseconds() < 500) {
        }
        Intake.setPower(0);
    }

    private void DisableEncoders() {
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
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
}
