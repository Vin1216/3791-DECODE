package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class RampUpEncoderTest extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor RearLeft;
    private DcMotor RearRight;
    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor
    double  power   = 0.1;
    boolean rampUp  = true;
    int ticksperRevolution;
    double wheelCircumference;
    double ticksPerInch;
    int tickstoDestination;
    ElapsedTime myTimer;

    public void runOpMode() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        RearLeft = hardwareMap.get(DcMotor.class, "RearLeft");
        RearRight = hardwareMap.get(DcMotor.class, "RearRight");
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.REVERSE);
        ticksperRevolution = 480;
        wheelCircumference = 12.56;
        ticksPerInch = ticksperRevolution / wheelCircumference;
        myTimer = new ElapsedTime();
        waitForStart();
        if(opModeIsActive()) {
            MoveForwardEncoder(44);
        }
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
        while (FrontRight.isBusy()) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT ;
                if (power >= MAX_FWD ) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
                else if (FrontRight.getCurrentPosition() >= (tickstoDestination * 0.75)) {
                    rampUp = !rampUp;
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT ;
                if (power <= MAX_REV ) {
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
    private void DisableEncoders() {
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
