package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

@Config
@TeleOp(name = "FlywheelCustomPIDTuning")
public class FlywheelCustomPIDTuning extends LinearOpMode {
    public static double P = 0.001;
    public static double D = 0.00002;
    public static double F = 0.00042;
    DcMotorEx flywheel1;
    DcMotorEx flywheel2;
    boolean flywheelActive = true;
    double velocity = 700;
    Servo Kicker;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    ElapsedTime flywheelTimer = new ElapsedTime();
    double maxVelocity = 2800;
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        Kicker = hardwareMap.get(Servo.class, "wshoot");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.xWasPressed() || gamepad2.rightBumperWasPressed()) {
                Kicker.setPosition(0.7);
            }
            if(gamepad1.yWasPressed() || gamepad2.right_trigger > 0) {
                Kicker.setPosition(0);
            }
            if(gamepad1.dpadUpWasPressed()) {
                velocity = 1120;
            }
            if(gamepad1.dpadRightWasPressed()) {
                velocity = 1040;
            }
            if(gamepad1.dpadDownWasPressed()) {
                velocity = 940;
            }
            if(gamepad1.rightBumperWasPressed()) {
                velocity -= 20;
            }
            if(gamepad1.rightBumperWasPressed()) {
                velocity += 20;
            }
            if(gamepad1.aWasPressed()) {
                flywheelActive = true;
            }
            if(gamepad1.bWasPressed()) {
                flywheelActive = false;
            }
            if(flywheelActive) {
                flywheel1.setPower(P*(velocity - flywheel1.getVelocity())
                        + D * (velocity - flywheel1.getVelocity()) / flywheelTimer.seconds()
                        + F * velocity);
                flywheel2.setPower(P*(velocity - flywheel2.getVelocity())
                        + D * (velocity - flywheel2.getVelocity()) / flywheelTimer.seconds()
                        + F * velocity);
                flywheelTimer.reset();
            }
            telemetry.addData("Target Velocity: ", velocity);
            telemetry.addLine(String.format(Locale.US, "Target Velocity %%: %.2f%%",(velocity / maxVelocity * 100) ));
            telemetry.addData("Flywheel 1: ", flywheel1.getVelocity());
            telemetry.addData("Flywheel 2: ", flywheel2.getVelocity());
            telemetry.addLine(String.format(Locale.US,"Flywheel 1 %%: %.2f%%", (flywheel1.getVelocity() / maxVelocity * 100)));
            telemetry.addLine(String.format(Locale.US,"Flywheel 2 %%: %.2f%%", (flywheel2.getVelocity() / maxVelocity * 100)));
            dashboardTelemetry.addData("Target Velocity", velocity);
            dashboardTelemetry.addData("Flywheel 1", flywheel1.getVelocity());
            dashboardTelemetry.addData("Flywheel 2", flywheel2.getVelocity());
            dashboardTelemetry.addData("nothing", 0);
            dashboardTelemetry.update();
            telemetry.update();
        }
    }
}
