package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelCustomPID {
    DcMotorEx flywheel1, flywheel2;
    public static double P = 0.001;
    public static double D = 0.00001;
    public static double F = 0.00042;
    ElapsedTime flywheelTimer = new ElapsedTime();
    double decelerationCoefficient = 1;
    public FlywheelCustomPID(HardwareMap hardwareMap) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setVelocity(double velocity) {
        if(velocity == 0) {
            decelerationCoefficient = 0.5;
        }
        else {
            decelerationCoefficient = 1;
        }
        flywheel1.setPower(
                (P*(velocity - flywheel1.getVelocity())
                + D * (velocity - flywheel1.getVelocity()) / flywheelTimer.seconds()
                + F * velocity)
                * decelerationCoefficient
        );
        flywheel2.setPower((P*(velocity - flywheel2.getVelocity())
                + D * (velocity - flywheel2.getVelocity()) / flywheelTimer.seconds()
                + F * velocity)
                * decelerationCoefficient
        );
        flywheelTimer.reset();
    }
    public void setVelocityP(double velocity) {
        if(velocity == 0) {
            decelerationCoefficient = 0.5;
        }
        else {
            decelerationCoefficient = 1;
        }
        flywheel1.setPower((P*(velocity - flywheel1.getVelocity())
                + F * velocity)
                * decelerationCoefficient
        );
        flywheel2.setPower((P*(velocity - flywheel2.getVelocity())
                + F * velocity)
                * decelerationCoefficient
        );
    }

}
