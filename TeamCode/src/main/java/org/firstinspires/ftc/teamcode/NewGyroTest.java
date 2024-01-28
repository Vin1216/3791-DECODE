package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
@TeleOp
public class NewGyroTest extends LinearOpMode {
    BNO055IMU imu;
    float Z_Rotation;
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Init_IMU();
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive()) {
                IMU_Telemetry();
            }
        }
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
}
