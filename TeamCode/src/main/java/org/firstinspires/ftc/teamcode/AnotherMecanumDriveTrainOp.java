package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "AnotherMecanumDriveTrainOp")
public class AnotherMecanumDriveTrainOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor1");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor2");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor3");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor4");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            double leftStickX = gamepad1.left_stick_x;
            
            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;
            
            double theta = Math.atan2(leftStickY, leftStickX);
            double power = Math.hypot(leftStickX, leftStickY);
            double frontLeftPow, frontRightPow, backLeftPow, backRightPow;
            
            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            
            frontLeftPow = power * cos / max + rightStickX;
            frontRightPow = power * sin / max - rightStickX;
            backLeftPow = power * sin / max + rightStickX;
            backRightPow = power * cos / max - rightStickX;
            
            if ((power + Math.abs(rightStickX)) > 1) {
                frontLeftPow /= power + rightStickX;
                frontRightPow /= power + rightStickX;
                backLeftPow /= power + rightStickX;
                backRightPow /= power + rightStickX;
            }
            
            motorFrontLeft.setPower(frontLeftPow);
            motorBackLeft.setPower(backLeftPow);
            motorFrontRight.setPower(frontRightPow);
            motorBackRight.setPower(backRightPow);
            
        }
    }
}
