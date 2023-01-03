package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SimpleRotationOp")
public class SimpleRotationOp extends LinearOpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private BNO055IMU imu;
    

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.dcMotor.get("motor1");
        motorBackLeft = hardwareMap.dcMotor.get("motor2");
        motorFrontRight = hardwareMap.dcMotor.get("motor3");
        motorBackRight = hardwareMap.dcMotor.get("motor4");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        double initialAngle = -imu.getAngularOrientation().firstAngle;
        
        telemetry.addData("Inital angle", initialAngle);
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double speed = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            
            initialAngle -= (turn * 1e-1);
            double turnOffset = -2.0*(imu.getAngularOrientation().firstAngle-initialAngle)/(2.0*Math.PI);
            turn = (turnOffset <= 0.4 && turnOffset >= 0.1 ) ? turnOffset : turnOffset <= 0.1 ? 0 : turnOffset; 
            telemetry.addData("Turn offset", turnOffset);
            telemetry.addData("Turn", turn);
            
            motorFrontLeft.setPower(speed + turn + strafe);
            motorFrontRight.setPower(speed - turn - strafe);
            motorBackLeft.setPower(speed + turn - strafe);
            motorBackRight.setPower(speed - turn + strafe);
            
            
            telemetry.update();
        }
    }
    
}
