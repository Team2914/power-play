package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SimpleEncoderRotationOp")
public class SimpleEncoderRotationOp extends LinearOpMode {

    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorBackRight;
    private BNO055IMU imu;
    

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motor1");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motor2");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motor3");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motor4");

        // Enable Encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        waitForStart();
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double speed = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            turn += Math.pow(strafe * 0.4, 2)*Math.signum(strafe);
            
            /*initialAngle -= (turn * 1e-1);
            double turnOffset = -2.0*(imu.getAngularOrientation().firstAngle-initialAngle)/(2.0*Math.PI);
            turn = (turnOffset <= 0.4 && turnOffset >= 0.1 ) ? turnOffset : turnOffset <= 0.1 ? 0 : turnOffset; 
            telemetry.addData("Turn offset", turnOffset);
            telemetry.addData("Turn", turn);*/

            telemetry.addData("Current Speed ",motorFrontLeft.getVelocity());
            
            motorFrontLeft.setVelocity((speed + turn + strafe) * 2000);
            motorFrontRight.setVelocity((speed - turn - strafe) * 2000);
            motorBackLeft.setVelocity((speed + turn - strafe) * 2000);
            motorBackRight.setVelocity((speed - turn + strafe) * 2000);
            
            
            telemetry.update();
        }
    }
    
}
