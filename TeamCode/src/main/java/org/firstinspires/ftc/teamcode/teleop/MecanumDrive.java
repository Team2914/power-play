package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Config;
import org.firstinspires.ftc.teamcode.utils.Utils;

@TeleOp(name = "MecanumDriveOpMode")
public class MecanumDrive extends LinearOpMode {
    private DcMotorEx m_frontLeft;
    private DcMotorEx m_frontRight;
    private DcMotorEx m_backLeft;
    private DcMotorEx m_backRight;
    private BNO055IMU imu;
    private Servo s_Claw;
    private DcMotorEx m_turnTable;
    private DcMotorEx m_Lift;

    private boolean isLiftMoving;
    private boolean isRotating;
    private int currentLiftLevel = 0;
    private boolean areBumpersPressed = false;

    private double oldLeftStickX2;
    private double oldRightStickX2;

    // Set the position of the lift, going up or down
    private void setLiftLevel(boolean goingUp) {
        if (!areBumpersPressed) {
            areBumpersPressed = true;
            if (goingUp && currentLiftLevel < 3) {
                currentLiftLevel++;
            } else if (!goingUp && currentLiftLevel > 0) {
                currentLiftLevel--;
            }
            m_Lift.setTargetPosition(Config.LIFT_LEVELS[currentLiftLevel]);
            sleep(200);
        } else if (areBumpersPressed) {
            areBumpersPressed = false;
        }
        m_Lift.setPower(1);
    }

    private void initHardware() {
        // Getting hardware from expansion hub(s)
        m_frontLeft = hardwareMap.get(DcMotorEx.class, "motor1");
        m_backLeft = hardwareMap.get(DcMotorEx.class, "motor2");
        m_frontRight = hardwareMap.get(DcMotorEx.class, "motor3");
        m_backRight = hardwareMap.get(DcMotorEx.class, "motor4");
        s_Claw = hardwareMap.get(Servo.class, "claw");
        m_turnTable = hardwareMap.get(DcMotorEx.class, "turnTable");
        m_Lift = hardwareMap.get(DcMotorEx.class, "elevator");

        // Setting motor types
        m_frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        m_backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        /*m_Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_Lift.setTargetPosition(0);
        m_Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        m_Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_turnTable.setTargetPosition(0);
        m_turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set claw to open position
        s_Claw.setPosition(0.95);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();



        double targetHeading = -imu.getAngularOrientation().firstAngle;

        isRotating = false;
        isLiftMoving = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            double leftStickX = gamepad1.left_stick_x;

            double leftStickX2 = gamepad2.left_stick_x;
            double rightStickX2 = gamepad2.right_stick_x;
            double leftTrigger2 = gamepad2.left_trigger;
            double rightTrigger2 = gamepad2.right_trigger;

            // TODO: record these values in the field
            // This is bad >:(
            if (gamepad1.right_bumper) {
                Config.COMY+=0.1;
            } else if (gamepad1.left_bumper) {
                Config.COMY-=0.1;
            }

            if (Math.abs(rightStickX) > 0) {
                isRotating = true;
            } else if (isRotating) {
                targetHeading = -imu.getAngularOrientation().firstAngle;
                isRotating = false;
            }

            telemetry.addData("Left Trigger", gamepad1.left_trigger);

            // Setting the claw position
            if (gamepad2.a) {
                s_Claw.setPosition(0);
            }
            if (gamepad2.b) {
                s_Claw.setPosition(1);
            }

            telemetry.addData("Claw Position", s_Claw.getPosition());



            if (Math.abs(leftStickX2)>0) {
                m_turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m_turnTable.setPower(leftStickX2);
            } else if (gamepad2.dpad_up) {
                m_turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m_turnTable.setTargetPosition(0);
                m_turnTable.setPower(1);
            } else if (gamepad2.dpad_left) {
                m_turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m_turnTable.setTargetPosition(-220);
                m_turnTable.setPower(1);
            } else if (gamepad2.dpad_right) {
                m_turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m_turnTable.setTargetPosition(220);
                m_turnTable.setPower(1);
            } else if (Math.abs(leftStickX2)==0) {
                m_turnTable.setPower(0);
            }

            /*if (gamepad2.right_bumper) {
                setLiftLevel(true);
            }
            if (gamepad2.left_bumper) {
                setLiftLevel(false);
            }*/

            /*if (leftTrigger2 > 0){
                m_Lift.setTargetPosition((int)(m_Lift.getCurrentPosition() + leftTrigger2 * 700));
            } else if (rightTrigger2 > 0) {
                m_Lift.setTargetPosition((int)(m_Lift.getCurrentPosition() - rightTrigger2 * 700));
            }
            m_Lift.setPower(1);*/

            if (leftTrigger2 > 0) {
                m_Lift.setVelocity(leftTrigger2*2200);
            } else if (rightTrigger2 > 0) {
                m_Lift.setVelocity(-rightTrigger2*2200);
            } else {
                m_Lift.setVelocity(0);
            }

            telemetry.addData("Lift Position", m_Lift.getCurrentPosition());
            telemetry.addData("Lift level", currentLiftLevel);

            telemetry.addLine(
                    String.format("COM X: %f, Y: %f", Config.   COMX, Config.COMY));

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double theta = Math.atan2(leftStickY, leftStickX);
            double power = Math.hypot(leftStickX, leftStickY);
            double frontLeftPow, frontRightPow, backLeftPow, backRightPow;

            double imuCompensation =
                    Math.abs(power) > 0 ? (botHeading - targetHeading) / (2*Math.PI) : 0;
            telemetry.addLine(String.format("Heading: %s Target: %s",botHeading, targetHeading));

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            frontLeftPow = power * cos / max + rightStickX - imuCompensation;
            frontRightPow = power * sin / max - rightStickX + imuCompensation;
            backLeftPow = power * sin / max + rightStickX - imuCompensation;
            backRightPow = power * cos / max - rightStickX + imuCompensation;

            if ((power + Math.abs(rightStickX)) > 1) {
                frontLeftPow /= power + rightStickX;
                frontRightPow /= power + rightStickX;
                backLeftPow /= power + rightStickX;
                backRightPow /= power + rightStickX;
            }

            telemetry.addData("Left Stick X",leftStickX);
            telemetry.addData("Left Stick Y",leftStickY);

            double[] power2 = Utils.findCompensation(leftStickX, leftStickY);
            // add code to circumvent strafe scalars if going straight
            // backwards or straight forwards (leftStickX == 0)
            // add gyro based straight driving
            telemetry.addLine(String.format("Velocity %f %f", power2[0], power2[1]));
            telemetry.addLine(String.format("%f %f",frontLeftPow,frontRightPow));
            m_frontLeft.setVelocity(frontLeftPow * 2200);
            m_backLeft.setVelocity(backLeftPow * power2[0] * 2200);
            m_frontRight.setVelocity(frontRightPow * 2200);
            m_backRight.setVelocity(backRightPow * power2[1] * 2200);

            oldLeftStickX2 = leftStickX2;
            oldRightStickX2 = rightStickX2;

            telemetry.update();
        }
    }
}