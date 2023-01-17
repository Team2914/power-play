package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.utils.Config.CAMERA_FORWARD_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.utils.Config.CAMERA_LEFT_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.utils.Config.CAMERA_VERTICAL_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.utils.Config.HALF_FIELD;
import static org.firstinspires.ftc.teamcode.utils.Config.MM_TARGET_HEIGHT;
import static org.firstinspires.ftc.teamcode.utils.Config.ONE_AND_HALF_TILE;
import static org.firstinspires.ftc.teamcode.utils.Config.VUFORIA_KEY;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.utils.Config;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.ArrayList;
import java.util.List;

// TODO: implement encoder movement (move in inches)
// TODO: implement claw and servo logic

@Autonomous(name = "BlueRearParking3")
public class BlueRearParking3 extends LinearOpMode {
    // Hardware
    private DcMotorEx m_frontLeft;
    private DcMotorEx m_frontRight;
    private DcMotorEx m_backLeft;
    private DcMotorEx m_backRight;
    private BNO055IMU imu;
    private Servo s_Claw;
    private DcMotorEx m_turnTable;
    private DcMotorEx m_Lift;

    // Vision
    protected static OpenGLMatrix lastLocation   = null;
    protected static VuforiaLocalizer vuforia    = null;
    protected static VuforiaTrackables targets   = null;
    protected static List<VuforiaTrackable> allTrackables;
    protected static WebcamName webcamName       = null;
    protected static boolean targetVisible       = false;
    // Cam preview takes up a lot of the RC's battery
    protected static final boolean showCamPreview = false;
    protected static int camViewId = 0;
    protected static boolean foundSignal = false;
    protected static int parkingLocation = 0;

    // Movement queue
    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backLeftTarget = 0;
    private int backRightTarget = 0;
    private int liftTarget = 0;
    private int servoTarget = 0;
    private boolean[] queueCompleted;
    private int currentQueuePosition;


    // COORDINATES SYSTEM (looking from the Red Alliance) :
    // X: center is origin, -x is left, +x is right;
    // Y: center is origin, +y is to Blue Alliance, -y is to Red Alliance
    // Z: center is origin, regular up down

    // pos : target offsets in xyz
    // rot : target rotations in xyz
    private void identifyTarget(int index, String name, float[] pos, float[] rot) {
        VuforiaTrackable target = targets.get(index);
        target.setName(name);
        target.setLocation(OpenGLMatrix.translation(pos[0], pos[1], pos[2])
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES,
                        rot[0], rot[1], rot[2]
                )));
    }

    private void initVision() {
        foundSignal = false;
        VuforiaLocalizer.Parameters parameters;

        if (showCamPreview) {
            parameters = new VuforiaLocalizer.Parameters(camViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        vuforia =
                ClassFactory.getInstance().createVuforia(parameters);
        targets =
                vuforia.loadTrackablesFromAsset("PowerPlay");
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targets);

        identifyTarget(3, "RedRear",
                new float[] {HALF_FIELD, -ONE_AND_HALF_TILE, MM_TARGET_HEIGHT},
                new float[] {90, 0, -90});
        identifyTarget(4, "RedAudience",
                new float[] {-HALF_FIELD, -ONE_AND_HALF_TILE, MM_TARGET_HEIGHT},
                new float[] {90, 0, 90});
        identifyTarget(5, "BlueRear",
                new float[] {HALF_FIELD, ONE_AND_HALF_TILE, MM_TARGET_HEIGHT},
                new float[] {90, 0, -90});
        identifyTarget(6, "BlueAudience",
                new float[] {-HALF_FIELD, ONE_AND_HALF_TILE, MM_TARGET_HEIGHT},
                new float[] {90, 0, 90});

        targets.get(0).setName("Parking1");
        targets.get(1).setName("Parking2");
        targets.get(2).setName("Parking3");

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT,
                        CAMERA_LEFT_DISPLACEMENT,
                        CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC,
                        XZY,
                        DEGREES,
                        90, 90, 0));

        // Tell the trackers where the camera is
        // TODO: get precise camera pos

        if (parameters.cameraName == null) return;

        for (VuforiaTrackable trackable : allTrackables) {
            VuforiaTrackableDefaultListener trackableListener =
                    (VuforiaTrackableDefaultListener)trackable.getListener();

            trackableListener.setCameraLocationOnRobot(parameters.cameraName,
                    cameraLocationOnRobot);
        }

    }

    @SuppressLint("DefaultLocale")
    private void updateVision() {
        if (foundSignal) return;

        targetVisible = false;

        for (VuforiaTrackable trackable : allTrackables) {
            VuforiaTrackableDefaultListener trackableListener =
                    (VuforiaTrackableDefaultListener)trackable.getListener();

            if (!trackableListener.isVisible()) continue;

            telemetry.addData("Visible target", trackable.getName());
            targetVisible = true;

            if (!foundSignal) {
                if (trackable.getName().equals("Parking1")) {
                    foundSignal = true;
                    parkingLocation = 1;
                }
                if (trackable.getName().equals("Parking2")) {
                    foundSignal = true;
                    parkingLocation = 2;
                }
                if (trackable.getName().equals("Parking3")) {
                    foundSignal = true;
                    parkingLocation = 3;
                }
            }

            OpenGLMatrix robotLocationTransform =
                    trackableListener.getUpdatedRobotLocation();

            if (robotLocationTransform == null) continue;

            lastLocation = robotLocationTransform;

            break;
        }
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

        m_frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_frontLeft.setTargetPosition(0);
        m_frontRight.setTargetPosition(0);
        m_backLeft.setTargetPosition(0);
        m_backRight.setTargetPosition(0);

        m_frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        m_Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_Lift.setTargetPosition(0);
        m_Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set claw to open position
        s_Claw.setPosition(0.1);

        // Vision
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camViewId = hardwareMap.appContext.
                getResources().
                getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName());
    }

    // Front left = 0, front right = 1, back left = 2, back right = 3;
    private void updateMotorEncoders(int[] ticks, int queuePos) {
        if (currentQueuePosition != queuePos) return;
        // Return if has run before
        if (queueCompleted[queuePos]) return;

        frontLeftTarget = m_frontLeft.getCurrentPosition() + ticks[0];
        frontRightTarget = m_frontRight.getCurrentPosition() + ticks[1];
        backLeftTarget = m_backLeft.getCurrentPosition() + ticks[2];
        backRightTarget = m_backRight.getCurrentPosition() + ticks[3];

        m_frontLeft.setTargetPosition(frontLeftTarget);
        m_frontRight.setTargetPosition(frontRightTarget);
        m_backLeft.setTargetPosition(backLeftTarget);
        m_backRight.setTargetPosition(backRightTarget);

        m_frontLeft.setPower(1);
        m_frontRight.setPower(1);
        m_backLeft.setPower(1);
        m_backRight.setPower(1);

        queueCompleted[queuePos] = true;
    }

    private void updateMotorEncodersAndLift(int[] ticks, int liftPos, int queuePos) {
        if (currentQueuePosition != queuePos) return;
        // Return if has run before
        if (queueCompleted[queuePos]) return;

        frontLeftTarget = m_frontLeft.getCurrentPosition() + ticks[0];
        frontRightTarget = m_frontRight.getCurrentPosition() + ticks[1];
        backLeftTarget = m_backLeft.getCurrentPosition() + ticks[2];
        backRightTarget = m_backRight.getCurrentPosition() + ticks[3];

        m_frontLeft.setTargetPosition(frontLeftTarget);
        m_frontRight.setTargetPosition(frontRightTarget);
        m_backLeft.setTargetPosition(backLeftTarget);
        m_backRight.setTargetPosition(backRightTarget);

        m_Lift.setTargetPosition(Config.LIFT_LEVELS[liftPos]);
        m_Lift.setPower(1);

        m_frontLeft.setPower(1);
        m_frontRight.setPower(1);
        m_backLeft.setPower(1);
        m_backRight.setPower(1);

        queueCompleted[queuePos] = true;
    }

    private void driveInches(double distance, int queuePos) {
        int d = Utils.inchesToTicks(distance, Config.ULTRA_PLANET_TICK, 1.5);
        updateMotorEncoders(new int[] {d, d, d, d}, queuePos);
    }

    private void driveInchesAndLift(double distance, int liftPos, int queuePos) {
        int d = Utils.inchesToTicks(distance, Config.ULTRA_PLANET_TICK, 1.5);
        updateMotorEncodersAndLift(new int[] {d, d, d, d}, liftPos, queuePos);
    }

    private void rotate45CW(int queuePos) {
        updateMotorEncoders(new int[] {380, -440, 390, -450}, queuePos);
    }


    private void rotate90CCW(int queuePos) {
        updateMotorEncoders(new int[] {-440*2, 380*2, -450*2, 390*2}, queuePos);
    }

    private void rotate90CW(int queuePos) {
        updateMotorEncoders(new int[] {380*2, -440*2, 390*2, -450*2}, queuePos);
    }


    private void rotateFaceConeStorage(int queuePos) {
        updateMotorEncoders(new int[] {-1050, 1150, -1150, 1200}, queuePos);
    }

    private void rotate2() {

    }

    private void setClawPos(int pos, int queuePos) {
        if (currentQueuePosition != queuePos) return;
        // Return if has run before
        if (queueCompleted[queuePos]) return;

        servoTarget = pos;
        s_Claw.setPosition(pos);

        queueCompleted[queuePos] = true;
    }

    @Override
    public void runOpMode() {
        // INIT SHIT
        initHardware();
        initVision();
        currentQueuePosition = 0;
        queueCompleted = new boolean[512];
        for (int i = 0; i < 512; i++) queueCompleted[i] = false;
        frontLeftTarget = 0;
        frontRightTarget = 0;
        backLeftTarget = 0;
        backRightTarget = 0;
        liftTarget = 0;
        servoTarget = (int)1e9;
        parkingLocation = 0;

        s_Claw.setPosition(0);

        targets.activate();

        telemetry.addLine("INIT COMPLETE");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {
            /*while (Math.toDegrees(-imu.getAngularOrientation().firstAngle) > -130) {
                m_frontLeft.setTargetPosition(m_frontLeft.getCurrentPosition() - 10);
                m_backLeft.setTargetPosition(m_backLeft.getCurrentPosition() - 10);
                m_frontRight.setTargetPosition(m_frontRight.getCurrentPosition() + 10);
                m_backRight.setTargetPosition(m_backRight.getCurrentPosition() + 10);

                m_frontLeft.setPower(1);
                m_frontRight.setPower(1);
                m_backLeft.setPower(1);
                m_backRight.setPower(1);
            }

            telemetry.addData("Front left tick", m_frontLeft.getCurrentPosition());
            telemetry.addData("Back left tick", m_backLeft.getCurrentPosition());
            telemetry.addData("Front right tick", m_frontRight.getCurrentPosition());
            telemetry.addData("Back right tick", m_backRight.getCurrentPosition());*/
            updateVision();

            driveInches(Config.ONE_TILE_INCHES + 5, 0);
            rotate90CW(1);
            driveInches(Config.ONE_TILE_INCHES - 2, 2);


            int flp = Math.abs(m_frontLeft.getCurrentPosition() - frontLeftTarget);
            int frp = Math.abs(m_frontRight.getCurrentPosition() - frontRightTarget);
            int blp = Math.abs(m_backLeft.getCurrentPosition() - backLeftTarget);
            int brp = Math.abs(m_backRight.getCurrentPosition() - backRightTarget);
            int lp = Math.abs(m_Lift.getCurrentPosition() - liftTarget);
            double sp = Math.abs(s_Claw.getPosition() - servoTarget);

            if (((flp >= 0 && flp <= 40) && (frp >= 0 && frp <= 40) && (blp >= 0 && blp <= 40) && (brp >= 0 && brp <= 40)) ||
                    (sp >= 0 && sp <= 1)) {

                currentQueuePosition++;
                frontLeftTarget = (int)1e9;
                frontRightTarget = (int)1e9;
                backLeftTarget = (int)1e9;
                backRightTarget = (int)1e9;
                servoTarget = (int)1e9;
                //liftTarget = (int)1e9;
            }

            telemetry.addData("Parking zone", parkingLocation);
            telemetry.addData("Current movement queue position", currentQueuePosition);
            telemetry.addLine(String.format("FL TARGET: %d DIFF: %d", frontLeftTarget, flp));
            telemetry.addLine(String.format("FR TARGET: %d DIFF: %d", frontRightTarget, frp));
            telemetry.addLine(String.format("BL TARGET: %d DIFF: %d", backLeftTarget, blp));
            telemetry.addLine(String.format("BR TARGET: %d DIFF: %d", backRightTarget, brp));
            //telemetry.addLine(String.format("LIFT TARGET: %d DIFF: %d", liftTarget, lp));
            telemetry.update();
        }

        // Shut down
        targets.deactivate();

        m_frontLeft.setPower(0);
        m_frontRight.setPower(0);
        m_backLeft.setPower(0);
        m_backRight.setPower(0);
    }
}
