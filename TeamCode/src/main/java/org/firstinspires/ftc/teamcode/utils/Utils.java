package org.firstinspires.ftc.teamcode.utils;

public class Utils {
    private static double inchToUnit(double l) {
        return l * Config.scale;
    }

    public static double[] findCompensation(double headingX, double headingY) {
        final double frontAxleLen = Config.frontAxleLen;
        final double backAxleLen = Config.backAxleLen;
        final double axleDist = Config.axleDistance;
        final double comY = Config.comY;
        final double comX = Config.comX;
        final double wheelWidth = Config.wheelWidth;
        final double wheelHeight = Config.wheelHeight;
        final double planeSize = Config.planeSize;
        final double scale = Config.scale;
        final double unitLen = Config.unitLen;
        final double unitLen2 = inchToUnit(unitLen) / planeSize;

        double scaleLeft = 1;
        double scaleRight = 1;

        double axleUnitDist = inchToUnit(axleDist / 2.0) / planeSize;
        double frontAxleY = 0.5 + axleUnitDist;
        double backAxleY = 0.5 - axleUnitDist;

        double frontAxleUnitLen = inchToUnit(frontAxleLen / 2.0) / planeSize;
        double frontLWheelX = 0.5 - frontAxleUnitLen;
        double frontRWheelX = 0.5 + frontAxleUnitLen;

        double backAxleUnitLen = inchToUnit(backAxleLen / 2.0) / planeSize;
        double backLWheelX = 0.5 - backAxleUnitLen;
        double backRWheelX = 0.5 + backAxleUnitLen;

        // The center of mass in our plane
        double unitCOMX = 0.5 + (inchToUnit(comX) / planeSize);
        double unitCOMY = 0.5 + (inchToUnit(comY) / planeSize);

        // Heading vector
        Vector2 v_heading =
                new Vector2(unitCOMX, unitCOMY, inchToUnit(headingX) / planeSize, inchToUnit(headingY) / planeSize);

        // Center of mass to wheel vectors
        Vector2 v_frontLeftCOM =
                new Vector2(unitCOMX, unitCOMY, frontLWheelX - unitCOMX, frontAxleY - unitCOMY);
        Vector2 v_frontRightCOM =
                new Vector2(unitCOMX, unitCOMY, frontRWheelX - unitCOMX, frontAxleY - unitCOMX);
        Vector2 v_backLeftCOM =
                new Vector2(unitCOMX, unitCOMY, backLWheelX - unitCOMX, backAxleY - unitCOMY);
        Vector2 v_backRightCOM =
                new Vector2(unitCOMX, unitCOMY, backRWheelX - unitCOMX, backAxleY - unitCOMY);

        // Wheel drive vectors
        Vector2 v_frontLeftDrive =
                new Vector2(frontLWheelX, frontAxleY, -unitLen2, unitLen2);
        Vector2 v_frontRightDrive =
                new Vector2(frontRWheelX, frontAxleY, unitLen2, unitLen2);
        Vector2 v_backLeftDrive =
                new Vector2(backLWheelX, backAxleY, unitLen2, unitLen2);
        Vector2 v_backRightDrive =
                new Vector2(backRWheelX, backAxleY, -unitLen2, unitLen2);

        // Project heading onto drive vectors
        Vector2 v_frontLeft =
                v_frontLeftDrive.parallelProj(v_heading, v_frontLeftDrive.x, v_frontLeftDrive.y);
        Vector2 v_frontRight =
                v_frontRightDrive.parallelProj(v_heading, v_frontRightDrive.x, v_frontRightDrive.y);
        Vector2 v_backLeft =
                v_backLeftDrive.parallelProj(v_heading, v_backLeftDrive.x, v_backLeftDrive.y);
        Vector2 v_backRight =
                v_backRightDrive.parallelProj(v_heading, v_backRightDrive.x, v_backRightDrive.y);

        v_backLeft.scale(scaleLeft);
        v_backRight.scale(scaleRight);

        // Calculate wheel velocity
        double frontLeftVel =
                v_frontLeft.len() * Math.signum(v_frontLeft.dot(v_frontLeftDrive));
        double frontRightVel =
                v_frontRight.len() * Math.signum(v_frontRight.dot(v_frontRightDrive));
        double backLeftVel =
                v_backLeft.len() * Math.signum(v_backLeft.dot(v_backLeftDrive));
        double backRightVel =
                v_backRight.len() * Math.signum(v_backRight.dot(v_backRightDrive));

        // Calculate torque vectors
        Vector2 v_frontLeftTorque =
                v_frontLeftCOM.perpendicularProj(v_frontLeftDrive, v_frontLeftDrive.x, v_frontLeftDrive.y);


    }
}
