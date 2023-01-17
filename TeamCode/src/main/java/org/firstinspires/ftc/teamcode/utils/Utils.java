package org.firstinspires.ftc.teamcode.utils;

public class Utils {
    private static double in(double l) {
        return l * Config.SCALE;
    }

    public static int inchesToTicks(double inches, double ticksPerRev, double wheelDiameter) {
        return (int)((ticksPerRev / wheelDiameter * Math.PI * inches) + 0.5);
    }

    public static double[] findCompensation(double headingI, double headingJ) {
        double SIZE = Config.SIZE;
        double UNITPIXLEN = in(Config.UNITLEN)/SIZE;
        double scaleLeft = 1;
        double scaleRight = 1;

        // Draw center line
        double axlePixelDistance = in(Config.axleDistance/2.0)/SIZE;
        double frontAxleY = 0.5+axlePixelDistance;
        double backAxleY = 0.5-axlePixelDistance;

        // Draw front axle line
        double frontAxlePixelLength = in(Config.frontAxleLength/2.0)/SIZE;
        double frontLeftWheelX = 0.5-frontAxlePixelLength;
        double frontRightWheelX = 0.5+frontAxlePixelLength;

        // Draw back axle line
        double backAxlePixelLength = in(Config.backAxleLength/2.0)/SIZE;
        double backLeftWheelX = 0.5-backAxlePixelLength;
        double backRightWheelX = 0.5+backAxlePixelLength;


        // Draw center of mass
        double pixelCOMX = in(Config.COMX)/SIZE;
        double pixelCOMY = in(Config.COMY)/SIZE;

        // Build COM to wheel vectors
        Vector frontLeftCOMVector = new Vector(0.5+pixelCOMX,0.5+pixelCOMY, frontLeftWheelX-0.5-pixelCOMX,frontAxleY-0.5-pixelCOMY);
        Vector frontRightCOMVector = new Vector(0.5+pixelCOMX,0.5+pixelCOMY, frontRightWheelX-0.5-pixelCOMX,frontAxleY-0.5-pixelCOMY);
        Vector backLeftCOMVector = new Vector(0.5+pixelCOMX,0.5+pixelCOMY, backLeftWheelX-0.5-pixelCOMX,backAxleY-0.5-pixelCOMY);
        Vector backRightCOMVector = new Vector(0.5+pixelCOMX,0.5+pixelCOMY, backRightWheelX-0.5-pixelCOMX,backAxleY-0.5-pixelCOMY);

        // Draw COM to wheel vectors
        /*frontLeftCOMVector.draw(canvas.RED, 0.005);
        frontRightCOMVector.draw(canvas.RED, 0.005);
        backLeftCOMVector.draw(canvas.RED, 0.005);
        backRightCOMVector.draw(canvas.RED, 0.005);*/

        // Build Heading Vector
        Vector headingVector = new Vector(0.5+pixelCOMX,0.5+pixelCOMY, in(headingI)/SIZE, in(headingJ)/SIZE);

        // Build wheel drive vectors
        Vector frontLeftDriveVector = new Vector(frontLeftWheelX, frontAxleY, UNITPIXLEN*(-1), UNITPIXLEN);
        Vector frontRightDriveVector = new Vector(frontRightWheelX, frontAxleY, UNITPIXLEN, UNITPIXLEN);
        Vector backLeftDriveVector = new Vector(backLeftWheelX, backAxleY, UNITPIXLEN, UNITPIXLEN);
        Vector backRightDriveVector = new Vector(backRightWheelX, backAxleY, UNITPIXLEN*(-1), UNITPIXLEN);
        System.out.println(frontLeftDriveVector.length());

        // Project heading onto drive vectors
        Vector frontLeftVector = frontLeftDriveVector.parallelProjection(headingVector, frontLeftDriveVector.x, frontLeftDriveVector.y);
        Vector frontRightVector = frontRightDriveVector.parallelProjection(headingVector, frontRightDriveVector.x, frontRightDriveVector.y);
        Vector backLeftVector = backLeftDriveVector.parallelProjection(headingVector, backLeftDriveVector.x, backLeftDriveVector.y);
        Vector backRightVector = backRightDriveVector.parallelProjection(headingVector, backRightDriveVector.x, backRightDriveVector.y);

        backLeftVector.scale(scaleLeft);
        backRightVector.scale(scaleRight);

        // Calc wheel velocity
        double wheelPixelWidth = in(Config.wheelWidth)/SIZE;
        double wheelPixelHeight = in(Config.wheelHeight)/SIZE;
        double frontLeftVel = frontLeftVector.length()*Math.signum(frontLeftVector.dot(frontLeftDriveVector));
        double frontRightVel = frontRightVector.length()*Math.signum(frontRightVector.dot(frontRightDriveVector));
        double backLeftVel = backLeftVector.length()*Math.signum(backLeftVector.dot(backLeftDriveVector));
        double backRightVel = backRightVector.length()*Math.signum(backRightVector.dot(backRightDriveVector));

        // Calc torque vectors
        Vector frontLeftTorqueVector = frontLeftCOMVector.perpendicularProjection(frontLeftDriveVector,frontLeftDriveVector.x, frontLeftDriveVector.y);
        Vector frontRightTorqueVector = frontRightCOMVector.perpendicularProjection(frontRightDriveVector,frontRightDriveVector.x, frontRightDriveVector.y);
        Vector backLeftTorqueVector = backLeftCOMVector.perpendicularProjection(backLeftDriveVector,backLeftDriveVector.x, backLeftDriveVector.y);
        Vector backRightTorqueVector = backRightCOMVector.perpendicularProjection(backRightDriveVector,backRightDriveVector.x, backRightDriveVector.y);

        // Scale vectors by radius and speed
        frontLeftTorqueVector.scale(frontLeftVel*frontLeftCOMVector.length()*SIZE);
        frontRightTorqueVector.scale(frontRightVel*frontRightCOMVector.length()*SIZE);
        backLeftTorqueVector.scale(backLeftVel*backLeftCOMVector.length()*SIZE);
        backRightTorqueVector.scale(backRightVel*backRightCOMVector.length()*SIZE);

        double resScaleRight = backRightTorqueVector.length()==0?1:frontLeftTorqueVector.length()/backRightTorqueVector.length();
        double resScaleLeft = backLeftTorqueVector.length()==0?1:frontRightTorqueVector.length()/backLeftTorqueVector.length();
        double[] result = {resScaleLeft,resScaleRight};
        return result;
    }


}
