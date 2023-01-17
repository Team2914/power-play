package org.firstinspires.ftc.teamcode.utils;

public class Config {
    public static double frontAxleLength = 3;
    public static double backAxleLength = 16;
    public static double axleDistance = 7;
    public static double COMY = 2.9;              // Center of mass Y
    public static double COMX = 0;              // Center of mass X
    public static double wheelWidth = 1;
    public static double wheelHeight = 2;

    public static double SIZE = 500.0;
    public static double SCALE = SIZE / 20.0;
    public static double UNITLEN = 1/Math.sqrt(2);

    public static final double VEX_CORE_HEX_TICK = 288;
    // I think 28 is correct...
    // It is listed as "at the motor," but Reddit says its fine :shrug:
    public static final double ULTRA_PLANET_TICK = 28;

    public static final String VUFORIA_KEY = "AYP9/xH/////AAABmV3Tt3E7I0X5jvXrfuN8EP11OHARxtAiPkw7bHB2rR+tug9c+WHPtp4pVdgVMvIuio5u8+5EZsLRBdscc6rdoG+9UrqyhApQIGskmsupzO6b3RYmX/GhITPM8uUg2TL2xmDD1HvY0cqaYwOoXu5fhTAUoInalV7k9qVtTyG1Zv60293uzjgZ8eY5OJX7eM9EsR1/Slfx5VnLQsSssNLqz65GW+O+WmcTgwV3ugjEwq8XLrtx0oeUz3dqBDhSKxZFut0rKjbd/T6AugsxhZBWudiFZW25Q7jYSIPJmuWldU15giYb0FSYapSEMWtRUD68/zMm+eAH1YtU54psOOTP19P7a7RvLpRF2xLqhRCUllMj";
    // Taken from ConceptVuforiaFieldNavigationWebcam
    // Since ImageTarget trackables use mm to specifiy their dimensions,
    // we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float MM_PER_INCH = 25.4f;
    // mmTargetHeight: the height of the center of the target image above the floor
    public static final float MM_TARGET_HEIGHT = 6 * MM_PER_INCH;
    public static final float HALF_FIELD = 72 * MM_PER_INCH;
    public static final float HALF_TILE = 12 * MM_PER_INCH;
    public static final float ONE_AND_HALF_TILE = 36 * MM_PER_INCH;
    public static final float ONE_TILE_INCHES = 22.75f;

    public static final float CAMERA_FORWARD_DISPLACEMENT  = 2.0f * MM_PER_INCH;
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 4.5f * MM_PER_INCH;
    public static final float CAMERA_LEFT_DISPLACEMENT     = 6.5f * MM_PER_INCH;

    public static final int[] LIFT_LEVELS = {0, -300, -1900, -2900};
}
