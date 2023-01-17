package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TEST_OP_BALLS (Blocks to Java)")
@Disabled
public class TEST_OP_BALLS extends LinearOpMode {

  private DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor2");
  private DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor3");

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    motorBackLeft = hardwareMap.get(DcMotor.class, "motor2");
    motorFrontRight = hardwareMap.get(DcMotor.class, "motor3");
    
    motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    
    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        motorBackLeft.setPower(1);
        motorFrontRight.setPower(1);
        
        telemetry.update();
      }
    }
  }
}
