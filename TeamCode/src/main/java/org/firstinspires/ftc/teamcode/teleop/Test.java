package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//blah blah blah

@TeleOp
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() {

        DcMotor motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        DcMotor motorTurret = hardwareMap.get(DcMotor.class, "motorTurret");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // lift
            if (gamepad2.dpad_up) {
                motorLift.setPower(1);
            }
            if (gamepad2.dpad_down) {
                motorLift.setPower(-1);
            }
            else {
                motorLift.setPower(0);
            }

            // turret
            if (gamepad2.dpad_left) {
                motorTurret.setPower(-1);
            }
            if (gamepad2.dpad_right) {
                motorTurret.setPower(1);
            }
            else {
                motorTurret.setPower(0);
            }

            // servo stuff


            telemetry.addData("Motor Lift Power:", motorLift.getPower());
            telemetry.addData("Motor Turret Power:", motorTurret.getPower());
            telemetry.update();

        }
    }
}
