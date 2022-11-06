package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//blah blah blah

@TeleOp
public class gamepad2 extends LinearOpMode {


    @Override
    public void runOpMode() {

        DcMotor motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        DcMotor motorTurret = hardwareMap.get(DcMotor.class, "motorTurret");
        Servo servoSlider = hardwareMap.get(Servo.class, "servoTurret");
        Servo servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        double sliderPos, clawPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        waitForStart();

        // reset slider pos and open claw
        sliderPos = 0.5;
        clawPos = 1;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // lift
            if (gamepad2.left_stick_y > 0) {
                motorLift.setPower(-1);
            }
            if (gamepad2.left_stick_y < 0) {
                motorLift.setPower(1);
            }
            else {
                motorLift.setPower(0);
            }

            // turret
            if (gamepad2.left_bumper) {
                motorTurret.setPower(-1);
            }
            if (gamepad2.right_bumper) {
                motorTurret.setPower(1);
            }
            else {
                motorTurret.setPower(0);
            }

            // horizontal slider
            if (gamepad2.dpad_up && sliderPos < MAX_POSITION) {
                sliderPos += 0.1;

            }
            if (gamepad2.dpad_down && sliderPos > MIN_POSITION) {
                sliderPos -= -0.1;
            }

            // claw a =
            if (gamepad2.a && clawPos > MIN_POSITION) {
                clawPos -= -0.1;
            }
            if (gamepad2.b && clawPos < MAX_POSITION) {
                clawPos += 0.1;
            }

            servoSlider.setPosition(Range.clip(sliderPos, MIN_POSITION, MAX_POSITION));
            servoClaw.setPosition(Range.clip(clawPos, MIN_POSITION, MAX_POSITION));

            telemetry.addData("Motor Lift Power:", motorLift.getPower());
            telemetry.addData("Motor Turret Power:", motorTurret.getPower());
            telemetry.addData("Horizontal Slider Position:", servoSlider.getPosition());
            telemetry.addData("Claw Position:", servoClaw.getPosition());
            telemetry.update();

        }
    }
}
