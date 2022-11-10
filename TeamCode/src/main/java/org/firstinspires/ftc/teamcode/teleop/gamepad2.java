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

        DcMotor motorLift = hardwareMap.get(DcMotor.class, "leftLift");
        DcMotor motorLift2 = hardwareMap.get(DcMotor.class, "rightLift");
        DcMotor motorTurret = hardwareMap.get(DcMotor.class, "motorTurret");
        Servo servoSlider = hardwareMap.get(Servo.class, "servoTurret");
        Servo servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        double sliderPos, clawPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;
        double MAXLIFT = 100;
        int liftPreset = 0;

        waitForStart();

        // reset slider pos and open claw
        sliderPos = 0;
        clawPos = 0;



        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // lift
            motorLift.setPower(gamepad2.left_stick_y);
            motorLift2.setPower(gamepad2.left_stick_y);


            if (motorLift2.getCurrentPosition() < MAXLIFT && motorLift.getCurrentPosition() < MAXLIFT) {
                motorLift2.setTargetPosition((int) gamepad2.left_stick_y + motorLift2.getCurrentPosition());
                motorLift.setTargetPosition((int) gamepad2.left_stick_y + motorLift.getCurrentPosition());
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
            if (sliderPos > MIN_POSITION && sliderPos < MAX_POSITION) {
                sliderPos += gamepad2.right_stick_y;
            }

            // claw a
            if (gamepad2.left_trigger > 0 && clawPos > MIN_POSITION) {
                clawPos -= -0.1;
            }
            if (gamepad2.right_trigger < 0 && clawPos < MAX_POSITION) {
                clawPos += 0.1;
            }


            if (gamepad2.dpad_up) {
                liftPreset++;

                if (liftPreset > 3) {
                    liftPreset = 0;
                }
            }

            if (liftPreset == 0) {

            } else if(liftPreset == 1) {

            } else if (liftPreset == 2) {

            } else if(liftPreset == 3) {

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
