package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@Disabled
public class FreightFrenzyTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() {

        // Movement Motors
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Intake Motors
        DcMotor motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        DcMotor motorTurret = hardwareMap.get(DcMotor.class, "motorTurret");
        Servo servoSlider = hardwareMap.get(Servo.class, "servoTurret");
        Servo servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Driving
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!

            //STRAFING VARIABLE
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing

            //THIS IS THE TURNING VARIABLE
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

            if (gamepad1.dpad_up) {
                motorFL.setPower(1);
                motorBL.setPower(1);
                motorFR.setPower(1);
                motorBR.setPower(1);
            }

            if (gamepad1.dpad_down) {
                motorFL.setPower(-1);
                motorBL.setPower(-1);
                motorFR.setPower(-1);
                motorBR.setPower(-1);
            }

            if (gamepad1.dpad_left) {
                motorFL.setPower(-1);
                motorBL.setPower(1);
                motorFR.setPower(1);
                motorBR.setPower(-1);
            }

            if (gamepad1.dpad_right) {
                motorFL.setPower(1);
                motorBL.setPower(-1);
                motorFR.setPower(-1);
                motorBR.setPower(1);
            }

            double sliderPos, clawPos;
            double MIN_POSITION = 0, MAX_POSITION = 1;

            // reset slider pos and open claw
            sliderPos = 0.5;
            clawPos = 1;


            // lift
            double liftPower = gamepad2.left_stick_y;

            motorLift.setPower(liftPower);

            // turret
            if (gamepad2.left_bumper) {
                motorTurret.setPower(-1);
            }
            if (gamepad2.right_bumper) {
                motorTurret.setPower(1);
            } else {
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
            if (gamepad2.left_trigger > 0 && clawPos > MIN_POSITION) {
                clawPos -= 0.1;
            }
            if (gamepad2.right_trigger > 0 && clawPos < MAX_POSITION) {
                clawPos += 0.1;
            }

            servoSlider.setPosition(Range.clip(sliderPos, MIN_POSITION, MAX_POSITION));
            servoClaw.setPosition(Range.clip(clawPos, MIN_POSITION, MAX_POSITION));


            telemetry.addData("Motor Lift Power:", motorLift.getPower());
            telemetry.addData("Motor Turret Power:", motorTurret.getPower());
            telemetry.addData("Horizontal Slider Position:", servoSlider.getPosition());
            telemetry.addData("Claw Position:", servoClaw.getPosition());
            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());
            telemetry.update();

            }
        }
    }

