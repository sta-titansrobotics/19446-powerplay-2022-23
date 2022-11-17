package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

//blah blah blah

@TeleOp
public class servoTesting extends LinearOpMode {


    @Override
    public void runOpMode() {

        // Movement Motors
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Other
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo horizontalServo = hardwareMap.get(Servo.class, "servoTurret");
        Servo servoScissor = hardwareMap.get(Servo.class, "servoScissor");
        Servo verticalServo = hardwareMap.get(Servo.class, "servoScissorLift");
        TouchSensor liftSensorRight = hardwareMap.get(TouchSensor.class, "liftSensorRight");
        TouchSensor liftSensorLeft = hardwareMap.get(TouchSensor.class, "liftSensorLeft");
        

        // Reverse right lift motor
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        double sliderPos, verticalServoPos, scissorPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        waitForStart();

        // reset slider pos
        sliderPos = 0.5;
        verticalServoPos = 0.5;
        scissorPos = 0.5;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // forward, back
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!

            // strafing
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing

            // turning
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

            // lift
            if (gamepad2.left_stick_y < 0) {

                if (!liftSensorLeft.isPressed()) {
                    leftLift.setPower(0.80);
                }
                if (!liftSensorRight.isPressed()) {
                    rightLift.setPower(0.80);
                }

            } else if (gamepad2.left_stick_y > 0) {

                if (!liftSensorLeft.isPressed()) {
                    leftLift.setPower(-0.5);
                }
                if (!liftSensorRight.isPressed()) {
                    rightLift.setPower(-0.5);
                }
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }

            // horizontal slider (rp)
            if (gamepad2.right_stick_y < 0 && sliderPos < MAX_POSITION) {
                sliderPos += 0.01;
            }

            if (gamepad2.right_stick_y > 0 && sliderPos > MIN_POSITION) {
                sliderPos -= 0.01;
            }

            // vertical slider
            if (gamepad2.right_stick_x > 0) {
                verticalServoPos += 0.1;
            }

            if (gamepad2.right_stick_x < 0) {
                verticalServoPos += 0.1;
            }

            // scissor intake
            if (gamepad2.left_stick_x < 0) {
                scissorPos += 0.01;
            }
            if (gamepad2.left_stick_x > 0) {
                scissorPos -= 0.01;
            }


            // set positions to servos
            horizontalServo.setPosition(Range.clip(sliderPos, MIN_POSITION, MAX_POSITION));
            verticalServo.setPosition(Range.clip(verticalServoPos, MIN_POSITION, MAX_POSITION));
            servoScissor.setPosition(Range.clip(scissorPos, MIN_POSITION, MAX_POSITION));

            // add telemetry data
            telemetry.addData("Left Lift Encoder: ", leftLift.getCurrentPosition());
            telemetry.addData("Left Lift Encoder: ", leftLift.getCurrentPosition());
            telemetry.addData("Horizontal Slider Position: ", horizontalServo.getPosition());
            telemetry.addData("Vertical Slider Position: ", verticalServo.getPosition());
            telemetry.addData("Scissor Intake Position: ", servoScissor.getPosition());

            telemetry.update();

        }

    }




}

