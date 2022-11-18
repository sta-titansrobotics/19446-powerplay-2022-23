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

    boolean pGA2UP = false;
    boolean pGA2DOWN = false;

    boolean pGA2A = false;
    boolean scissorToggle = false;

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


        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo servoScissor = hardwareMap.get(Servo.class, "servoScissor");
        Servo verticalServo = hardwareMap.get(Servo.class, "servoScissorLift");
        TouchSensor liftSensorRight = hardwareMap.get(TouchSensor.class, "liftSensorRight");
        TouchSensor liftSensorLeft = hardwareMap.get(TouchSensor.class, "liftSensorLeft");

        // Reverse right lift motor
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        double verticalServoPos, scissorPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        waitForStart();

        // set initial positions
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
                    leftLift.setPower(gamepad2.left_stick_y * 0.8);
                    rightLift.setPower(gamepad2.left_stick_y * 0.8);

            } else if (gamepad2.left_stick_y > 0) {

                if (leftLift.getCurrentPosition() > 0 || !liftSensorLeft.isPressed()) {
                    leftLift.setPower(gamepad2.left_stick_y * 0.4);
                }
                if (rightLift.getCurrentPosition() > 0 || !liftSensorRight.isPressed()) {
                    rightLift.setPower(gamepad2.left_stick_y * 0.4);
                }
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }

            // vertical slider
            boolean ga2UP = gamepad2.dpad_up;
            if (ga2UP && !pGA2UP && verticalServoPos > MIN_POSITION) {
                verticalServoPos -= 0.1;
            }
            pGA2UP = ga2UP;

            boolean ga2DOWN = gamepad2.dpad_down;
            if (ga2DOWN && !pGA2DOWN && verticalServoPos < MAX_POSITION) {
                verticalServoPos += 0.1;
            }
            pGA2DOWN = ga2DOWN;

            // scissor intake
            boolean ga2A = gamepad2.a;
            if (ga2A && !pGA2A) {
                scissorToggle = !scissorToggle;
            }

            // pick up (expand scissor)
            if (scissorToggle) {
                scissorPos = 0.67;
            }
            // release cone (neutral position)
            else {
                scissorPos = 0.5;
            }
            pGA2A = ga2A;


            // set positions to servos
            verticalServo.setPosition(rangeclip(verticalServoPos, MIN_POSITION, MAX_POSITION));
            servoScissor.setPosition(scissorPos);

            // add telemetry data
            telemetry.addData("Left Lift Power: ", leftLift.getPower());
            telemetry.addData("Right Lift Power: ", rightLift.getPower());
            telemetry.addData("Left Lift Encoder: ", leftLift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder: ", rightLift.getCurrentPosition());
            telemetry.addData("Vertical Slider Position: ", verticalServo.getPosition());
            telemetry.addData("Scissor Intake Position: ", servoScissor.getPosition());
            telemetry.addData("Left Touch Sensor: ", liftSensorLeft.isPressed());
            telemetry.addData("Right Touch Sensor: ", liftSensorRight.isPressed());



            telemetry.update();

        }

    }

    private double rangeclip(double number, double min, double max) {
        if(number > max) {
            return max;
        } else if (number < min) {
            return min;
        } else {
            return number;
        }
    }




}

