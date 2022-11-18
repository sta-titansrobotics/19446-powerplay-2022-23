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
public class Powerplay extends LinearOpMode {

    DcMotor motorFL, motorBL, motorFR, motorBR, leftLift, rightLift;

    boolean pGA2UP = false;
    boolean pGA2DOWN = false;

    boolean pGA2Y = false;

    boolean pGA2X = false;
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

        // lift motors
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

        int GROUND = 0;
        int LOW = 1700;
        int MIDDLE = 2900;
        int HIGH = 4000;

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

                leftLift.setPower(-gamepad2.left_stick_y * 0.85);
                rightLift.setPower(-gamepad2.left_stick_y * 0.85);

                if (verticalServoPos > MIN_POSITION) {
                    verticalServoPos -= 0.01;
                }

            } else if (gamepad2.left_stick_y > 0) {

                if (leftLift.getCurrentPosition() > 0) {
                    leftLift.setPower(-gamepad2.left_stick_y * 0.30);
                }

                if (rightLift.getCurrentPosition() > 0) {
                    rightLift.setPower(-gamepad2.left_stick_y * 0.30);
                }



                if (verticalServoPos < MAX_POSITION) {
                    verticalServoPos += 0.01;
                }
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }

            if (leftLift.getCurrentPosition() < 0 ) {
                leftLift.setPower(0.5);
            }

            if (rightLift.getCurrentPosition() < 0) {
                rightLift.setPower(0.5);
            }

            // vertical slider
            boolean ga2UP = gamepad2.dpad_up;
            if (ga2UP && !pGA2UP && verticalServoPos > MIN_POSITION) {
                verticalServoPos -= 0.25;
            }
            pGA2UP = ga2UP;

            boolean ga2DOWN = gamepad2.dpad_down;
            if (ga2DOWN && !pGA2DOWN && verticalServoPos < MAX_POSITION) {
                verticalServoPos += 0.25;
            }
            pGA2DOWN = ga2DOWN;

            // scissor intake
            boolean ga2Y = gamepad2.y;
            if (ga2Y && !pGA2Y) {
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
            pGA2Y = ga2Y;

            boolean ga2A = gamepad2.a;
            if (ga2A && !pGA2A) {
                moveLift(0.8, 3100);
            }
            pGA2A = ga2A;




            // set positions to servos
            verticalServo.setPosition(Range.clip(verticalServoPos, MIN_POSITION, MAX_POSITION));
            servoScissor.setPosition(Range.clip(scissorPos, MIN_POSITION, MAX_POSITION));

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

    /**
     * Powers lift to target position
     * @param power desired power
     * @param ticks target position
     */
    public void moveLift(double power, int ticks) {
        leftLift.setTargetPosition(ticks);
        rightLift.setTargetPosition(ticks);

        setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorPower(power);

        while(leftLift.isBusy() && rightLift.isBusy()) {

            telemetry.addData("encoder-left-lift", leftLift.getCurrentPosition() + " busy= " + leftLift.isBusy());
            telemetry.addData("encoder-right-lift", rightLift.getCurrentPosition() + " busy= " + rightLift.isBusy());
            telemetry.update();
        }

        motorPower(0);

        setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set power of both lift motors
     * @param power setPower
     */
    public void motorPower(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    /**
     * Change mode of cascading lift
     * @param mode setMode
     */
    public void setLiftMode(DcMotor.RunMode mode) {
        leftLift.setMode(mode);
        rightLift.setMode(mode);
    }




}

