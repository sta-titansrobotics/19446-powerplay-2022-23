package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//blah blah blah

@TeleOp
public class Powerplay extends LinearOpMode {

    DcMotor motorFL, motorBL, motorFR, motorBR, leftLift, rightLift;
    boolean pGA2UP = false;
    boolean pGA2DOWN = false;

    @Override
    public void runOpMode() {

        // Movement Motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Other
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        Servo servoScissor = hardwareMap.get(Servo.class, "servoScissor");
        Servo servoScissorLift = hardwareMap.get(Servo.class, "servoScissorLift");
        TouchSensor tSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        TouchSensor liftSensorRight = hardwareMap.get(TouchSensor.class, "liftSensorRight");
        TouchSensor liftSensorLeft = hardwareMap.get(TouchSensor.class, "liftSensorLeft");

        // create scissorintake object
        ScissorIntake intake = new ScissorIntake(servoScissorLift, servoScissor, tSensor);

        double MIN_POSITION = 0, MAX_POSITION = 1, MAX_LIFT_POSITION = 100000;
        int liftPreset = 0;
        int GROUND = 0;
        int LOW = 1700;
        int MIDDLE = 2900;
        int HIGH = 4000;

        waitForStart();

        // reset slider pos
        double servoScissorPos = 0.5;
        double scissorPos = 0;


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


            // lift

            if (gamepad2.left_stick_y > 0) {
                leftLift.setTargetPosition(leftLift.getTargetPosition() + 100);
                rightLift.setPower(leftLift.getTargetPosition() + 100);

                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if(leftLift.getTargetPosition() == MAX_LIFT_POSITION) {
                    if (tSensor.isPressed()) {
                        scissorPos -= gamepad2.left_stick_y;
                    }
                }

            }

            if(gamepad2.left_stick_y < 0) {
                leftLift.setTargetPosition(leftLift.getTargetPosition() + 100);
                rightLift.setPower(leftLift.getTargetPosition() + 100);

                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (tSensor.isPressed()) {
                    scissorPos += gamepad2.left_stick_y;
                }
            }


            // falling edge detectors for click once
            boolean ga2UP = gamepad2.dpad_up;
            if (ga2UP && !pGA2UP) {
                if (liftPreset < 3) {
                    liftPreset++;
                }
                if (liftPreset == 0) {
                    moveLift(0.5, GROUND);
                } else if(liftPreset == 1) {
                    moveLift(0.5, LOW);
                } else if (liftPreset == 2) {
                    moveLift(0.5, MIDDLE);
                } else if(liftPreset == 3) {
                    moveLift(0.5, HIGH);
                }
            }
            pGA2UP = ga2UP;

            boolean ga2DOWN = gamepad2.dpad_down;
            if (ga2DOWN && !pGA2DOWN) {
                if (liftPreset > 0) {
                    liftPreset--;
                }
                if (liftPreset == 0) {
                    moveLift(0.5, GROUND);
                } else if(liftPreset == 1) {
                    moveLift(0.5, LOW);
                } else if (liftPreset == 2) {
                    moveLift(0.5, MIDDLE);
                } else if(liftPreset == 3) {
                    moveLift(0.5, HIGH);
                }
            }
            pGA2DOWN = ga2DOWN;



            servoScissorLift.setPosition(Range.clip(servoScissorPos, MIN_POSITION, MAX_POSITION));
            servoScissor.setPosition(Range.clip(scissorPos, MIN_POSITION, MAX_POSITION));


            telemetry.addData("Motor Lift Power:", leftLift.getPower());
            telemetry.addData("Horizontal Slider Position:", servoScissorLift.getPosition());
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

