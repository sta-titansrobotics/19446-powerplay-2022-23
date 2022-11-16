package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        DcMotor motorLift = hardwareMap.get(DcMotor.class, "leftLift");
        DcMotor motorLift2 = hardwareMap.get(DcMotor.class, "rightLift");
        Servo servoSlider = hardwareMap.get(Servo.class, "servoTurret");
        Servo servoScissor = hardwareMap.get(Servo.class, "servoScissor");
        Servo servoScissorLift = hardwareMap.get(Servo.class, "servoScissorLift");
        TouchSensor tSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        TouchSensor liftSensorRight = hardwareMap.get(TouchSensor.class, "liftSensorRight");
        TouchSensor liftSensorLeft = hardwareMap.get(TouchSensor.class, "liftSensorLeft");

        motorLift2.setDirection(DcMotorSimple.Direction.REVERSE);

        // create scissorintake object
        ScissorIntake intake = new ScissorIntake(servoScissorLift, servoScissor, tSensor);

        double sliderPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;
        int liftPreset = 0;
        int GROUND = 0;
        int LOW = 0;
        int MIDDLE = 0;
        int HIGH = 0;

        waitForStart();

        // reset slider pos
        sliderPos = 0.5;


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
            motorLift.setPower(gamepad2.left_stick_y);
            motorLift2.setPower(gamepad2.left_stick_y);


            // intake
            if (gamepad2.a) {
                intake.releaseCone();
            }
            if (gamepad2.b) {
                intake.contractScissor();
            }
            if (gamepad2.x) {
                intake.moveScissorToBottom();
            }
            if (gamepad2.y) {
                intake.moveScissorToTop();
            }



            // horizontal slider
            if (sliderPos > MIN_POSITION && sliderPos < MAX_POSITION) {
                sliderPos += gamepad2.right_stick_y;
            }

            /*if (gamepad2.dpad_up) {
                liftPreset++;

                if (liftPreset > 3) {
                    liftPreset = 0;
                }

            }

            if (gamepad2.dpad_down) {
                liftPreset--;

                if (liftPreset < 0) {
                    liftPreset = 3;
                }

            }

            if(gamepad2.b) {
                liftPreset = 1;
            }




            if (liftPreset == 0) {
                motorLift2.setTargetPosition(GROUND);
                motorLift.setTargetPosition(GROUND);
            } else if(liftPreset == 1) {
                motorLift2.setTargetPosition(LOW);
                motorLift.setTargetPosition(LOW);
            } else if (liftPreset == 2) {
                motorLift2.setTargetPosition(MIDDLE);
                motorLift.setTargetPosition(MIDDLE);
            } else if(liftPreset == 3) {
                motorLift2.setTargetPosition(HIGH);
                motorLift.setTargetPosition(HIGH);
            }


            motorLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */

            servoSlider.setPosition(Range.clip(sliderPos, MIN_POSITION, MAX_POSITION));


            telemetry.addData("Motor Lift Power:", motorLift.getPower());
            telemetry.addData("Horizontal Slider Position:", servoSlider.getPosition());


            telemetry.update();

        }

    }




}

