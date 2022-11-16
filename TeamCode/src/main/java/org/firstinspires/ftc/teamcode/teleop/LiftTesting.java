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
public class LiftTesting extends LinearOpMode {


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

        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor motorTurret = hardwareMap.get(DcMotor.class, "motorTurret");
        Servo servoSlider = hardwareMap.get(Servo.class, "servoTurret");
        Servo servoScissor = hardwareMap.get(Servo.class, "servoScissor");
        Servo servoScissorLift = hardwareMap.get(Servo.class, "servoScissorLift");
        TouchSensor liftSensorRight = hardwareMap.get(TouchSensor.class, "liftSensorRight");
        TouchSensor liftSensorLeft = hardwareMap.get(TouchSensor.class, "liftSensorLeft");

        motorLift2.setDirection(DcMotorSimple.Direction.REVERSE);

        // create scissorintake object

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
            if (gamepad2.left_stick_y < 0) {
                motorLift.setPower(0.80);
                motorLift2.setPower(0.80);
            } else if (gamepad2.left_stick_y > 0) {
                motorLift.setPower(-0.5);
                motorLift2.setPower(-0.5);
            } else {
                motorLift.setPower(0);
                motorLift2.setPower(0);
            }

            if(gamepad2.b) {
                motorLift.setPower(1);
                motorLift2.setPower(1);
            }


            // turret
            if (gamepad2.left_bumper) {
                motorTurret.setPower(-0.5);
            }
            if (gamepad2.right_bumper) {
                motorTurret.setPower(0.5);
            }
            else {
                motorTurret.setPower(0);
            }


            // horizontal slider
            if (sliderPos > MIN_POSITION && sliderPos < MAX_POSITION) {
                sliderPos += gamepad2.right_stick_y;
            }





            servoSlider.setPosition(Range.clip(sliderPos, MIN_POSITION, MAX_POSITION));


            telemetry.addData("Motor Lift Power:", motorLift.getPower());
            telemetry.addData("Motor Turret Power:", motorTurret.getPower());
            telemetry.addData("Horizontal Slider Position:", servoSlider.getPosition());


            telemetry.update();

        }

    }




}

