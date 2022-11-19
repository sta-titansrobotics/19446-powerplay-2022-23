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
        DcMotor motorTurret = hardwareMap.get(DcMotor.class, "motorTurret");
        Servo servoSlider = hardwareMap.get(Servo.class, "servoTurret");
        Servo servoScissor = hardwareMap.get(Servo.class, "servoScissor");
        Servo servoScissorLift = hardwareMap.get(Servo.class, "servoScissorLift");
        TouchSensor tSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        DistanceSensor dSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // set motorTurret to encoder mode
        motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // create scissorintake object
        ScissorIntake intake = new ScissorIntake(servoScissorLift, servoScissor, tSensor, dSensor);

        double sliderPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

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
            motorLift2.setPower(-gamepad2.left_stick_y);


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


            // intake
            if (gamepad2.a) {
                intake.releaseCone();
            }
            if (gamepad2.x) {
                intake.autoPickUpCone();
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



    public class ScissorIntake {

        private Servo rpServo;  // the servo that controls the rack and pinion
        private Servo scissorServo; // servo that controls the scissor mechanism;
        private TouchSensor scissorTouch; // scissor touch sensor
        private DistanceSensor coneDistanceSensor; // cone touch sensor

        private final double CONE_DISTANCE_THRESHOLD_MM = 5;  // in mm, need to figure out this value
        private final double SCISSOR_HOME_POSITION = 0;
        private final double SCISSOR_CONTRACT_POSITION = 0.8;  // position of servo when closing to pick up the cone
        private final double SCISSOR_WIDE_OPEN_POSITION = -1.0; // position of servo to open wide (not really useful, but anyways)
        private final double RP_TOP_POSITION = -1.0;
        private final double RP_BOTTOM_POSITION = 1.0;
        private final double RP_HOME_POSITION = 0.0;

        /**
         * @param rbtRPServo            - rack & pinion servo from robot (vertical)
         * @param rbtScissorServo       - scissor servo from robot
         * @param rbtScissorTouch       - scissor touch sensor from robot
         * @param rbtConeDistanceSensor - distance sensor for cone detection
         */
        public ScissorIntake(Servo rbtRPServo, Servo rbtScissorServo, TouchSensor rbtScissorTouch, DistanceSensor rbtConeDistanceSensor) {
            this.rpServo = rbtRPServo;
            this.scissorServo = rbtScissorServo;
            this.scissorTouch = rbtScissorTouch;
            this.coneDistanceSensor = rbtConeDistanceSensor;
        }

        /**
         * Check if the cone is placed in the bracket
         *
         * @return true if the cone distance is less than the determined threshold
         */
        public boolean isConeInBracket() {
            return this.coneDistanceSensor.getDistance(DistanceUnit.MM) < this.CONE_DISTANCE_THRESHOLD_MM;
        }

        /**
         * Check if the scissor touch sensor is press
         *
         * @return true if the scissor touch sensor is pressed
         */
        public boolean isScissorInCone() {
            return this.scissorTouch.isPressed();
        }

        /**
         * Contract scissor arms to for cone pick up
         */
        public void contractScissor() {
            this.scissorServo.setPosition(this.SCISSOR_CONTRACT_POSITION);
        }

        /**
         * Move the scissor up to te top position on the rack and pinion
         */
        public void moveScissorToTop() {
            this.rpServo.setPosition(this.RP_TOP_POSITION);
        }

        /**
         * Move the scissor down to the bottom position on the rack and pinion
         */
        public void moveScissorToBottom() {
            this.rpServo.setPosition(this.RP_BOTTOM_POSITION);
            // maybe add a timeout safety stop here;
        }

        /**
         * Reset the position of the RP servo to home position
         */
        public void moveScissorToHome() {
            this.rpServo.setPosition(this.RP_HOME_POSITION);
            // maybe add a timeout safety stop here;
        }

        /**
         * Automated action to pick up the cone
         *
         * @return true if the action completes
         */
        public boolean autoPickUpCone() {
            // move to home position
            if (this.rpServo.getPosition() != this.RP_HOME_POSITION) {
                this.rpServo.setPosition(this.RP_HOME_POSITION);
            }

            // move the scissor down until bottom or cone detected in scissor
            if (isConeInBracket()) {
                while (this.rpServo.getPosition() > this.RP_BOTTOM_POSITION || !this.isScissorInCone()) {
                    this.rpServo.setPosition(this.rpServo.getPosition() + 0.1);
                }

                if (this.isScissorInCone()) {
                    this.contractScissor();
                    this.moveScissorToTop();
                    return true;
                }
            }

            return false;
        }

        /**
         * bring scissor arms to their position, releasing the cone
         */
        public void releaseCone() {
            this.scissorServo.setPosition(0);
        }
    }
}
