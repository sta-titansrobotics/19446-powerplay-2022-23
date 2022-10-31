package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//blah blah blah

@TeleOp
public class driveControlled extends LinearOpMode {


    @Override
    public void runOpMode() {

        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Arm
        DcMotor Arm = hardwareMap.get(DcMotor.class, "arm");
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Lift
        DcMotor Lift = hardwareMap.get(DcMotor.class, "lift");

        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        //Turret
        DcMotor Turret = hardwareMap.get(DcMotor.class, "turret");

        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

            //Arm
            double armPower = gamepad2.right_stick_y;

            Arm.setPower(armPower);


            //Lift

            if (gamepad2.right_bumper) {
                Lift.setPower(0.85);

            }
            if (gamepad2.left_bumper) {
                Lift.setPower(-0.85);

            }else{
                Lift.setPower(0);

            }

            //Turret
            double turretPower = gamepad2.left_stick_x;

            Turret.setPower(turretPower);

//            if(gamepad2.dpad_left) {
//                Turret.setPower(0.80);
//            }
//            if (gamepad2.dpad_right) {
//                Turret.setPower(-0.80);
//            }else{
//                Turret.setPower(0);
//            }



            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());
            telemetry.addData("Arm Power:", Arm.getPower());
            telemetry.addData("Lift Power:", Lift.getPower());
            telemetry.addData("Turret Power:", Turret.getPower());
            telemetry.addData("Arm Encoder Position: ", Arm.getCurrentPosition());
            telemetry.update();


        }
    }
}