package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

// odometry import
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.teamcode.util.Encoder;

public class AustinsOdometryTest extends LinearOpMode{

    // Movement Motors
    DcMotor motorFL, motorBL, motorFR, motorBR;

    // Odometry Motors

    // Odometry Variables
    double TICKS_PER_REV;
    double WHEEL_RADIUS;
    double GEAR_RATIO;

    double PARALLEL_X = 0;
    double PARALLEL_Y = 0;

    double PERPENDICULAR_X = 0;
    double PERPENDICULAR_Y = 0;

    //Encoder parallelEncoder, perpendicularEncoder;
    @Override
    public void runOpMode() {

        // Initialize Wheel Motors
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Initialize Odometry Encoders
        //parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        //perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

        float gp1ly = gamepad1.left_stick_y;
        float gp1lx = gamepad1.left_stick_x;

        float gp1ry = gamepad1.right_stick_y;
        float gp1rx = gamepad1.right_stick_x;

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {

            // forward, back
            double y = -gp1ly; // Remember, this is reversed!

            // strafing
            double x = gp1lx * 1.1; // Counteract imperfect strafing

            // turning
            double rx = gp1rx;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

            // output data
            telemetry.addData("Front Left Motor Busy: ", motorFL.isBusy() + " " + frontLeftPower);
            telemetry.addData("Front Right Motor Busy: ", motorFR.isBusy() + " " + frontRightPower);
            telemetry.addData("Front Left Motor Busy: ", motorBL.isBusy() + " " + backLeftPower);
            telemetry.addData("Front Right Motor Busy: ", motorBR.isBusy() + " " + backRightPower);



        }
    }

    public double encoderTicksToInches(double ticks){
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
