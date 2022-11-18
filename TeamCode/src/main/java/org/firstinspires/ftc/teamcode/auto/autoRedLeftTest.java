/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class autoRedLeftTest extends LinearOpMode {

    DcMotor leftLift, rightLift;
    @Override
    public void runOpMode()
    {

        int parkNum = 1;
        Pose2d vectorPark = new Pose2d(0, 0);

        switch (parkNum) {
            case 1:
                vectorPark = new Pose2d(-58, -12, Math.toRadians(180));
                break;
            case 2:
                vectorPark = new Pose2d(-35, -12, Math.toRadians(180));
                break;
            case 3:
                vectorPark = new Pose2d(-12, -12, Math.toRadians(180));
                break;

        }
        Pose2d finalVectorPark = vectorPark;

        // Other
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        // lift motors
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo servoScissor = hardwareMap.get(Servo.class, "servoScissor");
        Servo verticalServo = hardwareMap.get(Servo.class, "servoScissorLift");

        // Reverse right lift motor
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.5, -62.3, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                // preload cone

                .UNSTABLE_addTemporalMarkerOffset(0, () -> verticalServo.setPosition(1))
                .UNSTABLE_addTemporalMarkerOffset(3, () -> servoScissor.setPosition(0.67))
                .UNSTABLE_addTemporalMarkerOffset(6, () -> verticalServo.setPosition(0))
                .waitSeconds(6)

                .lineToSplineHeading(new Pose2d(-11.5, -62.3, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-11.5, -11.5, Math.toRadians(180)))
                .turn(Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveLift(0.8, 3300))
                .waitSeconds(1)
                .forward(7.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> verticalServo.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> servoScissor.setPosition(0.5))
                .waitSeconds(1)
                .back(7.25)
                .turn(Math.toRadians(45))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> moveLift(0.8, 0))
                .waitSeconds(1)
                // park
                .lineToSplineHeading((finalVectorPark))
                .build();


        waitForStart();

        drive.followTrajectorySequence(leftTraj);





    }

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