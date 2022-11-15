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

    @Override
    public void runOpMode()
    {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.5, -62, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                // preload cone
                .lineToConstantHeading(new Vector2d(-35, -4))
                .strafeLeft(-11)
                .turn(Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // first cycle
                .splineTo(new Vector2d(-58 + 9.5, -12), Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*pick up cone*/})
                .waitSeconds(0.4)
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(50)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // second cycle
                .lineToSplineHeading(new Pose2d(-58 + 9.5, -12, Math.PI))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(50)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // third cycle
                .lineToSplineHeading(new Pose2d(-58 + 9.5, -12, Math.PI))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(50)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // park
                .lineToSplineHeading(new Pose2d(-58, -11, 0))
                .build();

        TrajectorySequence middleTraj = drive.trajectorySequenceBuilder(startPose)
                // preload cone
                .lineToConstantHeading(new Vector2d(-35, -4))
                .strafeLeft(-11)
                .turn(Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // first cycle
                .splineTo(new Vector2d(-58 + 9.5, -12), Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*pick up cone*/})
                .waitSeconds(0.4)
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(50)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // second cycle
                .lineToSplineHeading(new Pose2d(-58 + 9.5, -12, Math.PI))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(50)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // third cycle
                .lineToSplineHeading(new Pose2d(-58 + 9.5, -12, Math.PI))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(50)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // park
                .lineToSplineHeading(new Pose2d(-36, -12, 0))
                .build();

        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                // preload cone
                .lineToConstantHeading(new Vector2d(-35, -4))
                .strafeLeft(-11)
                .turn(Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // first cycle
                .splineTo(new Vector2d(-58 + 9.5, -12), Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*pick up cone*/})
                .waitSeconds(0.4)
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(50)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // second cycle
                .lineToSplineHeading(new Pose2d(-58 + 9.5, -12, Math.PI))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(50)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // third cycle
                .lineToSplineHeading(new Pose2d(-58 + 9.5, -12, Math.PI))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    /*pick up cone*/})
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(50)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                .waitSeconds(0.5)

                // park
                .lineToSplineHeading(new Pose2d(-12, -11, Math.toRadians(90)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(leftTraj);

            //drive.followTrajectorySequence(middleTraj);
            //drive.followTrajectorySequence(rightTraj);




    }
}