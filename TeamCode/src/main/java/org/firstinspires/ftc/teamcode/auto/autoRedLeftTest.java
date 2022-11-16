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

        int parkNum = 1;
        Pose2d vectorPark = new Pose2d(0, 0);

        switch (parkNum) {
            case 1:
                vectorPark = new Pose2d(-58, -12, Math.toRadians(180));
                break;
            case 2:
                vectorPark = new Pose2d(-36, -12, Math.toRadians(180));
                break;
            case 3:
                vectorPark = new Pose2d(-12, -12, Math.toRadians(180));
                break;

        }
        Pose2d finalVectorPark = vectorPark;


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.5, -61.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                // preload cone
                .lineToSplineHeading(new Pose2d(-11.5, -61.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-11.6, -11.5, Math.toRadians(90)))
                .turn(Math.toRadians(42))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    /*
                     * lift up
                     * move turret
                     * drop cone
                     * lift down
                     */
                })
                .waitSeconds(1)
                .turn(Math.toRadians(48))

                // first cycle
                .lineToSplineHeading(new Pose2d(-48.5, -12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    /*
                     * move turret
                     * pick up cone
                     *
                     */
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    /*
                     * lift up (during traj)
                     * turret 90 degrees
                     * drop cone
                     * lift down (during traj)
                     */
                })
                .waitSeconds(1)

                // second cycle
                .lineToSplineHeading(new Pose2d(-48.5, -12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    /*
                     * move turret
                     * pick up cone
                     *
                     */
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    /*
                     * lift up (during traj)
                     * turret 90 degrees
                     * drop cone
                     * lift down (during traj)
                     */
                })
                .waitSeconds(1)

                // third cycle
                .lineToSplineHeading(new Pose2d(-48.5, -12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    /*
                     * move turret
                     * pick up cone
                     *
                     */
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    /*
                     * lift up (during traj)
                     * turret 90 degrees
                     * drop cone
                     * lift down (during traj)
                     */
                })
                .waitSeconds(1)

                // park
                .lineToSplineHeading((finalVectorPark))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(leftTraj);




    }

}