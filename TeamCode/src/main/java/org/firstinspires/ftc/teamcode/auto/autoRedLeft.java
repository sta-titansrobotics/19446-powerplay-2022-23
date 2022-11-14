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

@TeleOp
public class autoRedLeft extends LinearOpMode {
    //INTRODUCE VARIABLES HERE

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.5, -62, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                // preload cone
                .lineToConstantHeading(new Vector2d(-35, -4))
                .lineToSplineHeading(new Pose2d(-35.4, -11.1, Math.toRadians(45)))
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
                .lineToSplineHeading(new Pose2d(-35.4, -11.1, Math.toRadians(45)))
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
                .lineToSplineHeading(new Pose2d(-35.4, -11.1, Math.toRadians(45)))
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
                .lineToSplineHeading(new Pose2d(-12, -11, 0))
                .build();




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }





        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // autonomous code here
        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            // left trajectory
            drive.followTrajectorySequence(leftTraj);


        } else if (tagOfInterest.id == MIDDLE) {
            // middle trajectory
            drive.followTrajectorySequence(middleTraj);


        } else {
            // right trajectory
            drive.followTrajectorySequence(rightTraj);

        }


    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}