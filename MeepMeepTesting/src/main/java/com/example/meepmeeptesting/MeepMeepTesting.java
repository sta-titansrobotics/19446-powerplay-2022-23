package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        int parkNum = 3;

        RoadRunnerBotEntity myBot;
            if (parkNum == 1) {
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(26.01, 30
                                , Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.PI / 2))
                                        .forward(10)
                                        .turn(Math.toRadians(90))
                                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*move turret, drop cone, pick up cone, turn turret, drop cone*/})
                                        .waitSeconds(2)
                                        .turn(Math.toRadians(-90))
                                        .waitSeconds(2)
                                        .splineTo(new Vector2d(-58 + 9.5, -13), Math.PI)
                                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*pick up cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-58 + 9.5, -13, Math.PI))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-58 + 9.5, -13, Math.PI))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .build()
                        );
            } else if (parkNum == 2) {
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(26.01, 30
                                , Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.PI / 2))
                                        .forward(10)
                                        .turn(Math.toRadians(90))
                                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*move turret, drop cone, pick up cone, turn turret, drop cone*/})
                                        .waitSeconds(1)
                                        .turn(Math.toRadians(-90))
                                        .waitSeconds(2)
                                        .splineTo(new Vector2d(-58 + 9.5, -13), Math.PI)
                                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*pick up cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-58 + 9.5, -13, Math.PI))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-58 + 9.5, -13, Math.PI))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .splineToLinearHeading(new Pose2d(-36, -35.3, Math.toRadians(100)), 0.5)
                                        .build()
                        );
            } else {
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(26.01, 30
                                , Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.PI / 2))
                                        .forward(10)
                                        .turn(Math.toRadians(90))
                                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*move turret, drop cone, pick up cone, turn turret, drop cone*/})
                                        .waitSeconds(1)
                                        .turn(Math.toRadians(-90))
                                        .waitSeconds(1)
                                        .splineTo(new Vector2d(-58 + 9.5, -13), Math.PI)
                                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*pick up cone*/})
                                        .waitSeconds(0.4)
                                        .lineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.4)
                                        .lineToSplineHeading(new Pose2d(-58 + 9.5, -13, Math.PI))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                        .waitSeconds(0.4)
                                        .lineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.4)
                                        .lineToSplineHeading(new Pose2d(-58 + 9.5, -13, Math.PI))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                        .waitSeconds(0.4)
                                        .lineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .splineToLinearHeading(new Pose2d(-36, -35.3, Math.toRadians(100)), 0.5)
                                        .back(20)
                                        .build()
                        );
            }




        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}