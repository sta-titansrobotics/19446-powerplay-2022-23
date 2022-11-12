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
        Vector2d vectorPark = new Vector2d(0, 0);

        switch (parkNum) {
            case 1:
                vectorPark = new Vector2d(-58, -11);
                break;
            case 2:
                vectorPark = new Vector2d(-36, -12);
                break;
            case 3:
                vectorPark = new Vector2d(-12, -11);
                break;

        }


        Vector2d finalVectorPark = vectorPark;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(26.01, 30, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(16, 17)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, -70+8.5, Math.PI / 2))
                                        .forward(5)
                                        .turn(Math.toRadians(-90))
                                        .lineToConstantHeading(new Vector2d(-35, -4))
                                        .strafeRight(10)
                                        .turn(Math.toRadians(45))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.4)
                                        .turn(Math.toRadians(135))
                                        .splineTo(new Vector2d(-58 + 9.5, -13), Math.PI)
                                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*pick up cone*/})
                                        .waitSeconds(0.4)
                                        .lineToSplineHeading(new Pose2d(-35, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-58 + 9.5, -13, Math.PI))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-35, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-58 + 9.5, -13, Math.PI))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(new Pose2d(-35, -13, Math.toRadians(50)))
                                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                        .waitSeconds(0.5)
                                        .turn(Math.toRadians(50))
                                        .lineToConstantHeading(finalVectorPark)
                                        .build()
                        );




        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}