package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class blueLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        int parkNum = 1;
        Pose2d vectorPark = new Pose2d(0, 0);

        switch (parkNum) {
            case 1:
                vectorPark = new Pose2d(58, 11, Math.toRadians(-180));
                break;
            case 2:
                vectorPark = new Pose2d(36, 12, Math.toRadians(-180));
                break;
            case 3:
                vectorPark = new Pose2d(12, 11, Math.toRadians(-180));
                break;

        }


        Pose2d finalVectorPark = vectorPark;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.01, 30, 2.1322221755981445, Math.toRadians(180), 14.65)
                .setDimensions(16, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.5, 62, Math.toRadians(-180)))

                                // preload cone
                                .lineToConstantHeading(new Vector2d(35, 4))
                                .lineToSplineHeading(new Pose2d(35.4, 11.1, Math.toRadians(-135)))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                .waitSeconds(0.5)

                                // first cycle
                                .splineTo(new Vector2d(48.5, 12), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(2, () -> {/*pick up cone*/})
                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(35, 12, Math.toRadians(-130)))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                .waitSeconds(0.5)

                                // second cycle
                                .lineToSplineHeading(new Pose2d(48.5, 12, Math.toRadians(0)))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(35, 12, Math.toRadians(-130)))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                .waitSeconds(0.5)

                                // third cycle
                                .lineToSplineHeading(new Pose2d(48.5, 12, Math.toRadians(0)))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*pick up cone*/})
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(35, 12, Math.toRadians(-130)))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {/*drop cone*/})
                                .waitSeconds(0.5)

                                // park
                                .lineToSplineHeading((finalVectorPark))
                                .build()
                );




        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}