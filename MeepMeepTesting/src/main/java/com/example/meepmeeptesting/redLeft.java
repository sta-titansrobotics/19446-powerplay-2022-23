package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class redLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        int parkNum = 2;
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
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.01, 30, 2.1322221755981445, Math.toRadians(180), 14.65)
                .setDimensions(16, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -62.3, Math.toRadians(180)))

                                // preload cone
                                .lineToSplineHeading(new Pose2d(-11.5, -62.3, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-11.5, -11.5, Math.toRadians(180)))
                                .turn(Math.toRadians(-45))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    /* drop cone
                                    *
                                    *
                                    * */
                                })
                                .waitSeconds(1)
                                .turn(Math.toRadians(45))

                                // first cycle
                                .lineToSplineHeading(new Pose2d(-48.5, -11, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {/*pick up cone*/})
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-35, -11, Math.toRadians(180)))
                                .turn(Math.toRadians(-135))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {/*drop cone*/})
                                .waitSeconds(1)
                                .turn(Math.toRadians(135))
                                
                                // second cycle
                                .lineToSplineHeading(new Pose2d(-48.5, -11, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {/*pick up cone*/})
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-35, -11, Math.toRadians(180)))
                                .turn(Math.toRadians(-135))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {/*drop cone*/})
                                .waitSeconds(1)
                                .turn(Math.toRadians(135))

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