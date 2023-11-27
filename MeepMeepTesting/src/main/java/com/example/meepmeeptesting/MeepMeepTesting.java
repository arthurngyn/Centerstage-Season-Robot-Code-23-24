package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30.430746841508835, 36.48291908330528, Math.toRadians(212.96386402266293), Math.toRadians(212.96386402266293),13.53)
                //BLUE 1 LEFT
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(10, 65.1, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(32.5,33 , Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(50, 43, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(53, 60, Math.toRadians(180)))
//                                .build()
//                );
        //BLUE 1 Middle
//                .followTrajectorySequence(drive ->
//                drive.trajectorySequenceBuilder(new Pose2d(10, 65.1, Math.toRadians(270)))
//                        .lineToSplineHeading(new Pose2d(23,23 , Math.toRadians(180)))
//                        .lineToSplineHeading(new Pose2d(51, 33, Math.toRadians(180)))
//                        .lineToSplineHeading(new Pose2d(53, 60, Math.toRadians(180)))
//                        .build()
//        );
        //BLUE 1 RIGHT
//                .followTrajectorySequence(drive ->
//                drive.trajectorySequenceBuilder(new Pose2d(10, 65.1, Math.toRadians(180)))
//                        .lineToSplineHeading(new Pose2d(12,32 , Math.toRadians(180)))
//                        .lineToSplineHeading(new Pose2d(51, 23, Math.toRadians(180)))
//                        .lineToSplineHeading(new Pose2d(53, 60, Math.toRadians(180)))
//                        .build()
//        );
//        BLUE 2 LEFT
//                .followTrajectorySequence(drive ->
//                drive.trajectorySequenceBuilder(new Pose2d(-37, 65.1, Math.toRadians(270)))
//                        .lineToSplineHeading(new Pose2d(-35,31 , Math.toRadians(180)))
//                        .lineToSplineHeading(new Pose2d(51, 42, Math.toRadians(180)))
//                        .lineToSplineHeading(new Pose2d(50, 14, Math.toRadians(180)))
//                        .lineToSplineHeading(new Pose2d(58, 14, Math.toRadians(180)))
//                        .build()
//        );
////        BLUE 2 Middle
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-37, 65.1, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(-35,31 , Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(-35,36 , Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(51, 34, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(50, 14, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(58, 14, Math.toRadians(180)))
//                                .build()
//                );
//        BLUE 2 Right
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37, 65.1, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(-35,31 , Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-30,31 , Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-35,31 , Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-35, 56, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(9, 56, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(48,28, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(50, 14, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(58, 14, Math.toRadians(180)))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}