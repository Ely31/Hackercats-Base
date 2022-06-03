package com.example.meepmeeptwo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTwo {

    final static double distanceToWall = 142.0/2;
    final static double distanceToBotEdgeLong = 9;
    final static double distanceToBotEdgeWide = 9;
    final static Pose2d startPos = new Pose2d(11.4,-(distanceToWall-distanceToBotEdgeLong), Math.toRadians(-90));

    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 700 pixels at 30 fps
        MeepMeep meepMeep = new MeepMeep(740,60);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(distanceToBotEdgeWide*2,distanceToBotEdgeLong*2) // Width of 11.6 to match our thin bot
                .setConstraints(55, 30, Math.toRadians(167), Math.toRadians(167), 11)

                // The path we are simulating
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineToSplineHeading(new Pose2d(-7.0, -45,Math.toRadians(-70)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(0,-(distanceToWall- distanceToBotEdgeWide),Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(40,-(distanceToWall- distanceToBotEdgeWide), Math.toRadians(0)))
                                .strafeLeft(26)
                                .forward(18)
                                .build()
                );
        meepMeep
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(0.9f)
                .addEntity(bot)
                .start();
    }
}