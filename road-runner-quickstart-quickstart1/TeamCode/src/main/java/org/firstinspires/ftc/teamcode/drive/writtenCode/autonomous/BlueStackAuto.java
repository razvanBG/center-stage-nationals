package org.firstinspires.ftc.teamcode.drive.writtenCode.autonomous;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.autonomous.RedPreloadAuto.pose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeFourBar;

/** Blue alliance, backdrop side: preloads plus one stack cycle. */
@Config
@Autonomous(name = "Blue Stack Cycles", group = "Auto")
public class BlueStackAuto extends StackCycleAuto {
    public static double SPIKE_BACKOFF_3 = 6.75;

    public static double SPIKE_1_X = 35, SPIKE_1_Y = 35, SPIKE_1_HEADING = 0;
    public static double SPIKE_2_X = 25, SPIKE_2_Y = 25, SPIKE_2_HEADING = 0;
    public static double SPIKE_3_X = 16, SPIKE_3_Y = 33, SPIKE_3_HEADING = 0;

    public static double BACKDROP_1_X = 51.5, BACKDROP_1_Y = 46, BACKDROP_1_HEADING = 0;
    public static double BACKDROP_2_X = 51.5, BACKDROP_2_Y = 40, BACKDROP_2_HEADING = 0;
    public static double BACKDROP_3_X = 51.5, BACKDROP_3_Y = 32, BACKDROP_3_HEADING = 0;

    public static double PARK_X = 44, PARK_Y = 20, PARK_HEADING = 0;

    public static final double STACK_APPROACH_X = 20f, STACK_APPROACH_Y = 11.5f, STACK_APPROACH_HEADING = 1;
    public static final double STACK_X = -27f, STACK_Y = 11.5;
    public static final double CROSS_X = 20, CROSS_Y = 11, CROSS_HEADING = 0;
    public static final double SCORE_X = 53.2, SCORE_Y = 33.5f, SCORE_HEADING = 0;

    @Override
    protected String alliance() {
        return "blue";
    }

    @Override
    protected Pose2d startPose() {
        return new Pose2d(10, 62, Math.toRadians(270));
    }

    @Override
    protected int cycles() {
        return 1;
    }

    @Override
    protected IntakeFourBar.State noPixelFallbackHeight() {
        return IntakeFourBar.State.AUTO_STACK_4;
    }

    @Override
    protected boolean scoreAfterTimeLimit() {
        return true;
    }

    @Override
    protected boolean keepLiftUpWhileLeavingBackdrop() {
        return true;
    }

    @Override
    protected void buildPaths(Pose2d start) {
        TrajectoryVelocityConstraint slow = getVelocityConstraint(40, 5, 12.05);

        toSpike[0] = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(pose(SPIKE_1_X, SPIKE_1_Y, SPIKE_1_HEADING))
                .build();
        toBackdrop[0] = drive.trajectorySequenceBuilder(toSpike[0].end())
                .lineToLinearHeading(pose(BACKDROP_1_X, BACKDROP_1_Y, BACKDROP_1_HEADING))
                .build();
        toSpike[1] = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(pose(SPIKE_2_X, SPIKE_2_Y, SPIKE_2_HEADING))
                .build();
        toBackdrop[1] = drive.trajectorySequenceBuilder(toSpike[1].end())
                .lineToLinearHeading(pose(BACKDROP_2_X, BACKDROP_2_Y, BACKDROP_2_HEADING))
                .build();
        toSpike[2] = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(slow)
                .lineToLinearHeading(pose(SPIKE_3_X, SPIKE_3_Y, SPIKE_3_HEADING))
                .back(SPIKE_BACKOFF_3)
                .build();
        toBackdrop[2] = drive.trajectorySequenceBuilder(toSpike[2].end())
                .setVelConstraint(slow)
                .lineToLinearHeading(pose(BACKDROP_3_X, BACKDROP_3_Y, BACKDROP_3_HEADING))
                .build();

        Pose2d stackApproach = pose(STACK_APPROACH_X, STACK_APPROACH_Y, STACK_APPROACH_HEADING);
        for (int i = 0; i < 3; i++) {
            toStack[i] = drive.trajectorySequenceBuilder(toBackdrop[i].end())
                    .lineToLinearHeading(stackApproach)
                    .build();
        }
        intoStack = drive.trajectorySequenceBuilder(stackApproach)
                .lineTo(new Vector2d(STACK_X, STACK_Y))
                .build();
        stackToBackdrop = drive.trajectorySequenceBuilder(intoStack.end())
                .lineToLinearHeading(pose(CROSS_X, CROSS_Y, CROSS_HEADING))
                .lineToLinearHeading(pose(SCORE_X, SCORE_Y, SCORE_HEADING))
                .build();
        backdropToStack = drive.trajectorySequenceBuilder(stackToBackdrop.end())
                .lineToLinearHeading(stackApproach)
                .build();
        park = drive.trajectorySequenceBuilder(stackToBackdrop.end())
                .back(7)
                .lineToLinearHeading(pose(PARK_X, PARK_Y, PARK_HEADING))
                .build();
    }
}
