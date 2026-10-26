package org.firstinspires.ftc.teamcode.drive.writtenCode.autonomous;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.autonomous.RedPreloadAuto.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeFourBar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Red alliance, backdrop side: preloads plus two stack cycles.
 * Spike, backdrop and park positions are shared with {@link RedPreloadAuto}.
 */
@Config
@Autonomous(name = "Red Stack Cycles", group = "Auto")
public class RedStackAuto extends StackCycleAuto {
    public static final double STACK_APPROACH_X = 20f, STACK_APPROACH_Y = -11.5f, STACK_APPROACH_HEADING = -1.7;
    public static final double STACK_X = -30.25, STACK_Y = -11.5;
    public static final double CROSS_X = 20, CROSS_Y = -11, CROSS_HEADING = 0;
    public static final double SCORE_X = 49, SCORE_Y = -34, SCORE_HEADING = 0;

    @Override
    protected String alliance() {
        return "red";
    }

    @Override
    protected Pose2d startPose() {
        return new Pose2d(10, -62, Math.toRadians(90));
    }

    @Override
    protected int cycles() {
        return 2;
    }

    @Override
    protected IntakeFourBar.State noPixelFallbackHeight() {
        return IntakeFourBar.State.AUTO_STACK_LOW;
    }

    @Override
    protected boolean scoreAfterTimeLimit() {
        return false;
    }

    @Override
    protected boolean keepLiftUpWhileLeavingBackdrop() {
        return false;
    }

    @Override
    protected void buildPaths(Pose2d start) {
        toSpike[0] = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(pose(SPIKE_1_X, SPIKE_1_Y, SPIKE_1_HEADING))
                .back(SPIKE_BACKOFF_1)
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
        // Case 3 approaches the spike with the backdrop heading.
        toSpike[2] = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(pose(SPIKE_3_X, SPIKE_3_Y, BACKDROP_3_HEADING))
                .build();
        toBackdrop[2] = drive.trajectorySequenceBuilder(toSpike[2].end())
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
                .lineToLinearHeading(pose(PARK_3_X, PARK_3_Y, PARK_3_HEADING))
                .build();
    }
}
