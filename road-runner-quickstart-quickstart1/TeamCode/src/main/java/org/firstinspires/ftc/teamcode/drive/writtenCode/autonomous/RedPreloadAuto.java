package org.firstinspires.ftc.teamcode.drive.writtenCode.autonomous;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/** Red alliance, backdrop side: preloads only. Field positions are tunable from FTC Dashboard. */
@Config
@Autonomous(name = "Red Preload", group = "Auto")
public class RedPreloadAuto extends PreloadAuto {
    public static double SPIKE_BACKOFF_1 = 6.75;

    public static double SPIKE_1_X = 15, SPIKE_1_Y = -36.5, SPIKE_1_HEADING = -5;
    public static double SPIKE_2_X = 25, SPIKE_2_Y = -25, SPIKE_2_HEADING = -5;
    public static double SPIKE_3_X = 32, SPIKE_3_Y = -35.5, SPIKE_3_HEADING = -5;

    public static double BACKDROP_1_X = 49, BACKDROP_1_Y = -31, BACKDROP_1_HEADING = 0;
    public static double BACKDROP_2_X = 49, BACKDROP_2_Y = -37.5, BACKDROP_2_HEADING = 0;
    public static double BACKDROP_3_X = 49, BACKDROP_3_Y = -46, BACKDROP_3_HEADING = 0;

    public static double PARK_1_X = 48, PARK_1_Y = -17, PARK_1_HEADING = 0;
    public static double PARK_2_X = 48, PARK_2_Y = -17, PARK_2_HEADING = 0;
    public static double PARK_3_X = 48, PARK_3_Y = -17, PARK_3_HEADING = 0;

    @Override
    protected String alliance() {
        return "red";
    }

    @Override
    protected Pose2d startPose() {
        return new Pose2d(10, -62, Math.toRadians(90));
    }

    @Override
    protected void buildPaths(Pose2d start) {
        TrajectoryVelocityConstraint slow = getVelocityConstraint(40, 5, 12.05);

        toSpike[0] = drive.trajectorySequenceBuilder(start)
                .setVelConstraint(slow)
                .lineToLinearHeading(pose(SPIKE_1_X, SPIKE_1_Y, SPIKE_1_HEADING))
                .back(SPIKE_BACKOFF_1)
                .build();
        toBackdrop[0] = drive.trajectorySequenceBuilder(toSpike[0].end())
                .setVelConstraint(slow)
                .lineToLinearHeading(pose(BACKDROP_1_X, BACKDROP_1_Y, BACKDROP_1_HEADING))
                .build();
        toPark[0] = parkFrom(toBackdrop[0], PARK_1_X, PARK_1_Y, PARK_1_HEADING);

        toSpike[1] = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(pose(SPIKE_2_X, SPIKE_2_Y, SPIKE_2_HEADING))
                .build();
        toBackdrop[1] = drive.trajectorySequenceBuilder(toSpike[1].end())
                .lineToLinearHeading(pose(BACKDROP_2_X, BACKDROP_2_Y, BACKDROP_2_HEADING))
                .build();
        toPark[1] = parkFrom(toBackdrop[1], PARK_2_X, PARK_2_Y, PARK_2_HEADING);

        toSpike[2] = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(pose(SPIKE_3_X, SPIKE_3_Y, SPIKE_3_HEADING))
                .build();
        toBackdrop[2] = drive.trajectorySequenceBuilder(toSpike[2].end())
                .lineToLinearHeading(pose(BACKDROP_3_X, BACKDROP_3_Y, BACKDROP_3_HEADING))
                .build();
        toPark[2] = parkFrom(toBackdrop[2], PARK_3_X, PARK_3_Y, PARK_3_HEADING);
    }

    private TrajectorySequence parkFrom(TrajectorySequence previous, double x, double y, double heading) {
        return drive.trajectorySequenceBuilder(previous.end())
                .back(5)
                .lineToLinearHeading(pose(x, y, heading))
                .strafeLeft(5)
                .forward(10)
                .build();
    }

    static Pose2d pose(double x, double y, double headingDegrees) {
        return new Pose2d(x, y, Math.toRadians(headingDegrees));
    }
}
