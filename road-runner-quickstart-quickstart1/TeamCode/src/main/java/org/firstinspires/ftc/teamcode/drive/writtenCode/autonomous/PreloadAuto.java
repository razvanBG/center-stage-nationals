package org.firstinspires.ftc.teamcode.drive.writtenCode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Intake;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Lift;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Scoring;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Scores only the two preloaded pixels: the purple one on the spike mark, the yellow one on
 * the backdrop, then parks. Each alliance supplies its own paths.
 */
public abstract class PreloadAuto extends BaseAuto {
    enum Step { START, DRIVE_TO_SPIKE, DROP_PURPLE, DRIVE_TO_BACKDROP, PARK, LOWER_LIFT, DONE }

    protected double purpleDropTime = 1;
    protected double liftDelay = 0.4;
    protected double backdropDropTime = 1;

    // Paths for prop cases 1, 2 and 3.
    protected TrajectorySequence[] toSpike = new TrajectorySequence[3];
    protected TrajectorySequence[] toBackdrop = new TrajectorySequence[3];
    protected TrajectorySequence[] toPark = new TrajectorySequence[3];

    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime liftTimer = new ElapsedTime();

    protected abstract String alliance();

    protected abstract Pose2d startPose();

    protected abstract void buildPaths(Pose2d start);

    @Override
    public void runOpMode() throws InterruptedException {
        setUp(alliance(), startPose());
        Step step = Step.START;
        buildPaths(startPose());
        detectPropUntilStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            switch (step) {
                case START:
                    followForCase(toSpike[0], toSpike[1], toSpike[2]);
                    step = Step.DRIVE_TO_SPIKE;
                    break;
                case DRIVE_TO_SPIKE:
                    if (!drive.isBusy()) {
                        // The purple pixel is still in the intake; reversing it drops it on the spike mark.
                        s.intake.state = Intake.State.REVERSE_SLOW;
                        stepTimer.reset();
                        step = Step.DROP_PURPLE;
                    }
                    break;
                case DROP_PURPLE:
                    if (stepTimer.seconds() > purpleDropTime) {
                        s.intake.state = Intake.State.STOP;
                        followForCase(toBackdrop[0], toBackdrop[1], toBackdrop[2]);
                        liftTimer.reset();
                        step = Step.DRIVE_TO_BACKDROP;
                    }
                    break;
                case DRIVE_TO_BACKDROP:
                    if (s.lift.state == Lift.State.REST && liftTimer.seconds() > liftDelay) {
                        s.lift.state = Lift.State.AUTO_LOW;
                    }
                    if (!drive.isBusy()) {
                        s.scoring.state = Scoring.State.DROP_BOTH;
                        stepTimer.reset();
                        step = Step.PARK;
                    }
                    break;
                case PARK:
                    if (stepTimer.seconds() > backdropDropTime) {
                        followForCase(toPark[0], toPark[1], toPark[2]);
                        liftTimer.reset();
                        step = Step.LOWER_LIFT;
                    }
                    break;
                case LOWER_LIFT:
                    if (liftTimer.seconds() > 0.3) {
                        s.lift.state = Lift.State.REST;
                        step = Step.DONE;
                    }
                    break;
                default:
                    break;
            }

            updateRobot();
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Step", step);
            telemetry.update();
        }
    }
}
