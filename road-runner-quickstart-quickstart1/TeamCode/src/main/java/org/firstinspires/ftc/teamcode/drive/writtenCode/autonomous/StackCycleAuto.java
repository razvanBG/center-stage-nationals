package org.firstinspires.ftc.teamcode.drive.writtenCode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Extender;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Intake;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeFourBar;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Lift;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Scoring;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ServoLatch;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Transfer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Scores the preloads, then cycles between the pixel stack and the backdrop.
 *
 * At the stack the extender pushes the intake out and the four-bar lowers one pixel at a time.
 * The break beams tell when both pixels are in; with only one pixel after a timeout the robot
 * leaves anyway. After 28 s the slides and lift are brought back in so the robot ends parked.
 */
public abstract class StackCycleAuto extends BaseAuto {
    enum Step {
        START,
        DRIVE_TO_SPIKE,
        DROP_PURPLE,
        SCORE_PRELOAD,
        NEXT_CYCLE,
        LOWER_LIFT,
        COLLECT,
        LEAVE_STACK,
        SCORE_STACK_PIXELS,
        DONE
    }

    protected double purpleDropTime = 0.2;
    protected double liftDelay = 0.4;
    protected double backdropDropTime = 0.3;
    protected double extendDelay = 0.4;
    protected int onePixelTimeout = 3;
    protected int noPixelTimeout = 5;

    // Paths for prop cases 1, 2 and 3.
    protected TrajectorySequence[] toSpike = new TrajectorySequence[3];
    protected TrajectorySequence[] toBackdrop = new TrajectorySequence[3];
    protected TrajectorySequence[] toStack = new TrajectorySequence[3];
    // Paths used on every cycle.
    protected TrajectorySequence intoStack;
    protected TrajectorySequence stackToBackdrop;
    protected TrajectorySequence backdropToStack;
    protected TrajectorySequence park;

    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime liftTimer = new ElapsedTime();
    private final ElapsedTime extendTimer = new ElapsedTime();
    private final ElapsedTime collectTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();

    private Step step = Step.START;
    private int cycle = 0;

    protected abstract String alliance();

    protected abstract Pose2d startPose();

    protected abstract void buildPaths(Pose2d start);

    /** Number of stack cycles before parking. */
    protected abstract int cycles();

    /** Four-bar height to try when nothing has been collected for a while. */
    protected abstract IntakeFourBar.State noPixelFallbackHeight();

    /** Whether the lift may still rise for a stack cycle after 28 s. */
    protected abstract boolean scoreAfterTimeLimit();

    /** Whether the lift is allowed to stay up while driving away from the backdrop after 28 s. */
    protected abstract boolean keepLiftUpWhileLeavingBackdrop();

    @Override
    public void runOpMode() throws InterruptedException {
        setUp(alliance(), startPose());
        buildPaths(startPose());
        detectPropUntilStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            runStep();
            endOfMatchSafety();
            updateRobot();
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Step", step);
            telemetry.addData("Match time", matchTimer.seconds());
            telemetry.addData("Extender", s.extender.state);
            telemetry.update();
        }
    }

    private void runStep() {
        switch (step) {
            case START:
                matchTimer.reset();
                followForCase(toSpike[0], toSpike[1], toSpike[2]);
                step = Step.DRIVE_TO_SPIKE;
                break;
            case DRIVE_TO_SPIKE:
                if (!drive.isBusy()) {
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
                    step = Step.SCORE_PRELOAD;
                }
                break;
            case SCORE_PRELOAD:
                if (s.lift.state == Lift.State.REST && liftTimer.seconds() > liftDelay) {
                    s.lift.state = Lift.State.AUTO_LOW;
                }
                if (!drive.isBusy()) {
                    dropPixels();
                }
                break;
            case NEXT_CYCLE:
                if (stepTimer.seconds() > backdropDropTime) {
                    cycle++;
                    if (cycle == 1) {
                        followForCase(toStack[0], toStack[1], toStack[2]);
                    } else if (cycle > cycles()) {
                        drive.followTrajectorySequenceAsync(park);
                    } else {
                        drive.followTrajectorySequenceAsync(backdropToStack);
                    }
                    liftTimer.reset();
                    step = Step.LOWER_LIFT;
                }
                break;
            case LOWER_LIFT:
                if (liftTimer.seconds() > 0.3
                        && s.lift.state != Lift.State.GOING_DOWN && s.lift.state != Lift.State.REST) {
                    s.lift.state = Lift.State.GOING_DOWN;
                }
                if (!drive.isBusy()) {
                    if (cycle > cycles()) {
                        step = Step.DONE;
                    } else {
                        step = Step.COLLECT;
                        extendTimer.reset();
                        drive.followTrajectorySequenceAsync(intoStack);
                    }
                }
                break;
            case COLLECT:
                collect();
                break;
            case LEAVE_STACK:
                pause(300);
                s.secondPixelLatch.state = ServoLatch.State.OPEN;
                s.transfer.extraWait = Transfer.extenderReturnTime;
                s.extender.state = Extender.State.RETRACTED;
                s.intake.state = Intake.State.REVERSE;
                drive.followTrajectorySequenceAsync(stackToBackdrop);
                step = Step.SCORE_STACK_PIXELS;
                scoreStackPixels();
                break;
            case SCORE_STACK_PIXELS:
                scoreStackPixels();
                break;
            default:
                break;
        }
    }

    private void collect() {
        if (extendTimer.seconds() > extendDelay && s.extender.state != Extender.State.EXTENDED) {
            s.extender.state = Extender.State.EXTENDED;
            s.intakeFourBar.state = cycle == 2 ? IntakeFourBar.State.AUTO_STACK_LOW : IntakeFourBar.State.AUTO_STACK_5;
            s.intake.state = Intake.State.STACK;
            collectTimer.reset();
        }

        boolean frontBlocked = !s.robot.frontBeam.getState();
        boolean backBlocked = !s.robot.backBeam.getState();
        if (frontBlocked && backBlocked) {
            s.transfer.state = Transfer.State.LOCK_GATES;
            step = Step.LEAVE_STACK;
        } else if (backBlocked) {
            // One pixel in: on the first cycle, lower the four-bar to reach the next pixel.
            if (cycle == 1) {
                s.intakeFourBar.state = IntakeFourBar.State.AUTO_STACK_4;
            }
            if (collectTimer.seconds() > onePixelTimeout) {
                s.startTransfer(true, false);
                s.extender.state = Extender.State.RETRACTED;
                s.intake.state = Intake.State.REVERSE;
                drive.followTrajectorySequenceAsync(stackToBackdrop);
                step = Step.SCORE_STACK_PIXELS;
            }
        } else if (!frontBlocked) {
            if (collectTimer.seconds() > noPixelTimeout) {
                s.intakeFourBar.state = noPixelFallbackHeight();
            }
        }
    }

    private void scoreStackPixels() {
        if (s.lift.state == Lift.State.REST && s.transfer.state == Transfer.State.IDLE
                && (scoreAfterTimeLimit() || matchTimer.seconds() < 28)) {
            if (propCase == 1) {
                s.lift.state = cycle == 2 ? Lift.State.AUTO_CYCLE2_CASE1 : Lift.State.AUTO_CYCLE1_CASE1;
            } else {
                s.lift.state = cycle == 2 ? Lift.State.AUTO_CYCLE2_CASE23 : Lift.State.AUTO_CYCLE1_CASE23;
            }
        }
        if (!drive.isBusy() && s.lift.isScoringInAuto()) {
            dropPixels();
        }
    }

    private void dropPixels() {
        s.scoring.state = Scoring.State.DROP_BOTH;
        stepTimer.reset();
        step = Step.NEXT_CYCLE;
    }

    /** Near the end of the match, pull everything in so nothing is left outside the robot. */
    private void endOfMatchSafety() {
        boolean timeUp = matchTimer.seconds() > 28;
        if (timeUp) {
            s.extender.state = Extender.State.RETRACTED;
        }
        boolean scoring = step == Step.SCORE_PRELOAD || step == Step.SCORE_STACK_PIXELS
                || (keepLiftUpWhileLeavingBackdrop() && step == Step.NEXT_CYCLE);
        if (timeUp && !scoring) {
            s.lift.state = Lift.State.GOING_DOWN;
        }
        if (s.robot.liftMotor.getCurrentPosition() >= -120 && s.lift.state == Lift.State.GOING_DOWN) {
            s.lift.state = Lift.State.REST;
        }
    }
}
