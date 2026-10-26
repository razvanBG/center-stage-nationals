package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/**
 * Vertical lift on a single motor. Encoder counts become more negative as the lift goes up.
 * Raising the lift also moves the extender out of the way and swings the outtake to the backdrop.
 */
@Config
public class Lift extends StateController<Lift.State> {
    public enum State {
        REST,
        GOING_DOWN,
        LOW,
        MID,
        HIGH,
        AUTO_LOW,
        AUTO_CYCLE2_CASE1,
        AUTO_CYCLE1_CASE1,
        AUTO_CYCLE1_CASE23,
        AUTO_CYCLE2_CASE23,
        MOSAIC
    }

    public static int restPosition = 10;
    public static int downPosition = -100;
    public static int lowPosition = -700;
    public static int midPosition = -700;
    public static int highPosition = -1630;
    public static int autoLowPosition = -360;
    public static int mosaicPosition = -317;
    public static int autoCycle1Case1Position = -570;
    public static int autoCycle2Case1Position = -900;
    public static int autoCycle1Case23Position = -540;
    public static int autoCycle2Case23Position = -800;

    /** Manual control never moves the lift below this target. */
    public int manualLimit = 100;
    /** Encoder target sent every loop. Manual control edits it directly. */
    public int targetPosition = restPosition;

    private final OuttakeFourBar outtakeFourBar;
    private final Extender extender;
    private final DcMotor motor;

    public Lift(OuttakeFourBar outtakeFourBar, Extender extender, RobotHardware robot) {
        super(State.REST);
        this.outtakeFourBar = outtakeFourBar;
        this.extender = extender;
        this.motor = robot.liftMotor;
    }

    @Override
    protected void everyLoop() {
        RobotHardware.allowFullSpeed(motor);
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    @Override
    protected void apply(State s) {
        switch (s) {
            case REST:
                outtakeFourBar.state = OuttakeFourBar.State.RECEIVE;
                targetPosition = restPosition;
                break;
            case GOING_DOWN:
                extender.state = Extender.State.RETRACTED;
                outtakeFourBar.state = OuttakeFourBar.State.DOWN;
                targetPosition = downPosition;
                break;
            default:
                raiseTo(scoringTarget(s));
                break;
        }
    }

    private int scoringTarget(State s) {
        switch (s) {
            case LOW:                return lowPosition;
            case MID:                return midPosition;
            case HIGH:               return highPosition;
            case AUTO_LOW:           return autoLowPosition;
            case AUTO_CYCLE2_CASE1:  return autoCycle2Case1Position;
            case AUTO_CYCLE1_CASE1:  return autoCycle1Case1Position;
            case AUTO_CYCLE1_CASE23: return autoCycle1Case23Position;
            case AUTO_CYCLE2_CASE23: return autoCycle2Case23Position;
            default:                 return mosaicPosition;
        }
    }

    private void raiseTo(int target) {
        if (extender.state == Extender.State.RETRACTED) {
            extender.state = Extender.State.HOLD;
        }
        targetPosition = target;
        // Starting from the bottom, the outtake waits a moment before swinging out.
        boolean startingFromBottom = motor.getCurrentPosition() >= 0;
        outtakeFourBar.state = startingFromBottom
                ? OuttakeFourBar.State.SCORE_AFTER_DELAY
                : OuttakeFourBar.State.SCORE;
    }

    public boolean isScoringInAuto() {
        return state == State.AUTO_CYCLE1_CASE1 || state == State.AUTO_CYCLE2_CASE1
                || state == State.AUTO_CYCLE1_CASE23 || state == State.AUTO_CYCLE2_CASE23;
    }
}
