package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/**
 * Moves the two pixels from the intake box into the outtake box, as one timed sequence:
 * lock the gates, flip the intake box, push the pixels across, then return the box.
 * The driver (or a break-beam sensor) only starts it; the rest runs by itself.
 */
public class Transfer extends StateController<Transfer.State> {
    public enum State { IDLE, LOCK_GATES, FLIP_BOX, MOVE_PIXELS, RETURN_BOX }

    /** Where the intake box servo is set when the OpMode starts. */
    public static double startPosition = 0.96;
    /** Extra time the box needs when the slides are still retracting from the extended position. */
    public static double extenderReturnTime = 0.3;

    public double flipTime = 0.8;
    public double pushTime = 0.3;
    public double reverseDelay = 0.1;
    public double extraWait = 0;

    public double boxTransferPosition = 0.19;
    public double boxCollectPosition = 0.96;

    private final ElapsedTime flipTimer = new ElapsedTime();
    private final ElapsedTime pushTimer = new ElapsedTime();
    private final ElapsedTime reverseTimer = new ElapsedTime();

    private final Intake intake;
    private final TransferGates gates;
    private final ServoLatch outtakeLock;
    private final Servo box;

    public Transfer(Intake intake, TransferGates gates, ServoLatch outtakeLock, RobotHardware robot) {
        super(State.IDLE);
        this.intake = intake;
        this.gates = gates;
        this.outtakeLock = outtakeLock;
        this.box = robot.intakeBoxServo;
    }

    /** Starts a transfer. Pass true when the slides are still coming back in. */
    public void start(boolean extenderReturning) {
        extraWait = extenderReturning ? extenderReturnTime : 0;
        state = State.LOCK_GATES;
    }

    @Override
    protected boolean isTimed(State s) {
        return s == State.MOVE_PIXELS || s == State.RETURN_BOX;
    }

    @Override
    protected void apply(State s) {
        switch (s) {
            case LOCK_GATES:
                outtakeLock.state = ServoLatch.State.OPEN;
                gates.state = TransferGates.State.LOCKED;
                intake.state = Intake.State.STOP;
                state = State.FLIP_BOX;
                break;
            case FLIP_BOX:
                box.setPosition(boxTransferPosition);
                flipTimer.reset();
                reverseTimer.reset();
                state = State.MOVE_PIXELS;
                break;
            case MOVE_PIXELS:
                // Shortly after the flip starts, the intake runs in reverse.
                if (reverseTimer.seconds() > reverseDelay) {
                    intake.state = Intake.State.REVERSE;
                }
                if (flipTimer.seconds() > flipTime + extraWait) {
                    intake.state = Intake.State.STOP;
                    gates.state = TransferGates.State.PUSH;
                    state = State.RETURN_BOX;
                    pushTimer.reset();
                }
                break;
            case RETURN_BOX:
                if (pushTimer.seconds() > pushTime) {
                    gates.state = TransferGates.State.OPEN;
                    outtakeLock.state = ServoLatch.State.CLOSED;
                    box.setPosition(boxCollectPosition);
                    state = State.IDLE;
                }
                break;
            default:
                break;
        }
    }
}
