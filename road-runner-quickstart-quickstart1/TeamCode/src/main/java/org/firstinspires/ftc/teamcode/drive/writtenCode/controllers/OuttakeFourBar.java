package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/**
 * Four-bar that swings the outtake box from the transfer position out to the backdrop.
 * When the lift starts from the bottom, the swing is delayed slightly so the box clears the robot.
 */
@Config
public class OuttakeFourBar extends StateController<OuttakeFourBar.State> {
    public enum State {
        RECEIVE,
        SCORE,
        SCORE_AFTER_DELAY,
        WAITING_TO_SCORE,
        DOWN
    }

    public static double receivePosition = 0.07;
    public static double scorePosition = 0.69;
    public static double downPosition = 0.25;

    private final double scoreDelay = 0.05;
    private final ElapsedTime delayTimer = new ElapsedTime();
    private final Servo servo;

    public OuttakeFourBar(RobotHardware robot) {
        super(State.RECEIVE);
        servo = robot.outtakeFourBarServo;
    }

    @Override
    protected boolean isTimed(State s) {
        return s == State.WAITING_TO_SCORE;
    }

    @Override
    protected void apply(State s) {
        switch (s) {
            case RECEIVE:
                servo.setPosition(receivePosition);
                break;
            case SCORE:
                servo.setPosition(scorePosition);
                break;
            case SCORE_AFTER_DELAY:
                delayTimer.reset();
                state = State.WAITING_TO_SCORE;
                break;
            case WAITING_TO_SCORE:
                if (delayTimer.seconds() > scoreDelay) {
                    state = State.SCORE;
                }
                break;
            case DOWN:
                servo.setPosition(downPosition);
                break;
        }
    }
}
