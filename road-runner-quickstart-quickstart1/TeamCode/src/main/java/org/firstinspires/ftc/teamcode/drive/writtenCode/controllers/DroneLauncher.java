package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/** Releases the paper drone. */
public class DroneLauncher extends StateController<DroneLauncher.State> {
    public enum State { HOLD, LAUNCH }

    public double holdPosition = 0.7;
    public double launchPosition = 0;
    private final Servo servo;

    public DroneLauncher(RobotHardware robot) {
        super(State.HOLD);
        servo = robot.droneServo;
    }

    @Override
    protected void apply(State s) {
        servo.setPosition(s == State.LAUNCH ? launchPosition : holdPosition);
    }
}
