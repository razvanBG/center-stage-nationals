package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/** Intake roller motor. Powers are static so they can be tuned live from FTC Dashboard. */
@Config
public class Intake extends StateController<Intake.State> {
    public enum State {
        STOP,
        COLLECT,
        REVERSE,
        /** Slow reverse used in autonomous to drop the purple pixel on the spike mark. */
        REVERSE_SLOW,
        STACK
    }

    public static double collectPower = 1;
    public static double reversePower = -1;
    public static double slowReversePower = -0.45;
    public static double stackPower = 0.8;

    private final DcMotorEx motor;

    public Intake(RobotHardware robot) {
        super(State.STOP);
        motor = robot.intakeMotor;
    }

    @Override
    protected void everyLoop() {
        RobotHardware.allowFullSpeed(motor);
    }

    @Override
    protected void apply(State s) {
        switch (s) {
            case STOP:
                motor.setPower(0);
                break;
            case COLLECT:
                motor.setPower(collectPower);
                break;
            case REVERSE:
                motor.setPower(reversePower);
                break;
            case REVERSE_SLOW:
                motor.setPower(slowReversePower);
                break;
            case STACK:
                motor.setPower(stackPower);
                break;
        }
    }
}
