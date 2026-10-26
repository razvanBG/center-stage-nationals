package org.firstinspires.ftc.teamcode.drive.writtenCode.hardwareTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/**
 * Lets the lift and extender motors float so they can be moved by hand, and shows their
 * encoder counts. Used to read off target positions.
 */
public abstract class EncoderReadout extends LinearOpMode {
    protected abstract DcMotor[] motors(RobotHardware robot);

    protected abstract String[] labels();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RobotHardware robot = new RobotHardware(hardwareMap);
        DcMotor[] motors = motors(robot);
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            for (int i = 0; i < motors.length; i++) {
                telemetry.addData(labels()[i], motors[i].getCurrentPosition());
            }
            telemetry.update();
        }
    }
}
