package org.firstinspires.ftc.teamcode.drive.writtenCode.hardwareTesters;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

/**
 * Holds the claw rotation servo at a position set live from FTC Dashboard.
 * Driver Y runs the intake while held.
 */
@Config
@TeleOp(name = "Claw Servo Tester", group = "Linear OpMode")
public class ClawServoTester extends LinearOpMode {
    public static double servoPosition = 0.115;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RobotHardware robot = new RobotHardware(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            robot.clawRotationServo.setPosition(servoPosition);
            robot.intakeMotor.setPower(gamepad1.y ? 0.8 : 0);
            telemetry.addData("Servo position", robot.clawRotationServo.getPosition());
            telemetry.update();
        }
    }
}
