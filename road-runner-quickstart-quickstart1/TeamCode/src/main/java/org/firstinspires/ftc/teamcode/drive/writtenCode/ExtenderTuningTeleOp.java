package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Extender;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeFourBar;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Lift;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Subsystems;

/**
 * Test mode for finding lift and extender encoder targets. The drivetrain is configured but not driven.
 * Operator left stick moves the lift, right stick moves the extender, both without limits.
 */
@TeleOp(name = "Extender Tuning", group = "Linear OpMode")
public class ExtenderTuningTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Subsystems s = new Subsystems(new RobotHardware(hardwareMap));
        s.initialize();
        new DriveMotors(hardwareMap);

        waitForStart();
        s.intakeFourBar.state = IntakeFourBar.State.DRIVE;
        s.intakeFourBar.update();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            int liftPosition = s.robot.liftMotor.getCurrentPosition();
            if (liftPosition >= -120 && s.lift.state == Lift.State.GOING_DOWN) {
                s.lift.state = Lift.State.REST;
            }

            if (gamepad2.left_stick_y > 0) {
                s.lift.targetPosition += 10;
                s.extender.state = Extender.State.HOLD;
            } else if (gamepad2.left_stick_y < 0) {
                s.lift.targetPosition -= 10;
                s.extender.state = Extender.State.HOLD;
            }

            if (gamepad2.right_stick_y > 0) {
                s.extender.targetPosition -= 5;
            }
            if (gamepad2.right_stick_y < 0) {
                s.extender.targetPosition += 5;
            }

            s.update();
        }
    }
}
