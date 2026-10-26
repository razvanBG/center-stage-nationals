package org.firstinspires.ftc.teamcode.drive.writtenCode.hardwareTesters;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotHardware;

@TeleOp(name = "Lift Tester", group = "Linear OpMode")
public class LiftTester extends EncoderReadout {
    @Override
    protected DcMotor[] motors(RobotHardware robot) {
        return new DcMotor[]{robot.liftMotor};
    }

    @Override
    protected String[] labels() {
        return new String[]{"Lift position"};
    }
}
