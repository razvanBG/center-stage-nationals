package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Every motor, servo and sensor on the robot, looked up once by its name in the
 * Driver Station configuration. The strings must match the configuration exactly.
 */
public class RobotHardware {
    public final DcMotorEx liftMotor;
    public final DcMotorEx intakeMotor;
    public final DcMotorEx leftExtension;
    public final DcMotorEx rightExtension;

    public final Servo droneServo;
    public final Servo outtakeLockServo;
    public final Servo clawRotationServo;
    public final Servo flapServo;
    public final Servo secondPixelServo;
    public final Servo outtakeFourBarServo;
    public final Servo intakeBoxServo;
    public final Servo leftGateServo;
    public final Servo rightGateServo;
    public final Servo intakeFourBarServo;

    public final CRServo leftHangServo;
    public final CRServo rightHangServo;

    /** Break-beam sensors in the transfer box. A beam reads false when a pixel blocks it. */
    public final DigitalChannel frontBeam;
    public final DigitalChannel backBeam;

    public RobotHardware(HardwareMap map) {
        liftMotor = map.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder(liftMotor);

        intakeMotor = map.get(DcMotorEx.class, "intakeMotor");
        allowFullSpeed(intakeMotor);

        leftExtension = map.get(DcMotorEx.class, "leftExtension");
        resetEncoder(leftExtension);
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtension.setDirection(DcMotorSimple.Direction.REVERSE);
        allowFullSpeed(leftExtension);

        rightExtension = map.get(DcMotorEx.class, "rightExtension");
        resetEncoder(rightExtension);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        allowFullSpeed(rightExtension);

        droneServo = map.get(Servo.class, "droneServo");
        outtakeLockServo = map.get(Servo.class, "outtakeLock");
        clawRotationServo = map.get(Servo.class, "clawRotation");
        flapServo = map.get(Servo.class, "outtakeFlap");
        secondPixelServo = map.get(Servo.class, "secondPixelLatch");
        outtakeFourBarServo = map.get(Servo.class, "outtakeFourBar");
        intakeBoxServo = map.get(Servo.class, "intakeBox");
        leftGateServo = map.get(Servo.class, "leftGate");
        rightGateServo = map.get(Servo.class, "rightGate");
        intakeFourBarServo = map.get(Servo.class, "intakeFourBar");

        leftHangServo = map.get(CRServo.class, "leftHang");
        rightHangServo = map.get(CRServo.class, "rightHang");

        frontBeam = map.get(DigitalChannel.class, "frontBeam");
        backBeam = map.get(DigitalChannel.class, "backBeam");
    }

    /** Zeroes the encoder, then lets the motor run with encoder feedback. */
    private static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * By default the SDK caps the motor below its rated RPM so the velocity loop has headroom.
     * We remove that cap so the motor can use its full speed.
     */
    public static void allowFullSpeed(DcMotor motor) {
        MotorConfigurationType type = motor.getMotorType().clone();
        type.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(type);
    }
}
