package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "hearts_teleop_new_robot")
public class HeartsNewRobotTeleOP extends LinearOpMode {

    private DcMotor left_back;
    private DcMotor left_front;
    private DcMotor right_back;
    private DcMotor right_front;
    private DcMotor m1;
    private DcMotor m2;
    private Servo claw_left;
    private Servo claw_right;
    private Servo claw;
    private Speeds speeds;
    private static int ticks_per_rotation = 1440;
    double pos = 0.60;
    double angle = 0;
    private IMU imu_IMU;
    double rotate = 0.0;

    @Override
    public void runOpMode() {
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        claw_left = hardwareMap.get(Servo.class, "claw_left");
        claw_right = hardwareMap.get(Servo.class, "claw_right");
        m1 = hardwareMap.get(DcMotor.class, "m_right");
        m2 = hardwareMap.get(DcMotor.class, "m_left");


        speeds = new Speeds();

        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));


        waitForStart();

        if (opModeIsActive()) {
            left_back.setDirection(DcMotorSimple.Direction.REVERSE);
            left_front.setDirection(DcMotorSimple.Direction.REVERSE);

            claw_left.setDirection(Servo.Direction.REVERSE);


            m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            m2.setDirection(DcMotorSimple.Direction.REVERSE);

            m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            imu_IMU.resetYaw();


            // Put run blocks here.
            while (opModeIsActive()) {

                orientation = imu_IMU.getRobotYawPitchRollAngles();
                angularVelocity = imu_IMU.getRobotAngularVelocity(AngleUnit.RADIANS);

                angle = orientation.getYaw(AngleUnit.RADIANS);

                if (Math.abs(gamepad1.right_stick_x) >= 0.5) {
                    rotate = gamepad1.right_stick_x;
                }

                speeds.control(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.triangle, angle, rotate);

                left_back.setPower(speeds.left_back);
                right_back.setPower(speeds.right_back);
                left_front.setPower(speeds.left_front);
                right_front.setPower(speeds.right_front);
                telemetry.addData("lf", speeds.left_front);
                telemetry.addData("rf", speeds.right_front);


                telemetry.addData("m1", m1.getCurrentPosition());
                telemetry.addData("m2", m2.getCurrentPosition());

                if ((gamepad2.right_trigger >= 0.2) & (m2.getCurrentPosition() < 5800)) {
                    if (m2.getCurrentPosition() - m1.getCurrentPosition() >= 15) {
                        m1.setPower(gamepad2.right_trigger);
                        m2.setPower(gamepad2.right_trigger * 0.8);
                    } else if (m2.getCurrentPosition() - m1.getCurrentPosition() <= -15) {
                        m1.setPower(gamepad2.right_trigger * 0.8);
                        m2.setPower(gamepad2.right_trigger);
                    } else {
                        m1.setPower(gamepad2.right_trigger);
                        m2.setPower(gamepad2.right_trigger);
                    }
                } else if ((gamepad2.left_trigger >= 0.2) & (m2.getCurrentPosition() > 50)) {
                    if (m2.getCurrentPosition() - m1.getCurrentPosition() >= 15) {
                        m1.setPower(-gamepad2.left_trigger * 0.8);
                        m2.setPower(-gamepad2.left_trigger);
                    } else if (m2.getCurrentPosition() - m1.getCurrentPosition() <= -15) {
                        m1.setPower(-gamepad2.left_trigger);
                        m2.setPower(-gamepad2.left_trigger * 0.8);
                    } else {
                        m1.setPower(-gamepad2.left_trigger);
                        m2.setPower(-gamepad2.left_trigger);
                    }
                } else {
                    m1.setPower(0);
                    m2.setPower(0);
                }

                if (gamepad2.right_bumper) {
                    pos = 0.75;
                } else if (gamepad2.left_bumper) {
                    pos = 0.60;
                }

                claw_left.setPosition(pos);
                claw_right.setPosition(pos);

                telemetry.update();
            }
        }
    }
}