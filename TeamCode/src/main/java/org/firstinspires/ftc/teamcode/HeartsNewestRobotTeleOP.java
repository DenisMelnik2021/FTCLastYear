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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "hearts_teleop_newest_robot")
public class HeartsNewestRobotTeleOP extends LinearOpMode {

    private DcMotor left_back;
    private DcMotor left_front;
    private DcMotor right_back;
    private DcMotor right_front;
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor vertelka;
    private Servo horizontal;
    private Servo left_claw;
    private Servo right_claw;
    private Servo rotate_cone;
    private Speeds speeds;
    double angle = 0;
    private IMU imu_IMU;
    double rotate = 0.0;
    private static final int vertelka_top_position;
    private static final int[] vertelka_positions;
    private static final double[] horizontal_positions;

    static {
        vertelka_positions = new int[]{10, 300, 600, 800, 1000, 1330};
        horizontal_positions = new double[]{0.87, 0.45, 0.00, 0.68, 0.35, 0.00};
        vertelka_top_position = 900;
    }

    boolean statement = true;
    boolean boolshit1 = true;
    boolean boolshit2 = true;

    public HeartsNewestRobotTeleOP() {
    }


    @Override
    public void runOpMode() {
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_front = hardwareMap.get(DcMotor.class, "left_front");

        vertelka = hardwareMap.get(DcMotor.class, "vertelka");

        m1 = hardwareMap.get(DcMotor.class, "m_right");
        m2 = hardwareMap.get(DcMotor.class, "m_left");
        horizontal = hardwareMap.get(Servo.class, "horizontal");
        left_claw = hardwareMap.get(Servo.class, "left_claw");
        right_claw = hardwareMap.get(Servo.class, "right_claw");
        rotate_cone = hardwareMap.get(Servo.class, "rotate");


        speeds = new Speeds();

        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        horizontal.setDirection(Servo.Direction.REVERSE);
        horizontal.setPosition(0);
        telemetry.addData("You can start!!!) Go!Go!go!", 0);
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            horizontal.setDirection(Servo.Direction.FORWARD);
            left_back.setDirection(DcMotorSimple.Direction.REVERSE);
            left_front.setDirection(DcMotorSimple.Direction.REVERSE);



            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            m2.setDirection(DcMotorSimple.Direction.FORWARD);
            m1.setDirection(DcMotorSimple.Direction.REVERSE);

            m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            vertelka.setDirection(DcMotorSimple.Direction.REVERSE);
            vertelka.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertelka.setTargetPosition(vertelka_positions[0]);
            vertelka.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vertelka.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            right_claw.setDirection(Servo.Direction.REVERSE);
            left_claw.scaleRange(0.52, 0.75); //w
            right_claw.scaleRange(0.15, 0.4);
            horizontal.setPosition(0.95);

            imu_IMU.resetYaw();


            // Put run blocks here.
            while (opModeIsActive()) {
                vertelka.setPower(0.2);


                orientation = imu_IMU.getRobotYawPitchRollAngles();

                angle = orientation.getYaw(AngleUnit.RADIANS);

                if (gamepad1.a) {
                    imu_IMU.resetYaw();
                }

                if (Math.abs(gamepad1.right_stick_x) >= 0.2) {
                    rotate = gamepad1.right_stick_x;
                } else {
                    rotate = 0;
                }


                if (gamepad1.a) {
                    imu_IMU.resetYaw();
                }


                if (gamepad1.dpad_down) {
                    for (int i=0; i < vertelka_positions.length; ++i) {
                        vertelka_positions[i] -= 1;
                    }
                    sleep(50);
                }

                if (gamepad1.dpad_up) {
                    for (int i=0; i < vertelka_positions.length; ++i) {
                        vertelka_positions[i] += 1;
                    }
                    sleep(50);
                }



                speeds.control(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.triangle, angle, -rotate);

                left_back.setPower(speeds.left_back);
                right_back.setPower(speeds.right_back);
                left_front.setPower(speeds.left_front);
                right_front.setPower(speeds.right_front);
                telemetry.addData("lf", speeds.left_front);
                telemetry.addData("rf", speeds.right_front);
                telemetry.addData("angle", orientation.getYaw(AngleUnit.DEGREES));


                if ((gamepad2.right_trigger >= 0.2) & (m2.getCurrentPosition() < 3700)) {
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
                } else if ((gamepad2.left_trigger >= 0.2) & (m2.getCurrentPosition() + m1.getCurrentPosition() > 150)) {
                    if (m2.getCurrentPosition() - m1.getCurrentPosition() >= 15 ) {
                        m1.setPower(-gamepad2.left_trigger * 0.8);
                        m2.setPower(-gamepad2.left_trigger);
                    } else if (m2.getCurrentPosition() - m1.getCurrentPosition() <= -15) {
                        m1.setPower(-gamepad2.left_trigger);
                        m2.setPower(-gamepad2.left_trigger * 0.8);
                    } else {
                        m1.setPower(-gamepad2.left_trigger);
                        m2.setPower(-gamepad2.left_trigger);
                    }
                } else if ((gamepad2.left_trigger >= 0.2) & (m2.getCurrentPosition() + m1.getCurrentPosition() > 0)) {
                    if (m2.getCurrentPosition() - m1.getCurrentPosition() >= 15 ) {
                        m1.setPower(-gamepad2.left_trigger * 0.8 * m1.getCurrentPosition() * 0.002);
                        m2.setPower(-gamepad2.left_trigger * m2.getCurrentPosition() * 0.002);
                    } else if (m2.getCurrentPosition() - m1.getCurrentPosition() <= -15) {
                        m1.setPower(-gamepad2.left_trigger * m1.getCurrentPosition() * 0.002);
                        m2.setPower(-gamepad2.left_trigger * 0.8 * m2.getCurrentPosition() * 0.002);
                    } else {
                        m1.setPower(-gamepad2.left_trigger * m1.getCurrentPosition() * 0.002);
                        m2.setPower(-gamepad2.left_trigger * m2.getCurrentPosition() * 0.002);
                    }
                } else if (m2.getCurrentPosition() < 0 | m2.getCurrentPosition() < 0) {
                    if (m2.getCurrentPosition() < 0) {
                        m2.setPower(0.002 * -m2.getCurrentPosition());
                    }
                    if (m1.getCurrentPosition() < 0) {
                        m1.setPower(0.002 * -m1.getCurrentPosition());
                    }
                } else {
                    m1.setPower(0);
                    m2.setPower(0);
                }




                /*if (Math.abs(gamepad2.left_stick_y) >= 0.2) {
                    vertelka.setPower(gamepad2.left_stick_y);
                } else {
                    vertelka.setPower(0);
                }*/
                if (gamepad2.a) {
                    vertelka.setTargetPosition(vertelka_positions[0]);
                } else if (gamepad2.b) {
                    vertelka.setTargetPosition(vertelka_positions[1]);
                } else if (gamepad2.y) {
                    vertelka.setTargetPosition(vertelka_positions[2]);
                } else if (gamepad2.dpad_up) {
                    vertelka.setTargetPosition(vertelka_positions[3]);
                } else if (gamepad2.dpad_left) {
                    vertelka.setTargetPosition(vertelka_positions[4]);
                } else if (gamepad2.dpad_down) {
                    vertelka.setTargetPosition(vertelka_positions[5]);
                }

                boolshit1 = vertelka.getCurrentPosition() > vertelka_positions[0] + 180;
                boolshit2 = vertelka.getCurrentPosition() < vertelka_positions[5] - 180;
                statement = boolshit1 & boolshit2;



                if (vertelka.getTargetPosition() == vertelka_positions[0] & boolshit2) {
                    horizontal.setPosition(horizontal_positions[0]);
                } else if (vertelka.getTargetPosition() == vertelka_positions[1] & statement) {
                    horizontal.setPosition(horizontal_positions[1]);
                } else if (vertelka.getTargetPosition() == vertelka_positions[2] & statement) {
                    horizontal.setPosition(horizontal_positions[2]);
                } else if (vertelka.getTargetPosition() == vertelka_positions[3] & statement) {
                    horizontal.setPosition(horizontal_positions[3]);
                } else if (vertelka.getTargetPosition() == vertelka_positions[4] & statement) {
                    horizontal.setPosition(horizontal_positions[4]);
                } else if (vertelka.getTargetPosition() == vertelka_positions[5] & boolshit1) {
                    horizontal.setPosition(horizontal_positions[5]);
                }


                if (vertelka.getTargetPosition() >= vertelka_positions[3] &
                        vertelka.getCurrentPosition() > vertelka_positions[2]) {
                    rotate_cone.setPosition(0);
                } else if (vertelka.getTargetPosition() <= vertelka_positions[2] &
                        vertelka.getCurrentPosition() < vertelka_positions[3]) {
                    rotate_cone.setPosition(0.95);
                }


                left_claw.setPosition(gamepad1.right_bumper | gamepad2.right_bumper ? 0.55 : 0);
                right_claw.setPosition(gamepad1.right_bumper | gamepad2.right_bumper ? 0.55 : 0);


                telemetry.addData("height2", m2.getCurrentPosition());
                telemetry.addData("height1", m1.getCurrentPosition());
                telemetry.addData("vertelka", vertelka.getCurrentPosition());
                telemetry.addData("dpad_down", gamepad2.dpad_down);

                telemetry.update();
            }
        }

    }



}