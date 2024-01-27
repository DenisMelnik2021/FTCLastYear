package org.firstinspires.ftc.teamcode;

....................import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "govnaha_kakaha")
public class govnaha_kakaha extends LinearOpMode {
    private DcMotor arm;
    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "arm");
        waitForStart();
        arm.setPower(0.2);

    }
}
