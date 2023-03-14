package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "HeartsAutonomusRighrt")
public class HeartsAutonomusRight extends LinearOpMode {

    private DcMotor m_left;
    private DcMotor m_right;
    private Servo claw_left;
    private Servo claw_right;
    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;

    int square_to_move_robot;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        m_left = hardwareMap.get(DcMotor.class, "m_left");
        m_right = hardwareMap.get(DcMotor.class, "m_right");
        claw_left = hardwareMap.get(Servo.class, "claw_left");
        claw_right = hardwareMap.get(Servo.class, "claw_right");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        waitForStart();
        set_zero_behavior();
        m_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw_left.setDirection(Servo.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        telemetry.update();
        waitForStart();
        set_zero_behavior();

        for(int i = 0; i < 100; i++)
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null){
            square_to_move_robot = 2;
        }else if(tagOfInterest.id == LEFT){
            square_to_move_robot = 1;
        }else if(tagOfInterest.id == MIDDLE){
            square_to_move_robot = 2;
        }else{
            square_to_move_robot = 3;
        }

        telemetry.addData("where_to_park", square_to_move_robot);
        telemetry.update();
        ONE_CUBE_CAMERA_DIAG_1();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    /**
     * Describe this function...
     */
    private void ONE_CUBE_CAMERA_DIAG_1() {
        claw_close(false);
        sleep(700);
        lift_move(1);
        forward_backward_left_right(105, "r");
        sleep(300);
        forward_backward_left_right(15, "f");
        sleep(1600);
        claw_close(true);
        forward_backward_left_right(13, "b");
        sleep(300);
        lift_move(0);
        claw_close(false);
        forward_backward_left_right(32, "l");
        sleep(300);
        if (square_to_move_robot == 1) {
            forward_backward_left_right(62, "f");
        } else if (square_to_move_robot == 3) {
            forward_backward_left_right(70, "b");
        } else {
        }
    }


    private void lift_move(int lift_pos) {
        if (lift_pos == 1) {
            m_left.setDirection(DcMotorSimple.Direction.REVERSE);
            m_right.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            m_left.setDirection(DcMotorSimple.Direction.FORWARD);
            m_right.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        m_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_left.setTargetPosition(4000);
        m_right.setTargetPosition(4000);
        m_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_left.setPower(1);
        m_right.setPower(1);
        while (!(!m_left.isBusy() && !m_right.isBusy())  && opModeIsActive()) {
            // Put loop blocks here.
            telemetry.update();
        }
        m_left.setPower(0);
        m_right.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void claw_close(boolean closed) {
        if (closed) {
            claw_left.setPosition(0.6);
            claw_right.setPosition(0.6);
        } else {
            claw_left.setPosition(0.75);
            claw_right.setPosition(0.75);
        }
    }


    private void rotate(int degrees, String left_right) {
        if (left_right.equals("left")) {
            left_front.setDirection(DcMotorSimple.Direction.FORWARD);
            left_back.setDirection(DcMotorSimple.Direction.FORWARD);
            right_front.setDirection(DcMotorSimple.Direction.FORWARD);
            right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            left_front.setDirection(DcMotorSimple.Direction.REVERSE);
            left_back.setDirection(DcMotorSimple.Direction.REVERSE);
            right_front.setDirection(DcMotorSimple.Direction.REVERSE);
            right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        reset_encoders();
        set_target_position(degrees * 5.3, degrees * 5.3, degrees * 5.3, degrees * 5.3);
        run_to_position();
        set_power(0.3, 0.3, 0.3, 0.3);
        while (!(!left_back.isBusy() && !left_front.isBusy() && !right_back.isBusy() && !right_front.isBusy())  && opModeIsActive()) {
            // Put loop blocks here.
            telemetry.update();
        }
        set_power(0, 0, 0, 0);
    }


    private void forward_backward_left_right(double amount_sm, String forward) {
        double wheel_len;

        wheel_len = Math.PI * 10.16;
        if (forward.equals("b")) {
            left_front.setDirection(DcMotorSimple.Direction.REVERSE);
            left_back.setDirection(DcMotorSimple.Direction.REVERSE);
            right_front.setDirection(DcMotorSimple.Direction.FORWARD);
            right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (forward.equals("f")) {
            left_front.setDirection(DcMotorSimple.Direction.FORWARD);
            left_back.setDirection(DcMotorSimple.Direction.FORWARD);
            right_front.setDirection(DcMotorSimple.Direction.REVERSE);
            right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (forward.equals("r")) {
            left_front.setDirection(DcMotorSimple.Direction.FORWARD);
            left_back.setDirection(DcMotorSimple.Direction.REVERSE);
            right_front.setDirection(DcMotorSimple.Direction.FORWARD);
            right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            left_front.setDirection(DcMotorSimple.Direction.REVERSE);
            left_back.setDirection(DcMotorSimple.Direction.FORWARD);
            right_front.setDirection(DcMotorSimple.Direction.REVERSE);
            right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        reset_encoders();
        double targetPosition = (amount_sm / wheel_len) * 480 * Math.sin(((double)45 / 180) * Math.PI);
        set_target_position(targetPosition, targetPosition, targetPosition, targetPosition);
        run_to_position();
        set_power(0.2, 0.2, 0.2, 0.2);
        telemetry.addData("aboba", left_back.getTargetPosition());
        while (!(!left_back.isBusy() && !left_front.isBusy() && !right_back.isBusy() && !right_front.isBusy())  && opModeIsActive()) {
            set_power(0.2, 0.2, 0.2, 0.2);
            telemetry.update();
        }
        set_power(0, 0, 0, 0);
    }

    /**
     * Describe this function...
     */
    private void reset_encoders() {
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void run_to_position() {
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void set_target_position(double left_front2, double left_back2, double right_front2, double right_back2) {
        left_back.setTargetPosition((int) left_back2);
        left_front.setTargetPosition((int) left_front2);
        right_back.setTargetPosition((int) right_back2);
        right_front.setTargetPosition((int) right_front2);
    }

    /**
     * Describe this function...
     */
    private void set_power(double left_front2, double left_back2, double right_front2, double right_back2) {
        left_back.setPower(left_back2);
        left_front.setPower(left_front2);
        right_back.setPower(right_back2);
        right_front.setPower(right_front2);
    }

    /**
     * Describe this function...
     */
    private void set_zero_behavior() {
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}