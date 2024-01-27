

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "testautonomus (Blocks to Java)")
public class  testautonomus extends LinearOpMode {

    private DcMotor m_left;
    private DcMotor m_right;
    private Servo horizontal;
    private Servo rotate;
    private Servo left_claw;
    private Servo right_claw;
    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor vertelka;

    double delta_power_per_tick;
    double power_in_moment;
    double wheel_len;
    double amount_ticks_to_rotate;
    long half_of_the_ticks;
    int last_current_position;
    int param_changer;
    private static final int[] vertelka_positions = {10, 300, 600, 800, 1000, 1330};
    private BNO055IMU imu_IMU;
    @Override
    public void runOpMode() {
        m_left = hardwareMap.get(DcMotor.class, "m_left");
        m_right = hardwareMap.get(DcMotor.class, "m_right");
        horizontal = hardwareMap.get(Servo.class, "horizontal");
        rotate = hardwareMap.get(Servo.class, "rotate");
        left_claw = hardwareMap.get(Servo.class, "left_claw");
        right_claw = hardwareMap.get(Servo.class, "right_claw");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu_IMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu_IMU.initialize(parameters);

        vertelka = hardwareMap.get(DcMotor.class, "vertelka");

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            set_zero_behavior();
            m_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotate.setDirection(Servo.Direction.FORWARD);

            vertelka.setDirection(DcMotorSimple.Direction.REVERSE);
            vertelka.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertelka.setTargetPosition(vertelka_positions[0]);
            vertelka.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vertelka.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





            rotate.setPosition(0.95);
            horizontal.setPosition(0.87);
            set_vertelka_pos(0);
            claw_close(true);
            set_vertelka_pos(2);
            lift_move(1, 750);
            sleep(200);

            sleep(500);
            // claw_close(false);
            sleep(500);
            forward_backward_left_right(103, "r");
            sleep(500);
            forward_backward_left_right(10, "f");
            sleep(1000);
            horizontal.setPosition(0);
            sleep(2000);
            claw_close(false);
            sleep(2000);
            forward_backward_left_right(10, "b");
            sleep(500);
            horizontal.setPosition(0.87);
            sleep(1000);
            sleep(700);
            claw_close(true);
            sleep(500);
            sleep(200);
            forward_backward_left_right(33, "l");
            sleep(300);
            lift_move(0, 751);
            sleep(1000);
            set_vertelka_pos(0);
            third_position();


        }
    }


    private void first_position()
    {
        forward_backward_left_right(55, "b");
    }
    private void second_position()
    {
        telemetry.addLine("2");
    }
    private void third_position()
    {
        forward_backward_left_right(55, "f");

    }
    private void lift_move(int lift_pos, int bebra) {
        if (lift_pos == 1) {
            m_left.setDirection(DcMotorSimple.Direction.REVERSE);
            m_right.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            m_left.setDirection(DcMotorSimple.Direction.FORWARD);
            m_right.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        m_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_left.setTargetPosition(bebra);
        m_right.setTargetPosition(bebra);
        m_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        m_left.setPower(1);
        m_right.setPower(1);
        while (!(!m_left.isBusy() && !m_right.isBusy()) & opModeIsActive()) {
            telemetry.update();
        }
    }


    private void set_vertelka_pos(int pos) {
        vertelka.setTargetPosition(vertelka_positions[pos]);
        vertelka.setPower(0.2);
        while (vertelka.isBusy()) {
            continue;
        }
    }


    private void claw_close(boolean closed) {
        if (closed) {
            left_claw.setPosition(0.5);
            right_claw.setPosition(0.45);
        } else {
            left_claw.setPosition(0.75);
            right_claw.setPosition(0.25);
        }
    }


    private void rotate2(int degrees, String left_right) {
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
        while (!(!left_back.isBusy() && !left_front.isBusy() && !right_back.isBusy() && !right_front.isBusy()) & opModeIsActive()) {
            // Put loop blocks here.
            telemetry.update();
        }
        set_power(0, 0, 0, 0);
    }


    private void reset_encoders() {
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    private void run_to_position() {
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void set_target_position(double left_front2, double left_back2, double right_front2, double right_back2) {
        left_back.setTargetPosition((int) left_back2);
        telemetry.addData("lb", left_back2);
        left_front.setTargetPosition((int) left_front2);
        telemetry.addData("lf", left_front.getTargetPosition());
        right_back.setTargetPosition((int) right_back2);
        telemetry.addData("rb", right_back.getTargetPosition());
        right_front.setTargetPosition((int) right_front2);
        telemetry.addData("rf", right_front.getTargetPosition());
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void set_power(double left_front2, double left_back2, double right_front2, double right_back2) {
        left_back.setPower(left_back2);
        right_back.setPower(right_back2);
        left_front.setPower(left_front2);
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

    /**
     * Describe this function...
     */
    private void move_left(int amount_sm) {
        wheel_len = Math.PI * 10.16;
        left_front.setDirection(DcMotorSimple.Direction.FORWARD);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        reset_encoders();
        set_target_position((amount_sm / wheel_len) * 480 * Math.sin(45 / 180 * Math.PI), (amount_sm / wheel_len) * 480 * Math.sin(45 / 180 * Math.PI), (amount_sm / wheel_len) * 480 * Math.sin(45 / 180 * Math.PI), (amount_sm / wheel_len) * 480 * Math.sin(45 / 180 * Math.PI));
        run_to_position();
        waitForStart();
        set_power(0.3, 0.3, 0.3, 0.3);
    }
    private void forward_backward_left_right(int amount_sm, String forward) {
        wheel_len = Math.PI * 10.16;
        if (forward.equals("b")) {
            left_front.setDirection(DcMotorSimple.Direction.FORWARD);
            left_back.setDirection(DcMotorSimple.Direction.FORWARD);
            right_front.setDirection(DcMotorSimple.Direction.REVERSE);
            right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (forward.equals("f")) {
            left_front.setDirection(DcMotorSimple.Direction.REVERSE);
            left_back.setDirection(DcMotorSimple.Direction.REVERSE);
            right_front.setDirection(DcMotorSimple.Direction.FORWARD);
            right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (forward.equals("l")) {
            left_front.setDirection(DcMotorSimple.Direction.REVERSE);
            left_back.setDirection(DcMotorSimple.Direction.FORWARD);
            right_front.setDirection(DcMotorSimple.Direction.REVERSE);
            right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            left_front.setDirection(DcMotorSimple.Direction.FORWARD);
            left_back.setDirection(DcMotorSimple.Direction.REVERSE);
            right_front.setDirection(DcMotorSimple.Direction.FORWARD);
            right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        reset_encoders();
        set_target_position((amount_sm / wheel_len) * 480 * Math.sin(Math.PI / 4.0), (amount_sm / wheel_len) * 480 * Math.sin(Math.PI / 4.0), (amount_sm / wheel_len) * 480 * Math.sin(Math.PI / 4.0), (amount_sm / wheel_len) * 480 * Math.sin(Math.PI / 4.0));
        run_to_position();
        set_power(0.15, 0.15, 0.15, 0.15);
        telemetry.addData("", !(!right_back.isBusy() || !right_front.isBusy() || !left_back.isBusy() || !left_front.isBusy()) & opModeIsActive());
        telemetry.update();
        while (!(!right_back.isBusy() || !right_front.isBusy() || !left_back.isBusy() || !left_front.isBusy()) & opModeIsActive()) {

            telemetry.addData("", 3.0 / 4.0);
            telemetry.update();
        }
        set_zero_behavior();
        set_power(0, 0, 0, 0);
    }
    // PID-константы
    double Kp = 0.03;
    double Ki = 0.001;
    double Kd = 0.05;

    // Переменные PID-регулятора
    double previousError = 0;
    double integral = 0;

    // Переменные управления роботом
    double power = 0;
    double error = 0;


    // Функция PID-регулятора для поворота робота
    void pidRotate(double targetAngle) {

        double currentAngle = imu_IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        error = targetAngle - currentAngle;

        // Вычисление PID-выхода
        double pTerm = Kp * error;
        integral += error;
        double iTerm = Ki * integral;
        double dTerm = Kd * (error - previousError);
        power = pTerm + iTerm + dTerm;
        previousError = error;

        // Ограничение мощности
        if (power > 1) {
            power = 1;
        } else if (power < -1) {
            power = -1;
        }

        // Управление роботом
        left_front.setPower(-power);
        right_front.setPower(power);
        left_back.setPower(-power);
        right_back.setPower(power);
    }

}