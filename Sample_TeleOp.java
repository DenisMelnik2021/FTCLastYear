package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
// написан примитивный телеоп для:
// движения колесной базы (написан полный телеоп)(без тестирования в полевых условиях на ошибки)
// движения мотора уточки(движение в 2 стороны по нажатию кнопок)(без тестирования в полевых условиях на ошибки)
// движения заборщика кубика(вперед назад по нажатию кнопок)(без тестирования в полевых условиях на ошибки)
// движения лифта(вверх вниз по нажатию кнопок todo (без стоп ограничений)
// движение севры (по кнопке для сбрасывания шарика и автоматическое возвращения сервы в исходное положение)(без тестирования в полевых условиях на ошибки)
// todo автоматические комбинации забора шарика
public class Sample_TeleOp extends LinearOpMode {
    DcMotor left_ass, right_ass, left_forward, right_forward, duck_motor, lift_motor, swamper_motor;
    Servo lift_servo;
    public void runOpMode(){
        left_ass = hardwareMap.dcMotor.get("left_ass");
        right_ass = hardwareMap.dcMotor.get("right_ass");
        left_forward = hardwareMap.dcMotor.get("left_forward");
        right_forward = hardwareMap.dcMotor.get("right_forward");
        duck_motor = hardwareMap.dcMotor.get("duck_motor");
        lift_motor = hardwareMap.dcMotor.get("lift_motor");
        swamper_motor = hardwareMap.dcMotor.get("swamper_motor");
        lift_servo = hardwareMap.servo.get("servo_rotate_lift");
        lift_servo.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_stick_x > gamepad1.left_stick_y && gamepad1.left_stick_x != 0) {
                // право считается значением по дефолту
                left_ass.setDirection(DcMotorSimple.Direction.REVERSE);
                right_ass.setDirection(DcMotorSimple.Direction.REVERSE);
                left_forward.setDirection(DcMotorSimple.Direction.FORWARD);
                right_forward.setDirection(DcMotorSimple.Direction.FORWARD);
                // на моторы подается пропорционально отклоеннию на контролере
                left_ass.setPower(gamepad1.left_stick_x);
                right_ass.setPower(gamepad1.left_stick_x);
                left_forward.setPower(gamepad1.left_stick_x);
                right_forward.setPower(gamepad1.left_stick_x);
            } else if (gamepad1.left_stick_x < gamepad1.left_stick_y && gamepad1.left_stick_y != 0) {
                // право считается значением по дефолту
                left_ass.setDirection(DcMotorSimple.Direction.FORWARD);
                right_ass.setDirection(DcMotorSimple.Direction.REVERSE);
                left_forward.setDirection(DcMotorSimple.Direction.FORWARD);
                right_forward.setDirection(DcMotorSimple.Direction.REVERSE);
                // на моторы подается пропорционально отклоеннию на контролере
                left_ass.setPower(gamepad1.left_stick_y);
                right_ass.setPower(gamepad1.left_stick_y);
                left_forward.setPower(gamepad1.left_stick_y);
                right_forward.setPower(gamepad1.left_stick_y);
            }
            if (gamepad1.right_stick_x != 0){
                left_ass.setDirection(DcMotorSimple.Direction.FORWARD);
                right_ass.setDirection(DcMotorSimple.Direction.FORWARD);
                left_forward.setDirection(DcMotorSimple.Direction.FORWARD);
                right_forward.setDirection(DcMotorSimple.Direction.FORWARD);
                left_ass.setPower(gamepad1.right_stick_x);
                right_ass.setPower(gamepad1.right_stick_x);
                left_forward.setPower(gamepad1.right_stick_x);
                right_forward.setPower(gamepad1.right_stick_x);
            } else{
                left_ass.setPower(0);
                right_ass.setPower(0);
                left_forward.setPower(0);
                right_forward.setPower(0);
            }
            if (gamepad1.x){
                duck_motor.setPower(1);
            } else if (gamepad1.b) {
                duck_motor.setPower(-1);
            } else{
                duck_motor.setPower(0);
            }
            if (gamepad1.y){
                lift_motor.setPower(1);
            } else if (gamepad1.a){
                lift_motor.setPower(-1);
            } else{
                lift_motor.setPower(0);
            }
            if (gamepad1.dpad_up){
                swamper_motor.setPower(1);
            } else if (gamepad1.dpad_down){
                swamper_motor.setPower(-1);
            } else{
                swamper_motor.setPower(0);
            }
            if (gamepad1.left_bumper) {
                lift_servo.setPosition(0.25);
                sleep(2000);
                lift_servo.setPosition(0);
            } else if (gamepad1.right_bumper) {
                lift_servo.setPosition(-0.25);
                sleep(2000);
                lift_servo.setPosition(0);
            }
        }
    }
}