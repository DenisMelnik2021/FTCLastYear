package org.firstinspires.ftc.teamcode;

import java.lang.Math;


public class Speeds {
    double left_front = 0;
    double left_back = 0;
    double right_front = 0;
    double right_back = 0;
    double slower_k = 1;
    private static double k = 1;
    private static double angle = 0;
    private static double magnitude = 0; // density of angle
    private static double max_speed = 0.0;

    void control(double x, double y, boolean slower, double aditional_angle, double rotate) {
        angle = Math.atan2(y, x);
        magnitude = Math.sqrt( x * x + y * y );

        left_back = -Math.sin(-aditional_angle + angle + Math.PI / 4);
        //left_back = 1;
        right_front = left_back;

        left_front = Math.sin(-aditional_angle + angle - Math.PI / 4);
        //left_front = 1;
        right_back = left_front;

        if (slower) {
            slower_k = 0.7;
        } else {
            slower_k = 1.0;
        }

        
        k = 1 / Math.max( Math.abs(left_front), Math.abs(left_back) ) * magnitude;


        left_back = left_back * k * slower_k + rotate;
        left_front = left_front * k * slower_k + rotate;
        right_back = right_back * k * slower_k - rotate;
        right_front = right_front * k * slower_k - rotate;

        // на данном этапе мы получили скорости с поворотом
        // теперь нам надо проверить выхождение за рамки скоростей мотора
        // если скорость хотя бы одного мотора выходит за рамки,
        // то мы пропрционально уменьшаем значение всех моторов

        // находим выходящую за рамки скорость
        max_speed = Math.max(
                Math.max(
                        Math.abs(left_back),
                        Math.abs(left_front)
                ),
                Math.max(
                        Math.abs(right_front),
                        Math.abs(right_back)
                )
        );

        // проверяем, выходит ли за рамки, если да, то пропорцианально домножаем
        if (max_speed > 1.0) {
            k = 1 / max_speed;
            left_front *= k;
            right_front *= k;
            left_back *= k;
            right_back *= k;
        }










    }

}