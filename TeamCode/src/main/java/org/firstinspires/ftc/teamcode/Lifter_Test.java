package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Lifter_Test extends LinearOpMode {
    public static DcMotorEx m1, m2, m3, m4;
    public static int hangValue = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m4 = hardwareMap.get(DcMotorEx.class, "m4");

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                runMotor(m1, hangValue, 0.5);
                runMotor(m2, hangValue, 0.5);
            }
            telemetry.addData("m1 ", m1.getCurrentPosition());
            telemetry.addData("m2 ", m2.getCurrentPosition());
            telemetry.addData("m3 ", m3.getCurrentPosition());
            telemetry.addData("m4 ", m4.getCurrentPosition());
            telemetry.update();
        }

    }

    public static void runMotor(DcMotorEx m, int value, double power){
        m.setTargetPosition(value);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(power);
    }
}
