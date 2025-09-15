package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@Config
@TeleOp(name = "Sensor: Limelight3A Servo", group = "Sensor")
public class Limelight_Servo extends LinearOpMode {

    private static double errCpr;
    private Limelight3A limelight;
    private static Servo s1;
    public static DcMotorEx m1;
    public static CRServoImplEx cr;
    public static DcMotorEx enc;
    public static Pose3D botpose;
    public static double kp = 0.25, ki = 0, kd = 0, setPoint = 0;
    public static double corr = 0, yaw = 0, dist = 0;
    public static double h1 = 10.3, h2 = 36.5, a1 = 36.6, a2 = 0, angle_radian=0;
    public static double min = 0;
    public static double max = 0;
    private int counts = 0;
    private double y_pose;
    private double x_pose;
    private double z_pose;
    MecanumDrive drive;
    private double roll;
    private double pitch;
    public static double llYaw = 0, tx = 0 , ty = 0, tYaw = 0 ;
    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        s1 = hardwareMap.get(Servo.class, "s1");
        cr = hardwareMap.get(CRServoImplEx.class, "cr");

//        m1 = hardwareMap.get(DcMotorEx.class, "m1");
//        enc =  hardwareMap.get(DcMotorEx.class,"m1");
        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
//        m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        m1.setZeroPowerBehavior(BRAKE);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(5);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        while (opModeInInit()) {
            s1.setPosition(0.5);
//            enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            if(gamepad1.back){
                drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
                    drive.updatePoseEstimate();

            }



            LLResult result = limelight.getLatestResult();


            if (result.isValid()) {
                // Access general information
                botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();




                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();


                for (LLResultTypes.FiducialResult fr : fiducialResults) {


//                    yaw = fr.getTargetPoseRobotSpace().getOrientation().getYaw();
                    yaw = fr.getRobotPoseFieldSpace().getOrientation().getYaw();
                    pitch = fr.getRobotPoseFieldSpace().getOrientation().getPitch();
                    roll = fr.getRobotPoseFieldSpace().getOrientation().getRoll();
                    x_pose = fr.getRobotPoseFieldSpace().getPosition().x;
                    y_pose = fr.getRobotPoseFieldSpace().getPosition().y;
                    z_pose = fr.getTargetPoseRobotSpace().getPosition().z;
                        a2 = fr.getTargetYDegrees();
//                    a2 = y_pose;
//                    a2 = fr.getTargetPoseRobotSpace().getOrientation().getPitch();
                    angle_radian = (a1+a2) * (3.14159 / 180.0);
                    dist = (h2 - h1)/ Math.tan(angle_radian);
                    tx = fr.getTargetXDegrees();
                    ty = fr.getTargetYDegrees();
                    tYaw = fr.getTargetPoseRobotSpace().getOrientation().getYaw();
                    telemetry.addData("Fiducial", " Yaw: %.2f", yaw);
                    telemetry.addData("Fiducial", " Pitch: %.2f", pitch);
                    telemetry.addData("Fiducial", " Roll: %.2f", roll);
                }
            }else {
                yaw = 0;
                corr = 0;
                telemetry.addData("Limelight", "No data available");
            }

//            telemetry.addData("enc: ", enc.getCurrentPosition());
//            correctTurretMotor(yaw, setPoint);
//            correctTurretEncoder(counts, setPoint, telemetry);
//            if (corr > 0){
//                cr.setPower(1);
//            }
//            else if (corr < 0){
//                cr.setPower(-1);
//            }
            telemetry.addData("corr", corr);
            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", ty);
            telemetry.addData("T Yaw", tYaw);
            telemetry.addData("x ", convert_m_to_inches(x_pose));
            telemetry.addData("y ", convert_m_to_inches(y_pose));
            telemetry.addData("z ", convert_m_to_inches(z_pose));
            telemetry.addData("x robot", drive.localizer.getPose().position.x);
            telemetry.addData("y robot", drive.localizer.getPose().position.y);
            telemetry.addData("heading real", Math.toDegrees(drive.localizer.getPose().heading.real));
            telemetry.addData("heading img", Math.toDegrees(drive.localizer.getPose().heading.imag));
            telemetry.addData("heading RAD", drive.localizer.getPose().heading.real);
            telemetry.addData("heading RAD to double",Math.toDegrees( drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("heading RAD to double Rad",drive.localizer.getPose().heading.toDouble());

//            telemetry.addData("HEAD",);
            telemetry.addData("Distance ", y_pose);
            telemetry.addData("a2 ", a2);
            telemetry.addData("a1 ", a1);
            telemetry.addData("dist ", dist);
            telemetry.update();
        }
        limelight.stop();
    }

//    public static void correctTurret(double angle, double sp){
//        corr = 0;
//        double err = 0;
//        double totalError = 0;
//        double prevError = 0;
//
//        double P = 0;
//        double I = 0;
//        double D = 0;
//        err = angle - sp;
//        P = kp*err;
//        totalError = totalError + err;
//        I = ki* totalError;
//        D = kd*(err - prevError);
//
//        corr =  P + I + D;
//        Range.clip(corr, 0, 1);
//        prevError = err;
//        s1.setPosition(abs(corr));
//
//
//    }


    public static void correctTurretMotor(double angle, double sp){
        corr = 0;
        double err = 0;
        double totalError = 0;
        double prevError = 0;

        double P = 0;
        double I = 0;
        double D = 0;
        err = angle - sp;
        P = kp*err;
        totalError = totalError + err;
        I = ki* totalError;
        D = kd*(err - prevError);

        corr =  P + I + D;
        if (err < 2 && err > -2){
            corr = 0;
        }
        Range.clip(corr, -1, 1);
        prevError = err;
//        m1.setPower(corr);
//        cr.setPwmRange(min);
        cr.setPower(corr);


    }

    public static double convert_m_to_inches(double meter){
        return meter * 39.370;
    }
    public static void correctTurretEncoder(double count, double sp, Telemetry telemetry){
        double cpr_sp = sp*8192/360;
//        double cpr_yaw = angle*8192/360;
        corr = 0;
        double err = 0;
        double totalError = 0;
        double prevError = 0;

        double P = 0;
        double I = 0;
        double D = 0;
        err = count - cpr_sp;

        P = kp*err;
        totalError = totalError + err;
        I = ki* totalError;
        D = kd*(err - prevError);

        corr =  P + I + D;
        if (err < 2 && err > -2){
            corr = 0;
        }

//        Range.clip(corr,-1,1);
        prevError = err;
//        m1.setPower(corr);
//        cr.setPwmRange(min);
        cr.setPower(corr);

        telemetry.addData("CPR_SP : ", cpr_sp);
        telemetry.addData("CPR_YAW : ", count);
        telemetry.addData("ERROR : ", err);


    }


}
