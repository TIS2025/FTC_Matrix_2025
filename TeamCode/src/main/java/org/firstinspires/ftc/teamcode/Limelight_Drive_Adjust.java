package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LimeUtils.LimeGlobals.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LimeUtils.LimeGlobals;

import java.util.List;

@Config
@TeleOp(name = "Sensor: Limelight3A Drive Adjust", group = "Sensor")
public class Limelight_Drive_Adjust extends LinearOpMode {

    private static double errCpr;
    private Limelight3A limelight;
    private static Servo s1;
    public static DcMotorEx m1;
    public static CRServoImplEx cr;
    public static DcMotorEx enc;
    public static double kp = 0.25, ki = 0, kd = 0, setPoint = 0;
    public static MecanumDrive drive;



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

//        limelight.pipelineSwitch(5);
        limelight.pipelineSwitch(9);

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
            targetFound = false;

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
                targetFound = true;
                botpose = result.getBotpose();


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
                    angle_radian = (LimeGlobals.a1 + LimeGlobals.a2) * (3.14159 / 180.0);
                    dist = (LimeGlobals.h2 - LimeGlobals.h1)/ Math.tan(LimeGlobals.angle_radian);
                    tx = fr.getTargetXDegrees();
                    ty = fr.getTargetYDegrees();
                    tYaw = fr.getTargetPoseRobotSpace().getOrientation().getYaw();
                    telemetry.addData("Fiducial", " Yaw: %.2f", LimeGlobals.yaw);
                    telemetry.addData("Fiducial", " Pitch: %.2f", pitch);
                    telemetry.addData("Fiducial", " Roll: %.2f", roll);
                }
            }else {
                yaw = 0;
                corr = 0;
                telemetry.addData("Limelight", "No data available");
            }

            if (gamepad1.left_bumper && !targetFound){

                Find_AprilTag(telemetry);

            }
            else if (gamepad1.left_bumper && targetFound) {

                Track_AprilTag(telemetry);

            } else {
                rangeError = 0;
                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                driveS  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveS, strafe, turn);
                drive.driveFieldCentric(strafe,  driveS, -turn, drive.localizer.getPose().heading.toDouble());

            }

            print_stuffs(telemetry);

            telemetry.update();

        }
        limelight.stop();
    }

    public static void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        drive.leftFront.setPower(frontLeftPower);
        drive.rightFront.setPower(frontRightPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightBack.setPower(backRightPower);
    }


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


    public static void Track_AprilTag(Telemetry telemetry){
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        rangeError      = convert_m_to_inches(z_pose) - DESIRED_DISTANCE - offset;
        double  headingError    = tx;
        double  yawError        = tYaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        driveS  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        moveRobot(driveS, strafe, -turn);
    }
    public static void Find_AprilTag(Telemetry telemetry){
        telemetry.addLine("No Target Found - Rotating to find target");
        if(Math.toDegrees( drive.localizer.getPose().heading.toDouble()) > 0){
            rangeError      = 0;
            double  headingError    = Math.toDegrees( drive.localizer.getPose().heading.toDouble()) - DESIRED_ANGLE;
            double  yawError        = 0;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            driveS  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -1, 1) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        }
        else if(Math.toDegrees( drive.localizer.getPose().heading.toDouble()) < 0){
            rangeError      = 0;
            double  headingError    = DESIRED_ANGLE - Math.toDegrees( drive.localizer.getPose().heading.toDouble()) ;
            double  yawError        = 0;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            driveS  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -1, 1) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        }
        moveRobot(driveS, strafe, -turn);
        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveS, strafe, turn);

    }

    public static void print_stuffs(Telemetry telemetry){
        telemetry.addData("Range error: ", rangeError);
        telemetry.addData("corr", corr);
        telemetry.addData("Tx", tx);
        telemetry.addData("Ty", ty);
        telemetry.addData("T Yaw", tYaw);
        telemetry.addData("x ", convert_m_to_inches(x_pose));
        telemetry.addData("y ", convert_m_to_inches(y_pose));
        telemetry.addData("z ", convert_m_to_inches(z_pose));
        telemetry.addData("dist", dist);
        telemetry.addData("x robot", drive.localizer.getPose().position.x);
        telemetry.addData("y robot", drive.localizer.getPose().position.y);
        telemetry.addData("heading real", Math.toDegrees(drive.localizer.getPose().heading.real));
        telemetry.addData("heading img", Math.toDegrees(drive.localizer.getPose().heading.imag));
        telemetry.addData("heading RAD", drive.localizer.getPose().heading.real);
        telemetry.addData("heading RAD to double",Math.toDegrees( drive.localizer.getPose().heading.toDouble()));
        telemetry.addData("heading RAD to double Rad",drive.localizer.getPose().heading.toDouble());
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
