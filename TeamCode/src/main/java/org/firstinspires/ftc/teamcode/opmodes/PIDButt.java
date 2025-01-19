package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.PI;
import static java.lang.Math.round;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

@Autonomous
public class PIDButt extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    Motor[] wheels;
    Chassis bobot;
    TwoPointServo claw;
    TwoPointServo clawWrist;
    Slide slides;
    Motor slideMotor;
    PIDFController controller;
    boolean clawToggle;
    boolean clawWristToggle;
    public static double p = -0.002, i = 0, d = 0.0001;
    public static double normalF = 0.01;
    public static double zeroF = 0.12;
    public static double fullF = 0.15;
    public static double f = normalF;
    //    public static double f = 0.8;
    public static int target;
    boolean runArm = true;
    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);
    @Override
    public void runOpMode() {

            // Initializing Hardware
            bobot = new Chassis(hardwareMap);
            wheels = new Motor[4];
            claw = new TwoPointServo(0.18, 0, 1, "claw", hardwareMap);
            clawWrist = new TwoPointServo(0.3, 0.6, 0.9, "clawWrist", hardwareMap);
            slides = new Slide(hardwareMap);
            slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));
            slideMotor.reverse();

            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target = 0;
            FeedforwardFun armFF = (position, velocity) -> {
                double distanceFromTop = (Math.abs(position - 650) / 100);
    //            Log.d("VEGAff", "v " + velocity + " p " + position + " factor " + distanceFromTop);
                if (velocity != null && position > 100) {
                    double ff = (position > 650 ? 1.0 : -1.0) * distanceFromTop * Math.abs(velocity / 1000) * f;
    //                Log.d("VEGAff", "positive case " + ff);
                    return ff;
                }
                else
                    return 0;
            };
            controller = new PIDFController(coefficients, armFF);

            wheels[0] = bobot.frontLeft;
            wheels[1] = bobot.frontRight;
            wheels[2] = bobot.backRight;
            wheels[3] = bobot.backLeft;


            wheels[0].setDirection(DcMotor.Direction.FORWARD);
            wheels[1].setDirection(DcMotor.Direction.FORWARD);
            wheels[2].setDirection(DcMotor.Direction.FORWARD); //these should not be all forward BUT DONT CHANGE
            wheels[3].setDirection(DcMotor.Direction.FORWARD); // BUT we already built everything around this butt strcture

            // Set up for each motor
            for (DcMotor wheel : wheels){
                wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            claw.positionA();
            sleep (50);
            clawWrist.positionA();
            double power;

            waitForStart();

            coefficients.setKP(p);
            coefficients.setKI(i);
            coefficients.setKD(d);
            f = normalF;



            PIDStrafe(-15, 1);
            PIDDrive(34, 2, 530, false);
            PIDDrive(-3, 1, 380, true);
    }

    public void PIDStrafe(double distance, double tolerance) { // TODO: Adjust Tolerance

//        double []newWheelTarget = new double[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
//            for (int i = 0; i < 4; i++) {
//                newWheelTarget[i] = wheels[i].getCurrentPosition()/WHEEL_COUNTS_PER_INCH + (distance); // TODO: Get avg position
//            }

            double kp;
            double kd; // Constant of derivation
            double ki;

            double kStatic;
            double kV;

            double a; // a can be anything from 0 < a < 1     0.55

            double previousFilterEstimate = 0;
            double currentFilterEstimate = 0;

            double referenceVelocity;

            double dt; //Delta time = 20 ms/cycle
            double dtS;

            double PID = 0;

            double power = 0;

            double P = 0;

            double previousError = 0;
            double error = tolerance + 1;

            double area = 0;
            double previousArea = 0;

            //1993.6
            double WHEEL_COUNTS_PER_INCH = (int) round((1 / (104 * PI)) * 537.7 * 25.4);

            kp = .75;
            kd = 0; // Constant of derivation
            ki = 0;

            kV = 0;
            kStatic = 0;

            referenceVelocity = 0;

            dt = 50; //Delta time = 20 ms/cycle
            dtS = dt/1000;

            error = tolerance + 1;

            while ((Math.abs(error) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

//
//                telemetry.addData("Friction FeedForward:", kStatic * Math.signum(PID));
//                telemetry.addData("Velocity FeedForward:", kV * (referenceVelocity * Math.signum(PID)));
//
//                telemetry.addData("Proportion:", P);
//                telemetry.addData("Derivative:", kd * ((error - previousError) / dtS));
//                telemetry.addData("Integral:", ki * area);
////
////                telemetry.addData("de(t)/dt", ((error - previousError) / dtS));
////
////                telemetry.addData("error:", error);
////                telemetry.addData("previous error:", previousError);
////
////                telemetry.addData("∫e(t)dt:", area);
////                telemetry.addData("previous ∫e(t)dt:", previousArea);
////
////                telemetry.addData("dtS", dtS);


                previousError = error;

                error = distance - wheels[0].getCurrentPosition()/WHEEL_COUNTS_PER_INCH;

                P = Math.abs(error)/distance;

                previousArea = area;

                if (error*(int)(distance) < 0) previousArea = 0;

                area = error * dtS + previousArea;

                PID = kp * P + kd * ((error - previousError) / dtS) + (ki * area);

//                power = PID + kStatic * Math.signum(PID) + kV * (referenceVelocity * Math.signum(PID));

                wheels[0].setPower(PID);
                wheels[1].setPower(PID);
                wheels[2].setPower(PID);
                wheels[3].setPower(PID);

                sleep((long) 50);
            }

            // Stop all motion;
            for (int i = 0; i < 4; i++){
                wheels[i].setPower(0);

                // Resets encoders
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void PIDDrive(double distance, double tolerance, int target, boolean open) { // TODO: Adjust Tolerance

//        double []newWheelTarget = new double[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            clip(target, open);
            // Determine new target position, and pass to motor controller
//            for (int i = 0; i < 4; i++) {
//                newWheelTarget[i] = wheels[i].getCurrentPosition()/WHEEL_COUNTS_PER_INCH + (distance); // TODO: Get avg position
//            }

            double kp;
            double kd; // Constant of derivation
            double ki;

            double kStatic;
            double kV;

            double a; // a can be anything from 0 < a < 1     0.55

            double previousFilterEstimate = 0;
            double currentFilterEstimate = 0;

            double referenceVelocity;

            double dt; //Delta time = 20 ms/cycle
            double dtS;

            double PID = 0;

            double power = 0;

            double P = 0;

            double previousError = 0;
            double error = tolerance + 1;

            double area = 0;
            double previousArea = 0;

            //1993.6
            double WHEEL_COUNTS_PER_INCH = (int) round((1 / (104 * PI)) * 537.7 * 25.4);

            kp = 0.7;
            kd = 0.0001; // Constant of derivation
            ki = 0.01;

            kV = 0;
            kStatic = 0;

            referenceVelocity = 0;

            dt = 50; //Delta time = 20 ms/cycle
            dtS = dt/1000;

            error = tolerance + 1;

            while ((Math.abs(error) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

//
//                telemetry.addData("Friction FeedForward:", kStatic * Math.signum(PID));
//                telemetry.addData("Velocity FeedForward:", kV * (referenceVelocity * Math.signum(PID)));
//
//                telemetry.addData("Proportion:", P);
//                telemetry.addData("Derivative:", kd * ((error - previousError) / dtS));
//                telemetry.addData("Integral:", ki * area);
////
////                telemetry.addData("de(t)/dt", ((error - previousError) / dtS));
////
////                telemetry.addData("error:", error);
////                telemetry.addData("previous error:", previousError);
////
////                telemetry.addData("∫e(t)dt:", area);
////                telemetry.addData("previous ∫e(t)dt:", previousArea);
////
////                telemetry.addData("dtS", dtS);


                previousError = error;

                error = distance - wheels[0].getCurrentPosition()/WHEEL_COUNTS_PER_INCH;

                P = Math.abs(error)/distance;

                previousArea = area;

                if (error*(int)(distance) < 0) previousArea = 0;

                area = error * dtS + previousArea;

                PID = kp * P + kd * ((error - previousError) / dtS) + (ki * area);

//                power = PID + kStatic * Math.signum(PID) + kV * (referenceVelocity * Math.signum(PID));
                clip(target, true);
                wheels[0].setPower(PID);
                wheels[1].setPower(-PID);
                wheels[2].setPower(PID);
                wheels[3].setPower(-PID);

                sleep((long) 150);
            }

            // Stop all motion;
            for (int i = 0; i < 4; i++){
                wheels[i].setPower(0);

                // Resets encoders
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

    }

    public void PIDTurn(double distance, double tolerance) { // TODO: Adjust Tolerance

//        double []newWheelTarget = new double[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
//            for (int i = 0; i < 4; i++) {
//                newWheelTarget[i] = wheels[i].getCurrentPosition()/WHEEL_COUNTS_PER_INCH + (distance); // TODO: Get avg position
//            }

            double kp;
            double kd; // Constant of derivation
            double ki;

            double kStatic;
            double kV;

            double a; // a can be anything from 0 < a < 1     0.55

            double previousFilterEstimate = 0;
            double currentFilterEstimate = 0;

            double referenceVelocity;

            double dt; //Delta time = 20 ms/cycle
            double dtS;

            double PID = 0;

            double power = 0;

            double P = 0;

            double previousError = 0;
            double error = tolerance + 1;

            double area = 0;
            double previousArea = 0;

            //1993.6
            double WHEEL_COUNTS_PER_INCH = (int) round((1 / (104 * PI)) * 537.7 * 25.4);

            kp = 0.8;
            kd = 0.001; // Constant of derivation
            ki = 0.05;

            kV = 0;
            kStatic = 0;

            referenceVelocity = 0;

            dt = 50; //Delta time = 20 ms/cycle
            dtS = dt/1000;

            error = tolerance + 1;

            while ((Math.abs(error) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

//
//                telemetry.addData("Friction FeedForward:", kStatic * Math.signum(PID));
//                telemetry.addData("Velocity FeedForward:", kV * (referenceVelocity * Math.signum(PID)));
//
//                telemetry.addData("Proportion:", P);
//                telemetry.addData("Derivative:", kd * ((error - previousError) / dtS));
//                telemetry.addData("Integral:", ki * area);
////
////                telemetry.addData("de(t)/dt", ((error - previousError) / dtS));
////
////                telemetry.addData("error:", error);
////                telemetry.addData("previous error:", previousError);
////
////                telemetry.addData("∫e(t)dt:", area);
////                telemetry.addData("previous ∫e(t)dt:", previousArea);
////
////                telemetry.addData("dtS", dtS);


                previousError = error;

                error = distance - wheels[0].getCurrentPosition()/WHEEL_COUNTS_PER_INCH;

                P = Math.abs(error)/distance;

                previousArea = area;

                if (error*(int)(distance) < 0) previousArea = 0;

                area = error * dtS + previousArea;

                PID = kp * P + kd * ((error - previousError) / dtS) + (ki * area);

                wheels[0].setPower(PID);
                wheels[1].setPower(PID);
                wheels[2].setPower(-PID);
                wheels[3].setPower(-PID);
//
//                wheels[0].setPower(Math.abs(error)/(int)distance);
//                wheels[1].setPower(Math.abs(error)/(int)distance);
//                wheels[2].setPower(-Math.abs(error)/(int)distance);
//                wheels[3].setPower(-Math.abs(error)/(int)distance);

                sleep((long) 50);
            }

            // Stop all motion;
            for (int i = 0; i < 4; i++){
                wheels[i].setPower(0);

                // Resets encoders
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void clip(int target, boolean open) {
        double power;

        while (Math.abs(slideMotor.getPosition() - target) > 40) {
            if (Math.abs(slideMotor.getPosition() - 250) < 50 && open) {
                claw.positionB();
            } else {
                claw.positionA();
            }
            coefficients.setKP(p);
            int armPos = slideMotor.getPosition();
            power = controller.update(System.nanoTime(), armPos, slideMotor.getVelocity());
            controller.setTargetPosition(target);
            slideMotor.setPower(power);

        }

        if (open) {
            claw.positionA();
        } else {
            claw.positionB();
        }

    }

}
