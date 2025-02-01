package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.round;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

@Autonomous
public class GoToAPlace extends LinearOpMode {
    TwoPointServo claw;
    TwoPointServo clawWrist;
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
    private final double ticks_in_degree = round(1993.6 / 360);
    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);

    @Override
    public void runOpMode() {
        claw = new TwoPointServo(0.18, 0, 1, "claw", hardwareMap);
        clawWrist = new TwoPointServo(0.3, 0.6, 0.9, "clawWrist", hardwareMap);
        claw.positionA();
        clawWrist.positionB();
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
        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        Action traj = robot.actionBuilder(robot.localizer.getPose())
                .splineTo(new Vector2d(30, -40), -Math.PI)
                .build();
        Action firstGrab = robot.actionBuilder(robot.localizer.getPose())
                        .strafeTo(new Vector2d(10.0, 60.0))

                        .build();
        waitForStart();
        coefficients.setKI(i);
        coefficients.setKD(d);
        int armPos = slideMotor.getPosition();
        double power = controller.update(System.nanoTime(), armPos, slideMotor.getVelocity());
        slideMotor.setPower(power);
        f = normalF;
        target = (350);
        coefficients.setKP(p);
        controller.setTargetPosition(target);
        Actions.runBlocking(firstGrab

        );
    }
}
