package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
@Configurable
@Config
@TeleOp
public class multithreadteleop extends LinearOpMode {
    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    public ServoImplEx transfermover;
    private DcMotorEx spindexer;
    private CRServoImplEx transfer;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private DcMotorEx rb;
    private ControlSystem cs;
    public static double transfermoveridle = 0.6;
    public static double transfermoverscore = 0.73;
    public static double transfermoverfull = 1;
    public static int movespindexer = 2731;
    public static double p=0.0039,i=0,d=0.0000005;
    public static double v=0.000372,a=0.7,s=0.0000005;
    ControlSystem cs1;
    public static double p1=0.0009,i1=0,d1=0;
    private static int targetpos;
    private CRServo turretL;
    private CRServo turretR;
    private Servo hood;
    public static double targetTicksPerSecond=200;
    public static double shootclose = 1250;
    public static double shootfar=1600;
    public static double shooteridle = 200;
    public static double hoodtop = 0;
    public static double hoodbottom = 0.1;
    public ElapsedTime x;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        x= new ElapsedTime();
        x.reset();
        turretL = hardwareMap.crservo.get("turretL");
        turretR = hardwareMap.crservo.get("turretR");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.get(DcMotorEx.class,"rb");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel = hardwareMap.get(DcMotorEx.class,"shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        transfer=hardwareMap.get(CRServoImplEx.class, "transfer");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfermover=hardwareMap.get(ServoImplEx.class,"transfermover");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfermover.setPosition(transfermoveridle);
        int target = 0;
        Thread g1 = new Thread(() ->{
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            lf.setPower(frontLeftPower);
            lb.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rb.setPower(backRightPower);

            turretL.setPower(gamepad2.right_stick_x);
            turretR.setPower(gamepad2.right_stick_x);
            if(gamepad2.right_trigger>0 && !gamepad2.right_bumper){
                transfermover.setPosition(transfermoverscore);
                transfer.setPower(gamepad2.right_trigger);
            }
            if(gamepad2.right_trigger>0 && gamepad2.right_bumper){
                transfermover.setPosition(transfermoverfull);
                transfer.setPower(gamepad2.right_trigger);
            }
            if(gamepad2.right_trigger==0){
                transfermover.setPosition(transfermoveridle);
                transfer.setPower(0);
            }

            if(gamepad2.y){
                targetTicksPerSecond=shootfar;
            }
            if(gamepad2.b){
                targetTicksPerSecond=shootclose;
            }
            if(gamepad2.a){
                targetTicksPerSecond=shooteridle;
            }
        });
        Thread g2 = new Thread(()->{
            if(gamepad2.left_bumper){
                moveSpindexer(movespindexer,1);
                sleep(300);
            } else if(gamepad2.left_stick_y!=0){
                spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                spindexer.setPower(-gamepad2.left_stick_y);
            } else{
                spindexer.setPower(0);
            }
        });
        waitForStart();
        g1.start();
        g2.start();
        while (opModeIsActive()){



            KineticState current2 = new KineticState(spindexer.getCurrentPosition(),spindexer.getVelocity());
            cs1 = ControlSystem.builder()
                    .posPid(p1,i1,d1)
                    .build();
            cs1.setGoal(new KineticState(target));
            if(gamepad2.a){
                target+=movespindexer;
                sleep(300);
            }
            if(Math.abs(gamepad2.left_stick_y)==0) {
                spindexer.setPower(cs1.calculate(current2));
            } else{
                spindexer.setPower(-gamepad2.left_stick_y);
            }

            cs =  ControlSystem.builder()
                    .velPid(p, i, d)
                    .basicFF(v,a,s)
                    .build();
            cs.setGoal(new KineticState(0,targetTicksPerSecond));
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            KineticState current1 = new KineticState(flywheel.getCurrentPosition(), flywheel.getVelocity());
            flywheel.setPower(cs.calculate(current1));
            //flywheel.setPower(-gamepad2.right_stick_y);
            telemetry.addData("motor current speed",Math.abs(flywheel.getVelocity()));
            telemetry.addData("target",targetTicksPerSecond);
            telemetry.addData("output of shooter",cs.calculate(current1));
            telemetry.update();
        }

        try {
            g1.join();
            g2.join();
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread Interrupted");
        }
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
    public void moveSpindexer(int pos, double speed){
        spindexer.setTargetPosition((spindexer.getCurrentPosition()+pos));
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(speed);
    }
}
