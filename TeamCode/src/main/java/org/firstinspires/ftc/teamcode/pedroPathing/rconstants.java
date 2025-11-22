package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;

public class rconstants {

    public static double transfermoveridle = 0.6;

    public static Limelight3A limelight;
    public static double transfermoverscore = 0.73;
    public static double transfermoverfull = 1;
    public static int movespindexer = 2731;

    public static Servo hood;
    public static IMU imu;
    public static DcMotor lf;
    public static DcMotor lb;
    public static DcMotor rf;
    public static ServoImplEx transfermover;
    public static DcMotorEx spindexer;
    public static CRServoImplEx transfer;
    public static DcMotorEx flywheel;
    public static DcMotorEx intake;
    public static DcMotorEx rb;
    public static double targetTicksPerSecond=200;
    public static double shootclose = 1250;
    public static double shootfar=1600;
    public static double shooteridle = 200;
    public static double hoodtop = 0;
    public static double hoodbottom = 0.1;
    public static CRServo turretL;
    public static CRServo turretR;


    public static void initHardware(HardwareMap hardwareMap){

        turretL = hardwareMap.crservo.get("turretL");
        turretR = hardwareMap.crservo.get("turretR");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        hood=hardwareMap.servo.get("hood");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

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
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }

}
