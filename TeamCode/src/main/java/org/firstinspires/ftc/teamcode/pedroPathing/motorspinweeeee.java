package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Configurable
@Config
@TeleOp

public class motorspinweeeee extends LinearOpMode {
    private DcMotorImplEx m;
    public static double p=0.01;
    public static double i=0.01;
    public static double d=0.01;
    public static double f=0;
    public static double vel = 1000;
    @Override
    public void runOpMode(){
        m =  hardwareMap.get(DcMotorImplEx.class,"m");
        //m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            m.setVelocityPIDFCoefficients(p,i,d,f);
            m.setVelocity(vel, AngleUnit.DEGREES);
            m.setPower(1);
        }
    }
}
