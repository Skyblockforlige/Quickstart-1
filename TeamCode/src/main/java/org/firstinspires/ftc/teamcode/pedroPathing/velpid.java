package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.FeedbackElement;


@Config
@Configurable
@TeleOp(name="velpid")
public class velpid extends OpMode {
    public PIDFController controller;
    public static double p=0,i=0,d=0;
    public static double v=0,a=0,s=0;
    private ControlSystem cs;
    private DcMotorEx flywheel;
    public static double targetTicksPerSecond=300;
    @Override
    public void init(){
         flywheel = hardwareMap.get(DcMotorEx.class,"m");
         flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         flywheel.setDirection(DcMotor.Direction.REVERSE);
         flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cs =  ControlSystem.builder()
                .velPid(p, i, d)
                //.basicFF(v,a,s)
                .build();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
    @Override
    public void loop(){

        cs.setGoal(new KineticState(0,targetTicksPerSecond));
        KineticState current = new KineticState(flywheel.getCurrentPosition(), Math.abs(flywheel.getVelocity()));
        double output = cs.calculate(current);
        double output2= cs.calculate();
        flywheel.setPower(output);
        telemetry.addData("motor current speed",Math.abs(flywheel.getVelocity()));
        telemetry.addData("target",targetTicksPerSecond);
        telemetry.addData("output",output);
        telemetry.addData("output2",output2);
        telemetry.update();

    }
    @Override
    public void stop(){

    }
}
