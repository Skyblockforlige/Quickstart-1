package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Sorting Test")
public class sortingsystem extends LinearOpMode {
    private Limelight3A limelight;
    public static int pattern;
    @Override
    public void runOpMode(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(1);
        limelight.pipelineSwitch(3);
        limelight.start();
        while(opModeInInit()){
            pattern = limelight.getLatestResult().getFiducialResults().get(0).getFiducialId();
        }
        //21 is GPP 22 is PGP 23 is PPG
        waitForStart();
        while (opModeIsActive()) {
            switch (pattern){
                case 21:
                    telemetry.addData("Motif: ", "GPP");
                case 22:
                    telemetry.addData("Motif: ", "PGP");
                case 23:
                    telemetry.addData("Motif: ", "PPG");
                default:
                    telemetry.addData("Motif: ", "No data found");
            }
            telemetry.update();
        }
    }
}
