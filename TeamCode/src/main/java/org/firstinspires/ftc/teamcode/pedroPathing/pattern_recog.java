package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class pattern_recog extends OpMode{
    private Limelight3A limelight3a;

    @Override
    public void init() {
        limelight3a= hardwareMap.get(Limelight3A.class,"ll");
        limelight3a.pipelineSwitch(0);

    }

    @Override
    public void loop() {
        limelight3a.start();
        LLResult llresult = limelight3a.getLatestResult();
        llresult.getColorResults();
        telemetry.addData("happened?","null");
        if(llresult!=null && llresult.isValid())
        {
            telemetry.addData("happened","yes");

            telemetry.addData("amount",llresult.getColorResults().size());






        }

    }
}
