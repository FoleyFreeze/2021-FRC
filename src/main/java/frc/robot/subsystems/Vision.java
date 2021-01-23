package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.cals.VisionCals;
import frc.robot.util.LimitedList;

public class Vision extends SubsystemBase{
    public class VisionData{
        public double dist;
        public double angle;
        public double timestamp;
        public double robotangle;

        public String toString(){
            return String.format("T:%.1f, d:%.1f, \u03B8:%.0f", timestamp, dist, angle);
        }
    }

    public VisionCals cals;
    public double lastFrameTime;
    public int lastFrameId;
    public LimitedList<VisionData> targetData;
    public LimitedList<VisionData> ballData;

    public Vision(VisionCals cals){
        targetData = new LimitedList<>(cals.historySize);
        ballData = new LimitedList<>(cals.historySize);
        addNTListener();
        this.cals = cals;
    }

    public boolean hasTargetImage(){
        if(targetData.size() > 0){
            VisionData vd = targetData.getFirst();
            return Timer.getFPGATimestamp() - vd.timestamp < cals.maxImageTime;
        }
        return false;
    }

    public boolean hasBallImage(){
        if(ballData.size() > 0){
            VisionData vd = ballData.getFirst();
            return Timer.getFPGATimestamp() - vd.timestamp < cals.maxImageTime;
        }
        return false;
    }

    private void addNTListener(){
        //NetworkTableInstance.getDefault().setUpdateRate(0.01);
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Vision");

        nt.addEntryListener("Target", (table, key, entry, value, flags) -> {
            try{
                VisionData vd = new VisionData();
                String data = value.getString();
                String[] parts = data.split(",");

                vd.dist = Double.parseDouble(parts[1]);
                vd.angle = Double.parseDouble(parts[2]);

                vd.timestamp = Timer.getFPGATimestamp();
                double dt = vd.timestamp - lastFrameTime;
                //SmartDashboard.putNumber("Image dt", dt);
                Display.put("Image DT", dt);
                lastFrameTime = vd.timestamp;
                int id = Integer.parseInt(parts[0]);
                int dFrame = id - lastFrameId;
                SmartDashboard.putNumber("Image dFrame", dFrame);
                lastFrameId = id;

                targetData.addFirst(vd);

                Display.put("Last Target", vd.toString());
            } catch(Exception e){
                e.printStackTrace();
            }

        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        
        nt.addEntryListener("Ball", (table, key, entry, value, flags) -> {
            try{
                VisionData vd = new VisionData();
                String data = value.getString();
                String[] parts = data.split(",");

                vd.dist = Double.parseDouble(parts[1]);
                vd.angle = Double.parseDouble(parts[2]);

                vd.timestamp = Timer.getFPGATimestamp();
                double dt = vd.timestamp - lastFrameTime;
                //SmartDashboard.putNumber("Image dt", dt);
                Display.put("Image DT", dt);
                lastFrameTime = vd.timestamp;
                int id = Integer.parseInt(parts[0]);
                int dFrame = id - lastFrameId;
                SmartDashboard.putNumber("Image dFrame", dFrame);
                lastFrameId = id;

                ballData.addFirst(vd);

                Display.put("Last Ball", vd.toString());
            } catch(Exception e){
                e.printStackTrace();
            }

        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public void NTEnablePiTgt(boolean set){
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Pi");
        nt.getInstance().getEntry("Tgt Enable").setBoolean(set);
    }

    public void NTEnablePiBall(boolean set){
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Pi");
        nt.getInstance().getEntry("Ball Enable").setBoolean(set);
    }
}