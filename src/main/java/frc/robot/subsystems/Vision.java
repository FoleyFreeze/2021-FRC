package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.cals.VisionCals;
import frc.robot.util.LimitedList;
import frc.robot.util.Vector;

public class Vision extends SubsystemBase{
    public class VisionData{
        public VisionData(int size){
            location = new Vector[size];
        }
        public Vector[] location;
        public double timestamp;
        public double robotangle;

        public String toString(){
            return String.format("T:%.1f, d:%.1f, \u03B8:%.0f", timestamp, location[0].r, Math.toDegrees(location[0].theta));
        }
    }

    public VisionCals cals;
    public double lastFrameTime;
    public double lastFrameId;
    public LimitedList<VisionData> targetData;
    public LimitedList<VisionData> ballData;

    RobotContainer m_subsystem;

    NetworkTable piTable;
    NetworkTable visionTable;

    public Vision(VisionCals cals, RobotContainer subsystem){
        targetData = new LimitedList<>(cals.historySize);
        ballData = new LimitedList<>(cals.historySize);
        addNTListener();
        this.cals = cals;
        m_subsystem = subsystem;
        NTEnablePiCones(cals.coneMode);
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
            return Timer.getFPGATimestamp() - vd.timestamp < cals.maxImageTime  && Math.abs(vd.location[0].theta) < Math.toRadians(45);
        }
        return false;
    }

    private void addNTListener(){
        NetworkTableInstance.getDefault().setUpdateRate(.03);
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        piTable = NetworkTableInstance.getDefault().getTable("Pi");

        visionTable.addEntryListener("Target", (table, key, entry, value, flags) -> {
            try{
                VisionData vd = new VisionData(1);
                String data = value.getString();
                String[] parts = data.split(",");
                Pose2d botPos = m_subsystem.m_drivetrain.drivePos;

                double dist = Double.parseDouble(parts[1]);
                double angle = Double.parseDouble(parts[2]);
                Vector v = new Vector(dist, Math.toRadians(angle));
                vd.location[0] = v;
                vd.robotangle = m_subsystem.m_drivetrain.robotAng;

                vd.timestamp = Timer.getFPGATimestamp();
                double dt = vd.timestamp - lastFrameTime;
                //SmartDashboard.putNumber("Image dt", dt);
                Display.put("Image DT", dt);
                lastFrameTime = vd.timestamp;
                double id = Double.parseDouble(parts[0]);
                double dFrame = id - lastFrameId;
                SmartDashboard.putNumber("Image dFrame", dFrame);
                lastFrameId = id;

                targetData.addFirst(vd);

                Display.put("Last Target", vd.toString());
            } catch(Exception e){
                e.printStackTrace();
            }

        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
        
        visionTable.addEntryListener("Ball", (table, key, entry, value, flags) -> {
            try{
                VisionData vd = new VisionData(3);
                String data = value.getString();
                String[] parts = data.split(",");

                double dist = Double.parseDouble(parts[1]);
                double angle = Double.parseDouble(parts[2]);
                Vector v = new Vector(dist, Math.toRadians(angle));
                vd.location[0] = v;

                dist = Double.parseDouble(parts[3]);
                angle = Double.parseDouble(parts[4]);
                v = new Vector(dist, Math.toRadians(angle));
                vd.location[1] = v;

                dist = Double.parseDouble(parts[5]);
                angle = Double.parseDouble(parts[6]);
                v = new Vector(dist, Math.toRadians(angle));
                vd.location[2] = v;
                
                vd.robotangle = m_subsystem.m_drivetrain.robotAng;

                vd.timestamp = Timer.getFPGATimestamp();
                double dt = vd.timestamp - lastFrameTime;
                //SmartDashboard.putNumber("Image dt", dt);
                Display.put("Image DT", dt);
                lastFrameTime = vd.timestamp;
                double id = Double.parseDouble(parts[0]);
                double dFrame = id - lastFrameId;
                SmartDashboard.putNumber("Image dFrame", dFrame);
                lastFrameId = id;

                ballData.addFirst(vd);

                Display.put("Last Ball", vd.toString());
            } catch(Exception e){
                e.printStackTrace();
            }

        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    }

    @Override
    public void periodic(){
        piTable.getEntry("Timer").setDouble(Timer.getFPGATimestamp());
    }

    public void NTEnablePiTgt(boolean set){
        piTable.getEntry("Tgt Enable").setBoolean(set);
    }

    public void NTEnablePiBall(boolean set){
        piTable.getEntry("Ball Enable").setBoolean(set);
    }

    public void NTEnablePiCones(boolean set){
        piTable.getEntry("Cone Enable").setBoolean(set);
    }
}