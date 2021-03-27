package frc.robot.cals;

public class ElectroKendro extends CalSet{

    //fly sky buttons and axes
    public final int FS_FIELDORIENT = 5;
    public final int FS_ANGRESET = 10;
    public final int FS_AUTOTRENCH = 4;
    public final int FS_EnblBallCam = 1;
    public final int FS_DRIVESTRAIGHT = 6;

    public final int FS_XAXIS = 0;
    public final int FS_YAXIS = 1;
    public final int FS_ROTAXIS = 4;
    public final int FS_AUTOSHOOT = 3;
    public final int FS_AUTOGATHER = 2;

    public double FS_LOWDEADBND = 0.05;
    public final double FS_HIGHDEADBND = 1.0;
    public final double FS_EXPONENT = 1.4;



    //xbox buttons and axes
    public final int XB_REVOLVE = 1;
    public final int XB_SHOOT = 2;
    public final int XB_FIELDORIENT = 3;
    public final int XB_ANGRESET = 5;
    public final int XB_AUTOTRENCH = 4;
    public final int XB_DRIVESTRAIGHT = -1;

    public final int XB_XAXIS = 0;
    public final int XB_YAXIS = 1;
    public final int XB_ROTAXIS = 4;
    public final int XB_AUTOGATHER = 2;

    public final double XB_LOWDEADBND = 0.1;
    public final double XB_HIGHDEADBND = 0.9;
    public final double XB_EXPONENT = 1.0;



    //driver station buttons and switches
    public final boolean DS_ENABLED = false;
    public final int DS_CLIMBUP = 2;
    public final int DS_CLIMBDN = 2;
    public final int DS_ENABLEBUDCLIMB = 9;
    public final int DS_DROPFOOT = 8;
    public final int DS_MINTAKE = 3;
    public final int DS_REVOLVE = 1;
    public final int DS_MSHOOT = 2;
    public final int DS_SHIFT = 5;
    public final int DS_CAMMODE = 1;
    public final int DS_LAYUP = 1;
    public final int DS_PITMODE = 4;
    public final int DS_JOGUP = 0;
    public final int DS_JOGDN = 180;
    public final int DS_JOGR = 90;
    public final int DS_JOGL = 270;
    public final int DS_TWOVTHREE = 6;
    public final int DS_CWACTIVATE = 7;
    public final int DS_CWROTPOS = 10;

    public ElectroKendro(){
        switch(type){
            case PRACTICE:
                FS_LOWDEADBND = 0.1;
            break;
            default:

            break;
        }
    }
}