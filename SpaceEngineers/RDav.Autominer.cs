using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;

using System;
using System.Collections.Generic;

using System.Linq;
using System.Text;

using VRageMath;
using Sandbox.Common;
using Sandbox.Engine;
using Sandbox.Game;
using VRage.Game.ModAPI.Ingame;
using System.Xml.Serialization;

using VRage.Game;
using Sandbox.Game.EntityComponents;
using VRage.Game.Components;
using VRage.Collections;
using VRage.Game.ObjectBuilders.Definitions;
using SpaceEngineers.Game.ModAPI.Ingame;


namespace SpaceEngineers.RDav.AutoMiner
{
    class Program:MyGridProgram
    {
        #region Copy/paste this to program block
        MiningVessel miner;

        public Program()
        {
            //Sets Update Frequency
            Runtime.UpdateFrequency = UpdateFrequency.Update10;

            try
            {
                if (!string.IsNullOrWhiteSpace(Storage))
                {
                    // load data from storage
                    var serializer = new XmlSerializer(typeof(Vessel));
                    var sb = new StringBuilder();
                    sb.Append(Storage);
                    System.IO.Stream stream = 
                    serializer.Deserialize(xmlWriter);
                    
                }
                else
                {
                    miner = new MiningVessel(this);
                }

                if (miner.ISNOTBURIED)
                    miner.Run();
            }
            catch (Exception ex)
            {
                Echo(ex.Message);
            }
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (miner != null)
            {
                try
                {
                    miner.Run();
                }
                catch (Exception ex)
                {
                    Echo(ex.Message);
                }
            }
            else
            { // somehow we don't have a miner

            }
        }

        void SaveCode()
        {
            var serializer = new XmlSerializer(typeof(Vessel));    
            var sb = new StringBuilder();
            var xmlWriter = System.Xml.XmlWriter.Create(sb);
            serializer.Serialize(xmlWriter, miner);
            Storage = sb.ToString();
        }

        abstract class Vessel
        {
            const double PrecisionMaxAngularVel = 0.6; //Maximum Precision Ship Angular Velocity
            const double RotationalSensitvity = 1; //Gain Applied To Gyros
            protected MyGridProgram _grid;
            protected IMyRemoteControl RC;
            protected IMyShipConnector CONNECTOR;
            protected IMySensorBlock SENSOR;
            protected IMyGyro GYRO;

            public Vessel(MyGridProgram grid)
            {
                _grid = grid;
                InitVessel(_grid, this);

                if (RC == null || RC.CubeGrid.GetCubeBlock(RC.Position) == null)
                    new Exception("No Remote Control Found,\nInstall Forward Facing Remote Control Block And Press Recompile");
                if (CONNECTOR == null || CONNECTOR.CubeGrid.GetCubeBlock(CONNECTOR.Position) == null)
                    new Exception("No Connector Found,\nInstall Connector And Press Recompile ");
                if (SENSOR == null || SENSOR.CubeGrid.GetCubeBlock(SENSOR.Position) == null)
                    new Exception("No Sensor Found,\nInstall Sensor For Asteroid Detection And Press Recompile,\n(all sensor settings will automatically be set)");
                if (GYRO == null || GYRO.CubeGrid.GetCubeBlock(GYRO.Position) == null)
                    new Exception("No Gyro Found,\nInstall Gyro And Press Recompile");

                if (CAF2_THRUST.Count > 15)
                {
                    new Exception("Large Amount Of Thrusters Detected\nProgram Terminated To Prevent Performance Issues\n" +
                      "Remove Unecessary Thrusters And Press Recompile (15 max)\n" +
                      "This safety measure can be disabled on line 56");
                }
            }

            private void InitVessel(MyGridProgram grid, Vessel vessel)
            {
                var Me = grid.Me;
                var GridTerminalSystem = grid.GridTerminalSystem;
                //Gathers Remote Control
                try
                {
                    List<IMyRemoteControl> TEMP_RC = new List<IMyRemoteControl>();
                    GridTerminalSystem.GetBlocksOfType(TEMP_RC, b => b.CubeGrid == Me.CubeGrid);
                    vessel.RC = TEMP_RC[0];
                }
                catch { }

                //GathersConnector  
                try
                {
                    List<IMyShipConnector> TEMP_CON = new List<IMyShipConnector>();
                    GridTerminalSystem.GetBlocksOfType(TEMP_CON, b => b.CubeGrid == Me.CubeGrid && b.CustomName.Contains("Ejector") == false);
                    vessel.CONNECTOR = TEMP_CON[0];
                }
                catch { }

                //Sets Sensor
                try
                {
                    List<IMySensorBlock> TEMP_SENSOR = new List<IMySensorBlock>();
                    GridTerminalSystem.GetBlocksOfType(TEMP_SENSOR, b => b.CubeGrid == Me.CubeGrid);
                    vessel.SENSOR = TEMP_SENSOR[0];
                }
                catch { }

                //Sets Gyro
                try
                {
                    List<IMyGyro> TEMP_GYRO = new List<IMyGyro>();
                    GridTerminalSystem.GetBlocksOfType(TEMP_GYRO, b => b.CubeGrid == Me.CubeGrid);
                    vessel.GYRO = TEMP_GYRO[0];
                }
                catch { }

                //Runs Thruster Setup 
                try
                {
                    CollectAndFire2(new Vector3D(), 0, 0, RC.GetPosition(), RC);
                    for (int j = 0; j < CAF2_THRUST.Count; j++)
                    { CAF2_THRUST[j].SetValue<float>("Override", 0.0f); CAF2_THRUST[j].ApplyAction("OnOff_On"); }
                }
                catch { }
            }

            static double Vector_Projection(Vector3D IN, Vector3D Axis)
            {
                double OUT = 0;
                OUT = Vector3D.Dot(IN, Axis) / IN.Length();
                if (OUT + "" == "NaN")
                { OUT = 0; }
                return OUT;
            }

            public bool Docked
            {
                get
                {
                    return CONNECTOR.Status == MyShipConnectorStatus.Connected;
                }
            }

            class Thrust_info                   //Basic Information For Axial Thrust
            {
                public double PositiveMaxForce;
                public double NegativeMaxForce;
                public List<IMyThrust> PositiveThrusters;
                public List<IMyThrust> NegativeThrusters;
                public double VCF;
                public Thrust_info(Vector3D DIRECT, IMyGridTerminalSystem GTS, IMyCubeGrid MEGRID)
                {
                    PositiveThrusters = new List<IMyThrust>(); NegativeThrusters = new List<IMyThrust>();
                    List<IMyTerminalBlock> TEMP_RC = new List<IMyTerminalBlock>();
                    GTS.GetBlocksOfType<IMyThrust>(PositiveThrusters, block => Vector3D.Dot(-1 * block.WorldMatrix.Forward, DIRECT) > 0.7 && block.CubeGrid == MEGRID);
                    GTS.GetBlocksOfType<IMyThrust>(NegativeThrusters, block => Vector3D.Dot(block.WorldMatrix.Forward, DIRECT) > 0.7 && block.CubeGrid == MEGRID);
                    double POWER_COUNT = 0;
                    foreach (var item in PositiveThrusters)
                    { POWER_COUNT = POWER_COUNT + item.MaxEffectiveThrust; }
                    PositiveMaxForce = POWER_COUNT;
                    POWER_COUNT = 0;
                    foreach (var item in NegativeThrusters)
                    { POWER_COUNT = POWER_COUNT + item.MaxEffectiveThrust; }
                    NegativeMaxForce = POWER_COUNT;
                }
            }
            Thrust_info CAF2_FORWARD;
            Thrust_info CAF2_UP;
            Thrust_info CAF2_RIGHT;
            List<Thrust_info> CAFTHI = new List<Thrust_info>();

            List<IMyTerminalBlock> CAF2_THRUST = new List<IMyTerminalBlock>();
            bool C_A_F_HASRUN = false;
            double CAF2_BRAKING_COUNT = 99999999;

            double CAF_SHIP_DECELLERATION;                        //Outputs current decelleration
            double CAF_STOPPING_DIST;                             //Outputs current stopping distance
            double CAF_DIST_TO_TARGET;                            //Outputs distance to target

            void CollectAndFire2(Vector3D INPUT_POINT, double INPUT_VELOCITY, double INPUT_MAX_VELOCITY, Vector3D REFPOS, IMyRemoteControl RC)
            {
                //Function Initialisation
                //-------------------------------------------------------------------- 
                if (C_A_F_HASRUN == false)
                {
                    //Initialise Classes And Basic System
                    CAF2_FORWARD = new Thrust_info(RC.WorldMatrix.Forward, _grid.GridTerminalSystem, _grid.Me.CubeGrid);
                    CAF2_UP = new Thrust_info(RC.WorldMatrix.Up, _grid.GridTerminalSystem, _grid.Me.CubeGrid);
                    CAF2_RIGHT = new Thrust_info(RC.WorldMatrix.Right, _grid.GridTerminalSystem, _grid.Me.CubeGrid);
                    CAFTHI = new List<Thrust_info>() { CAF2_FORWARD, CAF2_UP, CAF2_RIGHT };
                    _grid.GridTerminalSystem.GetBlocksOfType<IMyThrust>(CAF2_THRUST, block => block.CubeGrid == _grid.Me.CubeGrid);
                    C_A_F_HASRUN = true;

                    //Initialises Braking Component
                    foreach (var item in CAFTHI)
                    {
                        CAF2_BRAKING_COUNT = (item.PositiveMaxForce < CAF2_BRAKING_COUNT) ? item.PositiveMaxForce : CAF2_BRAKING_COUNT;
                        CAF2_BRAKING_COUNT = (item.NegativeMaxForce < CAF2_BRAKING_COUNT) ? item.PositiveMaxForce : CAF2_BRAKING_COUNT;
                    }
                }
                _grid.Echo("Running Thruster Control Program");

                //Generating Maths To Point and decelleration information etc.
                //-------------------------------------------------------------------- 
                double SHIPMASS = Convert.ToDouble(RC.CalculateShipMass().PhysicalMass);
                Vector3D INPUT_VECTOR = Vector3D.Normalize(INPUT_POINT - REFPOS);
                double VELOCITY = RC.GetShipSpeed();
                CAF_DIST_TO_TARGET = (REFPOS - INPUT_POINT).Length();
                CAF_SHIP_DECELLERATION = 0.75 * (CAF2_BRAKING_COUNT / SHIPMASS);
                CAF_STOPPING_DIST = (((VELOCITY * VELOCITY) - (INPUT_VELOCITY * INPUT_VELOCITY))) / (2 * CAF_SHIP_DECELLERATION);

                //If Within Stopping Distance Halts Programme
                //--------------------------------------------
                if (!(CAF_DIST_TO_TARGET > (CAF_STOPPING_DIST + 0.25)) || CAF_DIST_TO_TARGET < 0.25 || VELOCITY > INPUT_MAX_VELOCITY)
                { foreach (var thruster in CAF2_THRUST) { (thruster as IMyThrust).ThrustOverride = 0; } return; }
                //dev notes, this is the most major source of discontinuity between theorised system response

                //Reflects Vector To Cancel Orbiting
                //------------------------------------
                Vector3D DRIFT_VECTOR = Vector3D.Normalize(RC.GetShipVelocities().LinearVelocity + RC.WorldMatrix.Forward * 0.00001);
                Vector3D R_DRIFT_VECTOR = -1 * Vector3D.Normalize(Vector3D.Reflect(DRIFT_VECTOR, INPUT_VECTOR));
                R_DRIFT_VECTOR = ((Vector3D.Dot(R_DRIFT_VECTOR, INPUT_VECTOR) < -0.3)) ? 0 * R_DRIFT_VECTOR : R_DRIFT_VECTOR;
                INPUT_VECTOR = Vector3D.Normalize((4 * R_DRIFT_VECTOR) + INPUT_VECTOR);

                //Components Of Input Vector In FUR Axis
                //----------------------------------------
                double F_COMP_IN = Vector_Projection(INPUT_VECTOR, RC.WorldMatrix.Forward);
                double U_COMP_IN = Vector_Projection(INPUT_VECTOR, RC.WorldMatrix.Up);
                double R_COMP_IN = Vector_Projection(INPUT_VECTOR, RC.WorldMatrix.Right);

                //Calculate MAX Allowable in Each Axis & Length
                //-----------------------------------------------
                double F_COMP_MAX = (F_COMP_IN > 0) ? CAF2_FORWARD.PositiveMaxForce : -1 * CAF2_FORWARD.NegativeMaxForce;
                double U_COMP_MAX = (U_COMP_IN > 0) ? CAF2_UP.PositiveMaxForce : -1 * CAF2_UP.NegativeMaxForce;
                double R_COMP_MAX = (R_COMP_IN > 0) ? CAF2_RIGHT.PositiveMaxForce : -1 * CAF2_RIGHT.NegativeMaxForce;
                double MAX_FORCE = Math.Sqrt(F_COMP_MAX * F_COMP_MAX + U_COMP_MAX * U_COMP_MAX + R_COMP_MAX * R_COMP_MAX);

                //Apply Length to Input Components and Calculates Smallest Multiplier
                //--------------------------------------------------------------------
                double F_COMP_PROJ = F_COMP_IN * MAX_FORCE;
                double U_COMP_PROJ = U_COMP_IN * MAX_FORCE;
                double R_COMP_PROJ = R_COMP_IN * MAX_FORCE;
                double MULTIPLIER = 1;
                MULTIPLIER = (F_COMP_MAX / F_COMP_PROJ < MULTIPLIER) ? F_COMP_MAX / F_COMP_PROJ : MULTIPLIER;
                MULTIPLIER = (U_COMP_MAX / U_COMP_PROJ < MULTIPLIER) ? U_COMP_MAX / U_COMP_PROJ : MULTIPLIER;
                MULTIPLIER = (R_COMP_MAX / R_COMP_PROJ < MULTIPLIER) ? R_COMP_MAX / R_COMP_PROJ : MULTIPLIER;

                //Calculate Multiplied Components
                //---------------------------------
                CAF2_FORWARD.VCF = ((F_COMP_PROJ * MULTIPLIER) / F_COMP_MAX) * Math.Sign(F_COMP_MAX);
                CAF2_UP.VCF = ((U_COMP_PROJ * MULTIPLIER) / U_COMP_MAX) * Math.Sign(U_COMP_MAX);
                CAF2_RIGHT.VCF = ((R_COMP_PROJ * MULTIPLIER) / R_COMP_MAX) * Math.Sign(R_COMP_MAX);

                //Runs System Thrust Application 
                //----------------------------------
                Dictionary<IMyThrust, float> THRUSTVALUES = new Dictionary<IMyThrust, float>();
                foreach (var thruster in CAF2_THRUST) { THRUSTVALUES.Add((thruster as IMyThrust), 0f); }

                foreach (var THRUSTSYSTM in CAFTHI)
                {
                    List<IMyThrust> POSTHRUST = THRUSTSYSTM.PositiveThrusters;
                    List<IMyThrust> NEGTHRUST = THRUSTSYSTM.NegativeThrusters;
                    if (THRUSTSYSTM.VCF < 0) { POSTHRUST = THRUSTSYSTM.NegativeThrusters; NEGTHRUST = THRUSTSYSTM.PositiveThrusters; }
                    foreach (var thruster in POSTHRUST) { THRUSTVALUES[thruster as IMyThrust] = (float)(Math.Abs(THRUSTSYSTM.VCF)) * (thruster as IMyThrust).MaxThrust; }
                    foreach (var thruster in NEGTHRUST) { THRUSTVALUES[thruster as IMyThrust] = 1; }//(float)0.01001;}
                    foreach (var thruster in THRUSTVALUES) { thruster.Key.ThrustOverride = thruster.Value; } //thruster.Key.ThrustOverride = thruster.Value;
                }
            }

            protected void Vector_Thrust_Manager(Vector3D PM_START, Vector3D PM_TARGET, Vector3D PM_REF, double PR_MAX_VELOCITY, double PREC, IMyRemoteControl RC)
            {
                Vector3D VECTOR = Vector3D.Normalize(PM_START - PM_TARGET);
                Vector3D GOTOPOINT = PM_TARGET + VECTOR * MathHelper.Clamp((((PM_REF - PM_TARGET).Length() - 0.2)), 0, (PM_START - PM_TARGET).Length());
                double DIST_TO_POINT = MathHelper.Clamp((GOTOPOINT - PM_REF).Length(), 0, (PM_START - PM_TARGET).Length());

                if (DIST_TO_POINT > PREC)
                { CollectAndFire2(GOTOPOINT, 0, PR_MAX_VELOCITY * 2, PM_REF, RC); }
                else
                { CollectAndFire2(PM_TARGET, 0, PR_MAX_VELOCITY, PM_REF, RC); }
            }

            protected static void GyroTurn6(Vector3D TARGET, double GAIN, IMyGyro GYRO, IMyRemoteControl REF_RC, double ROLLANGLE, double MAXANGULARVELOCITY)
            {
                //Ensures Autopilot Not Functional
                REF_RC.SetAutoPilotEnabled(false);
                //Echo("Running Gyro Control Program");

                //Detect Forward, Up & Pos
                Vector3D ShipForward = REF_RC.WorldMatrix.Forward;
                Vector3D ShipUp = REF_RC.WorldMatrix.Up;
                Vector3D ShipPos = REF_RC.GetPosition();

                //Create And Use Inverse Quatinion                   
                Quaternion Quat_Two = Quaternion.CreateFromForwardUp(ShipForward, ShipUp);
                var InvQuat = Quaternion.Inverse(Quat_Two);
                Vector3D DirectionVector = Vector3D.Normalize(TARGET - ShipPos); //RealWorld Target Vector
                Vector3D RCReferenceFrameVector = Vector3D.Transform(DirectionVector, InvQuat); //Target Vector In Terms Of RC Block

                //Convert To Local Azimuth And Elevation
                double ShipForwardAzimuth = 0; double ShipForwardElevation = 0;
                Vector3D.GetAzimuthAndElevation(RCReferenceFrameVector, out ShipForwardAzimuth, out ShipForwardElevation);

                //Does Some Rotations To Provide For any Gyro-Orientation
                var RC_Matrix = REF_RC.WorldMatrix.GetOrientation();
                var Vector = Vector3.Transform((new Vector3D(ShipForwardElevation, ShipForwardAzimuth, ROLLANGLE)), RC_Matrix); //Converts To World
                var TRANS_VECT = Vector3.Transform(Vector, Matrix.Transpose(GYRO.WorldMatrix.GetOrientation()));  //Converts To Gyro Local

                //Applies To Scenario
                GYRO.Pitch = (float)MathHelper.Clamp((-TRANS_VECT.X * GAIN), -MAXANGULARVELOCITY, MAXANGULARVELOCITY);
                GYRO.Yaw = (float)MathHelper.Clamp(((-TRANS_VECT.Y) * GAIN), -MAXANGULARVELOCITY, MAXANGULARVELOCITY);
                GYRO.Roll = (float)MathHelper.Clamp(((-TRANS_VECT.Z) * GAIN), -MAXANGULARVELOCITY, MAXANGULARVELOCITY);
                GYRO.GyroOverride = true;

                //GYRO.SetValueFloat("Pitch", (float)((TRANS_VECT.X) * GAIN));     
                //GYRO.SetValueFloat("Yaw", (float)((-TRANS_VECT.Y) * GAIN));
                //GYRO.SetValueFloat("Roll", (float)((-TRANS_VECT.Z) * GAIN));
            }

            protected void DockingIterator(bool Docking, List<Vector3D> COORDINATES)
            {
                //Logic Check To Check Coords Are Within Limits
                if (COORDINATES.Count < 3) { return; }

                //Changes Increment Based on Dock/Undock Requirement
                int TargetID = 0;
                int CurrentID = 0;
                int iter_er = 0;
                if (Docking == true)
                { TargetID = 1; CurrentID = 0; iter_er = +1; }
                if (Docking == false)
                { TargetID = 0; CurrentID = 1; iter_er = -1; }

                //Toggles State Of Thrusters Connectors And Gyros On The Ship
                if (Docking == true) { CONNECTOR.Connect(); }
                if (Docking == true && CONNECTOR.IsWorking == false) { CONNECTOR.Enabled = true; }
                if (Docking == false && CONNECTOR.IsWorking == true) { CONNECTOR.Disconnect(); CONNECTOR.Enabled = true; }
                if (Docked && Docking == true)
                {
                    //for (int j = 0; j < CAF2_THRUST.Count; j++)
                    //{(CAF2_THRUST[j] as IMyThrust).Enabled = false; }
                    GYRO.GyroOverride = false;
                    return;
                }

                //Setting Up a Few Constants
                Vector3D RollOrienter = Vector3D.Normalize(COORDINATES[COORDINATES.Count - 1] - COORDINATES[COORDINATES.Count - 2]);
                Vector3D Connector_Direction = -1 * ReturnConnectorDirection(CONNECTOR, RC);
                double RollReqt = (float)(0.6 * (Vector3D.Dot(RollOrienter, Connector_Direction)));

                //Vertical Motion During Dock
                if (COORD_ID == COORDINATES.Count - 1)
                {
                    Vector3D DockingHeading = Vector3D.Normalize(COORDINATES[COORDINATES.Count - 3] - COORDINATES[COORDINATES.Count - 2]) * 9000000; //Heading
                    GyroTurn6(DockingHeading, RotationalSensitvity, GYRO, RC, RollReqt, PrecisionMaxAngularVel); //Turn to heading
                    if (Vector3D.Dot(RC.WorldMatrix.Forward, Vector3D.Normalize(DockingHeading)) > 0.98) //Error check for small rotational velocity
                    { Vector_Thrust_Manager(COORDINATES[COORD_ID.Val - TargetID], COORDINATES[COORD_ID.Val - CurrentID], CONNECTOR.GetPosition(), 5, 0.7, RC); }  //Thrusts to point
                }
                else if (COORD_ID.Val == 0)//Last/First External Coord During Dock
                { //Standard Auto for first location
                    RC_Manager(COORDINATES[0], RC, false);
                }  

                //Horizontal And Iterative Statement
                else
                {
                    var HEADING = Vector3D.Normalize(COORDINATES[COORD_ID.Val - CurrentID] - COORDINATES[COORD_ID.Val - TargetID]) * 9000000;
                    Vector_Thrust_Manager(COORDINATES[COORD_ID.Val - TargetID], COORDINATES[COORD_ID.Val - CurrentID], CONNECTOR.GetPosition(), 8, 1, RC); //Runs docking sequence 
                    GyroTurn6(HEADING, RotationalSensitvity, GYRO, RC, RollReqt, PrecisionMaxAngularVel);
                }

                //Logic checks and iterates
                if (Docking == false && COORD_ID.Val == 0) { }
                else if ((CONNECTOR.GetPosition() - COORDINATES[COORD_ID.Val - CurrentID]).Length() < 1 || ((RC.GetPosition() - COORDINATES[COORD_ID.Val - CurrentID]).Length() < 10 && COORD_ID.Val == 0))
                {
                    COORD_ID.Val = COORD_ID.Val + iter_er;
                    if (COORD_ID.Val == COORDINATES.Count)
                    { COORD_ID.Val = COORDINATES.Count - 1; }
                    if (COORD_ID.Val < 0)
                        COORD_ID.Val = 0;
                }
            }
        }

        class MiningVessel:Vessel
        {
            List<IMyLargeTurretBase> DIRECTORS = new List<IMyLargeTurretBase>();
            IMyRadioAntenna RADIO;
            List<IMyShipController> CONTROLLERS = new List<IMyShipController>();
            List<IMyCargoContainer> Cargo = new List<IMyCargoContainer>();
            List<IMyUserControllableGun> DIRECTIONAL_FIRE = new List<IMyUserControllableGun>();  //Directional ship weaponry
            List<IMyShipDrill> SHIP_DRILLS = new List<IMyShipDrill>();     //List Of all the ships drills

            
            
            public Asteroid TargetAsteroid { get; set; }
            public bool ISNOTBURIED { get; set; }
            public string MiningStatus { get; internal set; }
            public bool FinishedBore { get; internal set; }

            public MiningVessel(MyGridProgram grid):base(grid)
            {
                InitMiner(grid, this);
                TargetAsteroid = null;
            }

            private double SHIPSIZE => Math.Sqrt(SHIP_DRILLS.Count) * 0.9 * (_grid.Me.CubeGrid.ToString().Contains("Large") ? 1.5 : 1.0);

            private static void InitMiner(MyGridProgram grid, MiningVessel miner)
            {
                var GridTerminalSystem = grid.GridTerminalSystem;
                var Me = grid.Me;

                //Initialising Dedicated Cargo
                try
                {
                    GridTerminalSystem.GetBlocksOfType(miner.Cargo, b => b.CubeGrid == Me.CubeGrid);
                }
                catch
                { }

                //Gathers Antennae
                try
                {
                    List<IMyRadioAntenna> TEMP = new List<IMyRadioAntenna>();
                    GridTerminalSystem.GetBlocksOfType(TEMP, b => b.CubeGrid == Me.CubeGrid);
                    miner.RADIO = TEMP[0] as IMyRadioAntenna;
                    miner.RADIO.SetValue("PBList", Me.EntityId);
                    miner.RADIO.EnableBroadcasting = true;
                    miner.RADIO.Enabled = true;
                }
                catch { }

                //GathersControllers   
                try
                {
                    GridTerminalSystem.GetBlocksOfType(miner.CONTROLLERS, b => b.CubeGrid == Me.CubeGrid);
                }
                catch { }

                //Gathers Director Turret
                try
                {
                    GridTerminalSystem.GetBlocksOfType(miner.DIRECTORS, b => b.CubeGrid == Me.CubeGrid);
                }
                catch { }

                //Gathers Drills
                try
                {
                    GridTerminalSystem.GetBlocksOfType(miner.SHIP_DRILLS, b => b.CubeGrid == Me.CubeGrid);
                }
                catch { }

                //Gathers Directional Weaponry
                try
                {
                    GridTerminalSystem.GetBlocksOfType(miner.DIRECTIONAL_FIRE,
                        (block => block.GetType().Name == "MySmallMissileLauncher" || block.GetType().Name == "MySmallGatlingGun"
                            || block.GetType().Name == "MySmallMissileLauncherReload")); //Collects the directional weaponry (in a group)
                }
                catch { }

                //Creates User Interface
                try
                {
                    
                    Me.CustomData = "Paste Asteroid GPS Here: \n===========================\n" + 
                        (miner.TargetAsteroid != null?$"@GPS:Mine:{miner.TargetAsteroid.Location.X}:{miner.TargetAsteroid.Location.Y}:{miner.TargetAsteroid.Location.Z}@\n":"@paste gps string here@\n") +
                        "\nInstructions:\n===========================\nPaste GPS coords of point NEAR asteroid\nbetween the symbols." +
                        "\nDo NOT include the 'at' symbol in the GPS name\nErrors Will be displayed in the terminal\n\n" +
                        "Hints/Tips:\n===========================\n" +
                        "-Look in the terminal for live mining progress\n" +
                        "-Paste GPS near the ore you want collected for faster mining\n" +
                        "-NEVER paste a GPS from inside an asteroid \n" +
                        "-Rename any connectors you don't want the code\n to use as docking points as 'Ejector' \n" +
                        "-The code uses the GPS as a starting point so for\n larger ships keep this GPS further away from the asteroid\n" +
                        "-The miner will remember the orientation you dock with \n so remember to dock in the direction you want to launch\n" +
                        "-You can reassign docking coordinates at any point\n by manually overriding and docking somewhere else \n" +
                        "-To cancel a task and return miner to base \n run the PB with the argument 'FIN'";
                }
                catch
                { }
            }

            public void BoreMine(Asteroid asteroid, bool reset = false)
            {
                //Setup Of Common Variables                         
                Vector3D DronePosition = RC.GetPosition();
                Vector3D Drone_To_Target = Vector3D.Normalize(asteroid.Centre - DronePosition);

                //Generates XYZ Vectors
                Vector3D X_ADD = Vector3D.Normalize(asteroid.Centre - asteroid.Location);//Characteristic 'Forward' vector 
                Vector3D Y_ADD = Vector3D.CalculatePerpendicularVector(X_ADD); //Characteristic 'Left' vector
                Vector3D Z_ADD = Vector3D.Cross(X_ADD, Y_ADD); //Characteristic 'Up' vector

                //Generates Array Of Starting Vectors
                int Steps = MathHelper.Clamp((int)((asteroid.Diameter * 0.3) / SHIPSIZE), 1, 16); //How many horizontal passes of the ship are required to eat the roid
                //double StepSize = SHIPSIZE;  //How big are those passes
                Vector3D[,] GridCoords = new Vector3D[Steps + 1, Steps + 1]; //i as asteroid.Row, j as asteroid.Column
                for (int i = 0; i < (Steps + 1); i++)
                {
                    for (int j = 0; j < (Steps + 1); j++)
                    {
                        Vector3D Ipos = (Math.Pow(-1, i) == -1) ? asteroid.Location + SHIPSIZE * (i - 1) * -1 * Z_ADD : asteroid.Location + SHIPSIZE * i * Z_ADD;
                        Vector3D Jpos = (Math.Pow(-1, j) == -1) ? Ipos + SHIPSIZE * (j - 1) * -1 * Y_ADD : Ipos + SHIPSIZE * j * Y_ADD;
                        GridCoords[i, j] = Jpos;
                    }
                }

                //Readouts
                //Echo("Has Finished Tunnel: " + asteroid.Finished);
                //Echo(SHIPSIZE + " 'Step' Size");
                //Echo(asteroid.Row + " /" + Steps + " Rows");
                //Echo(asteroid.Column + " /" + Steps + " Columns"); 
                //Echo((CurrentVectorEnd - DronePosition).Length() + " dist to iter++");

                //Generates Currently Targeted Vector As A Function Of 2 integers, asteroid.Row and Depth 
                Vector3D CurrentVectorStart = GridCoords[asteroid.Row, asteroid.Column]; //Start Vector
                Vector3D CurrentVectorEnd = CurrentVectorStart + X_ADD * (((asteroid.Centre - asteroid.Location).Length() - asteroid.Diameter / 2) + asteroid.Diameter * 0.8); //Accounts for small input

                //Sets IsBuried And Has Finished
                ISNOTBURIED = (CurrentVectorStart - RC.GetPosition()).Length() < 4; //If Retracted Allows Switching Of Case
                if ((CurrentVectorEnd - DronePosition).Length() < 1)
                    FinishedBore = true; //If Reached End Toggle Finished

                //Inputs To Autopilot Function
                double RollReqt = (float)(0.6 * (Vector3D.Dot(Z_ADD, RC.WorldMatrix.Down)));
                GyroTurn6(X_ADD * 999999999999999999, RotationalSensitvity, GYRO, RC, RollReqt, PrecisionMaxAngularVel);

                if (FinishedBore) //Reverses Once Finished
                {
                    Vector_Thrust_Manager(CurrentVectorEnd, CurrentVectorStart, RC.GetPosition(), 2, 0.5, RC);
                }
                else //else standard forward
                {
                    Vector_Thrust_Manager(CurrentVectorStart, CurrentVectorEnd, RC.GetPosition(), 1, 0.5, RC);
                }

                //Iterates Based On Proximity
                if ((CurrentVectorStart - DronePosition).Length() < 1 && asteroid.Row == Steps && asteroid.Column == Steps && FinishedBore)
                {
                    MiningStatus = "FIN";
                    return;
                }

                if ((CurrentVectorStart - DronePosition).Length() < 1 && asteroid.Row == Steps && FinishedBore)
                {
                    asteroid.Column++;
                    asteroid.Row = 1;
                    FinishedBore = false;
                }

                if ((CurrentVectorStart - DronePosition).Length() < 1 && FinishedBore)
                {
                    asteroid.Row++;
                    FinishedBore = false;
                }

            }

            internal void Run()
            {
                if(ISNOTBURIED)
                { // update target asteroid
                    string[] InputData = Me.CustomData.Split('@');
                    string[] InputGpsList = InputData[1].Split(':');
                    Vector3D TryVector = new Vector3D(double.Parse(InputGpsList[2]), double.Parse(InputGpsList[3]), double.Parse(InputGpsList[4]));

                    if (TargetAsteroid == null || TryVector != TargetAsteroid.Location)
                    {
                        TargetAsteroid = new Asteroid(TryVector);
                        MiningLogic(DOCKLIST);
                        Echo("Updated Input Correctly");
                    }
                }

                //Sets If Full Or Not
                bool IsEmpty = Cargo.All(cargo => cargo.GetInventory(0).CurrentMass <= 900) && SHIP_DRILLS[0].GetInventory().CurrentMass < 100;
                bool IsMassAboveThreshold = (double)SHIP_DRILLS[0].GetInventory().CurrentVolume > (double)SHIP_DRILLS[0].GetInventory().MaxVolume * 0.80;

                //if (IsMassAboveThreshold)
                //    Echo("Drill Inventory Is Currently Full");
                //if (IsEmpty)
                //    Echo("Drill Inventory Is Currently Empty");

                if (IsEmpty && MiningStatus != "FIN")
                    MiningStatus = "MINE";
                else if (IsMassAboveThreshold && MiningStatus != "FIN")
                {
                    MiningStatus = "FULL";
                    FinishedBore = true;
                }
            }

            void MiningLogic(List<Vector3D> DOCK_ROUTE)
            {
                //Echo("Mining Logic:\n--------------");

                //Sets Drills:
                if (SHIP_DRILLS[0].IsWorking == false && MiningStatus == "MINE")
                {
                    for (int j = 0; j < SHIP_DRILLS.Count; j++)
                        SHIP_DRILLS[j].Enabled = true;
                }
                else if (SHIP_DRILLS[0].IsWorking == true && MiningStatus != "MINE")
                {
                    for (int j = 0; j < SHIP_DRILLS.Count; j++)
                        SHIP_DRILLS[j].Enabled = false;
                }
                
                //If Empty And Docked And Want To Go Mine, Undock (stage 2)
                if (COORD_ID.Val != 0 && MiningStatus == "MINE")
                {
                    DockingIterator(false, DOCK_ROUTE, GYRO, CONNECTOR, RC);
                    //Echo("Status: Undocking");
                    return;
                }

                //If Fin, then Stop operations (stage 4)
                if ((MiningStatus == "FIN" || MiningStatus == "FULL") && ISNOTBURIED)
                {
                    DockingIterator(true, DOCK_ROUTE, GYRO, CONNECTOR, RC);

                    //if(MiningStatus == "FIN")
                    //  Echo("Status: Finished Task, Returning To Drop Off");
                    //if(MiningStatus == "FULL")
                    //  Echo("Status: Docking To Offload");
                    return;
                }

                //Always Calls Mine Function If Undocked And Not Full
                if (COORD_ID.Val == 0) //If Undocked
                {

                    //Uses Sensor To Update Information On Asteroid Within 250m of Selected Asteroid (prevents Close Proximity Asteroid Failure)
                    if (SENSOR.IsActive && (TargetAsteroid.Location - RC.GetPosition()).Length() < 50 && TargetAsteroid.Centre == TargetAsteroid.Location)
                        TargetAsteroid.UpdateInfo(SENSOR.LastDetectedEntity.BoundingBox);

                    //If No Asteroid Detected Goes To Location To Detect Asteroid
                    if (TargetAsteroid.Diameter == 0)
                    {
                        RC_Manager(TargetAsteroid.Location, RC, false);
                        return; //No Need For Remainder Of Logic
                    }
                    
                    //Goes To Location And Mines
                    if (TargetAsteroid.AbleToMineFrom(RC.GetPosition()) == false)
                    {
                        RC_Manager(TargetAsteroid.Location, RC, false);
                        ISNOTBURIED = true;
                    }
                    else
                    {
                        if (TargetAsteroid.Diameter > SHIPSIZE) //Only if drill size is bigger than ship 9prevents array handlign issues
                            BoreMine(TargetAsteroid, false);
                        //else
                        //    Echo("No Asteroid Detected, Drill array too large");
                    }

                    //Echo("Status: Mining");
                    //Echo(SHIP_DRILLS[0].GetInventory().MaxVolume + " Inventory Count"); // - SHIP_DRILLS[0].GetInventory().CurrentVolume + 
                }

            }
        }
        
        public class Asteroid
        {
            public double Diameter { get; set; }
            public int Row { get; set; }
            public int Column { get; set; }
            public Vector3D Centre { get; set; }
            public Vector3D Location { get; set; }

            public Asteroid(Vector3D location)
            {
                Centre = Location = location;
                Row = 1;
                Column = 1;
                Diameter = 0.0;
            }

            public void UpdateInfo(BoundingBoxD box)
            {
                Diameter = box.Size.Length();
                Centre = box.Center;
            }

            public bool AbleToMineFrom(Vector3D vesselPosition)
            {
                //Toggles Should-be-Mining Based On Proximity
                double Dist_To_Mine_Start = (Location - vesselPosition).Length();
                double Dist_To_Mine_Centre = (Centre - vesselPosition).Length();
                if (Dist_To_Mine_Start < 4) //Toggles Mining Mode On
                    return true;
                else if (Dist_To_Mine_Centre > Diameter + 40) //Toggles Mining Mode Off
                    return false;
                else
                    return false;
            }
        }
        #endregion

    }

    
}
