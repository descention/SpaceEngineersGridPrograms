using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using System.Collections.Generic;
using System;
using VRage.Game.ModAPI.Ingame;
using VRageMath;
using System.Linq;
using VRage.Game.ModAPI.Ingame.Utilities;

namespace IngameScript
{
    partial class Program
    {
        public abstract class Vessel
        {
            public const double PrecisionMaxAngularVel = 0.6; //Maximum Precision Ship Angular Velocity
            public const double RotationalSensitvity = 1; //Gain Applied To Gyros
            protected MyGridProgram _grid;
            protected IMyRemoteControl RC;
            protected IMyShipConnector CONNECTOR;
            protected IMySensorBlock SENSOR;
            protected IEnumerable<IMyGyro> GYRO;
            internal int COORD_ID = 0;
            internal List<Vector3D> DOCK_ROUTE = new List<Vector3D>();

            public Vector3D Position => RC.CubeGrid.GetPosition();

            public Vessel(MyGridProgram grid)
            {
                _grid = grid;
                InitVessel(_grid, this);

                PartValidation();

                _grid.Echo("Vessel initialization complete");
            }

            void PartValidation()
            {
                if (RC == null || RC.CubeGrid.GetCubeBlock(RC.Position) == null)
                    new Exception("No Remote Control Found,\nInstall Forward Facing Remote Control Block And Press Recompile");
                if (CONNECTOR == null || CONNECTOR.CubeGrid.GetCubeBlock(CONNECTOR.Position) == null)
                    new Exception("No Connector Found,\nInstall Connector And Press Recompile ");
                if (SENSOR == null || SENSOR.CubeGrid.GetCubeBlock(SENSOR.Position) == null)
                    new Exception("No Sensor Found,\nInstall Sensor For Asteroid Detection And Press Recompile,\n(all sensor settings will automatically be set)");
                if (GYRO == null || GYRO.Any(g=>g.CubeGrid.GetCubeBlock(g.Position) == null))
                    new Exception("Gyro Error,\nInstall gyro if missing and press Recompile");

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

                {
                    List<IMyRemoteControl> TEMP_RC = new List<IMyRemoteControl>();
                    GridTerminalSystem.GetBlocksOfType(TEMP_RC, b => b.CubeGrid == Me.CubeGrid);
                    if (!TEMP_RC.Any())
                        throw new Exception("No Remote Control Found. Install Forward Facing Remote Control Block And Press Recompile");
                    vessel.RC = TEMP_RC.First();
                }

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
                    vessel.GYRO = TEMP_GYRO;
                }
                catch { }

                //Runs Thruster Setup 
                try
                {
                    CollectAndFire2(new Vector3D(), 0, 0, RC.CubeGrid.GetPosition(), RC);
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
                public double PositiveMaxForce => PositiveThrusters.Sum(item => item.MaxEffectiveThrust);
                public double NegativeMaxForce => NegativeThrusters.Sum(item => item.MaxEffectiveThrust);
                public List<IMyThrust> PositiveThrusters = new List<IMyThrust>();
                public List<IMyThrust> NegativeThrusters = new List<IMyThrust>();
                public double VCF;
                public Thrust_info(Vector3D DIRECT, IMyGridTerminalSystem GTS, IMyCubeGrid MEGRID)
                {
                    GTS.GetBlocksOfType(PositiveThrusters, block => Vector3D.Dot(-1 * block.WorldMatrix.Forward, DIRECT) > 0.7 && block.CubeGrid == MEGRID);
                    GTS.GetBlocksOfType(NegativeThrusters, block => Vector3D.Dot(block.WorldMatrix.Forward, DIRECT) > 0.7 && block.CubeGrid == MEGRID);
                }
            }
            Thrust_info CAF2_FORWARD;
            Thrust_info CAF2_UP;
            Thrust_info CAF2_RIGHT;
            List<Thrust_info> CAFTHI = new List<Thrust_info>();

            List<IMyThrust> CAF2_THRUST = new List<IMyThrust>();
            bool C_A_F_HASRUN = false;
            double CAF2_BRAKING_COUNT = 99999999;

            void CollectAndFire2(Vector3D INPUT_POINT, double INPUT_VELOCITY, double INPUT_MAX_VELOCITY, Vector3D REFPOS, IMyRemoteControl RC)
            {
                var GridTerminalSystem = _grid.GridTerminalSystem;
                var Me = _grid.Me;
                var Echo = _grid.Echo;
                
                //Function Initialisation
                //-------------------------------------------------------------------- 
                if(CAF2_FORWARD == null)
                    CAF2_FORWARD = new Thrust_info(RC.WorldMatrix.Forward, GridTerminalSystem, Me.CubeGrid);

                if(CAF2_UP == null)
                    CAF2_UP = new Thrust_info(RC.WorldMatrix.Up, GridTerminalSystem, Me.CubeGrid);

                if(CAF2_RIGHT == null)
                    CAF2_RIGHT = new Thrust_info(RC.WorldMatrix.Right, GridTerminalSystem, Me.CubeGrid);

                if (C_A_F_HASRUN == false)
                {
                    //Initialise Classes And Basic System
                    CAFTHI = new List<Thrust_info>() { CAF2_FORWARD, CAF2_UP, CAF2_RIGHT };
                    GridTerminalSystem.GetBlocksOfType<IMyThrust>(CAF2_THRUST, block => block.CubeGrid == Me.CubeGrid);
                    C_A_F_HASRUN = true;

                    //Initialises Braking Component
                    foreach (var item in CAFTHI)
                    {
                        CAF2_BRAKING_COUNT = (item.PositiveMaxForce < CAF2_BRAKING_COUNT) ? item.PositiveMaxForce : CAF2_BRAKING_COUNT;
                        CAF2_BRAKING_COUNT = (item.NegativeMaxForce < CAF2_BRAKING_COUNT) ? item.NegativeMaxForce : CAF2_BRAKING_COUNT;
                    }
                }
                Echo("Running Thruster Control Program");

                //Generating Maths To Point and decelleration information etc.
                //-------------------------------------------------------------------- 
                double SHIPMASS = Convert.ToDouble(RC.CalculateShipMass().PhysicalMass);
                Vector3D INPUT_VECTOR = Vector3D.Normalize(INPUT_POINT - REFPOS);
                double VELOCITY = RC.GetShipSpeed();

                //Outputs distance to target
                double CAF_DIST_TO_TARGET = (REFPOS - INPUT_POINT).Length();

                //Outputs current decelleration
                double CAF_SHIP_DECELLERATION = 0.75 * (CAF2_BRAKING_COUNT / SHIPMASS);

                //Outputs current stopping distance
                double CAF_STOPPING_DIST = (((VELOCITY * VELOCITY) - (INPUT_VELOCITY * INPUT_VELOCITY))) / (2 * CAF_SHIP_DECELLERATION);

                //If Within Stopping Distance Halts Programme
                //--------------------------------------------
                if ((CAF_DIST_TO_TARGET <= (CAF_STOPPING_DIST + 0.25)) || CAF_DIST_TO_TARGET < 0.25 || VELOCITY > INPUT_MAX_VELOCITY)
                { foreach (var thruster in CAF2_THRUST) { thruster.ThrustOverride = 0; } return; }
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
                
                MULTIPLIER = Math.Min(F_COMP_MAX / F_COMP_PROJ , MULTIPLIER);
                MULTIPLIER = Math.Min(U_COMP_MAX / U_COMP_PROJ , MULTIPLIER);
                MULTIPLIER = Math.Min(R_COMP_MAX / R_COMP_PROJ , MULTIPLIER);

                //Calculate Multiplied Components
                //---------------------------------
                CAF2_FORWARD.VCF = ((F_COMP_PROJ * MULTIPLIER) / F_COMP_MAX) * Math.Sign(F_COMP_MAX);
                CAF2_UP.VCF = ((U_COMP_PROJ * MULTIPLIER) / U_COMP_MAX) * Math.Sign(U_COMP_MAX);
                CAF2_RIGHT.VCF = ((R_COMP_PROJ * MULTIPLIER) / R_COMP_MAX) * Math.Sign(R_COMP_MAX);

                //Runs System Thrust Application 
                //----------------------------------
                Dictionary<IMyThrust, float> THRUSTVALUES = new Dictionary<IMyThrust, float>();
                foreach (var thruster in CAF2_THRUST) { THRUSTVALUES.Add(thruster, 0f); }

                foreach (var THRUSTSYSTM in CAFTHI)
                {
                    List<IMyThrust> POSTHRUST = THRUSTSYSTM.PositiveThrusters;
                    List<IMyThrust> NEGTHRUST = THRUSTSYSTM.NegativeThrusters;
                    if (THRUSTSYSTM.VCF < 0) { POSTHRUST = THRUSTSYSTM.NegativeThrusters; NEGTHRUST = THRUSTSYSTM.PositiveThrusters; }
                    foreach (var thruster in POSTHRUST) { THRUSTVALUES[thruster] = (float)(Math.Abs(THRUSTSYSTM.VCF)) * (thruster as IMyThrust).MaxThrust; }
                    foreach (var thruster in NEGTHRUST) { THRUSTVALUES[thruster] = 1; }//(float)0.01001;}
                    foreach (var thruster in THRUSTVALUES) { thruster.Key.ThrustOverride = thruster.Value; } //thruster.Key.ThrustOverride = thruster.Value;
                }
            }

            internal void Run()
            {
                PartValidation();
            }

            protected void Vector_Thrust_Manager(Vector3D PM_START, Vector3D PM_TARGET, Vector3D PM_REF, double PR_MAX_VELOCITY, double PREC, IMyRemoteControl RC)
            {
                Vector3D VECTOR = Vector3D.Normalize(PM_START - PM_TARGET);
                Vector3D GOTOPOINT = PM_TARGET + VECTOR * MathHelper.Clamp((((PM_REF - PM_TARGET).Length() - 0.2)), 0, (PM_START - PM_TARGET).Length());
                double DIST_TO_POINT = MathHelper.Clamp((GOTOPOINT - PM_REF).Length(), 0, (PM_START - PM_TARGET).Length());

                // get max thrust per 6 degrees, take min as thrust
                var twr = CAFTHI.Min(t => Math.Min(t.NegativeMaxForce, t.PositiveMaxForce)) / (double)RC.CalculateShipMass().PhysicalMass;
                
                if (DIST_TO_POINT > PREC)
                {
                    _grid.Echo($"Goto Point: {twr}");
                    CollectAndFire2(GOTOPOINT, 0, twr, PM_REF, RC);
                }
                else
                {
                    _grid.Echo($"Forward from point: {twr}");
                    CollectAndFire2(PM_TARGET, 0, twr, PM_REF, RC);
                }
            }

            protected static void GyroTurn6(Vector3D TARGET, double GAIN, IEnumerable<IMyGyro> GYRO, IMyRemoteControl REF_RC, double ROLLANGLE, double MAXANGULARVELOCITY)
            {
                //Ensures Autopilot Not Functional
                REF_RC.SetAutoPilotEnabled(false);
                //Echo("Running Gyro Control Program");

                //Detect Forward, Up & Pos
                Vector3D ShipForward = REF_RC.WorldMatrix.Forward;
                Vector3D ShipUp = REF_RC.WorldMatrix.Up;
                Vector3D ShipPos = REF_RC.CubeGrid.GetPosition();

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
                foreach (var gyro in GYRO)
                {
                    var TRANS_VECT = Vector3.Transform(Vector, Matrix.Transpose(gyro.WorldMatrix.GetOrientation()));  //Converts To Gyro Local

                    //Applies To Scenario
                    gyro.Pitch = (float)MathHelper.Clamp((-TRANS_VECT.X * GAIN), -MAXANGULARVELOCITY, MAXANGULARVELOCITY);
                    gyro.Yaw = (float)MathHelper.Clamp(((-TRANS_VECT.Y) * GAIN), -MAXANGULARVELOCITY, MAXANGULARVELOCITY);
                    gyro.Roll = (float)MathHelper.Clamp(((-TRANS_VECT.Z) * GAIN), -MAXANGULARVELOCITY, MAXANGULARVELOCITY);
                    gyro.GyroOverride = true;
                }
                
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
                    foreach (var gyro in GYRO)
                        gyro.GyroOverride = false;
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
                    { Vector_Thrust_Manager(COORDINATES[COORD_ID - TargetID], COORDINATES[COORD_ID - CurrentID], CONNECTOR.GetPosition(), 5, 0.7, RC); }  //Thrusts to point
                }
                else if (COORD_ID == 0)//Last/First External Coord During Dock
                { //Standard Auto for first location
                    RC_Manager(COORDINATES[0], RC, false);
                }

                //Horizontal And Iterative Statement
                else
                {
                    var HEADING = Vector3D.Normalize(COORDINATES[COORD_ID - CurrentID] - COORDINATES[COORD_ID - TargetID]) * 9000000;
                    Vector_Thrust_Manager(COORDINATES[COORD_ID - TargetID], COORDINATES[COORD_ID - CurrentID], CONNECTOR.GetPosition(), 8, 1, RC); //Runs docking sequence 
                    GyroTurn6(HEADING, RotationalSensitvity, GYRO, RC, RollReqt, PrecisionMaxAngularVel);
                }

                //Logic checks and iterates
                if (Docking == false && COORD_ID == 0) { }
                else if ((CONNECTOR.GetPosition() - COORDINATES[COORD_ID - CurrentID]).Length() < 1 || ((RC.CubeGrid.GetPosition() - COORDINATES[COORD_ID - CurrentID]).Length() < 10 && COORD_ID == 0))
                {
                    COORD_ID = COORD_ID + iter_er;
                    if (COORD_ID == COORDINATES.Count)
                    { COORD_ID = COORDINATES.Count - 1; }
                    if (COORD_ID < 0)
                        COORD_ID = 0;
                }
            }

            Vector3D ReturnConnectorDirection(IMyShipConnector CONNECTOR, IMyRemoteControl RC)
            {
                if (CONNECTOR.Orientation.Forward == RC.Orientation.TransformDirection(Base6Directions.Direction.Down))
                { return RC.WorldMatrix.Left; }  //Connector is the bottom of ship
                if (CONNECTOR.Orientation.Forward == RC.Orientation.TransformDirection(Base6Directions.Direction.Up))
                { return RC.WorldMatrix.Right; }  //Connector is on the top of the ship
                if (CONNECTOR.Orientation.Forward == RC.Orientation.TransformDirection(Base6Directions.Direction.Right))
                { return RC.WorldMatrix.Up; }  //Connector is on the left of the ship
                if (CONNECTOR.Orientation.Forward == RC.Orientation.TransformDirection(Base6Directions.Direction.Left))
                { return RC.WorldMatrix.Down; }  //Connector is on the right of the ship
                return RC.WorldMatrix.Down;
            }

            protected void RC_Manager(Vector3D TARGET, IMyRemoteControl RC, bool TURN_ONLY = false)
            {
                //Uses Rotation Control To Handle Max Rotational Velocity
                //---------------------------------------------------------
                // _grid.Echo($"RotVel: {RC.GetShipVelocities().AngularVelocity.AbsMax()} / {PrecisionMaxAngularVel}");
                if (RC.GetShipVelocities().AngularVelocity.AbsMax() > PrecisionMaxAngularVel)
                {
                    _grid.Echo("Slowing Rotational Velocity");
                    if (RC.IsAutoPilotEnabled)
                    {
                        RC.SetAutoPilotEnabled(false);

                        // update gyro power to limit max rotation
                        foreach (var gyro in GYRO)
                            gyro.GyroPower -= 0.01f;
                    }
                    return;
                }

                //Setup Of Common Variables                         
                //--------------------------------------------
                Vector3D DronePosition = RC.CubeGrid.GetPosition();
                Vector3D Drone_To_Target = Vector3D.Normalize(TARGET - DronePosition);

                //Override Direction Detection
                //-------------------------------
                double To_Target_Angle = Vector3D.Dot(Vector3D.Normalize(RC.GetShipVelocities().LinearVelocity), Drone_To_Target);
                double Ship_Velocity = RC.GetShipVelocities().LinearVelocity.Length();
                List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
                RC.GetWaypointInfo(waypoints);
                //Turn Only: (Will drift ship automatically)
                //--------------------------------------------
                if (TURN_ONLY)
                {
                    _grid.Echo("Turning");
                    RC.ClearWaypoints();
                    RC.AddWaypoint(TARGET, "1");
                    RC.FlightMode = FlightMode.OneWay;
                    if (!RC.IsAutoPilotEnabled)
                        RC.SetAutoPilotEnabled(true);
                    RC.ApplyAction("CollisionAvoidance_Off");
                    RC.ControlThrusters = false;
                    return;
                }

                //Drift Cancellation Enabled:
                //-----------------------------
                else if (To_Target_Angle < 0.4 && Ship_Velocity > 3)
                {
                    _grid.Echo("Drift Cancellation Enabled");

                    //Aim Gyro To Reflected Vector
                    Vector3D DRIFT_VECTOR = Vector3D.Normalize(RC.GetShipVelocities().LinearVelocity);
                    Vector3D REFLECTED_DRIFT_VECTOR = -1 * (Vector3D.Normalize(Vector3D.Reflect(DRIFT_VECTOR, Drone_To_Target)));
                    Vector3D AIMPINGPOS = (-1 * DRIFT_VECTOR * 500) + DronePosition;

                    //Sets Autopilot To Turn
                    RC.ClearWaypoints();
                    RC.AddWaypoint(AIMPINGPOS, "1");
                    RC.FlightMode = FlightMode.OneWay;
                    if (!RC.IsAutoPilotEnabled)
                        RC.SetAutoPilotEnabled(true);
                    RC.ApplyAction("CollisionAvoidance_Off");

                }

                //System Standard Operation:
                //---------------------------
                else
                {
                    _grid.Echo("Drift Cancellation Disabled");

                    //Assign To RC, Clear And Refresh Command
                    
                    if(!waypoints.Any() || RC.CurrentWaypoint.Coords != TARGET)
                    {
                        RC.ClearWaypoints();
                        RC.AddWaypoint(TARGET, "1");
                    }
                    
                    RC.ControlThrusters = true;
                    RC.FlightMode = FlightMode.OneWay;

                    if (!RC.IsAutoPilotEnabled)
                        RC.SetAutoPilotEnabled(true);
                    RC.ApplyAction("DockingMode_Off");              //Precision Mode
                    RC.ApplyAction("CollisionAvoidance_On");        //Col avoidance

                }
            }

            public string ToIni()
            {
                var ini = new MyIni();
                ini.Set("Vessel", "RC", $"{RC.Position}");

                return ini.ToString();
            }
        }
    }
}