using Sandbox.ModAPI.Ingame;
using System.Collections.Generic;
using System;
using VRageMath;
using System.Linq;
using Sandbox.ModAPI.Interfaces;

namespace IngameScript
{
    partial class Program
    {
        class MiningVessel : Vessel
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

            public MiningVessel(MyGridProgram grid) : base(grid)
            {
                InitMiner(grid, this);
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
                    if (string.IsNullOrWhiteSpace(Me.CustomData))
                        Me.CustomData = "Paste Asteroid GPS Here: \n===========================\n" +
                            (miner.TargetAsteroid != null ? $"@GPS:Mine:{miner.TargetAsteroid.Location.X}:{miner.TargetAsteroid.Location.Y}:{miner.TargetAsteroid.Location.Z}@\n" : "@paste gps string here@\n") +
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
                if (ISNOTBURIED)
                { // update target asteroid
                    try
                    {
                        string[] InputData = _grid.Me.CustomData.Split('@');
                        string[] InputGpsList = InputData[1].Split(':');
                        double X;
                        double.TryParse(InputGpsList[2], out X);

                        double Y;
                        double.TryParse(InputGpsList[3], out Y);

                        double Z;
                        double.TryParse(InputGpsList[4], out Z);

                        Vector3D TryVector = new Vector3D(X, Y, Z);

                        if (TargetAsteroid == null || TryVector != TargetAsteroid.Location)
                        {
                            TargetAsteroid = new Asteroid(TryVector);
                            //MiningLogic(DOCKLIST);
                            MiningLogic();
                            _grid.Echo("Updated Input Correctly");
                            return;
                        }
                    }
                    catch (Exception ex)
                    {
                        _grid.Echo(ex.Message);
                        _grid.Echo(ex.StackTrace);
                        if (ex.InnerException != null)
                        {
                            _grid.Echo(ex.InnerException.Message);
                            _grid.Echo(ex.InnerException.StackTrace);
                        }
                    }
                }

                //Sets If Full Or Not
                bool IsEmpty = Cargo.All(cargo => cargo.GetInventory(0).CurrentMass <= 900) && SHIP_DRILLS.All(drill=>drill.GetInventory().CurrentMass < 100);
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

                //Updates A Sensor If Ship Has One
                //---------------------------------------------
                if (SENSOR != null && SENSOR.DetectAsteroids == false)
                {
                    SENSOR.DetectAsteroids = true;
                    SENSOR.DetectPlayers = false;
                    SENSOR.DetectOwner = false;
                    SENSOR.DetectLargeShips = false;
                    SENSOR.DetectSmallShips = false;
                    SENSOR.LeftExtend = 50;
                    SENSOR.RightExtend = 50;
                    SENSOR.TopExtend = 50;
                    SENSOR.FrontExtend = 50;
                    SENSOR.BottomExtend = 50;
                    SENSOR.BackExtend = 50;
                }

                //Runs Primary Mnining Logic (only if not 0,0,0)
                //-------------------------------------------------
                _grid.Echo("CurrentRoid: " + Vector3D.Round(TargetAsteroid.Location));
                _grid.Echo("CurrentCentre: " + Vector3D.Round(TargetAsteroid.Centre));
                _grid.Echo("CurrentRoidSize: " + Math.Round(TargetAsteroid.Diameter) + " Metres");

                //Echo("Is Vanilla RC A: " + Math.Round(TargetAsteroid.Diameter) + " Metres");
                _grid.Echo("Outside Asteroid? " + ISNOTBURIED);
                //Echo("Runtime: " + Math.Round(Runtime.LastRunTimeMs, 3) + " Ms");

                MiningLogic();
            }

            void MiningLogic()
            {
                //Echo("Mining Logic:\n--------------");

                //Sets Drills:
                if (SHIP_DRILLS[0].IsWorking == false && MiningStatus == "MINE" && TargetAsteroid.AbleToMineFrom(_grid.Me.GetPosition()))
                {
                    foreach(var drill in SHIP_DRILLS)
                        drill.Enabled = true;
                }
                else
                {
                    foreach (var drill in SHIP_DRILLS)
                        drill.Enabled = false;
                }

                //If Empty And Docked And Want To Go Mine, Undock (stage 2)
                if (COORD_ID != 0 && MiningStatus == "MINE")
                {
                    DockingIterator(false, DOCK_ROUTE);
                    _grid.Echo("Status: Undocking");
                    return;
                }

                //If Fin, then Stop operations (stage 4)
                if ((MiningStatus == "FIN" || MiningStatus == "FULL") && ISNOTBURIED)
                {
                    DockingIterator(true, DOCK_ROUTE);

                    if(MiningStatus == "FIN")
                        _grid.Echo("Status: Finished Task, Returning To Drop Off");
                    if(MiningStatus == "FULL")
                        _grid.Echo("Status: Docking To Offload");
                    return;
                }

                //Always Calls Mine Function If Undocked And Not Full
                if (!Docked) //If Undocked
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
                        else
                            _grid.Echo("No Asteroid Detected, Drill array too large");
                    }

                    _grid.Echo("Status: Mining");
                    _grid.Echo(SHIP_DRILLS[0].GetInventory().MaxVolume + " Inventory Count"); // - SHIP_DRILLS[0].GetInventory().CurrentVolume + 
                }

            }
        }
    }
}