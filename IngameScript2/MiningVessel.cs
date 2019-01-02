using Sandbox.ModAPI.Ingame;
using System.Collections.Generic;
using System;
using VRageMath;
using System.Linq;
using Sandbox.ModAPI.Interfaces;
using VRage.Game.ModAPI.Ingame.Utilities;

namespace IngameScript
{
    partial class Program
    {
        public class MiningVessel : Vessel
        {
            List<IMyLargeTurretBase> DIRECTORS = new List<IMyLargeTurretBase>();
            IMyRadioAntenna RADIO;
            List<IMyShipController> CONTROLLERS = new List<IMyShipController>();
            List<IMyCargoContainer> Cargo = new List<IMyCargoContainer>();
            List<IMyUserControllableGun> DIRECTIONAL_FIRE = new List<IMyUserControllableGun>();  //Directional ship weaponry
            List<IMyShipDrill> SHIP_DRILLS = new List<IMyShipDrill>();     //List Of all the ships drills
            List<IMyRadioAntenna> ANTENNAS = new List<IMyRadioAntenna>();
            internal MiningJob job;

            public string MiningStatus { get; internal set; }
            

            public MiningVessel(MyGridProgram grid) : base(grid)
            {
                InitMiner(grid, this);
            }

            private double _shipSize = 0.0;
            public double SHIPSIZE
            {
                get
                {
                    if (_shipSize == 0)
                        _shipSize = Math.Sqrt(SHIP_DRILLS.Count) * 0.9 * (_grid.Me.CubeGrid.ToString().Contains("Large") ? 1.5 : 1.0);
                    return _shipSize;
                }
            }

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
                            (miner.job.TargetAsteroid != null ? $"@GPS:Mine:{miner.job.TargetAsteroid.Location.X}:{miner.job.TargetAsteroid.Location.Y}:{miner.job.TargetAsteroid.Location.Z}@\n" : "@paste gps string here@\n") +
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

                GridTerminalSystem.GetBlocksOfType(miner.ANTENNAS, b => b.CubeGrid == Me.CubeGrid);
            }

            internal void Broadcast()
            {
                /*
                let other ships know that you're working on a job
                they need to know 
                    where the job is at (asteroid center)
                    what part of the job you're working on (Row, column)
                    how far you are through the job 
                        If bore in: (distance from start to ship position) / (distance from start to end)
                        if bore out: 100%
                broadcast distance should be 200% roid diameter

                */
                if (ANTENNAS.Any(t => t.IsWorking))
                {
                    var antenna = ANTENNAS.DefaultIfEmpty(null).First(t => t.IsWorking);
                    if (antenna != null)
                    {
                        // increase the antenna radius if needed
                        var desiredRadius = Math.Min((float)job.TargetAsteroid.Diameter * 2, 1000);
                        var currentRadius = antenna.Radius;
                        antenna.Radius = Math.Max(currentRadius, desiredRadius);
                        antenna.TransmitMessage(job.ToIini(), MyTransmitTarget.Owned | MyTransmitTarget.Ally);

                        // set radius back to 
                        if (currentRadius != desiredRadius)
                            antenna.Radius = currentRadius;
                    }
                }
            }

            public void GetBroadcast(string configurationData)
            {
                var otherShipJob = MiningJob.FromIni(configurationData);

                if(otherShipJob.TargetAsteroid.Location == job.TargetAsteroid.Location)
                { // asteroid bros!

                    // check for info they have that we don't
                    if (job.TargetAsteroid.RequiresUpdate && !otherShipJob.TargetAsteroid.RequiresUpdate)
                        job.TargetAsteroid.UpdateInfo(otherShipJob.TargetAsteroid);

                    // check to see if we're working on the same task
                    if(job.Row == otherShipJob.Row && job.Column == otherShipJob.Column)
                    { // oh no! we need to fix this
                        // check for progress

                        // if I'm less far along, I should go back
                        // if I didn't start, just pick the next option
                        if (job.Progress < otherShipJob.Progress || job.Progress <= 0.0)
                        {
                            job.FinishedBore = true;
                            if(job.Progress <= 0.0)
                                job.GotoNext();
                        }
                    }

                }
            }

            public void BoreMine(Asteroid asteroid, bool reset = false)
            {
                _grid.Echo($"BoreMine: {job.Row}, {job.Column}");
                _grid.Echo($"Distance: {job.GetDistanceFromBoreStart}, {job.GetDistanceFromBoreEnd}");
                //Setup Of Common Variables
                Vector3D DronePosition = RC.GetPosition();
                Vector3D Drone_To_Target = Vector3D.Normalize(asteroid.Centre - DronePosition);

                //Readouts
                //Echo("Has Finished Tunnel: " + asteroid.Finished);
                //Echo(SHIPSIZE + " 'Step' Size");
                //Echo(asteroid.Row + " /" + Steps + " Rows");
                //Echo(asteroid.Column + " /" + Steps + " Columns"); 
                //Echo((CurrentVectorEnd - DronePosition).Length() + " dist to iter++");

                //Generates Currently Targeted Vector As A Function Of 2 integers, asteroid.Row and Depth 
                
                if(!job.IsValid)
                {
                    _grid.Echo("Looks like an invalid asteroid");
                }

                //Sets IsBuried And Has Finished
                if (job.GetDistanceFromBoreEnd < 1)
                    job.FinishedBore = true; //If Reached End Toggle Finished

                //Inputs To Autopilot Function
                double RollReqt = (float)(0.6 * (Vector3D.Dot(job.Up, RC.WorldMatrix.Down)));
                GyroTurn6(job.Forward * 999999999999999999, RotationalSensitvity, GYRO, RC, RollReqt, PrecisionMaxAngularVel);

                if (job.FinishedBore) //Reverses Once Finished
                {
                    _grid.Echo("Bore out");
                    Vector_Thrust_Manager(job.CurrentVectorEnd, job.CurrentVectorStart, Position, 2, 0.5, RC);
                }
                else //else standard forward
                {
                    _grid.Echo("Bore in");
                    List<MyDetectedEntityInfo> entities = new List<MyDetectedEntityInfo>();
                    SENSOR.DetectedEntities(entities);
                    var asteroids = entities.Where(t => t.Type == MyDetectedEntityType.Asteroid);

                    Vector_Thrust_Manager(job.CurrentVectorStart, job.CurrentVectorEnd, Position, asteroids.Any() ? 1 : 10, asteroids.Any() ? 0.5 : 5, RC);
                }

                //Iterates Based On Proximity
                if (job.GetDistanceFromBoreStart < 1) {
                    if (job.Complete)
                    {
                        MiningStatus = "FIN";
                        return;
                    }

                    job.GotoNext();
                }
            }

            internal new void Run()
            {
                base.Run();

                if (job == null)
                { // update target asteroid
                    try
                    {
                        if (string.IsNullOrWhiteSpace(_grid.Me.CustomData))
                            throw new Exception("Initialization error. Missing @GPS@ in Custom Data.");

                        string[] InputData = _grid.Me.CustomData.Split('@');

                        string[] InputGpsList = InputData[1].Split(':');
                        double X;
                        double.TryParse(InputGpsList[2], out X);

                        double Y;
                        double.TryParse(InputGpsList[3], out Y);

                        double Z;
                        double.TryParse(InputGpsList[4], out Z);

                        Vector3D TryVector = new Vector3D(X, Y, Z);

                        if (job == null || job.TargetAsteroid == null || TryVector != job.TargetAsteroid.Location)
                        {
                            var asteroid = new Asteroid(TryVector);
                            job = new MiningJob(asteroid, this);
                            
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
                        return;
                    }
                }

                //Sets If Full Or Not
                bool IsEmpty = Cargo.All(cargo => cargo.GetInventory(0).CurrentMass <= 900) && SHIP_DRILLS.All(drill=>drill.GetInventory().CurrentMass < 100);
                bool IsMassAboveThreshold = SHIP_DRILLS.Sum(drill=> (double)drill.GetInventory().CurrentVolume) > SHIP_DRILLS.Sum(drill=> (double)drill.GetInventory().MaxVolume) * 0.80;

                //if (IsMassAboveThreshold)
                //    Echo("Drill Inventory Is Currently Full");
                //if (IsEmpty)
                //    Echo("Drill Inventory Is Currently Empty");
                if(string.IsNullOrWhiteSpace(MiningStatus))
                    MiningStatus = "MINE";

                if (IsEmpty && MiningStatus != "FIN")
                    MiningStatus = "MINE";
                else if (IsMassAboveThreshold && MiningStatus != "FIN")
                {
                    MiningStatus = "FULL";
                    job.FinishedBore = true;
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
                    
                    SENSOR.LeftExtend = SENSOR.MaxRange;
                    SENSOR.RightExtend = SENSOR.MaxRange;
                    SENSOR.TopExtend = SENSOR.MaxRange;
                    SENSOR.FrontExtend = SENSOR.MaxRange;
                    SENSOR.BottomExtend = SENSOR.MaxRange;
                    SENSOR.BackExtend = SENSOR.MaxRange;
                }

                //Runs Primary Mnining Logic (only if not 0,0,0)
                //-------------------------------------------------
                _grid.Echo(job.ToString());

                //Echo("Is Vanilla RC A: " + Math.Round(TargetAsteroid.Diameter) + " Metres");
                if (job?.TargetAsteroid != null)
                    _grid.Echo("Inside Asteroid? " + job.IsInsideAsteroidSoi);
                else
                    _grid.Echo("No asteroid set");

                //Echo("Runtime: " + Math.Round(Runtime.LastRunTimeMs, 3) + " Ms");
                _grid.Echo($"Status: {MiningStatus}");
                MiningLogic();
            }

            void MiningLogic()
            {
                if((SENSOR.LastDetectedEntity.BoundingBox.Center - Position).Length() < SENSOR.LastDetectedEntity.BoundingBox.Size.Length())
                { // we're inside the asteroid
                    if(job.TargetAsteroid.RequiresUpdate)
                        job.UpdateAsteroid(SENSOR.LastDetectedEntity.BoundingBox);
                }

                //Echo("Mining Logic:\n--------------");

                //Sets Drills:
                if (MiningStatus == "MINE" && job.IsInsideAsteroidSoi)
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
                if ((MiningStatus == "FIN" || MiningStatus == "FULL") && !job.IsInsideAsteroidSoi)
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
                    if (SENSOR.IsActive && job.GetDistanceFrom(Position) < 50 && job.TargetAsteroid.RequiresUpdate)
                        job.UpdateAsteroid(SENSOR.LastDetectedEntity.BoundingBox);

                    //If No Asteroid Detected Goes To Location To Detect Asteroid
                    if (job.TargetAsteroid.Diameter == 0)
                    {
                        _grid.Echo("Target needs update, travelling to asteroid location");
                        RC_Manager(job.TargetAsteroid.Location, RC);
                        return; //No Need For Remainder Of Logic
                    }

                    //Goes To Location And Mines
                    if (job.TargetAsteroid.AbleToMineFrom(RC.GetPosition()) == false && !job.IsInsideAsteroidSoi && job.GetDistanceFromBoreEnd > 4)
                    {
                        RC_Manager(job.TargetAsteroid.Location, RC);
                        _grid.Echo("Status: Locating");
                    }
                    else
                    {
                        if (job.TargetAsteroid.Diameter > SHIPSIZE) //Only if drill size is bigger than ship prevents array handlign issues
                        {
                            BoreMine(job.TargetAsteroid, false);
                            _grid.Echo("Status: Mining");
                        }
                        else
                            _grid.Echo("No Asteroid Detected, Drill array too large");
                    }

                    _grid.Echo(SHIP_DRILLS.Sum(drill=>(double)drill.GetInventory().MaxVolume).ToString("0,000.00") + " Inventory Count"); // - SHIP_DRILLS[0].GetInventory().CurrentVolume + 
                }

            }
        }
    }
}