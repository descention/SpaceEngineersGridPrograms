using System;
using System.Collections.Generic;
using System.Text;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        public class MiningJob
        {
            public Asteroid TargetAsteroid { get; set; }
            public int Row { get; set; }
            public int Column { get; set; }
            internal MiningVessel _vessel;
            public bool FinishedBore { get; internal set; }
            internal Vector2D BoreSize{get;set;}

            public List<Vector2I> CompletedSections = new List<Vector2I>();
            
            public MiningJob(Asteroid asteroid, MiningVessel vessel)
            {
                TargetAsteroid = asteroid;
                Row = 0;
                Column = 0;
                if (vessel != null)
                {
                    _vessel = vessel;
                    BoreSize = _vessel.ShipMiningSize;
                }
            }

            public Vector3D CurrentVectorStart => (TargetAsteroid.Location + BoreSize.Y * (Row) * Up) + BoreSize.X * (Column) * Left;
            public Vector3D CurrentVectorEnd => CurrentVectorStart + Forward * (((TargetAsteroid.Centre - TargetAsteroid.Location).Length() - TargetAsteroid.Diameter / 2) + TargetAsteroid.Diameter * 0.8); //Accounts for small input

            /// <summary>
            /// Characteristic 'Forward' vector 
            /// </summary>
            internal Vector3D Forward => Vector3D.Normalize(TargetAsteroid.Centre - TargetAsteroid.Location);

            /// <summary>
            /// Characteristic 'Left' vector
            /// </summary>
            internal Vector3D Left => Vector3D.CalculatePerpendicularVector(Forward);

            /// <summary>
            /// Characteristic 'Up' vector
            /// </summary>
            internal Vector3D Up => Vector3D.Cross(Forward, Left);

            /// <summary>
            /// How many horizontal passes of the ship are required to eat the roid
            /// </summary>
            public int Steps => Math.Max(1, (int)((TargetAsteroid.Diameter * 0.6666) / Math.Min(BoreSize.Y, BoreSize.X)));

            public bool IsValid => CurrentVectorStart != CurrentVectorEnd;
            
            public void UpdateAsteroid(BoundingBoxD box)
            {
                if(box.Size.Length() != TargetAsteroid.Diameter || TargetAsteroid.Centre != box.Center)
                {
                    TargetAsteroid.UpdateInfo(box);
                }
            }

            public double GetDistanceFrom(Vector3D position) => (TargetAsteroid.Location - position).Length();

            internal double? _distanceFromStart = null;
            public double GetDistanceFromBoreStart
            {
                get
                {
                    if (_distanceFromStart.HasValue)
                        return _distanceFromStart.Value;
                    var distance = (CurrentVectorStart - _vessel.Position).Length();
                    return distance >= 0.0 ? distance : double.MaxValue;
                }
                set
                {
                    _distanceFromStart = value;
                }
            }
            public double GetDistanceFromBoreEnd => (CurrentVectorEnd - _vessel.Position).Length();

            public bool IsInsideAsteroidSoi => (TargetAsteroid.Centre - _vessel.Position).Length() < (TargetAsteroid.Diameter / 2) + 5.0;

            internal double? _progressFromIni = null;
            public double Progress
            {
                get
                {
                    if (_progressFromIni.HasValue)
                        return _progressFromIni.Value;
                    try
                    {
                        if(GetDistanceFromBoreEnd < (CurrentVectorStart - CurrentVectorEnd).Length())
                        { // we're travelling the bore
                            return FinishedBore ? 1.0 : (GetDistanceFromBoreStart / (CurrentVectorStart - CurrentVectorEnd).Length());
                        }
                        else
                        { // outside the bore
                            return 0.0;
                        }
                    }
                    catch
                    {
                        return 0.0;
                    }
                }
                internal set
                {
                    _progressFromIni = value;
                }
            }

            public bool Complete => Row == Steps && Column == Steps && FinishedBore;

            public void GotoNext()
            {
                if (Row == Steps)
                {
                    Column++;
                    Row = 1;
                    FinishedBore = false;
                }
                else
                {
                    Row++;
                    FinishedBore = false;
                }
            }

            public override string ToString()
            {
                StringBuilder echo = new StringBuilder();
                echo.AppendLine($"CurrentRoid: {Vector3D.Round(TargetAsteroid.Location)}");
                echo.AppendLine($"CurrentCentre: {Vector3D.Round(TargetAsteroid.Centre)}");
                echo.AppendLine($"CurrentRoidSize: {Math.Round(TargetAsteroid.Diameter)} Metres");
                echo.AppendLine($"Bore size: {BoreSize}");
                echo.AppendLine($"Steps: {Steps}");
                echo.AppendLine($"Progress: {Progress * 100:000.00}%");
                return echo.ToString();
            }

            public string ToIini(MyIni ini = null, string section = nameof(MiningJob))
            {
                if (ini == null)
                    ini = new MyIni();

                ini.Set(section, nameof(Row), Row);
                ini.Set(section, nameof(Column), Column);

                ini.Set(section, nameof(Progress), Progress);
                ini.Set(section, nameof(GetDistanceFromBoreStart), GetDistanceFromBoreStart);

                TargetAsteroid.ToIni(ini);

                return ini.ToString();
            }

            public static MiningJob FromIni(MyIni ini, string section = nameof(MiningJob))
            {
                var target = Asteroid.FromIni(ini);
                if (target == null)
                    return null;

                var job = new MiningJob(target, null);

                job.Row = ini.Get(section, nameof(job.Row)).ToInt32();
                job.Column = ini.Get(section, nameof(job.Column)).ToInt32();

                job.Progress = ini.Get(section, nameof(job.Progress)).ToDouble();
                job.GetDistanceFromBoreStart = ini.Get(section, nameof(job.GetDistanceFromBoreStart)).ToDouble();
                return job;
            }
        }
    }
}