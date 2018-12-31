using System;
using System.Text;
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

            public MiningJob(Asteroid asteroid, MiningVessel vessel)
            {
                _vessel = vessel;
                TargetAsteroid = asteroid;
                Row = 0;
                Column = 0;
            }

            public Vector3D CurrentVectorStart => (TargetAsteroid.Location + _vessel.SHIPSIZE * (Row - 1) * Math.Pow(-1, Row) * Up) + _vessel.SHIPSIZE * (Column - 1) * Math.Pow(-1, Column) * Left;
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
            public int Steps => MathHelper.Clamp((int)((TargetAsteroid.Diameter * 0.3) / _vessel.SHIPSIZE), 1, 16);

            public bool IsValid => CurrentVectorStart != CurrentVectorEnd;
            
            public void UpdateAsteroid(BoundingBoxD box)
            {
                if(box.Size.Length() != TargetAsteroid.Diameter || TargetAsteroid.Centre != box.Center)
                {
                    TargetAsteroid.UpdateInfo(box);
                }
            }

            public double GetDistanceFrom(Vector3D position) => (TargetAsteroid.Location - position).Length();

            public double GetDistanceFromBoreStart => (CurrentVectorStart - _vessel.Position).Length();
            public double GetDistanceFromBoreEnd => (CurrentVectorEnd - _vessel.Position).Length();

            public bool IsInsideAsteroidSoi => (TargetAsteroid.Centre - _vessel.Position).Length() < TargetAsteroid.Diameter + 5.0;

            public override string ToString()
            {
                StringBuilder echo = new StringBuilder();
                echo.AppendLine($"CurrentRoid: {Vector3D.Round(TargetAsteroid.Location)}");
                echo.AppendLine($"CurrentCentre: {Vector3D.Round(TargetAsteroid.Centre)}");
                echo.AppendLine($"CurrentRoidSize: {Math.Round(TargetAsteroid.Diameter)} Metres");
                
                return echo.ToString();
            }
        }
    }
}