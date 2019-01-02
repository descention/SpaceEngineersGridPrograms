using VRage.Game.ModAPI.Ingame.Utilities;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        public class Asteroid
        {
            public double Diameter { get; internal set; }
            public Vector3D Centre { get; set; }
            public Vector3D Location { get; internal set; }

            public Asteroid(Vector3D location)
            {
                Centre = Location = location;
                Diameter = 0.0;
            }

            public double DistanceFromWaypointToCenter => (Centre - Location).Length();

            public bool RequiresUpdate => Centre == Location || Centre.Equals(new Vector3D());

            public void UpdateInfo(BoundingBoxD box)
            {
                Diameter = box.Size.Length();
                Centre = box.Center;
            }

            public void UpdateInfo(Asteroid asteroid)
            {
                this.Diameter = asteroid.Diameter;
                this.Centre = asteroid.Centre;
            }

            public bool AbleToMineFrom(Vector3D vesselPosition)
            {
                if (Diameter == 0)
                    return false;

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

            public string ToIni(MyIni ini = null, string section = nameof(Asteroid))
            {
                if (ini == null)
                    ini = new MyIni();
                ini.Set(section, "LocX", Location.X);
                ini.Set(section, "LocY", Location.Y);
                ini.Set(section, "LocZ", Location.Z);

                ini.Set(section, "CenX", Centre.X);
                ini.Set(section, "CenY", Centre.Y);
                ini.Set(section, "CenZ", Centre.Z);

                ini.Set(section, nameof(Diameter), Diameter);

                return ini.ToString();
            }

            public static Asteroid FromIni(MyIni ini, string section = nameof(Asteroid))
            {
                try
                {
                    var LocX = ini.Get(section, "LocX").ToDouble();
                    var LocY = ini.Get(section, "LocY").ToDouble();
                    var LocZ = ini.Get(section, "LocZ").ToDouble();

                    var location = new Vector3D(LocX, LocY, LocZ);
                    var asteroid = new Asteroid(location);

                    var CenX = ini.Get(section, "CenX").ToDouble();
                    var CenY = ini.Get(section, "CenY").ToDouble();
                    var CenZ = ini.Get(section, "CenZ").ToDouble();

                    var centre = new Vector3D(CenX, CenY, CenZ);

                    if (location != centre)
                        asteroid.Centre = centre; // update with known data

                    asteroid.Diameter = ini.Get(section, nameof(Diameter)).ToDouble();

                    return asteroid;
                }
                catch
                {
                    return null;
                }
            }
        }
    }
}