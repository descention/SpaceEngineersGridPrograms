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

            public string ToIni()
            {
                var ini = new VRage.Game.ModAPI.Ingame.Utilities.MyIni();
                ini.Set(nameof(Asteroid), "LocX", Location.X);
                ini.Set(nameof(Asteroid), "LocY", Location.Y);
                ini.Set(nameof(Asteroid), "LocZ", Location.Z);

                ini.Set(nameof(Asteroid), "CenX", Centre.X);
                ini.Set(nameof(Asteroid), "CenY", Centre.Y);
                ini.Set(nameof(Asteroid), "CenZ", Centre.Z);

                ini.Set(nameof(Asteroid), nameof(Diameter), Diameter);

                return ini.ToString();
            }

            public static Asteroid TryParseFromIni(string configurationData)
            {
                try
                {
                    var ini = new VRage.Game.ModAPI.Ingame.Utilities.MyIni();
                    if (ini.TryParse(configurationData))
                    {
                        var LocX = ini.Get(nameof(Asteroid), "LocX").ToDouble();
                        var LocY = ini.Get(nameof(Asteroid), "LocY").ToDouble();
                        var LocZ = ini.Get(nameof(Asteroid), "LocZ").ToDouble();

                        var location = new Vector3D(LocX, LocY, LocZ);
                        var asteroid = new Asteroid(location);

                        var CenX = ini.Get(nameof(Asteroid), "CenX").ToDouble();
                        var CenY = ini.Get(nameof(Asteroid), "CenY").ToDouble();
                        var CenZ = ini.Get(nameof(Asteroid), "CenZ").ToDouble();

                        var centre = new Vector3D(CenX, CenY, CenZ);

                        if (location != centre)
                            asteroid.Centre = centre; // update with known data

                        asteroid.Diameter = ini.Get(nameof(Asteroid), nameof(Diameter)).ToDouble();

                        return asteroid;
                    }
                    return null;
                }
                catch
                {
                    return null;
                }
            }
        }
    }
}