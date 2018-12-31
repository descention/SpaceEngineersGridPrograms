using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        public class Asteroid
        {
            public double Diameter { get; internal set; }
            public Vector3D Centre { get; internal set; }
            public Vector3D Location { get; internal set; }

            public Asteroid(Vector3D location)
            {
                Centre = Location = location;
                Diameter = 0.0;
            }

            public double DistanceFromWaypointToCenter => (Centre - Location).Length();

            public bool RequiresUpdate => Centre == Location;

            public void UpdateInfo(BoundingBoxD box)
            {
                Diameter = box.Size.Length();
                Centre = box.Center;
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
        }
    }
}