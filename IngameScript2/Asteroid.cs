using VRageMath;

namespace IngameScript
{
    partial class Program
    {
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
    }
}