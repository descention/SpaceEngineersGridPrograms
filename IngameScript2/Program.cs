using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System.Collections;
using System.Linq;
using System.Text;
using System;
using VRage.Collections;
using VRage.Game.Components;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Game;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
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
                    //var serializer = new XmlSerializer(typeof(Vessel));
                    //var sb = new StringBuilder();
                    //sb.Append(Storage);
                    //System.IO.Stream stream =
                    //serializer.Deserialize(xmlWriter);

                }
                
                if(miner == null)
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

        public void Save()
        {
            // Called when the program needs to save its state. Use
            // this method to save your state to the Storage field
            // or some other means. 
            // 
            // This method is optional and can be removed if not
            // needed.
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (miner != null)
            {
                try
                {
                    if (miner != null)
                        miner.Run();
                }
                catch (Exception ex)
                {
                    Echo(ex.Message);
                    Echo(ex.StackTrace);
                    if (ex.InnerException != null)
                    {
                        Echo(ex.InnerException.Message);
                        Echo(ex.InnerException.StackTrace);
                    }
                }
            }
            else
            { // somehow we don't have a miner
                Echo("no miner loaded");
            }
        }
    }
}