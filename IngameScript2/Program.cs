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

                //Sets Update Frequency
                Runtime.UpdateFrequency = UpdateFrequency.Update10 | UpdateFrequency.Update100;
            }
            catch (Exception ex)
            {
                miner = null;
                Echo(ex.Message);

                Runtime.UpdateFrequency = UpdateFrequency.None;
            }
        }

        public void Save()
        {
            Storage = miner.ToIni();
        }

        public void Main(string argument, UpdateType updateSource)
        {
            

            if (miner != null)
            {
                

                try
                {
                    if (miner != null)
                    {

                        if (updateSource == UpdateType.Antenna)
                            miner.GetBroadcast(argument); // parse info from other miners

                        else if (updateSource == UpdateType.Update10)
                            miner.Run(); // run miner control

                        else if (updateSource == UpdateType.Update100)
                            miner.Broadcast(); // update other miners of our progress
                    }
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
                    miner.Report(ex.Message + ex.StackTrace);
                }
            }
            else
            { // somehow we don't have a miner
                Echo("no miner loaded");
            }
        }
    }
}