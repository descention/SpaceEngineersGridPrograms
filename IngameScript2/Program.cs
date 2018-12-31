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

                miner.Run();

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
            
        }

        public void Main(string argument, UpdateType updateSource)
        {
            

            if (miner != null)
            {
                if (updateSource == UpdateType.Antenna)
                {

                }

                try
                {
                    if (miner != null)
                    {
                        // run miner control
                        if (updateSource == UpdateType.Update10)
                            miner.Run();

                        if(updateSource == UpdateType.Update100)
                        { // update other miners of our progress
                            miner.Broadcast();
                        }
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
                }
            }
            else
            { // somehow we don't have a miner
                Echo("no miner loaded");
            }
        }
    }
}