using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        public const string CockpitName = "Fighter Cockpit 4";

        public const string ResetCommand = "Reset";
        public const string ExportToPBCustomDataCommand = "Export";

        public const string RemovePrev = "RemovePrev";

        public const string RecordApproachCommand = "RecordApproach";
        public const string RecordUpRunningCommand = "RecordUpRunning";
        public const string RecordFittingCommand = "RecordFitting";
        public const string RecordStoppingCommand = "RecordStopping";

        // interface cache
        private IMyCockpit cockpit;
        private IMyTextSurfaceProvider panel;

        private AirRoute recordingTarget = new AirRoute("", new List<AirWaypoint>());

        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update100;
            var cockpitBlock = GridTerminalSystem.GetBlockWithName(CockpitName); 
            cockpit = cockpitBlock as IMyCockpit;
            panel = cockpitBlock as IMyTextSurfaceProvider;
        }

        public void Save()
        {
            
        }

        public void Main(string argument, UpdateType updateSource)
        {
            switch(argument)
            {
                case ResetCommand:
                    Reset();
                    break;
                case ExportToPBCustomDataCommand:
                    Export();
                    break;

                case RemovePrev:
                    RemovePrevious();
                    break;

                case RecordApproachCommand:
                    AppendWaypoint(AirWaypoint.AirWaypointType.Approach);
                    break;
                case RecordUpRunningCommand:
                    AppendWaypoint(AirWaypoint.AirWaypointType.UpRunning);
                    break;
                case RecordFittingCommand:
                    AppendWaypoint(AirWaypoint.AirWaypointType.Fitting);
                    break;
                case RecordStoppingCommand:
                    AppendWaypoint(AirWaypoint.AirWaypointType.Stopping);
                    break;
                default:
                    break;
            }
            Show();
        }

        private void AppendWaypoint(AirWaypoint.AirWaypointType type)
        {
            recordingTarget.waypoints.Add(new AirWaypoint(cockpit.GetPosition(), type));
        }

        private void RemovePrevious()
        {
            var ways = recordingTarget.waypoints;
            if (ways.Count != 0)
            {
                ways.RemoveAt(ways.Count - 1);
            }
        }

        private void Export()
        {
            Me.CustomData = recordingTarget.Stringify();
        }

        private void Reset()
        {
            recordingTarget = new AirRoute("", new List<AirWaypoint>());
        }

        private void Show()
        {
            Echo($"[{recordingTarget.name}]");
            foreach(var w in recordingTarget.waypoints)
            {
                Echo(w.Stringify());
            }

            string str = $"[{recordingTarget.name}]" + Environment.NewLine;
            foreach (var w in recordingTarget.waypoints)
            {
                str += w.Stringify() + Environment.NewLine;
            }
            panel.GetSurface(0).WriteText(str);
        }
    }

    #region dependency
    public class AirRoute
    {
        public readonly string name;
        public readonly List<AirWaypoint> waypoints = new List<AirWaypoint>();

        public AirRoute(string name, List<AirWaypoint> waypoints)
        {
            this.name = name;
            this.waypoints = waypoints;
        }

        public string Stringify()
        {
            string str = "";
            str += name;
            str += '@';
            str += '[';
            foreach(var p in waypoints)
            {
                str += p.Stringify();
                str += ',';
            }
            str.TrimEnd(',');
            str += ']';
            return str;
        }

        public static AirRoute Parse(string dataStr)
        {
            var t = dataStr;
            var compStr1 = t.Split('@');

            string name = compStr1[0];

            var wayStr = compStr1[1];
            wayStr.TrimStart('[');
            wayStr.TrimEnd(']');
            var compStr2 = wayStr.Split(',');

            var list = new List<AirWaypoint>();
            foreach(var c in compStr2)
            {
                list.Add(AirWaypoint.Parse(c));
            }

            return new AirRoute(name, list);
        }

        public static List<AirRoute> ParseBundle(string dataStr)
        {
            var t = dataStr;
            var compStrs = t.Split('$');
            return compStrs.Select(x => Parse(x)).ToList();
        }
    }

    public struct AirWaypoint
    {
        public readonly Vector3 position;
        public readonly AirWaypointType waypointType;

        public enum AirWaypointType
        {
            Approach,
            UpRunning,
            Braking,
            Fitting,
            Stopping
        }

        public AirWaypoint(Vector3 position, AirWaypointType waypointType)
        {
            this.position = position;
            this.waypointType = waypointType;
        }

        public string Stringify()
        {
            return $"({StringHelper.VectorStringify(position)}#{(int)waypointType})";
        }

        public static AirWaypoint Parse(string dataStr)
        {
            var t = dataStr;
            t.TrimStart('(');
            t.TrimEnd(')');
            var compStrs = t.Split('#');
            var pos = StringHelper.ParseVectorString(compStrs[0]);
            var typeInt = int.Parse(compStrs[1]);
            return new AirWaypoint(pos, (AirWaypointType)typeInt);
        }
    }

    public static class StringHelper
    {
        public static string MessageStringify(MyIGCMessage message)
        {
            return $"Tag: {message.Tag}" +
                $"Data: {message.Data.ToString()}" +
                $"Source: {message.Source}";
        }

        public static string VectorStringify(Vector3 vec)
        {
            return $"({vec.X},{vec.Y},{vec.Z})";
        }

        public static Vector3 ParseVectorString(string vectorString)
        {
            var t = vectorString;
            t = t.TrimStart('(');
            t = t.TrimEnd(')');
            var compStrs = t.Split(',');
            return new Vector3(
                float.Parse(compStrs[0]),
                float.Parse(compStrs[1]),
                float.Parse(compStrs[2]));
        }

        public static string QuaternionStringify(Quaternion rot)
        {
            return $"({rot.X},{rot.Y},{rot.Z},{rot.W})";
        }

        public static Quaternion ParseQuaternionString(string rotString)
        {
            var t = rotString;
            t = t.TrimStart('(');
            t = t.TrimEnd(')');
            var compStrs = t.Split(',');
            return new Quaternion(
                float.Parse(compStrs[0]),
                float.Parse(compStrs[1]),
                float.Parse(compStrs[2]),
                float.Parse(compStrs[3]));
        }
    }
    #endregion
}
