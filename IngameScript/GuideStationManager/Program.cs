using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System;
using VRage.Collections;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Game;
using VRage;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        private readonly string ConnectorGroups = "ConnectorSys-Auto-#01";
        private const float VerticalOffset = 10.0f;
        private const float PortOffset = 2.0f;


        private List<IMyShipConnector> connectors = new List<IMyShipConnector>();
        private bool isInitialized = false;

        private List<string> history = new List<string>();
        
        public Program()
        {
            IGC.RegisterBroadcastListener(ConnectorInfo.RequestTag);
            Runtime.UpdateFrequency = UpdateFrequency.Update100;
        }

        public void Save()
        {
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (isInitialized == false)
            {
                isInitialized = Initialize();
                return;
            }

            EchoConnectorStatus();

            var listeners = new List<IMyBroadcastListener>();
            IGC.GetBroadcastListeners(listeners);
            while (listeners[0].HasPendingMessage)
            {
                var message = listeners[0].AcceptMessage();
                history.Add(message.Tag + ":" + message.Data as string);
                if (message.Tag == ConnectorInfo.RequestTag)
                {
                    var availables = GetAvailableConnector();
                    var connecterStr = ConvertToMessage(availables);
                    IGC.SendUnicastMessage(message.Source, ConnectorInfo.ResponseTag, connecterStr);
                }
            }
        }

        private bool Initialize()
        {
            var group = GridTerminalSystem.GetBlockGroupWithName(ConnectorGroups);
            var list = new List<IMyTerminalBlock>();
            group?.GetBlocks(list);
            foreach (var block in list)
            {
                if (block is IMyShipConnector)
                {
                    var con = block as IMyShipConnector;
                    connectors.Add(con);
                }
            }
            if (connectors.Count != 0)
            {
                return true;
            }
            return false;
        }

        private void EchoConnectorStatus()
        {
            foreach(var con in connectors)
            {
                Echo($"{con.CustomName}: {con.Status}");
            }

            foreach (var hi in history)
            {
                Echo(hi);
            }
        }

        private string ConvertToMessage(List<IMyShipConnector> availables)
        {
            return ConnectorInfo
                .StringifyList(availables
                                    .Select(c => new ConnectorInfo(c, VerticalOffset, PortOffset))
                                    .ToList());
        }

        private List<IMyShipConnector> GetAvailableConnector()
        {
            return connectors
                .Where(c => c.Status == MyShipConnectorStatus.Unconnected && c.Enabled)
                .ToList();
        }
    }

    // Tools for connector info communication
    public struct ConnectorInfo
    {
        public readonly string name;
        public readonly Vector3 position;
        public readonly Quaternion rotation;
        public Vector3 approachOffset; // safety course to approach

        public const string RequestTag = "RequestConnectorInfo";
        public const string ResponseTag = "ResponseConnectorInfo";

        public ConnectorInfo(IMyShipConnector connector, float verticalOffset, float portOffset)
        {
            name = connector.Name;
            rotation = Quaternion.CreateFromRotationMatrix(connector.WorldMatrix);
            position = connector.GetPosition() + rotation.Forward * portOffset;
            approachOffset = rotation.Forward * verticalOffset;
        }

        public ConnectorInfo(string cname, Vector3 cposition, Quaternion crot, Vector3 capproachOffset)
        {
            name = cname;
            position = cposition;
            rotation = crot;
            approachOffset = capproachOffset;
        }

        public string Stringify()
        {
            return $"{name}:{StringHelper.VectorStringify(position)}:{StringHelper.QuaternionStringify(rotation)}:{StringHelper.VectorStringify(approachOffset)}";
        }

        private static ConnectorInfo Parse(string connectorString)
        {
            var compStrs = connectorString.Split(':');
            return new ConnectorInfo(
                    compStrs[0],
                    StringHelper.ParseVectorString(compStrs[1]),
                    StringHelper.ParseQuaternionString(compStrs[2]),
                    StringHelper.ParseVectorString(compStrs[3])
                );
        }

        public static string StringifyList(List<ConnectorInfo> infos)
        {
            var str = "";
            foreach (var info in infos)
            {
                str += info.Stringify() + ";";
            }
            str = str.TrimEnd(';');
            return str;
        }

        public static List<ConnectorInfo> ParseList(string connectorListString)
        {
            var infoStrs = connectorListString.Split(';');
            var connectorList = new List<ConnectorInfo>();
            foreach (var s in infoStrs)
            {
                connectorList.Add(ConnectorInfo.Parse(s));
            }
            return connectorList;
        }
    };

    #region dependency
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
