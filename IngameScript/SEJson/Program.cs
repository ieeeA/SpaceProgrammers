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
        public Program()
        {
        }

        public void Save()
        {
        }

        public void Main(string argument, UpdateType updateSource)
        {
        }

        public class SEJson
        {
            private int cursor;
            private string targetStr;
            private int indentLevel;
            private char Current => targetStr[cursor];

            public object Parse(string targetStr)
            {
                this.targetStr = targetStr;
                cursor = 0;
                return ParseAny();
            }

            private void FeedNext()
            {
                cursor++;
                while (Current == ' ' || Current == '\r' || Current == '\n')
                {
                    cursor++;
                }
            }

            private object ParseAny()
            {
                switch (Current)
                {
                    case '{':
                        return ParseObject();
                    case '[':
                        return ParseArray();
                    case '"':
                        return ParseString();
                    default:
                        return null;
                }
            }

            // Grammer parser

            // Object string starting with '{' end with '}
            private Dictionary<string, object> ParseObject()
            {
                var dict = new Dictionary<string, object>();
                FeedNext();

                while (Current != '}')
                {
                    var key = ParseString();
                    FeedNext();

                    if (Current != ':')
                        return null;

                    FeedNext();
                    var value = ParseAny();
                    dict.Add(key, value);

                    FeedNext();

                    if (Current == ',')
                        FeedNext();
                }
                return dict;
            }

            // Array string starting with '[' end with ']'
            private object ParseArray()
            {
                var list = new List<object>();
                FeedNext();
                while (Current != ']')
                {
                    var res = ParseAny();
                    if (res != null)
                    {
                        list.Add(res);
                    }
                    else
                    {
                        return null;
                    }

                    FeedNext();
                    if (Current == ',')
                        FeedNext();
                }
                return list;
            }

            // "String" string starting with '"' end with '"'
            private string ParseString()
            {
                string res = "";
                FeedNext();
                while (Current != '"')
                {
                    res += Current;
                    cursor++;
                }
                return res;
            }

            public string Stringify(object target)
            {
                indentLevel = 0;
                return StringifyAny(target);
            }

            private string StringifyAny(object target)
            {
                string res;
                if (target is Dictionary<string, object>)
                {
                    res = StringifyObject(target as Dictionary<string, object>);
                }
                else if (target is List<object>)
                {
                    res = StringifyArray(target as List<object>);
                }
                else if (target is string)
                {
                    res = $"\"{target}\"";
                }
                else
                {
                    res = string.Empty;
                }
                return res;
            }

            private string StringifyObject(Dictionary<string, object> target)
            {
                string str = "{";
                foreach (var kv in target)
                {
                    str += $"\"{kv.Key}\"";
                    str += ":";
                    str += StringifyAny(kv.Value);
                    str += ",";
                }
                str = str.TrimEnd(',');
                return str + "}";
            }

            private string StringifyArray(List<object> target)
            {
                string str = "[";
                foreach (var elem in target)
                {
                    str += StringifyAny(elem);
                    str += ",";
                }
                str = str.TrimEnd(',');
                return str + "]";
            }
        }
    }
}
