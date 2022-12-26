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
            Runtime.UpdateFrequency = UpdateFrequency.Update100;
        }

        public void Save()
        {

        }

        // constants
        public readonly string StatusLCDName = "CargoStatusLCD-00";
        public readonly string GaugeLCDName = "CargoStatusLCD-01";
        public readonly string SystemTitle = "PrinterCargoStatus";

        // # !!!REMARK!!!!
        // # please append cargoblockName to this array
        // # when you add cargos into this cargo system

        public readonly string[] RegisteredInventoryBlockNames = new string[]
        {
            "Large Cargo Container Printer",
        };

        private readonly int AmountResolution = 100;
        private readonly int MaxCharacter = 60;

        private bool initialized = false;
        private InventoryStatusPresenter presenter;

        public void Main(string argument, UpdateType updateSource)
        {
            if (initialized == false)
            {
                presenter = new InventoryStatusPresenter(
                    GridTerminalSystem,
                    SystemTitle,
                    StatusLCDName,
                    GaugeLCDName,
                    RegisteredInventoryBlockNames);
                presenter.AmountResolution = AmountResolution;
                presenter.MaxCharacter = MaxCharacter;

                initialized = true;
            }
            presenter.Update();
        }

        public class InventoryStatusPresenter
        {
            public int AmountResolution { get; set; } = 100;
            public int MaxCharacter { get; set; } = 60;

            private string presenterTitle;

            // interface cache
            private IMyTextPanel statusLcdPanel = null;
            private IMyTextPanel gaugeLcdPanel = null;
            private List<IMyCubeBlock> cachedBlockes = new List<IMyCubeBlock>();
            IMyGridTerminalSystem gts;

            public InventoryStatusPresenter(
                IMyGridTerminalSystem _gts,
                string _presenterTitle,
                string statusLcdName, string gaugeLcdName, string[] targetBlockNames)
            {
                gts = _gts;
                presenterTitle = _presenterTitle;

                statusLcdPanel = gts.GetBlockWithName(statusLcdName) as IMyTextPanel;
                gaugeLcdPanel = gts.GetBlockWithName(gaugeLcdName) as IMyTextPanel;
                foreach (var name in targetBlockNames)
                {
                    var block = gts.GetBlockWithName(name);
                    if (block != null)
                    {
                        cachedBlockes.Add(block);
                    }
                }
            }

            public void Update()
            {
                List<IMyInventory> inventories = new List<IMyInventory>();
                foreach (var block in cachedBlockes)
                {
                    for (int i = 0; i < block.InventoryCount; i++)
                    {
                        var inv = block.GetInventory(i);
                        inventories.Add(inv);
                    }
                }

                Dictionary<string, List<MyInventoryItem>> itemDict = new Dictionary<string, List<MyInventoryItem>>();
                foreach (var inv in inventories)
                {
                    ClassifyType(inv, itemDict);
                }
                ShowClassifiedItems(itemDict);
            }

            private void ShowClassifiedItems(Dictionary<string, List<MyInventoryItem>> itemDict)
            {
                string statusStr = $"[{this.presenterTitle}]" + Environment.NewLine;
                string gaugeStr = CreateScaleIndicatorString() + Environment.NewLine;

                foreach (var kv in itemDict)
                {
                    var itemList = kv.Value;
                    var itemType = itemList[0].Type;

                    MyFixedPoint amount = MyFixedPoint.Zero;
                    foreach (var item in itemList)
                    {
                        amount += item.Amount;
                    }

                    string amountStr = "";
                    if (itemType.TypeId.Contains("Ingot"))
                    {
                        amountStr = ((float)amount).ToString("F2");
                    }
                    else
                    {
                        amountStr = amount.ToString();
                    }
                    statusStr += $"{itemType.SubtypeId} x{amountStr}" + Environment.NewLine;
                    gaugeStr += CreateGaugeString(amount) + Environment.NewLine;
                }

                statusLcdPanel.WriteText(statusStr);
                gaugeLcdPanel.WriteText(gaugeStr);
            }

            private string CreateScaleIndicatorString()
            {
                string str = "";
                for (int i = 0; i < MaxCharacter; i++)
                {
                    if (i % 10 == 0)
                    {
                        int unit = i / 10;
                        str += $"{unit}k";
                        i++;
                    }
                    else
                    {
                        str += "_";
                    }
                }
                return str;
            }

            private string CreateGaugeString(MyFixedPoint amount)
            {
                var count = (int)amount;
                count /= AmountResolution;
                string str = "";
                int i = 0;
                for (i = 0; i < count; i++)
                {
                    if (MaxCharacter <= i)
                    {
                        return str;
                    }
                    str += (i % 10 == 0) ? "#" : "x";
                }
                return str;
            }

            private void ClassifyType(IMyInventory inv, Dictionary<string, List<MyInventoryItem>> itemDict)
            {
                List<MyInventoryItem> itemList = new List<MyInventoryItem>();
                inv.GetItems(itemList);
                for (int i = 0; i < itemList.Count; i++)
                {
                    var item = itemList[i];
                    var key = item.Type.ToString();
                    if (itemDict.ContainsKey(key) == false)
                    {
                        itemDict.Add(key, new List<MyInventoryItem>());
                    }
                    itemDict[key].Add(item);
                }
            }
        }
    }
}
