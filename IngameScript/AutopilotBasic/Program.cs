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
        /*
 * R e a d m e
 * -----------
 * 
 * In this file you can include any instructions or other comments you want to have injected onto the 
 * top of your final script. You can safely delete this file if you do not want any such comments.
 */

        // interface caching
        IMyCockpit cockpit;
        IMyShipConnector mainConnector;
        IMyRemoteControl remoteControl;
        IMyProgrammableBlock programmableBlock;

        List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
        List<IMyGasTank> tanks = new List<IMyGasTank>();

        // controllers
        ConnectorClient client;
        FlightRecorder recorder;
        GyroController gyroController = null;
        ThrusterController thrusterController = null;
        FlightController flightController = null;
        AutomaticConnectionManager connectionManager = null;

        // constant value
        readonly float ReachConstant = 2.0f; // stoppingRange
        readonly float StoppingSpeed = 1f;

        readonly string ProgrammableBlockName = "Programmable block";
        readonly string MainConnectorName = "Connector";
        readonly string HydrogenTankGroupName = "HydrogenTanks";
        readonly string CockpitName = "Cockpit";
        readonly string ControlSeatName = "Control Seat";
        readonly string GyroName = "Gyro";
        readonly string RemoteControlName = "Remote";

        readonly string StartCruiseCommand = "StartCruise";
        readonly string StopCommand = "Stop";

        // state value
        InertiaAutoPilotMode mode = InertiaAutoPilotMode.Initial;
        AutoConnectionPhase autoConnectionPhase = AutoConnectionPhase.RequestingConnectorInfo;

        Vector3D currentDestination;
        ConnectorInfo targetConnector;

        int currentWaypointIndex = 0;
        int recordCounter = 0;

        // state enum
        enum InertiaAutoPilotMode
        {
            Initial,
            TakeOff,
            Idling,
            Cruising,
            OrientationSetting,
            Approaching,
            AutoConnection,
        }

        enum AutoConnectionPhase
        {
            RequestingConnectorInfo, ApproachOffset, FittingConnector
        }

        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
            mode = InertiaAutoPilotMode.Initial;
            client = new ConnectorClient(IGC);
        }

        public void Save()
        {

        }

        private void StartAutomaticConnection()
        {
            mode = InertiaAutoPilotMode.AutoConnection;
            autoConnectionPhase = AutoConnectionPhase.RequestingConnectorInfo;
            client.TryToGetConnectorInfo(mainConnector.GetPosition());
        }

        private Vector3 GetDirection()
        {
            return currentDestination - cockpit.GetPosition();
        }

        private void InitializaCache()
        {
            List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlocks(blocks);
            // retrieve and caching thrusters and a cockpit

            List<IMyGyro> gyros = new List<IMyGyro>();
            List<IMyThrust> thrusters = new List<IMyThrust>();
            List<IMyThrust> atmosphericThrusters = new List<IMyThrust>();
            List<IMyThrust> hydrogenThrusters = new List<IMyThrust>();
            List<IMyThrust> ionThrusters = new List<IMyThrust>();
            foreach (var block in blocks)
            {
                if (block.CustomName.Contains("Thruster"))
                {
                    var thr = block as IMyThrust;
                    thrusters.Add(thr);
                    if (block.CustomName.Contains("Atmospheric"))
                    {
                        atmosphericThrusters.Add(thr);
                    }
                    else if (block.CustomName.Contains("Hydrogen"))
                    {
                        hydrogenThrusters.Add(thr);
                    }
                    else
                    {
                        ionThrusters.Add(thr);
                    }
                }
                if (block.CustomName.Contains(CockpitName))
                {
                    var temp_cock = block as IMyCockpit;
                    cockpit = temp_cock;
                }
                if (block.CustomName.Contains(ControlSeatName))
                {
                    if (cockpit == null)
                    {
                        var temp_cock = block as IMyCockpit;
                        cockpit = temp_cock;
                    }
                }
                if (block.CustomName.Contains(GyroName))
                {
                    gyros.Add(block as IMyGyro);
                }
                if (block.CustomName.Contains(RemoteControlName))
                {

                    remoteControl = block as IMyRemoteControl;
                    remoteControl.GetWaypointInfo(waypoints);
                    if (waypoints.Count() == 0)
                    {
                        currentDestination = cockpit.GetPosition();
                    }
                    else
                    {
                        currentWaypointIndex = 0;
                        currentDestination = waypoints[currentWaypointIndex].Coords;
                    }
                }
                if (block.CustomName.Contains(MainConnectorName))
                {
                    mainConnector = block as IMyShipConnector;
                }
                if (block.CustomName.Contains(ProgrammableBlockName))
                {
                    programmableBlock = block as IMyProgrammableBlock;
                }
            }

            if (remoteControl == null)
            {
                currentDestination = cockpit.GetPosition();
            }

            // recorder
            var tankGroup = GridTerminalSystem.GetBlockGroupWithName(HydrogenTankGroupName);
            blocks.Clear();
            if (tankGroup != null)
            {
                tankGroup.GetBlocks(blocks);
                foreach (var tank in blocks)
                {
                    tanks.Add(tank as IMyGasTank);
                }
                recorder = new FlightRecorder(tanks);
            }

            // controller initialization
            gyroController = new GyroController(GridTerminalSystem, cockpit, gyros);
            thrusterController = new ThrusterController(GridTerminalSystem, cockpit, atmosphericThrusters, hydrogenThrusters, ionThrusters);
            flightController = new FlightController(GridTerminalSystem, cockpit, thrusterController, gyroController, this);
            connectionManager = new AutomaticConnectionManager(GridTerminalSystem, cockpit, mainConnector, flightController, client, this);
        }

        public void Main(string argument, UpdateType updateSource)
        {
            Echo("current mode:" + mode.ToString());
            if (cockpit != null)
            {
                Echo("cockpit direction:" + cockpit.Orientation.Forward);
            }

            if (mode == InertiaAutoPilotMode.Initial)
            {
                InitializaCache();
                mode = InertiaAutoPilotMode.OrientationSetting;

                thrusterController.AutoSwitchThruster();
                flightController.FreeFunctionalBlocks();
            }
            else if (mode == InertiaAutoPilotMode.TakeOff)
            {
                // TODO:
                // - when cruising request comes, disconnect a connector and staring cruising protocol
            }
            else if (mode == InertiaAutoPilotMode.Idling)
            {
                flightController.FreeFunctionalBlocks();
            }
            else if (mode == InertiaAutoPilotMode.OrientationSetting)
            {
                if (flightController.Stabilize(GetDirection()))
                {
                    mode = InertiaAutoPilotMode.Cruising;
                }
            }
            else if (mode == InertiaAutoPilotMode.Cruising)
            {
                var direction = GetDirection();
                if (direction.Length() <= ReachConstant)
                {
                    if (cockpit.GetShipVelocities().LinearVelocity.Length() < StoppingSpeed)
                    {
                        currentWaypointIndex++;
                        if (currentWaypointIndex < waypoints.Count)
                        {
                            var currentDestination = waypoints[currentWaypointIndex].Coords;
                            mode = InertiaAutoPilotMode.OrientationSetting;
                        }
                        else
                        {
                            mode = InertiaAutoPilotMode.Idling;
                        }
                    }
                }
                flightController.Move(direction);

            }
            else if (mode == InertiaAutoPilotMode.AutoConnection)
            {
                connectionManager.UpdateProtocol();   
            }
            thrusterController.AutoSwitchThruster();
            client.UpdateClient();
        }


        public class FlightController
        {
            // constant value
            public readonly float AntiGravityCoeff = 1.1f;
            public readonly float GravityMergin = 0.05f;

            public readonly float StoppingDistance = 2.0f; // stoppingRange
            public readonly float StoppingSpeed = 1f;

            public readonly float BrakingDistance = 100f;
            public readonly float BrakingSpeed = 5f;

            public readonly float FittingDistance = 1f;
            public readonly float FittingSpeed = 2f;

            public readonly float UpRunningDistance = 1000f;
            public readonly float UpRunningSpeed = 40f;

            public readonly float ApproachDistance = 1000f;
            public readonly float ApproachSpeed = 98f;

            public readonly float CruisingSpeed = 98f;

            readonly float EPS = 0.001f;

            // interface cache
            private IMyGridTerminalSystem gts;
            private IMyCockpit cockpit;
            private ThrusterController thrusterController;
            private GyroController gyroController;
            private Program program;

            public FlightController(
                IMyGridTerminalSystem _gts,
                IMyCockpit _cockpit,
                ThrusterController thrusterController,
                GyroController gyroController,
                Program program)
            {
                this.gts = _gts;
                this.cockpit = _cockpit;
                this.thrusterController = thrusterController;
                this.gyroController = gyroController;
                this.program = program;
            }

            public bool Stabilize(Vector3 direction)
            {
                var gravity = cockpit.GetTotalGravity();
                if (gravity.Length() > GravityMergin) // high-gravity environment
                {
                    return gyroController.AlignToGround(direction, cockpit.GetTotalGravity());
                }
                else // low-gravity enviroment
                {
                    if (direction.Length() < EPS)
                    {
                        direction = Vector3.Forward;
                    }

                    Vector3 freeGroundAxis;
                    if (Vector3.Dot(direction, Vector3.Right) < 9.0f)
                    {
                        freeGroundAxis = Vector3.Cross(direction, Vector3.Right);
                    }
                    else
                    {
                        freeGroundAxis = Vector3.Cross(direction, Vector3.Up);
                    }
                    return gyroController.AlignToGround(direction, freeGroundAxis);
                }
            }

            public void Move(Vector3 direction)
            {
                SwitchDampener(false);
                var distance = direction.Length();
                var controlledSpeed =
                    distance < FittingDistance ? FittingSpeed :
                    distance < BrakingDistance ? BrakingSpeed :
                    distance < UpRunningDistance ? UpRunningSpeed : CruisingSpeed;
                var clampedSpeed = Math.Max(FittingDistance, Math.Min(distance, controlledSpeed));
                var targetVelocity = clampedSpeed * Vector3.Normalize(direction);

                program.Echo($"controlledSpeed: {controlledSpeed:f2}");

                var gravity = cockpit.GetTotalGravity() * AntiGravityCoeff;
                if (gravity.Length() < GravityMergin)
                {
                    thrusterController.Accelerate(targetVelocity, Vector3.Zero);
                }
                else
                {
                    thrusterController.Accelerate(targetVelocity, gravity);
                }
                Stabilize(direction);
            }

            public void SwitchDampener(bool active)
            {
                cockpit.DampenersOverride = active;
            }

            public void FreeFunctionalBlocks()
            {
                gyroController.FreeGyro();
                thrusterController.FreeAllThruster();
                SwitchDampener(true);
            }
        }

        public class ThrusterController
        {
            // constants
            readonly float ReactionConstant = 0.8f;
            readonly float CutoffEfficiency = 0.3f;

            // Interface caches
            private IMyGridTerminalSystem gts;
            private IMyCockpit cockpit;
            private List<IMyThrust> thrusters = new List<IMyThrust>();
            private List<IMyThrust> atmosphericThrusters = new List<IMyThrust>();
            private List<IMyThrust> hydrogenThrusters = new List<IMyThrust>();
            private List<IMyThrust> ionThrusters = new List<IMyThrust>();
            private List<IMyThrust> activeThrusters = new List<IMyThrust>();

            // calculation caching
            // If cockpit.Orientation.Forward != Base6Direction.Direction.Forward, this ship goes wrong directions.
            // We use the map to solve this problem.
            private Dictionary<IMyThrust, Base6Directions.Direction> thruster2direction = new Dictionary<IMyThrust, Base6Directions.Direction>();
            private float shipMass;

            public ThrusterController(
                IMyGridTerminalSystem _gts,
                IMyCockpit _cockpit,
                List<IMyThrust> atmosphericThrusters,
                List<IMyThrust> hydrogenThrusters,
                List<IMyThrust> ionThrusters)
            {
                this.gts = _gts;
                this.cockpit = _cockpit;
                shipMass = cockpit.CalculateShipMass().TotalMass;

                thrusters = atmosphericThrusters.Concat(hydrogenThrusters).Concat(ionThrusters).ToList();
                this.atmosphericThrusters = atmosphericThrusters;
                this.hydrogenThrusters = hydrogenThrusters;
                this.ionThrusters = ionThrusters;

                InitializeDirectionOfThrusters();
            }

            public void Accelerate(Vector3 targetVelocity, Vector3 gravity)
            {
                // 6-maximum force calculation
                // calculation thruster unit count (large thruster = n * small thruster)
                Dictionary<Base6Directions.Direction, float> maxF = new Dictionary<Base6Directions.Direction, float>();
                Dictionary<Base6Directions.Direction, int> units = new Dictionary<Base6Directions.Direction, int>();
                Dictionary<Base6Directions.Direction, List<IMyThrust>> thrusterDict = new Dictionary<Base6Directions.Direction, List<IMyThrust>>();
                Dictionary<IMyThrust, float> assignWeight = new Dictionary<IMyThrust, float>();
                for (int i = 0; i <= (int)Base6Directions.Direction.Down; i++)
                {
                    maxF[(Base6Directions.Direction)i] = 0;
                    units[(Base6Directions.Direction)i] = 0;
                    thrusterDict.Add((Base6Directions.Direction)i, new List<IMyThrust>());
                }
                foreach (var thr in activeThrusters)
                {
                    if (thr.IsWorking)
                    {
                        var dir = thruster2direction[thr];

                        // map to relative direction6
                        maxF[dir] += thr.MaxEffectiveThrust;
                        thrusterDict[dir].Add(thr);

                        //Echo($"{dir.ToString()}: {thr.CustomName}");
                    }
                }

                // calculate target force using kinematic equation considering gravity
                var shipRot = Quaternion.CreateFromRotationMatrix(cockpit.WorldMatrix);
                var currentVelocity = cockpit.GetShipVelocities().LinearVelocity;
                var relative = targetVelocity - currentVelocity;

                //DebugIndicate(relative, "re;");
                //DebugIndicate(currentVelocity, "cV;");

                var accelerateForce = (relative / ReactionConstant) * shipMass; // F = ma
                var gravityForce = -gravity * shipMass;

                // calculate anti-gravity force components(anti-gravity force is more important than acceleration)
                Vector3[] axises = new Vector3[]
                {
                    Vector3.Forward, Vector3.Backward,
                    Vector3.Left, Vector3.Right,
                    Vector3.Up, Vector3.Down
                };
                var leftMaxF = new Dictionary<Base6Directions.Direction, float>();
                Dictionary<Base6Directions.Direction, float> components = new Dictionary<Base6Directions.Direction, float>();
                var localGravityForce = Vector3.Transform(gravityForce, Quaternion.Inverse(shipRot));
                float antiGravityGiveUpRatio = 1f;

                for (int i = 0; i <= (int)Base6Directions.Direction.Down; i++)
                {
                    var dir = (Base6Directions.Direction)i;
                    var axis = axises[i];
                    var component = Vector3.Dot(axis, localGravityForce);
                    if (component <= 0) continue;
                    antiGravityGiveUpRatio = Math.Min(maxF[dir] / component, antiGravityGiveUpRatio);
                }
                var targetGravityForce = localGravityForce * antiGravityGiveUpRatio;
                for (int i = 0; i <= (int)Base6Directions.Direction.Down; i++)
                {
                    var dir = (Base6Directions.Direction)i;
                    var axis = axises[i];
                    components[dir] = Math.Max(Vector3.Dot(axis, targetGravityForce), 0);
                    leftMaxF[dir] = maxF[dir] - components[dir];
                }

                //DebugIndicate(localGravityForce, $"giv:{antiGravityGiveUpRatio} g:");

                // calculate accelerate force components
                var giveUpRatio = 1f;
                var localAccelerateForce = Vector3.Transform(accelerateForce, Quaternion.Inverse(shipRot));// localize targetForce
                for (int i = 0; i <= (int)Base6Directions.Direction.Down; i++)
                {
                    var dir = (Base6Directions.Direction)i;
                    var axis = axises[i];
                    var component = Vector3.Dot(axis, localAccelerateForce);
                    if (component <= 0) continue;
                    giveUpRatio = Math.Min(leftMaxF[dir] / component, giveUpRatio);
                }
                var targetAccelerateForce = localAccelerateForce * giveUpRatio;

                //DebugIndicate(accelerateForce, $"giv:{giveUpRatio} aF:");
                //DebugIndicate(targetAccelerateForce, $"giv:{giveUpRatio} tAF:");

                for (int i = 0; i <= (int)Base6Directions.Direction.Down; i++)
                {
                    var dir = (Base6Directions.Direction)i;
                    var axis = axises[i];
                    components[dir] += Math.Max(Vector3.Dot(axis, targetAccelerateForce), 0);
                }

                //Echo($"mass: {shipMass}kg");
                // force assignment
                for (int i = 0; i <= (int)Base6Directions.Direction.Down; i++)
                {
                    var dir = (Base6Directions.Direction)i;
                    //var forcePerUnit = components[dir] / units[dir];
                    foreach (var thr in thrusterDict[dir])
                    {
                        //Echo(thr.CustomName + ";" +
                        //    "; fPU" + units[dir].ToString("f2") +
                        //    "; comp; " + components[dir]);
                        //thr.ThrustOverride = forcePerUnit;

                        // balanced assignment
                        thr.ThrustOverride = components[dir] * (thr.MaxEffectiveThrust / maxF[dir]);
                    }
                }
            }

            public void AutoSwitchThruster()
            {
                activeThrusters.Clear();

                // about atmospheric trusters
                foreach (var thr in atmosphericThrusters)
                {
                    //Echo($"{thr.CustomName}: {(thr.MaxEffectiveThrust / thr.MaxThrust)}({CutoffEfficiency})");
                    if ((thr.MaxEffectiveThrust / thr.MaxThrust) < CutoffEfficiency)
                    {
                        thr.ThrustOverride = -1;
                        thr.Enabled = false;
                    }
                    else
                    {
                        thr.Enabled = true;
                        activeThrusters.Add(thr);
                    }
                }

                // about hydrogen trusters
                // if only atmospheric trusters can't lift ship, turn on hydrogen trusters
                var lifableForce = shipMass * GetGravityAsN().Length();
                var gravityAxis = Vector3.Normalize(cockpit.GetTotalGravity());
                var totalLiftPower = 0f;
                foreach (var thr in atmosphericThrusters)
                {
                    var rot = thr.WorldMatrix;
                    var effectiveForce = Vector3.Transform(Vector3.Forward, Matrix.Invert(rot)) * thr.MaxEffectiveThrust;
                    totalLiftPower += Vector3.Dot(-gravityAxis, effectiveForce);
                }
                //Echo($"lF: {lifableForce:f2}, tL: {totalLiftPower:f2}, en: {lifableForce <= totalLiftPower}");
                if (lifableForce <= totalLiftPower) // capa
                {
                    foreach (var thr in hydrogenThrusters) thr.Enabled = true;
                }
                else // can't lift
                {
                    foreach (var thr in hydrogenThrusters) thr.Enabled = false;
                }
                activeThrusters.AddRange(hydrogenThrusters);
            }

            public void FreeAllThruster()
            {
                foreach (var thr in thrusters)
                {
                    if (thr.IsWorking)
                    {
                        thr.ThrustOverridePercentage = -1;
                    }
                }
            }

            private Vector3 GetGravityAsN()
            {
                return cockpit.GetTotalGravity();
            }

            private void InitializeDirectionOfThrusters()
            {
                var shipRot = Quaternion.CreateFromRotationMatrix(cockpit.WorldMatrix);
                var shipForward = shipRot.Forward;
                var shipRight = shipRot.Right;
                var shipUp = shipRot.Up;

                foreach (var thruster in thrusters)
                {
                    var thrusterRot = Quaternion.CreateFromRotationMatrix(thruster.WorldMatrix);
                    var thrusterForward = thrusterRot.Forward;

                    var forwardDot = Vector3.Dot(thrusterForward, shipForward);
                    var rightDot = Vector3.Dot(thrusterForward, shipRight);
                    var upDot = Vector3.Dot(thrusterForward, shipUp);

                    // OMG code... this is OMG code. 
                    if (forwardDot > 0.9f) thruster2direction.Add(thruster, Base6Directions.Direction.Backward);
                    if (forwardDot < -0.9f) thruster2direction.Add(thruster, Base6Directions.Direction.Forward);
                    if (rightDot > 0.9f) thruster2direction.Add(thruster, Base6Directions.Direction.Left);
                    if (rightDot < -0.9f) thruster2direction.Add(thruster, Base6Directions.Direction.Right);
                    if (upDot > 0.9f) thruster2direction.Add(thruster, Base6Directions.Direction.Down);
                    if (upDot < -0.9f) thruster2direction.Add(thruster, Base6Directions.Direction.Up);
                }
            }
        }

        public class GyroController
        {
            // constants
            readonly float RotationMergin = 0.07f;
            readonly float GyroSensitivity = 1.5f;
            readonly float ReachConstant = 2.0f; // stoppingRange

            // interface cache
            private List<IMyGyro> gyros = new List<IMyGyro>();
            private IMyCockpit cockpit;

            // state
            private RotationPhase rotationPhase = RotationPhase.Neutral;

            enum RotationPhase
            {
                Roll, Pitch, Yaw, Neutral
            }

            public GyroController(IMyGridTerminalSystem _gts, IMyCockpit _cockpit, List<IMyGyro> _gyros)
            {
                this.gyros = _gyros;
                this.cockpit = _cockpit;
            }

            public bool AlignToGround(Vector3 direction, Vector3 groundAxis)
            {
                Quaternion shipRot = Quaternion.CreateFromRotationMatrix(cockpit.WorldMatrix);
                groundAxis.Normalize();

                // gravity alignment
                var rollAxis = shipRot.Forward;
                var pitchAxis = shipRot.Right;
                var yawAxis = shipRot.Up;

                var Erg = Vector3.Dot(pitchAxis, groundAxis) * pitchAxis + Vector3.Dot(yawAxis, groundAxis) * yawAxis;
                var Epg = Vector3.Dot(rollAxis, groundAxis) * rollAxis + Vector3.Dot(yawAxis, groundAxis) * yawAxis;
                Erg.Normalize();
                Epg.Normalize();

                var rollSignAxis = Vector3.Cross(groundAxis, shipRot.Forward);
                var rollSign = -Math.Sign(Vector3.Dot(-shipRot.Up, rollSignAxis));
                var rollAngleAbs = Math.Acos(Vector3.Dot(Erg, -shipRot.Up) / (Erg.Length() * groundAxis.Length()));

                var pitchSignAxis = Vector3.Cross(groundAxis, shipRot.Right);
                var pitchSign = -Math.Sign(Vector3.Dot(-shipRot.Up, pitchSignAxis));
                var pitchAngleAbs = Math.Acos(Vector3.Dot(Epg, -shipRot.Up) / (Epg.Length() * groundAxis.Length()));

                if (rotationPhase == RotationPhase.Neutral) // neutral
                {
                    // choose more efficient rotation axis
                    rotationPhase = (groundAxis - Erg).Length() < (groundAxis - Epg).Length() ? RotationPhase.Pitch : RotationPhase.Roll;
                }
                else if (rotationPhase == RotationPhase.Roll) // rolling
                {
                    //DebugIndicate($"p[0]{rollAngleAbs:f3},  {pitchAngleAbs:f3}");

                    if (rollAngleAbs < RotationMergin)
                    {
                        SetGyro(0, 0, 0);
                        if (pitchAngleAbs < RotationMergin) rotationPhase = RotationPhase.Yaw;
                        else rotationPhase = RotationPhase.Pitch;
                    }
                    else
                    {
                        SetGyro(rollSign * rollAngleAbs, 0, 0);
                    }
                }
                else if (rotationPhase == RotationPhase.Pitch) // pitch
                {
                    //DebugIndicate($"p[0]{rollAngleAbs:f3},  {pitchAngleAbs:f3}");

                    if (pitchAngleAbs < RotationMergin)
                    {
                        SetGyro(0, 0, 0);
                        if (rollAngleAbs < RotationMergin) rotationPhase = RotationPhase.Yaw;
                        else rotationPhase = RotationPhase.Roll;
                    }
                    else
                    {
                        SetGyro(0, pitchSign * pitchAngleAbs, 0);
                    }
                }
                else if (rotationPhase == RotationPhase.Yaw)
                {
                    // othogonalization by removing groundAxis-component

                    if (direction.Length() < ReachConstant)
                    {
                        SetGyro(0, 0, 0);
                        FreeGyro();
                        return true;
                    }
                    //Echo($"reach phase 2 {direction.Length():f2}");

                    var ndir = Vector3D.Normalize(direction);
                    var groundComponent = Vector3D.Dot(groundAxis, direction);
                    var groundPlaneDirection = direction - (float)groundComponent * groundAxis;
                    if (groundPlaneDirection.Length() < ReachConstant)
                    {
                        SetGyro(0, 0, 0);
                        FreeGyro();
                        return true;
                    }

                    groundPlaneDirection.Normalize();

                    var yawSignAxis = Vector3.Cross(groundPlaneDirection, shipRot.Up);
                    var yawSign = Math.Sign(Vector3.Dot(yawSignAxis, shipRot.Forward));
                    var yawAngleAbs = Math.Acos(Vector3.Dot(groundPlaneDirection, shipRot.Forward) / (groundPlaneDirection.Length() * shipRot.Forward.Length()));

                    // check gravity instability
                    if (pitchAngleAbs >= RotationMergin || rollAngleAbs >= RotationMergin)
                    {
                        rotationPhase = pitchAngleAbs > rollAngleAbs ? RotationPhase.Pitch : RotationPhase.Roll;
                    }
                    else if (yawAngleAbs < RotationMergin) // oriantation setting was completed
                    {
                        SetGyro(0, 0, 0);
                        FreeGyro();
                        return true;
                    }
                    else
                    {
                        SetGyro(0, 0, yawSign * yawAngleAbs);
                    }
                }
                return false;
            }

            public void FreeGyro()
            {
                foreach (var gyro in gyros)
                {
                    gyro.GyroOverride = false;
                }
            }

            private void SetGyro(double roll, double pitch, double yaw)
            {
                // TODO: multi-direction gyroscopes
                foreach (var gyro in gyros)
                {
                    gyro.GyroOverride = true;
                    gyro.Roll = (float)roll * GyroSensitivity;
                    gyro.Pitch = (float)pitch * GyroSensitivity;
                    gyro.Yaw = (float)yaw * GyroSensitivity;
                }
            }
        }

        public class AutomaticConnectionManager
        {
            // interface cache
            private IMyGridTerminalSystem gts;
            private IMyCockpit cockpit;
            private FlightController flightController;
            private ConnectorClient client;
            private IMyShipConnector mainConnector;
            private Program program;

            // state
            private AutoConnectionPhase autoConnectionPhase;
            private ConnectorInfo targetConnector;
            private bool workInProgress = false;

            enum AutoConnectionPhase
            {
                RequestingConnectorInfo, ApproachOffset, FittingConnector
            }

            public AutomaticConnectionManager(
                IMyGridTerminalSystem gts,
                IMyCockpit cockpit,
                IMyShipConnector mainConnector,
                FlightController flightController,
                ConnectorClient client,
                Program program)
            {
                this.gts = gts;
                this.cockpit = cockpit;
                this.mainConnector = mainConnector;
                this.flightController = flightController;
                this.client = client;
                this.program = program;
            }

            public void StartProtocol()
            {
                autoConnectionPhase = AutoConnectionPhase.RequestingConnectorInfo;
                client.TryToGetConnectorInfo(cockpit.GetPosition());
                workInProgress = true;
            }

            public void UpdateProtocol()
            {
                program.Echo($"Connection Phase: {autoConnectionPhase}");
                switch (autoConnectionPhase)
                {
                    case AutoConnectionPhase.RequestingConnectorInfo:
                        if (client.Finished && 0 < client.KnownInfos.Count)
                        {
                            var infos = client.KnownInfos;
                            targetConnector = infos[0];
                            foreach (var i in client.KnownInfos)
                            {
                                // get nearest connectable connector
                                if ((targetConnector.position - mainConnector.GetPosition()).Length() >
                                    (i.position - mainConnector.GetPosition()).Length())
                                {
                                    targetConnector = i;
                                }
                            }
                            autoConnectionPhase = AutoConnectionPhase.ApproachOffset;
                        }
                        else if (client.Finished) // there is no connectable connector
                        {
                            workInProgress = false;
                        }
                        break;
                    case AutoConnectionPhase.ApproachOffset:
                        program.Echo($"cock: {StringHelper.VectorStringify(cockpit.GetPosition())} con: {StringHelper.VectorStringify(mainConnector.GetPosition())}");
                        var offsetDir = targetConnector.approachOffset - mainConnector.GetPosition();
                        flightController.Move(offsetDir);
                        if (offsetDir.Length() <= flightController.StoppingDistance)
                        {
                            autoConnectionPhase = AutoConnectionPhase.FittingConnector;
                        }
                        break;
                    case AutoConnectionPhase.FittingConnector:
                        var dir = targetConnector.position - mainConnector.GetPosition();
                        flightController.Move(dir);
                        mainConnector.Connect();
                        if (mainConnector.Status == MyShipConnectorStatus.Connected)
                        {
                            workInProgress = false;
                        }
                        break;
                    default: break;
                }
            }

            public bool CheckFinished()
            {
                // TODO implement
                return workInProgress;
            }
        }

        public class ConnectorClient
        {
            // cache
            IMyIntergridCommunicationSystem IGC;

            List<ConnectorInfo> knownInfos = new List<ConnectorInfo>();
            public bool Requesting { get; private set; } = false;
            public bool Finished { get; private set; } = false;

            // prop
            public List<ConnectorInfo> KnownInfos => knownInfos;
            public ConnectorClient(IMyIntergridCommunicationSystem IGC)
            {
                this.IGC = IGC;
                IGC.RegisterBroadcastListener(ConnectorInfo.ResponseTag);
            }

            public void TryToGetConnectorInfo(Vector3 currentPosition)
            {
                Requesting = true;
                Finished = false;
            }

            public void UpdateClient()
            {
                if (Requesting)
                {
                    IMyUnicastListener listener = IGC.UnicastListener;
                    while (listener != null && listener.HasPendingMessage)
                    {
                        var message = listener.AcceptMessage();
                        //EchoMessage(message);
                        if (message.Tag == ConnectorInfo.ResponseTag)
                        {
                            if (message.Data is string)
                            {
                                var str = message.Data as string;

                                var vec = str.Split(';')[0].Split(':')[1].ToString();

                                knownInfos = ConnectorInfo.ParseList(str);
                                Finished = true;
                                Requesting = false;
                            }
                        }
                    }
                }
                IGC.SendBroadcastMessage<string>(ConnectorInfo.RequestTag, "");
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

            public ConnectorInfo(string cname, Vector3 cposition, Quaternion crot, Vector3 capproachOffset)
            {
                name = cname;
                position = cposition;
                rotation = crot;
                approachOffset = capproachOffset;
            }

            public ConnectorInfo(IMyShipConnector connector)
            {
                name = connector.CustomName;
                position = connector.GetPosition();
                rotation = Quaternion.CreateFromRotationMatrix(connector.WorldMatrix);
                this.approachOffset = Vector3.Zero;
            }

            public string Stringify()
            {
                return $"{name}:{StringHelper.VectorStringify(position)}:{StringHelper.QuaternionStringify(rotation)}:{StringHelper.VectorStringify(approachOffset)}";
            }

            public static ConnectorInfo Parse(string connectorString)
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

        // Ship Status Recorder
        public class FlightRecorder
        {
            private readonly List<IMyGasTank> tanks;
            private List<FlightRecordEntry> entries = new List<FlightRecordEntry>();

            public FlightRecorder(List<IMyGasTank> tanks)
            {
                this.tanks = tanks;
            }

            private double GetCurrentTotalGas()
            {
                double sum = 0;
                foreach (var tank in tanks)
                {
                    sum += tank.Capacity * tank.FilledRatio;
                }
                return sum;
            }

            public void Record()
            {
                var r = new FlightRecordEntry(
                        GetCurrentTotalGas(),
                        DateTime.UtcNow
                    );
                entries.Add(r);
            }

            public new string ToString()
            {
                var str = "";
                foreach (var r in entries)
                {
                    str += r.ToString() + Environment.NewLine;
                }
                return str;
            }
        }

        public class FlightRecordEntry
        {
            public readonly double fuel;
            public readonly DateTime datatime;

            public FlightRecordEntry(double fuel, DateTime date)
            {
                this.fuel = fuel;
                this.datatime = date;
            }

            public new string ToString()
            {
                return $"{datatime.Ticks}:{fuel}";
            }
        }
    }
}