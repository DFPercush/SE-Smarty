// desmos
// time
// \left(\left(.5\cdot-9.81\cdot x^{2}\right)+\left(v\cdot\sin\left(a\cdot\frac{\pi}{180}\right)\cdot x\right)+h\right)\ 
// distance
// \ \frac{v\cos\left(x\cdot\frac{\pi}{180}\right)}{9.81}\ \left(v\sin\left(x\cdot\frac{\pi}{180}\right)+\sqrt{\left(v\sin\left(x\cdot\frac{\pi}{180}\right)\right)^{2}+2\cdot9.8h}\right)-d

/*
TODO:

*/

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
        #region mdk preserve
        //#################################################################
        // CONFIG

        // Calibration,
        // if things always seem to need the same amount of adjustment.
        double CalibrateRight = 0.0;
        double CalibrateForward = 0.0;

        // Block names
		const string AzimuthRotor = "Advanced Rotor 3x3";
        const string ElevationRotor = "Hinge 3x3";
        const string Gun = "Artillery"; // single block or group
        
        const string Antenna = "Antenna"; // optional
        const string RadioChannel = "smArty";
        const string IndicatorLight = "Interior Light";

        // LCD output echoes what is printed in terminal
        const string DisplayBlock = "LCD Panel";

        // Stow / storage position
        const float Stow_Angle_Elevation = 89;
        const float Stow_Angle_Azimuth = 0;

        // How long will output messages stay in terminal
        int messageTimeout = 10;


        // For blocks that have multiple displays like button panels or cockpits
        const int DisplayIndex = 0;

        const double aimSettledThreshold_SpeedRPM = 0.05; // rpm
        const double aimSettledThreshold_AngleDeg = 0.05;

        // How often to report status to targeting craft
        const int RadioUpdateInterval = 1000;


        // AmmoVelocities may apply to any weapon based on WeaponCore.
        // This is the barrel speed of each ammo variant.
        // There should be at least one number here.
        readonly double[] AmmoVelocities = { 125, 225, 325 };
        
        //const int ShotDelay = 60 * 8; // frames at 60 fps


        // GunLocalAxis defines the coordinate system of the gun model.
        // Don't worry about this unless the aiming is broken.
        // The Northwind 203mm Howitzer's barrel points along -Z.
        // If you are using another mod or a different model,
        // this might need to change.
        // If you need to recalibrate, you can use an argument of "orient"
        // to inspect the current position.
        // Best done at the south pole of Earthlike (0, about 61800, 0).
        // Create two GPS markers, one at the pole, and another at the same coordinates
        // plus a few hundred X. Line up the gun at the pole and point it at the other marker.
        // The readout should then say approximately (1, 0, 0).
        // Test 1 and -1 in each axis here until it reads right.
        readonly Vector3D GunLocalAxis = new Vector3D(0, 0, -1);
        //readonly Vector3D GunLocalUp = new Vector3D(-1, 0, 0);
        readonly Vector3D GunLocalUp = new Vector3D(0, 1, 0);

        // Same deal with cockpit
        readonly Vector3D cockpitLocalForward = new Vector3D(1, 0, 0);



        //#################################################################
        // CODE
        // edit only if you are 1337 h4x0r
        #endregion


        PID pidAzimuth = new PID
        {
            Kp = 0.5, //.3,
            Ki = 1.0, // 0.1
            Kd = 0.0,
            limMin = -3,
            limMax = 3,
            limMinInt = -1.0, //-0.5,
            limMaxInt = 1.0, //0.5,
            lowPass = 1.0,
            dt = 1.0 / 60,
            setpoint = 0
        };

        PID pidElevation = new PID
        {
            Kp = 0.5, //0.3,
            Ki = 1.0, //0.1,
            Kd = 0.0,
            limMin = -3,
            limMax = 3,
            limMinInt = -1.0, //-0.5,
            limMaxInt = 1.0, //0.5,
            lowPass = 1.0,
            dt = 1.0 / 60,
            setpoint = 0
        };


        const double pi = Math.PI;
        const double pi2 = pi / 2;
        const double d90 = pi2;
        const double deg2rad = Math.PI / 180.0;
        const double rad2deg = 180.0 / Math.PI;

        //bool aiming = false;
        //bool stowing = false;

        Vector3D targetLoc = new Vector3D();

        //double ammoVel = 350.0; // read from gun

        //SpaceEngineers.Game.EntityComponents.GameLogic.Discovery.
        //Sandbox.ModAPI.
        IMyMotorStator azimuthRotor;
        IMyMotorStator elevationHinge;
        IMyShipController cockpit;
        //IMySmallMissileLauncher gun;
        IMyUserControllableGun gun;
        List<IMyUserControllableGun> guns = new List<IMyUserControllableGun>();
        IMyLightingBlock light;
        IMyRadioAntenna antenna;
        IMyBroadcastListener listener;
        IMyTextSurface disp;

        bool steady = false;
        bool prevSteady = false;
        bool useLowAngle = false;

        //readonly Vector3D rotorLocalAxis = new Vector3D(0, 1, 0);
        //readonly Vector3D hingeLocalFacingAxis = new Vector3D(1, 0, 0);

        enum ProgramState
		{
            Offline,
            Aiming,
            Stowing,
            Idle,
			Firing,
            OutOfRange
		}
		ProgramState state = ProgramState.Offline;

        DateTime lastBroadcastTime = DateTime.Now;

        struct Message
		{
            public DateTime when;
            public string text;
		}
        Queue<Message> messages = new Queue<Message>();
        HashSet<long> notifyAddresses = new HashSet<long>();
        IMyGridTerminalSystem G;
        bool fireCommandGiven = false;
        //bool firing = false;


        void msg(string s)
		{
            messages.Enqueue(new Message { when = DateTime.Now, text = s });
            Print(s);
            //msg(messages.ElementAt(0).text);
		}
        void notify(string s)
		{
            msg(s);
            foreach (var addr in notifyAddresses)
			{
                IGC.SendUnicastMessage(addr, "smArty", s);
			}
		}


        public Program()
        {
            G = GridTerminalSystem;
            var conts = new List<IMyShipController>();
            G.GetBlocksOfType(conts);
            if (conts.Count > 0)
            {
                cockpit = conts[0];
            }
            azimuthRotor = G.GetBlockWithName(AzimuthRotor) as IMyMotorStator;
            elevationHinge = G.GetBlockWithName(ElevationRotor) as IMyMotorStator;

            var gunsGroup = G.GetBlockGroupWithName(Gun);
            if (gunsGroup != null)
			{
                gunsGroup.GetBlocksOfType(guns);
                if (guns.Count > 0)
				{
                    gun = guns[0];
				}
			}
            else
			{
                gun = G.GetBlockWithName(Gun) as IMyUserControllableGun; //IMySmallMissileLauncher;
                guns.Add(gun);
			}
            antenna = G.GetBlockWithName(Antenna) as IMyRadioAntenna;
            light = G.GetBlockWithName(IndicatorLight) as IMyLightingBlock;


            if ((azimuthRotor == null))
            {
                msg("Error: Missing azimuth rotor. Check name in script.");
            }
            if (elevationHinge == null)
            {
                msg("Error: Missing elevation rotor. Check name in script.");
            }
            if (cockpit == null)
            {
                msg("Error: No ship controller found. Build a cockpit, seat, or remote control.");
            }
            if (light == null)
			{
                msg("(optional) Operating without indicator light.");
			}
            if (gun == null)
			{
                msg("Error: Gun not found. Check name in script.");
			}
            if (antenna == null)
			{
                msg("(optional) Operating without antenna.");
			}
            else
			{
                listener = IGC.RegisterBroadcastListener(RadioChannel);
                if (listener != null) { msg("Broadcast listener enabled."); }
                else { msg("Warning: Broadcast listener unavailable."); }
			}

            var db = G.GetBlockWithName(DisplayBlock);
            if (db == null)
            {
                msg("(optional) Display block not found");
            }
            else
            {
                if (db is IMyTextSurface)
                {
                    disp = db as IMyTextSurface;
                    disp.ContentType = ContentType.TEXT_AND_IMAGE;
                }
                else if (db is IMyTextSurfaceProvider)
                {
                    var tsp = db as IMyTextSurfaceProvider;
                    if (DisplayIndex >= 0 && DisplayIndex < tsp.SurfaceCount)
                    {
                        disp = tsp.GetSurface(DisplayIndex);
                        disp.ContentType = ContentType.TEXT_AND_IMAGE;
                    }
                    else
                    {
                        msg("Warning: DisplayIndex out of bounds");
                    }
                }
            }

            Idle();
            msg("System online.");
        }
        public void Save()
        {
            //this.Storage == "";
        }

        FiringSolution fs = new FiringSolution();

        double azimuthErr = 1.0;
        double elevationErr = 1.0;

        FindZeroParams fz = new FindZeroParams();
        Vector3D tmpvv = new Vector3D();
        Vector3D tmprv = new Vector3D();


        double CalcDistanceRK4(double g, double rPlanet, double rStart, double rTarget, double barrelVel, double fireAngleRad, double dt)
        {
            //double prev = rPlanet;
            //double vx = barrelVel * cos(fireAngle);
            //double vy = barrelVel * sin(fireAngle);
            //double dt = 0.001; // 1ms
            double r = rPlanet;
            Vector2D pos; pos.X = 0; pos.Y =rPlanet;
            Vector2D startPos = pos;
            Vector2D down;
            Vector2D v;
            //vec2d vGravity;
            v.X = barrelVel * Math.Cos(fireAngleRad);
            v.Y = barrelVel * Math.Sin(fireAngleRad);
            double vr = v.Y;
            Vector2D accel;
            double t = 0;
            while ((vr > 0 || r > rTarget)) // && (r > rPlanet / 2.0))
            {
                //pos += v * dt;
                down.X = -pos.X;
                down.Y = -pos.Y;
                down.Normalize();
                accel = down * g;
                pos.X = rk4ss((double x, double y) => v.X, t, pos.X, t + dt);
                pos.Y = rk4ss((double x, double y) => v.Y, t, pos.Y, t + dt);
                v.X = rk4ss((double x, double y) => accel.X, t, v.X, t + dt);
                v.Y = rk4ss((double x, double y) => accel.Y, t, v.Y, t + dt);
                r = pos.Length();
                //vr = v.Dot(-down); // glm::dot(v, -down);
                vr = (v.X * -down.X) + (v.Y * -down.Y);
                t += dt;
            }

            //return (pi/2) - atan2(pos.y, pos.x);
            //return glm::length(pos - startPos);
            return (pos - startPos).Length();
        }

        void Recalc()
		{
            Recalc_parabolic();
            //Recalc_elliptical();
            //Recalc_RK4();
		}

        void Recalc_RK4()
		{
            //Recalc_parabolic();
            // Recalc_elliptical();
            double g = cockpit.GetNaturalGravity().Length();
            Vector3D pp;
            if (!cockpit.TryGetPlanetPosition(out pp)) { return; }
            var myPos = gun.WorldMatrix.Translation;
            FindZeroParams fz;
            fz.dx = 0.01 * deg2rad;
            fz.errStep = fz.dx;
            fz.iterationLimit = 100;
            fz.margin = 0.01 * deg2rad;


            double altitude;
            cockpit.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out altitude);
            double rPlanet = (myPos - pp).Length() - altitude;
            var ammoIndex = gun.GetProperty("WC_PickAmmo").As<Int64>().GetValue(gun);
            double barrelVelocity = (ammoIndex < AmmoVelocities.Length) ? AmmoVelocities[ammoIndex] : AmmoVelocities[0];

            fz.min = 0;
            fz.max = 90 * deg2rad;
            fz.guess = 22.5 * deg2rad;
            fs.elevationAngleLowRad = FindZero((fireAngle) =>
                CalcDistanceRK4(
                    g,
                    rPlanet,
                    (myPos - pp).Length(),
                    (targetLoc - pp).Length(),
                    barrelVelocity,
                    fireAngle,
                    0.01),
                fz);

            fz.min = 0;
            fz.max = 90 * deg2rad;
            fz.guess = 67.5 * deg2rad;
            fs.elevationAngleHighRad = FindZero((fireAngle) =>
                CalcDistanceRK4(
                    g,
                    rPlanet,
                    (myPos - pp).Length(),
                    (targetLoc - pp).Length(),
                    barrelVelocity,
                    fireAngle,
                    0.01),
                fz);

            fs.elevationAngleHighDeg = fs.elevationAngleHighRad * rad2deg;
            fs.elevationAngleLowDeg = fs.elevationAngleLowRad * rad2deg;
            // TODO: Set time of flight etc
		}

        void Recalc_elliptical()
		{
            if (!cockpit.TryGetPlanetPosition(out pp))
            {
                msg("Not on a planet.");
                return;
            }
            
            var myPos = gun.WorldMatrix.Translation;
            // pc* = (relative to) planet center *
            Vector3D pcMe = myPos - pp;
            Vector3D pcTarget = targetLoc - pp;
            Vector3D pcMeNorm = pcMe; pcMeNorm.Normalize();
            Vector3D pcTargetNorm = pcTarget; pcTargetNorm.Normalize();
            double pcTargetAngle = Math.Acos(pcMeNorm.Dot(pcTargetNorm));
            double rMe = pcMe.Length();
            double rTarget = pcTarget.Length();

            var gravAccel = cockpit.GetNaturalGravity().Length();
            // M=ar^2/G
            // GM = ar^2
            double mu = gravAccel * rMe * rMe;

            fz.dx = 1.0 / 60000;
            fz.guess = Math.PI / 8;
            fz.errStep = fz.dx;
            fz.iterationLimit = 100;
            
            //fz.min = 0;
            //fz.max = Math.PI / 2;

            fz.min = 0.05; // 0.017453; // 1 deg
            fz.max = (Math.PI / 2) - 0.05; // 0.017453; // 89 deg

            fz.margin = 1.0 / 60000;

            tmprv.X = 0;
            tmprv.Y = rMe;
            tmprv.Z = 0;

            Zfn TestLaunchAngle = (launchAngle) =>
            {
                tmpvv.X = fs.barrelVelocity * Math.Cos(launchAngle);
                tmpvv.Y = fs.barrelVelocity * Math.Sin(launchAngle);
                tmpvv.Z = 0;
                var orb = OrbitAE.FromPosVel(tmprv, tmpvv, mu);
                //Echo("---");
                //Echo($"test launch angle {launchAngle*rad2deg:0.00}");
                //Echo($"orb.a = {orb.a}");
                //Echo($"orb.e = {orb.e}");
                //double pcTestAngle = 2.0 * FindCollision(orb.a, orb.e, rTarget);
                //double pcTestAngle = 2.0 * FindCollision(orb.a, orb.e, rTarget);
                //Echo($"pcTestAngle = {pcTestAngle*rad2deg:0.00}");

                // pca = planet centric angle
                double pcaTarget = FindCollision(orb.a, orb.e, rTarget);
                double pcaMe = FindCollision(orb.a, orb.e, rMe);

                //return pcTestAngle - pcTargetAngle;
                return pcaTarget + pcaMe - pcTargetAngle;
            };

            var ammoIndex = gun.GetProperty("WC_PickAmmo").As<Int64>().GetValue(gun);
            double barrelVelocity = (ammoIndex < AmmoVelocities.Length) ? AmmoVelocities[ammoIndex] : AmmoVelocities[0];
            fs.barrelVelocity = barrelVelocity;
            Echo($"barrelVelocity = {barrelVelocity}");
            Echo($"pcTargetAngle = {pcTargetAngle*rad2deg:0.00}");
            fs.OutOfRange = false;
            fs.elevationAngleLowRad = FindZeroBin(TestLaunchAngle, fz);
            fs.elevationAngleLowDeg = fs.elevationAngleLowRad * 180.0 / Math.PI;
            
            fz.guess = Math.PI * 3 / 8;
            fz.min = 0.05; // 0.017453; // 1 deg
            fz.max = (Math.PI / 2) - 0.05; // 0.017453; // 89 deg
            //fz.margin = 1.0 / 60000;
            //fz.errStep = 1.0 / 60000;
            //fz.iterationLimit = 100;
            fs.elevationAngleHighRad = FindZeroBin(TestLaunchAngle, fz);
            fs.elevationAngleHighDeg = fs.elevationAngleHighRad * 180.0 / Math.PI;
            //fs.OutOfRange // TODO: <--
        }

        //*********************
        void Recalc_parabolic()
        {
            var myPos = gun.WorldMatrix.Translation;
            Vector3D pp, toTarget, toTargetHoriz;
            if (!cockpit.TryGetPlanetPosition(out pp))
            {
                msg("Not on a planet.");
                return;
            }
            // pc* = (relative to) planet center *
            Vector3D pcMe = myPos - pp;
            var alt = pcMe.Length();
            var up = myPos - pp;
            up.Normalize();

            //Vector3D pcTarget = targetLoc - pp;
            //double pcTargetAngle = pcMe.Dot(pcTarget)


            var targetAlt = (targetLoc - pp).Length();
            var targetUp = targetLoc - pp;
            targetUp.Normalize();
            var altDiff = targetAlt - alt;
            //fs = CalcFiringSolution(ammoVel, (targetLoc - (altDiff * targetUp) - myPos).Length(), altDiff, cockpit.GetNaturalGravity().Length());

            var ammoIndex = gun.GetProperty("WC_PickAmmo").As<Int64>().GetValue(gun);
            double barrelSpeed = (ammoIndex < AmmoVelocities.Length) ? AmmoVelocities[ammoIndex] : AmmoVelocities[0];

            toTarget = targetLoc - myPos;
            toTargetHoriz = toTarget - (toTarget.Dot(up) * up);

            double planetCentricAngle = Math.Acos(up.Dot(targetUp));
            double arcLen = (myPos - pp).Length() * planetCentricAngle;
            //fs = CalcFiringSolution(barrelSpeed, toTargetHoriz.Length(), altDiff, -cockpit.GetNaturalGravity().Length());
            //fs = CalcFiringSolution(barrelSpeed, toTargetHoriz.Length(), altDiff, cockpit.GetNaturalGravity().Length());
            fs = CalcFiringSolution(barrelSpeed, arcLen, altDiff, cockpit.GetNaturalGravity().Length());

        }
        // ******************************/

        double[] dargs = new double[10];
        string[] sargs = new string[10];

        bool Parse(string args, string types, char sep = ' ')
		{
            types = " " + types;
            //msg("Parse() called");
            var argv = args.Split(sep);
            //msg($"{argv.Length} parameters");
            if (argv.Length != types.Length)
			{
                msg("Bad argument parameters");
                return false;
			}
            for (int i = 1; i < types.Length; i++)
			{
                //msg($"in [{i}]: {argv[i]}");
                switch (types[i])
				{
                    case 'd':
                        if (!double.TryParse(argv[i], out dargs[i]))
						{
                            return false;
						}
                        //msg($"[{i}] = {dargs[i]}");
                        break;
                    case 's':
                        sargs[i] = argv[i];
                        break;
                    default:
                        msg("Bug: Unknown argument type in Parse()");
                        break;
				}
			}
            return true;
		}

        void targetGPS(string arg)
		{

            myPos = gun.WorldMatrix.Translation;
            cockpit.TryGetPlanetPosition(out pp);
            up = myPos - pp; up.Normalize();
            toTarget = targetLoc - myPos;
            forward = toTarget; forward.Normalize();
            right = toTarget.Cross(up); right.Normalize();

            var gp = arg.Split(':');
            if (gp.Length < 6)
            {
                msg("Error: Bad GPS link format");
                return;
            }
            targetLoc.X = double.Parse(gp[2]);
            targetLoc.Y = double.Parse(gp[3]);
            targetLoc.Z = double.Parse(gp[4]);
            targetLoc += CalibrateRight * right;
            targetLoc += CalibrateForward * forward;
            Recalc();
            //aiming = true;
            state = ProgramState.Aiming;
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
            //if (Math.Abs(fs.elevationAngleDeg - 45) < 0.1)
            if (fs.OutOfRange)
            {
                // TODO: Make a program state
                Idle();
                state = ProgramState.OutOfRange;
                notify("Out of range!");
            }
            else
            {
                notify("Target data received.");
            }
            //notify($"Elev {fs.elevationAngleDeg:0.###}");
        }

        //int nShotsQueued = 0;
        //int prevRoundsLoaded = 0;
        //int shotDelayCountdown = 0;
        public void Main(string arg, UpdateType upd)
		{
            if (light != null) { light.Enabled = false; }
            if (arg.Length > 0)
			{
                Main2(arg, upd);
			}
            else
			{
                if (listener != null && listener.HasPendingMessage)
				{
                    var radioMsg = listener.AcceptMessage();
                    string radioArg = radioMsg.As<string>();
                    if (radioArg == "unlink")
					{
                        string wasLinked = notifyAddresses.Remove(radioMsg.Source) ? "unlinked" : "not linked";
                        IGC.SendUnicastMessage(radioMsg.Source, "smArty", $"{wasLinked} {IGC.Me}");
					}
                    else if (radioArg == "link")
					{
                        notifyAddresses.Add(radioMsg.Source);
                        IGC.SendUnicastMessage(radioMsg.Source, "smArty", $"linked {IGC.Me}");
					}
                    else
					{
                        notifyAddresses.Add(radioMsg.Source);
                        // command from ###:
                        msg($"{radioMsg.Source % 1000}: {radioArg}");
                        Main2(radioArg, UpdateType.IGC);
					}
				}
                else
				{
                    Main2(arg, upd);
				}


                // Send status to targeting craft
                if (lastBroadcastTime + TimeSpan.FromMilliseconds(RadioUpdateInterval) < DateTime.Now)
				{
                    BroadcastStatus();
				}
			}
            RunLCD();
            if (light != null) { light.Enabled = true; }
        }
        void RunLCD()
        {
            //Echo("RunLCD()");
            if (disp != null)
            {
                //Echo("RunLCD() disp not null");
                disp.WriteText(printBuf, false);
            }
            printBuf.Clear();
        }

        Vector3D myPos, pp, toTarget, toTargetHoriz, forward, right, up;
        List<TerminalActionParameter> reloadParams = new List<TerminalActionParameter>();

        public void Main2(string arg, UpdateType upd)
        {
            if ((azimuthRotor == null))
            {
                msg("Error: Missing azimuth rotor. Check name in script.");
                return;
            }
            if (elevationHinge == null)
            {
                msg("Error: Missing elevation rotor. Check name in script.");
                return;
            }
            if (cockpit == null)
            {
                msg("Error: No ship controller found. Build a cockpit, seat, or remote control.");
                return;
            }
            if (gun == null)
            {
                msg("Error: Gun not found. Check name in script.");
                return;
            }

            try
            {
                //Echo("Properties:");
                //var lst = new List<ITerminalProperty>();
                //gun.GetProperties(lst);
                //foreach(var p in lst) { Echo($"{p.Id}: {p.TypeName}"); }
                //
                //Echo("Actions:");
                //var lac = new List<ITerminalAction>();
                //gun.GetActions(lac);
                //foreach (var a in lac) { Echo($"{a.Name}"); }
                //return;
                //
                //Echo("Ammo is " + gun.GetProperty("WC_PickAmmo").TypeName);
                //Echo("Ammo int64 is " + gun.GetProperty("WC_PickAmmo").As<Int64>().GetValue(gun).ToString());
                //return;


                Print(state.ToString() + ' ' + Spinner());

                var myPos = gun.WorldMatrix.Translation;
                if (!cockpit.TryGetPlanetPosition(out pp))
                {
                    msg("Not on a planet.");
                    return;
                }
                var alt = (myPos - pp).Length();
                up = myPos - pp;
                up.Normalize();
                toTarget = targetLoc - myPos;
                forward = toTarget; forward.Normalize();
                right = toTarget.Cross(up); right.Normalize();

				if (arg == "stop")
				{
					Stop();
					return;
				}
				else if (arg == "idle")
				{
					Idle();
				}
                else if (arg == "hi" || arg == "high")
				{
                    useLowAngle = false;
                    notify("Using high angle.");
				}
                else if (arg == "low" || arg == "low")
				{
                    useLowAngle = true;
                    notify("Using low angle");
				}
				else if (arg.StartsWith("GPS:"))
				{
					targetGPS(arg);
					//msg($"tt {toTarget.Length():0}");
					//msg($"tth {toTargetHoriz.Length():0}");
					//msg($"fs.el {fs.elevationAngleDeg:0.###}");
					//msg($"fs.dist {fs.targetDistance:0.###}");
					//msg($"fs.hofs {fs.targetHeightOffset:0.###}");
					//return;
				}
				else if (arg == "orient")
				{
					msg("gun forward:");
					msg($"{mul(gun.WorldMatrix.GetOrientation(), GunLocalAxis)}");
					msg("gun up:");
					msg($"{mul(gun.WorldMatrix.GetOrientation(), GunLocalUp)}");
					msg("cockpit:");
					msg($"{mul(cockpit.WorldMatrix.GetOrientation(), cockpitLocalForward)}");
					//msg(mul(gun.WorldMatrix.GetOrientation(), GunLocalAxis).ToString());
					//msg("");
					//msg(div(GunLocalAxis, G.GetBlockWithName("203mm Howitzer").WorldMatrix.GetOrientation()).ToString());
					return;
				}
				else if (arg == "stow")
				{
                    //stowing = true;
                    //gun.SetValueBool("WC_Shoot", false);
                    foreach (var g in guns) { g.SetValueBool("WC_Shoot", false); }
                    state = ProgramState.Stowing;
					if (light != null)
					{
						light.Color = Color.Yellow;
						light.BlinkIntervalSeconds = 0.5f;
						light.BlinkLength = 50;
					}
				}
				else if (arg.StartsWith("open"))
				{
                    //if (!Parse(arg, "d")) { msg("'fire' expects number of shots"); }
                    //msg("'fire' dargs:");
                    //msg($"{dargs[0]}");
                    //msg($"{dargs[1]}");
                    //msg($"{dargs[2]}");
                    //msg($"{dargs[3]}");
                    //nShotsQueued += (int)dargs[1];
                    //msg($"Queued {dargs[1]} rounds.");

                    fireCommandGiven = true;
                    //if (state == ProgramState.Aiming)
					//{
                    //    state = ProgramState.Firing;
					//}
                    notify("Opening fire!");
				}
                else if (arg.StartsWith("cease"))
				{
                    //gun.Shoot = false;
                    //gun.ApplyAction("Shoot_Off");
                    if (state == ProgramState.Firing)
					{
                        state = ProgramState.Aiming;
					}
                    fireCommandGiven = false;
                    //gun.SetValueBool("WC_Shoot", false);
                    foreach (var g in guns) { g.SetValueBool("WC_Shoot", false); }
                    notify("Ceasing fire.");
				}
				else if (arg.StartsWith("ofs "))
				{
                    var ofs = arg.Split(' ');
                    if (ofs.Length < 4)
					{
                        msg("ofs requires x y z");
					}
                    else
					{
                        targetLoc.X += double.Parse(ofs[1]);
                        targetLoc.Y += double.Parse(ofs[2]);
                        targetLoc.Z += double.Parse(ofs[3]);
					}
				}
                else if (arg.StartsWith("right"))
				{
                    //double moveright;
                    //if (double.TryParse(arg.Substring(6), out moveright))
                    //{
                    //    targetLoc += right * moveright;
                    //}
                    if (!Parse(arg, "d")) { notify("'right' expects a distance"); return; }
                    targetLoc += right * dargs[1];
                    Recalc();
                    msg($"Adjust right {dargs[1]} m");
				}
                else if (arg.StartsWith("left"))
				{
                    if (!Parse(arg, "d")) { notify("'left' expects a distance"); return; }
                    targetLoc -= right * dargs[1];
                    Recalc();
                    msg($"Adjust left {dargs[1]} m");
				}
                else if (arg.StartsWith("forward"))
				{
                    if (!Parse(arg, "d")) { notify("'forward' expects a distance"); return; }
                    targetLoc += forward * dargs[1];
                    Recalc();
                    msg($"Adjust forward {dargs[1]} m");
				}
                else if (arg.StartsWith("back"))
				{
                    if (!Parse(arg, "d")) { notify("'back' expects a distance"); return; }
                    targetLoc -= forward * dargs[1];
                    Recalc();
                    msg($"Adjust back {dargs[1]} m");
				}
                else if (arg.StartsWith("ammo "))
				{
                    if (!Parse(arg, "d")) { notify("'ammo' requires a number for type of ammo"); return; }
                    foreach (var g in guns)
					{
                        g.SetValue<Int64>("WC_PickAmmo", (long)dargs[1]);
                        //ITerminalAction
                        //g.ApplyAction("Force Reload");
                        //g.ApplyAction("Force Reload", reloadParams);
                        //g.ApplyAction("Force_Reload", reloadParams);
                        //g.ApplyAction("Force_Reload", reloadParams);
					}
                    notify($"Ammo type {(long)dargs[1]}.");
				}

                prevSteady = steady;
                steady = settled();

                //if (stowing)
                //if (state == ProgramState.Stowing)
                //{
                //    targetLoc = cockpit.WorldMatrix.Translation +
                //            (1000 * mul(cockpit.WorldMatrix.GetOrientation(), cockpitLocalForward)) +
                //            (20 * up);
                //}
                //if (aiming || stowing)
                if (state == ProgramState.Aiming || state == ProgramState.Firing) // || state == ProgramState.Stowing)
                {
                    azimuthRotor.RotorLock = false;
                    elevationHinge.RotorLock = false;
                    if (fs.OutOfRange)
					{
                        Idle();
                        return;
					}
                    Runtime.UpdateFrequency = UpdateFrequency.Update1;
                    //Print("Running...");
                    //var aim = mul(elevationHinge.Top.WorldMatrix.GetOrientation(), hingeLocalFacingAxis);
                    var aim = mul(gun.WorldMatrix.GetOrientation(), GunLocalAxis);
                    var aimVert = aim.Dot(up) * up;
                    var aimHoriz = aim - aimVert;
                    toTarget = targetLoc - myPos;
                    var toTargetVert = toTarget.Dot(up) * up;
                    //toTargetHoriz = toTarget - toTarget.Dot(up);
                    toTargetHoriz = toTarget - toTargetVert;


                    //var azimuthErr = AngleBetweenDeg(aimHoriz, toTargetHoriz);
                    var inverted = Math.Sign(up.Dot(mul(gun.WorldMatrix.GetOrientation(), GunLocalUp)));
                    //var inverted = Math.Sign(up.Dot(div(GunLocalUp, gun.WorldMatrix.GetOrientation())));
                    //Print($"inv {inverted}");
                    //Print($"inv sign: {inverted}");
                    //azimuthErr = AngleDiffDeg(aimHoriz, toTargetHoriz, up) * inverted;
                    azimuthErr = AngleDiffDeg(aimHoriz, toTargetHoriz, up);
                    pidAzimuth.Update(azimuthErr);
                    Unlimit();
                    azimuthRotor.TargetVelocityRPM = (float)pidAzimuth.Output;

                    if (inverted > 0)
                    {
                        //elevationErr = ((Math.Acos(aimVert.Length()) * 180.0 / Math.PI) - fs.elevationAngleDeg) * inverted;
                        var selectAngle = useLowAngle ? fs.elevationAngleLowDeg : fs.elevationAngleHighDeg;
                        //elevationErr = ((Math.Acos(aimVert.Length()) * 180.0 / Math.PI) - selectAngle);
                        elevationErr = -((Math.Asin(aimVert.Length()) * rad2deg) - selectAngle);

                        //double targetHingeAngle = 90 - (useLowAngle ? fs.elevationAngleLowDeg : fs.elevationAngleHighDeg);
                        //Print($"targetElevationAngle = {targetElevationAngle:0.##}");
                        //elevationErr = ((Math.Acos(aimVert.Length()) * 180.0 / Math.PI) - selectAngle); // * inverted;

                        pidElevation.Update(elevationErr);
                        elevationHinge.TargetVelocityRPM = (float)pidElevation.Output;
                    }
                    else
					{
                        elevationHinge.TargetVelocityRPM = (float)pidElevation.limMax;
					}

                    if (fireCommandGiven && (state != ProgramState.Firing))
                    {
                        //gun.SetValueBool("WC_Shoot", true);
                        foreach (var g in guns) { g.SetValueBool("WC_Shoot", true); }
                        state = ProgramState.Firing;
                        //firing = true;
                    }

                    if (steady != prevSteady)
					{
                        notify(steady ? "Aim good" : "Adjusting aim");
					}

                    if (light != null)
                    {
                        if (state == ProgramState.Aiming)
                        {
                            //if (stowing)
                            //if (state == ProgramState.Stowing)
                            //{
                            //    light.Color = Color.Red;
                            //    light.BlinkIntervalSeconds = 0;
                            //}
                            //else
                            if (readyToFire())
                            {
                                light.Color = Color.Green;
                                light.BlinkIntervalSeconds = 0;
                            }
                            else
                            {
                                light.Color = Color.Yellow;
                                light.BlinkIntervalSeconds = 0.5f;
                                light.BlinkLength = 50f;
                            }
                        }
                        else if (state == ProgramState.Firing)
						{
                            light.BlinkIntervalSeconds = 0.5f;
                            light.BlinkLength = 50;
                            if (settled())
							{
                                light.Color = Color.Green;
							}
                            else
							{
                                light.Color = Color.Yellow;
							}
						}
                    }




                    //Print($"{nShotsQueued} shots in queue.");

                    //// ShootOnce() doesn't work, so how to detect if the gun has fired?
                    //// Look at delta ammo count
                    //int roundsLoaded = 0;
                    //if (gun.GetInventory().ItemCount > 0)
                    //{
                    //    var stack = gun.GetInventory().GetItemAt(0);
                    //    if (stack.HasValue)
                    //    {
                    //        roundsLoaded = stack.Value.Amount.ToIntSafe();
                    //    }
                    //    else
                    //	{
                    //        roundsLoaded = 0;
                    //	}
                    //}
                    //else
                    //{
                    //    roundsLoaded = 0;
                    //}
                    //if (roundsLoaded < prevRoundsLoaded)
                    //{
                    //    nShotsQueued--;
                    //    if (nShotsQueued < 0) { nShotsQueued = 0; }
                    //}
                    //prevRoundsLoaded = roundsLoaded;

                    //if (shotDelayCountdown > 0) { shotDelayCountdown--; }
                    //if (shotDelayCountdown == 0 && nShotsQueued > 0 && readyToFire())
                    //{
                    //    //Print("Firing...");
                    //    //gun.Shoot = true;
                    //    gun.ApplyAction("Shoot");
                    //    nShotsQueued--;
                    //    shotDelayCountdown = ShotDelay;
                    //}

                    //var dbgToTargetNormal = toTarget;
                    //dbgToTargetNormal.Normalize();
                    //msg($"myPos: {myPos}");
                    //msg($"pp: {pp}");
                    //msg($"up: {up}");
                    //msg($"tn X: {dbgToTargetNormal.X}");
                    //msg($"tn Y: {dbgToTargetNormal.Y}");
                    //msg($"tn Z: {dbgToTargetNormal.Z}");
                    //msg($"Aim horiz X: {aimHoriz.X}");
                    //msg($"Aim horiz Y: {aimHoriz.Y}");
                    //msg($"Aim horiz Z: {aimHoriz.Z}");

                    
                    Recalc();


                    var dispAngle = useLowAngle ? fs.elevationAngleLowDeg : fs.elevationAngleHighDeg;
                    Print($"Elev {dispAngle:0.###}");
                    Print($"Azim off {azimuthErr:0.###} deg");
                    Print($"Elev off {elevationErr:0.###} deg");

                }
                else if (state == ProgramState.Stowing)
				{
                    Runtime.UpdateFrequency = UpdateFrequency.Update1;
                    azimuthRotor.RotorLock = false;
                    elevationHinge.RotorLock = false;
                    Print("Stowing");

                    //azimuthErr = ((azimuthRotor.Angle * 180.0/pi + 180) % 360.0) - 180;

                    Unlimit();

                    azimuthErr = (((azimuthRotor.Angle * rad2deg) - Stow_Angle_Azimuth + 180) % 360.0) - 180;
                    pidAzimuth.setpoint = 0;
                    pidAzimuth.Update(azimuthErr);
                    azimuthRotor.TargetVelocityRPM = (float)pidAzimuth.Output;

                    //elevationErr = -(89 - (elevationHinge.Angle * 180 / pi));
                    elevationErr = (elevationHinge.Angle * 180 / pi) - Stow_Angle_Elevation;
                    pidElevation.setpoint = 0;
                    pidElevation.Update(elevationErr);
                    elevationHinge.TargetVelocityRPM = (float)pidElevation.Output;




                    Print($"Azimuth err: {azimuthErr:0.###} deg");
                    Print($"Elevation err: {elevationErr:0.###} deg");

                    if (settled())
					{
                        Idle();
					}
				}
                else if (state == ProgramState.Offline)
                {
                    Stop();
                }
                else if (state == ProgramState.Idle)
				{
                    //Print("Idle");
				}
                Print($"{100.0 * Runtime.CurrentInstructionCount / Runtime.MaxInstructionCount:0}% CPU");
            }
            catch (Exception e)
            {
                msg(e.Message + e.StackTrace);
                Idle();
                //azimuthRotor.TargetVelocityRPM = 0;
                //elevationHinge.TargetVelocityRPM = 0;
            }

            //Print($"{messages.Count} messages");
            //for (int imsg = 0; imsg < messages.Count; imsg++)
            for (int imsg = messages.Count - 1; imsg >= 0; imsg--)
			{
                Print(messages.ElementAt(imsg).text);
			}
            while (messages.Count > 0 && messages.Peek().when + TimeSpan.FromSeconds(messageTimeout) < DateTime.Now)
			{
                //Print("removed msg");
                messages.Dequeue();
			}
        }
        
        char[] SpinnerChars = { '/', '-', '\\', '|' };
        int spinnerIndex = 0;
        char Spinner()
		{
            spinnerIndex = (spinnerIndex + 1) % 4;
            return SpinnerChars[spinnerIndex];
		}
        bool readyToFire()
        {
            //return aiming && (!stowing) && settled();
            return (state == ProgramState.Aiming) && settled();
        }
        bool settled()
		{
            //Print($"azim {Math.Abs(azimuthErr):0.####}");
            //Print($"elev {Math.Abs(elevationErr):0.####}");
            //Print($"threshold {angleThresholdDeg}");
            //Print($"dAzim {azimuthRotor.TargetVelocityRPM:0.####}");
            //Print($"dElev {elevationHinge.TargetVelocityRPM:0.####}");
            //Print($"threshold {aimSettledThreshold}");
            return (Math.Abs(azimuthErr) < aimSettledThreshold_AngleDeg) &&
                (Math.Abs(elevationErr) < aimSettledThreshold_AngleDeg) &&
                (azimuthRotor.TargetVelocityRPM < aimSettledThreshold_SpeedRPM) &&
                (elevationHinge.TargetVelocityRPM < aimSettledThreshold_SpeedRPM);
        }
        void BroadcastStatus()
		{
            foreach (var addr in notifyAddresses)
			{
                IGC.SendUnicastMessage(addr, RadioChannel, $"state:{state}:{steady}");
			}
		}

        void Stop()
		{
            //gun.SetValueBool("WC_Shoot", false);
            foreach (var g in guns) { g.SetValueBool("WC_Shoot", false); }
            azimuthRotor.TargetVelocityRPM = 0;
            elevationHinge.TargetVelocityRPM = 0;
            azimuthRotor.RotorLock = true;
            elevationHinge.RotorLock = true;
            if (light != null)
			{
                //msg("blink Stop()"); //!rm
                light.Color = Color.Red;
                light.BlinkIntervalSeconds = 0;
			}
            Runtime.UpdateFrequency = UpdateFrequency.None;
            state = ProgramState.Offline;
            BroadcastStatus();
            // TODO: Broadcast offline to targeter
        }
        void Idle()
		{
            Stop();
            if (light != null)
			{
                //msg("blink Idle()"); //!rm
                light.BlinkIntervalSeconds = 2;
                light.BlinkLength = 84;
			}
            Runtime.UpdateFrequency = UpdateFrequency.Update100;
            azimuthRotor.RotorLock = true;
            elevationHinge.RotorLock = true;
            state = ProgramState.Idle;
		}

        public struct FiringSolution
        {
            public double barrelVelocity;
            public double elevationAngleHighRad;
            public double elevationAngleHighDeg;
            public double elevationAngleLowRad;
            public double elevationAngleLowDeg;
            
            public double targetDistance;
            public DateTime splashTime;
            public double targetHeightOffset;
            //public double flightDuration; // TODO: not being set
            public bool OutOfRange;
        }
        //public FiringSolution CalcFiringSolution(double barrelVelocity, Vector3D Vector3D targetPos, double targetHeightOffset, double gravity)
        
        //*************************
        public FiringSolution CalcFiringSolution(double barrelVelocity, double targetDistance, double targetHeightOffset, double gravity)
        {
            /***************
            const acceleration a
            Vy = a*t + V0*sin(θ)
            y(t) = 1/2*a*t^2 + V0*sin(θ)*t + y0
            tq = ^ quadratic formula ^
            qa = 0.5*g
            qb = V0*sin(θ)
            qc = y0
            V0*cos(θ) * (-(V0*sin(θ)) + sqrt((V0*sin(θ))^2 - 4*0.5*g*y0)) / 2*.5*g = 1/2*a*t^2 + V0*sin(θ)*t + y0
            V0*cos(θ) * (-(V0*sin(θ)) + sqrt((V0*sin(θ))^2 - 4*0.5*g*y0)) / g = 1/2*a*t^2 + V0*sin(θ)*t + y0
            -(V0*sin(θ)*t) + (V0*cos(θ) * (-(V0*sin(θ)) + sqrt((V0*sin(θ))^2 - 4*0.5*g*y0)) / g) = 1/2*a*t^2 + y0
            -(V0*sin(θ)*t) + V0^2*sin(2θ)/2 + sqrt((V0*sin(θ))^2 - 4*0.5*g*y0)) / g) = 1/2*a*t^2 + y0
            solve for θ ...lol


            D = V * cos(θ) * tq
            cos(θ) = D / (V * tq)

            cos(θ) = D / (V *   (-V0*sin(θ) + Sqrt(b*b - 4*a*c)) / (2 * a))
            sin(θ+pi2) = D / (V * (-V0*sin(θ) + Sqrt(b*b - 4*a*c)) / (2 * a))




            θ = arccos(D / (V * tq))
            θ = arccos(D / (V * (-V0*sin(θ) + Sqrt(b*b - 4*a*c)) / (2 * a)))
            *********************/

            //var g = gravity_gs * -9.81;
            var f = new FiringSolution();
            f.barrelVelocity = barrelVelocity;
            f.targetDistance = targetDistance;
            f.targetHeightOffset = targetHeightOffset;

            var y0 = -targetHeightOffset;


            //double tq = QuadraticFormulaRealMax(0.5*g, barrelVelocity * Math.Sin)


            //double angmin = 0;
            ////double angmax = Math.PI / 4;
            //double angmax = Math.PI / 2;
            //double d90 = Math.PI / 2;
            //int itcount = 0;
            //while (angmax - angmin > 0.0001) // .01 deg
            //{
            //    itcount++;
            //    f.elevationAngleRad = (angmin + angmax) / 2.0;
            //
            //    ////Print($"{f.targetDistance:0.}m / {f.flightDuration:0.#}s");
            //    ////f.flightDuration = QuadraticFormulaRealMax(0.5 * gravity, barrelVelocity * Math.Sin(f.elevationAngleRad), -targetHeightOffset);
            //    //f.flightDuration = QuadraticFormulaRealMax(0.5 * gravity, barrelVelocity * Math.Sin(d90 - f.elevationAngleRad), -targetHeightOffset);
            //    //f.targetDistance = barrelVelocity * Math.Cos(d90 - f.elevationAngleRad) * f.flightDuration;
            //    ////f.targetDistance = barrelVelocity * Math.Cos(f.elevationAngleRad) * f.flightDuration;
            //    //if (f.targetDistance > targetDistance) { angmax = f.elevationAngleRad; }
            //    //else { angmin = f.elevationAngleRad; }
            //
            //    //var vsin0 = barrelVelocity * Math.Sin(f.elevationAngleRad);
            //    //var vcos0 = barrelVelocity * Math.Cos(f.elevationAngleRad);
            //    //f.targetDistance = (vcos0 / gravity) *
            //    //    (vsin0 + Math.Sqrt((vsin0 * vsin0) + (2 * gravity * y0)));
            //    //var dd = 
            //    f.targetDistance = calcDist(barrelVelocity, f.elevationAngleRad, gravity, y0);
            //    double dd = calcDist(barrelVelocity, f.elevationAngleRad + 0.01, gravity, y0);
            //    var dir = Math.Sign(dd - f.targetDistance);
            //    if ((f.targetDistance - targetDistance) * dir > 0)
            //	{
            //
            //	}
            //}
            //f.elevationAngleRad = 90.0 - f.elevationAngleRad;
            //f.elevationAngleDeg = f.elevationAngleRad * 180.0 / Math.PI;

            var p = new FindZeroParams();
            p.dx = 0.01;
            p.errStep = 0.1;
            p.guess = 65 * deg2rad;
            p.iterationLimit = 100;
            p.margin = 0.0001 * deg2rad;
            p.max = d90;
            p.min = 0;
            //msg($"vel {f.barrelVelocity}");
            //msg($"gravity {gravity}");
            //msg($"hofs {-targetHeightOffset}");
            //msg($"t.dist {targetDistance:0.##}");


            double angle1, angle2;

            angle1 = FindZero((x) => calcDist(f.barrelVelocity, x, gravity, -targetHeightOffset) - targetDistance, p);
            if (double.IsNaN(angle1))
            {
                f.OutOfRange = true;
            }
            else
            {
                f.OutOfRange = false;
                p.guess = (90 * deg2rad) - angle1;
                //var toTargetAngle = Math.Abs(Math.Atan2(toTarget.Y, toTarget.X));
                //var toTargetAngle = Math.Atan(toTarget.Y, toTarget.X));
                var toTargetAngle = Math.Atan2(f.targetHeightOffset, f.targetDistance);
                var reflectAngle = (d90 + toTargetAngle) / 2.0;
                p.guess = (reflectAngle - (angle1 - reflectAngle));
                //msg($"tta {toTargetAngle:0.##}");
                //msg($"reflect {reflectAngle:0.##}");
                //msg($"pg {p.guess:0.##}");
                //Echo($"vel {f.barrelVelocity}");
                angle2 = FindZero((x) => calcDist(f.barrelVelocity, x, gravity, -targetHeightOffset) - targetDistance, p);
                f.elevationAngleHighRad = Math.Max(angle1, angle2);
                f.elevationAngleLowRad = Math.Min(angle1, angle2);
                f.elevationAngleHighDeg = f.elevationAngleHighRad * rad2deg;
                f.elevationAngleLowDeg = f.elevationAngleLowRad * rad2deg;
            }

            // Something's wrong with the this function
            //if (calcElevationAngle(f.barrelVelocity, gravity, targetDistance, targetHeightOffset, out angle1, out angle2))
            //{
            //    f.elevationAngleHighRad = Math.Max(angle1, angle2);
            //    f.elevationAngleLowRad = Math.Min(angle1, angle2);
			//	f.elevationAngleHighDeg = f.elevationAngleHighRad * rad2deg;
			//	f.elevationAngleLowDeg = f.elevationAngleLowRad * rad2deg;
			//}
			//else
            //{
            //    f.OutOfRange = true;
            //}

            return f;
        }
        // ***********************/

        // Parabolic trajectory
        double calcDist(double v, double theta, double g, double y0)
		{
            var vsin0 = v * Math.Sin(theta);
            var vcos0 = v * Math.Cos(theta);
            return (vcos0 / g) * (vsin0 + Math.Sqrt((vsin0 * vsin0) + (2 * g * y0)));
        }
        //public static double QuadraticFormulaRealMax(double a, double b, double c)
        //{
        //    double sqr = (b * b) - (4 * a * c);
        //    if (sqr < 0 || a == 0) { return 0; }
        //    double root = Math.Sqrt(sqr);
        //    double r1 = (-b - root) / (2 * a);
        //    double r2 = (-b + root) / (2 * a);
        //    return Math.Max(r1, r2);
        //    //return (-b + Math.Sqrt(sqr)) / (2 * a);
        //    //return (-b - Math.Sqrt(sqr)) / (2 * a);
        //}


        StringBuilder printBuf = new StringBuilder();
        void Print(string s)
		{
            Echo(s);
            printBuf.Append(s);
            printBuf.Append("\r\n");
		}


        class PID
        {
            /* Controller gains */
            public double Kp;
            public double Ki;
            public double Kd;

            /* Derivative low-pass filter time constant */
            public double lowPass;

            /* Output limits */
            public double limMin;
            public double limMax;

            /* Integrator limits */
            public double limMinInt;
            public double limMaxInt;

            /* Sample time (in seconds) */
            public double dt;

            public double setpoint;
            //public double lowPassOut;

            /* Controller "memory" */
            // TODO: make private again
            public double integrator;
            public double prevError;            /* Required for integrator */
            public double differentiator;
            public double prevMeasurement;      /* Required for differentiator */
            //private double prevOut;

            /* Controller output */
            private double m_output;
            public double Output { get { return m_output; } }

            public double Update(double measurement, bool debugPrint = false)
            {
                double error = setpoint - measurement;
                double proportional = Kp * error;
                //integrator = integrator + 0.5f * Ki * dt * (error + prevError);
                double integrator_delta = 0.5f * Ki * dt * (error + prevError);
                integrator += integrator_delta;
                //integrator = integrator + 0.5f * Ki * dt * (error + prevError);

                /* Anti-wind-up via integrator clamping */
                if (integrator > limMaxInt)
                {
                    integrator = limMaxInt;
                }
                else if (integrator < limMinInt)
                {
                    integrator = limMinInt;
                }

                // (band-limited differentiator)
                differentiator = -(2.0f * Kd * (measurement - prevMeasurement)   /* Note: derivative on measurement, therefore minus sign in front of equation! */
                                    + (2.0f * lowPass - dt) * differentiator)
                                    / (2.0f * lowPass + dt);


                //if (proportional == double.NaN) { throw new ArgumentException("proportional NaN"); }
                //if (integrator == double.NaN) { throw new ArgumentException("integrator NaN"); }
                //if (differentiator == double.NaN) { throw new ArgumentException("differentiator NaN"); }
                //if (proportional == double.NaN) { p.Disp("proportional NaN"); }
                //if (integrator == double.NaN) { p.Disp("integrator NaN"); }
                //if (differentiator == double.NaN) { p.Disp("differentiator NaN"); }

                // Compute output and apply limits
                m_output = proportional + integrator + differentiator;

                //if (m_output == double.NaN) { p.Disp("output NaN"); }
                if (m_output > limMax)
                {
                    m_output = limMax;
                }
                else if (m_output < limMin)
                {
                    m_output = limMin;
                }


                /* Store error and measurement for later use */
                prevError = error;
                prevMeasurement = measurement;

                if (double.IsNaN(integrator)) { integrator = 0; }
                if (double.IsNaN(prevError)) { prevError = 0; }
                if (double.IsNaN(differentiator)) { differentiator = 0; }
                if (double.IsNaN(prevMeasurement)) { prevMeasurement = 0; }
                if (double.IsNaN(m_output)) { m_output = 0; }


                /* Return controller output */
                return m_output;
            }
        } // class PID

        Vector3D mul(MatrixD m, Vector3D v)
        {
            return new Vector3(
                (m.M11 * v.X) + (m.M21 * v.Y) + (m.M31 * v.Z) + m.M41,
                (m.M12 * v.X) + (m.M22 * v.Y) + (m.M32 * v.Z) + m.M42,
                (m.M13 * v.X) + (m.M23 * v.Y) + (m.M33 * v.Z) + m.M43);
        }
        //Vector3D div(Vector3D v, MatrixD m)
        //{
        //    return mul(Inverse(m), v);
        //}
        //
        //MatrixD Inverse(MatrixD m)
        //{
        //    var r = new MatrixD();
        //    r.M11 = 0
        //      + (m.M22 * m.M33 * m.M44) + (m.M32 * m.M43 * m.M24) + (m.M42 * m.M23 * m.M34)
        //      - (m.M42 * m.M33 * m.M24) - (m.M32 * m.M23 * m.M44) - (m.M22 * m.M43 * m.M34);
        //    r.M21 = 0
        //      - (m.M21 * m.M33 * m.M44) - (m.M31 * m.M43 * m.M24) - (m.M41 * m.M23 * m.M34)
        //      + (m.M41 * m.M33 * m.M24) + (m.M31 * m.M23 * m.M44) + (m.M21 * m.M43 * m.M34);
        //    r.M31 = 0
        //      + (m.M21 * m.M32 * m.M44) + (m.M31 * m.M42 * m.M24) + (m.M41 * m.M22 * m.M34)
        //      - (m.M41 * m.M32 * m.M24) - (m.M31 * m.M22 * m.M44) - (m.M21 * m.M42 * m.M34);
        //    r.M41 = 0
        //      - (m.M21 * m.M32 * m.M43) - (m.M31 * m.M42 * m.M23) - (m.M41 * m.M22 * m.M33)
        //      + (m.M41 * m.M32 * m.M23) + (m.M31 * m.M22 * m.M43) + (m.M21 * m.M42 * m.M33);
        //
        //    r.M12 = 0
        //      - (m.M12 * m.M33 * m.M44) - (m.M32 * m.M43 * m.M14) - (m.M42 * m.M13 * m.M34)
        //      + (m.M42 * m.M33 * m.M14) + (m.M32 * m.M13 * m.M44) + (m.M12 * m.M43 * m.M34);
        //    r.M22 = 0
        //      + (m.M11 * m.M33 * m.M44) + (m.M31 * m.M43 * m.M14) + (m.M41 * m.M13 * m.M34)
        //      - (m.M41 * m.M33 * m.M14) - (m.M31 * m.M13 * m.M44) - (m.M11 * m.M43 * m.M34);
        //    r.M32 = 0
        //      - (m.M11 * m.M32 * m.M44) - (m.M31 * m.M42 * m.M14) - (m.M41 * m.M12 * m.M34)
        //      + (m.M41 * m.M32 * m.M14) + (m.M31 * m.M12 * m.M44) + (m.M11 * m.M42 * m.M34);
        //    r.M42 = 0
        //      + (m.M11 * m.M32 * m.M43) + (m.M31 * m.M42 * m.M13) + (m.M41 * m.M12 * m.M33)
        //      - (m.M41 * m.M32 * m.M13) - (m.M31 * m.M12 * m.M43) - (m.M11 * m.M42 * m.M33);
        //
        //    r.M13 = 0
        //      + (m.M12 * m.M23 * m.M44) + (m.M22 * m.M43 * m.M14) + (m.M42 * m.M13 * m.M24)
        //      - (m.M42 * m.M23 * m.M14) - (m.M22 * m.M13 * m.M44) - (m.M12 * m.M43 * m.M24);
        //    r.M23 = 0
        //      - (m.M11 * m.M23 * m.M44) - (m.M21 * m.M43 * m.M14) - (m.M41 * m.M13 * m.M24)
        //      + (m.M41 * m.M23 * m.M14) + (m.M21 * m.M13 * m.M44) + (m.M11 * m.M43 * m.M24);
        //    r.M33 = 0
        //      + (m.M11 * m.M22 * m.M44) + (m.M21 * m.M42 * m.M14) + (m.M41 * m.M12 * m.M24)
        //      - (m.M41 * m.M22 * m.M14) - (m.M21 * m.M12 * m.M44) - (m.M11 * m.M42 * m.M24);
        //    r.M43 =
        //      -(m.M11 * m.M22 * m.M43) - (m.M21 * m.M42 * m.M13) - (m.M41 * m.M12 * m.M23)
        //      + (m.M41 * m.M22 * m.M13) + (m.M21 * m.M12 * m.M43) + (m.M11 * m.M42 * m.M23);
        //
        //    r.M14 = 0
        //      - (m.M12 * m.M23 * m.M34) - (m.M22 * m.M33 * m.M14) - (m.M32 * m.M13 * m.M24)
        //      + (m.M32 * m.M23 * m.M14) + (m.M22 * m.M13 * m.M34) + (m.M12 * m.M33 * m.M24);
        //    r.M24 = 0
        //      + (m.M11 * m.M23 * m.M34) + (m.M21 * m.M33 * m.M14) + (m.M31 * m.M13 * m.M24)
        //      - (m.M31 * m.M23 * m.M14) - (m.M21 * m.M13 * m.M34) - (m.M11 * m.M33 * m.M24);
        //    r.M34 = 0
        //      - (m.M11 * m.M22 * m.M34) - (m.M21 * m.M32 * m.M14) - (m.M31 * m.M12 * m.M24)
        //      + (m.M31 * m.M22 * m.M14) + (m.M21 * m.M12 * m.M34) + (m.M11 * m.M32 * m.M24);
        //    r.M44 = 0
        //      + (m.M11 * m.M22 * m.M33) + (m.M21 * m.M32 * m.M13) + (m.M31 * m.M12 * m.M23)
        //      - (m.M31 * m.M22 * m.M13) - (m.M21 * m.M12 * m.M33) - (m.M11 * m.M32 * m.M23);
        //
        //    return r;
        //}




        static double AngleBetweenDeg(Vector3D pa, Vector3D pb)
        {
            return (180.0 / Math.PI) * Math.Acos(pa.Dot(pb) / (pa.Length() * pb.Length()));
        }

        static double AngleDiffDeg(Vector3D pa, Vector3D pb, Vector3D up)
		{
            return AngleBetweenDeg(pa, pb) * Math.Sign(pa.Cross(pb).Dot(up));
		}

        public delegate double Zfn(double x);
        public struct FindZeroParams
        {
            public double guess;
            public double min;
            public double max;
            public double dx; // to determine slope for next guess
            public double margin;  // get within this close of the answer
            public double errStep;  // How far to move x if slope is 0 (divide by zero protection)
            public double iterationLimit;
        }

        //z = FindZero((theta) => { return calcDist(barrelVelocity, theta, gravity, heightOfs) - targetDistance; }, p);
        //******************************
        double FindZero(Zfn f, FindZeroParams p)
        {
            double x = p.guess;
            double y = 9999999;
            //double dy;
            double slope;
            double prev_dx = p.dx;
            //while (f(x) > margin)
            long iterationCount = 0;
            while (true)
            {
                iterationCount++;
                if (iterationCount > p.iterationLimit)
                {
                    // Give up
                    return double.NaN;
                    //return x;
                }

                // Test our latest guess
                y = f(x);

                //msg($"{x * rad2deg:0.##} deg => {y:0.} m");

                if (double.IsNaN(y))
                {
                    // domain error, function has no value at this point
                    // Try backing up half the distance as the last move
                    if (x == p.guess)
                    {
                        // First iteration.
                        // We don't have much information on where a valid domain is.
                        // Try at the boundaries.
                        if (!double.IsNaN(f(p.min)))
                        {
                            // Go to min. If that's not valid, it will start "reversing" towards max.
                            x = p.min;
                            prev_dx = p.min - p.max;
                            continue;
                        }
                        if (!double.IsNaN(f(p.max)))
                        {
                            x = p.max;
                            prev_dx = p.max - p.min;
                            continue;
                        }
                        else
                        {
                            // Could try more guesses for a general solver,
                            // but in this particular case I know that
                            // if the max is invalid, the whole thing is.
                            Echo("FindZero: No solution");
                            return double.NaN;
                        }
                    }
                    if (prev_dx < 0) { p.min = x; }
                    else { p.max = x; }
                    prev_dx /= 2.0;
                    x -= prev_dx;
                    continue;
                }
                
                
                if (Math.Abs(y) < p.margin)
                {
                    return x;
                }


                //dy = f(x + dx) - y;
                //slope = (dres - res) / delta;
                slope = (f(x + p.dx) - y) / p.dx;
                if (slope != 0)
                {
                    prev_dx = -y / slope;
                }
                else
                {
                    prev_dx = -Math.Sign(y) * p.errStep;
                }
                if (x + prev_dx < p.min)
                {
                    prev_dx = p.min - x;
                    x = p.min;
                }
                else if (x + prev_dx > p.max)
                {
                    prev_dx = p.max - x;
                    x = p.max;
                }
                else
				{
                    x += prev_dx;
				}
            }
            //Console.WriteLine($"Found zero in {iterationCount} iterations.");
            //return x;
        }
        // **************************************/


        // Runge-Kutta 4 single step
        delegate double DifferentialFunction(double x, double y);
        double rk4ss(DifferentialFunction dydx, double x0, double y0, double x)
        {
            double h = x - x0;
            double k1, k2, k3, k4;
            //double y = y0;
            k1 = h * dydx(x0, y0);
            k2 = h * dydx(x0 + 0.5 * h, y0 + 0.5 * k1);
            k3 = h * dydx(x0 + 0.5 * h, y0 + 0.5 * k2);
            k4 = h * dydx(x0 + h, y0 + k3);
            return y0 + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
        }


        // TODO: Beware local mins and maxes
        double FindZeroBin(Zfn f, FindZeroParams p)
		{
            double max = p.max;
            double min = p.min;
            double x = p.guess;
            double y, dy;
            int itCount = 0;
            while (max - min > p.margin)
			{
                itCount++;
                if (itCount > p.iterationLimit) { return x; }
                //Echo($"Iter {itCount} / {p.iterationLimit}");
                //Echo($"Try {x*rad2deg}");
                y = f(x);
                dy = f(x + p.dx) - y;
                try
                {
                    if (Math.Sign(y) == Math.Sign(dy)) { max = x; }
                    else { min = x; }
                }
                catch (Exception e)
				{
                    Echo(e.Message);
                    Runtime.UpdateFrequency = UpdateFrequency.None;
                    if ((itCount & 1) != 0) { x = (min + min + min + max) / 4; }
                    else { x = (max + max + max + min) / 4; }
				}
                x = (min + max) / 2.0;
			}
            return x;
		}

        // Something's wrong with this function
        //bool calcElevationAngle(double v, double g, double x, double y, out double r1, out double r2)
        //{
        //    double v2 = v * v;
        //    double v4 = v2 * v2;
        //    double sqr = v4 - (g * ((g * x * x) + (2 * y * v2)));
        //    if (sqr < 0)
        //    {
        //        r1 = 0;
        //        r2 = 0;
        //        return false;
        //    }
        //    r1 = Math.Atan((v2 + sqr) / (g * x));
        //    r2 = Math.Atan((v2 - sqr) / (g * x));
        //    return true;
        //}



        class OrbitAE
        {

            // a = semi major axis
            // e = eccentricity
            // i = inclination
            // p = periapsis
            // Om = longitude of ascending node (capital omega)
            // w = argument of periapsis (little omega looks similar to latin w)
            // v = true anomaly?

            // Only a and e really matter for this script.
            // Just need the basic shape of the ellipse.

            public double a, e; //, p, i, Om, w, v;
            static Vector3D mul(double s, Vector3 v)
            {
                return new Vector3(s * v.X, s * v.Y, s * v.Z);
            }
            static Vector3D div(Vector3 v, double s)
            {
                return new Vector3(v.X / s, v.Y / s, v.Z / s);
            }

            public static OrbitAE FromPosVel(
                    Vector3D rv /*radial vector (position)*/,
                    Vector3D vv /*velocity vector*/,
                    double mu /* mu = GM, units m^3 s^-2, gravitational constant times mass of planet in kg */
                )

            //PQ M = 5.9722e+24_kg,
            ////PQ G = PQ("6.6743015e-11 N m^2 / kg^2")
            ////PQ G = 6.6743015e-11 * 1_N * 1_m * 1_m / 1_kg / 1_kg
            //PQ G = 6.6743015e-14 * 1_N * 1_m * 1_m / 1_kg / 1_kg
            {
                OrbitAE q = new OrbitAE();

                double r = rv.Length(); // radius
                double v = vv.Length(); // velocity

                // angular momentum
                Vector3D hv = rv.Cross(vv);
                double h = hv.Length();

                // basis vector
                Vector3D khat = new Vector3(0, 0, 1);

                // node vector (normal?)
                Vector3D nhat = khat.Cross(hv);
                double n = nhat.Length();

                // Eccentricity
                Vector3D ev = div(mul(v * v - (mu / r), rv) - (rv.Dot(vv) * vv), mu);
                q.e = ev.Length();

                // specific mechanical energy
                double E = (v * v / 2) - (mu / r);

                if (q.e <= 1.0)
                {
                    // semi major axis
                    q.a = -mu / (2 * E);

                    // periapsis
                    // Valid but optimized out, not needed
                    //q.p = q.a * (1 - q.e * q.e);
                }
                else
                {
                    // not an elliptical orbit
                    q.a = double.PositiveInfinity;
                    // Valid but optimized out, not needed
                    //q.p = h * h / mu;
                }

                // Valid but optimized out, not needed
                //q.i = Math.Acos(hv.Z / h);
                //q.Om = Math.Acos(nhat.X / n);
                //q.w = Math.Acos(nhat.Dot(ev) / (n * q.e));
                //q.v = Math.Acos(ev.Dot(rv) / (q.e * r));

                return q;
            }
        }


        static double FindCollision(double semiMajorAxis, double eccentricity, double radius)
        {
            // This is a highly constrained version of a circle-ellipse collision.
            // The ellipse is axis aligned, where the major axis is the X axis.
            // One focus of the ellipse is the center of the circle.
            // The center of the circle is also the origin.
            // Returns first quadrant result as an angle from center of circle/planet
            // and the X axis.
            // This angle should be multiplied by 2 to get the planet-centric angle between
            // launch and landing points.

            //double focus = ellipseCenterX + (semiMajorAxis * eccentricity);
            // Focus coords are relative to origin (other focus), not the centroid of the ellipse
            // Thus, * 2
            double focus = 2.0 * semiMajorAxis * eccentricity;
            //Console.WriteLine($"focus = {focus}");

            // Is the ellipse even long enough to reach the circle?
            if ((focus / 2.0) + semiMajorAxis < radius) { return 0; }

            // Using the property of an ellipse that the sum of the distances to the
            // two foci equals the major axis (2 * semi major axis), and the fact that
            // one term of that sum is going to be the radius of the circle/planet,
            // find the distance d from the focus to the intersection point.
            double d = 2.0 * semiMajorAxis - radius;
            //Console.WriteLine($"d = {d}");

            // We now have a SSS triangle of radius, focus, and d.
            // Use the law of cosines to find the angle at the center of
            // the circle/planet (which is the origin, due to constraints.)
            double cosTheta = ((radius * radius) + (focus * focus) - (d * d)) / (2 * radius * focus);
            //Console.WriteLine($"cosTheta = {cosTheta}");
            //return Math.Acos(((radius*radius) + (focus*focus) - (d*d)) / (2 * radius * focus));
            return Math.Acos(cosTheta);
        }

        void Unlimit()
		{
            azimuthRotor.UpperLimitDeg = 361;
            azimuthRotor.LowerLimitDeg = -361;
            elevationHinge.UpperLimitDeg = 90;
            elevationHinge.LowerLimitDeg = -90;
        }

    } // Program
} // namespace
