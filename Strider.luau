-- Discord: Hermabs | Roblox: Vell
--!strict

-- Strider, a six legged creature that walks with no animation data at all.
-- There is no Animator, no KeyframeSequence and no canned walk cycle. Every
-- joint angle on every frame is solved backwards from where the feet need to be.

-- The rig is an ordinary Motor6D assembly built by ServerStorage.BuildStrider.
-- Each leg is three joints in a chain: a yaw at the hip, a pitch at the thigh
-- and a bend at the knee. No bone length is written down in this file. They are
-- measured off the rig when the creature is built, so a leg can be resized in
-- the builder and nothing in here needs touching.

-- Two objects, and the split between them is the part worth understanding:

--   Leg      owns one leg. It knows where its foot is planted in the world,
--            whether that foot has strayed far enough to need moving, and how
--            to bend three joints so the foot sits where it is told to.

--   Strider  owns the body and the six Legs. It decides where to walk, holds
--            the body up, and referees which legs may move at any moment.

-- The order of work inside one frame matters, because the body and the legs
-- feed each other:

--   1: Strider picks a heading and a speed, then advances its own dead reckoned
--      position by that speed. This position is the creature's intent, not a
--      measurement of where it physically is.
--   2: Every Leg raycasts for the ground beneath its stance point, advances any
--      swing already running, and solves its three joints.
--   3: Strider reads what the legs found and starts a swing on the leg that is
--      most out of place, if the tripod rule allows that leg to move.
--   4: Strider averages the ground under all six feet and aims the body
--      constraints at that height. The body follows the legs; the legs do not
--      follow the body.

-- The body is never CFramed. It hangs unanchored from an AlignPosition and an
-- AlignOrientation, which is where the bob while idling and the tilt on slopes
-- come from without either being faked.

local Players = game:GetService("Players")
local RunService = game:GetService("RunService")
local ServerStorage = game:GetService("ServerStorage")

-- Has to agree with the stance BuildStrider posed the rig at. If it drifts the
-- body shoves itself up or down on the first frame before the legs catch up.
local RIDE_HEIGHT = 11.9

-- A foot lands where the body will be one swing from now, so the trigger has to
-- clear the ground the body covers during that swing. Set it any lower and every
-- leg re-steps the frame after it plants, which is a shuffle, not a walk.

-- These two also set the duty cycle: a leg spends STEP_TRIGGER/WALK_SPEED on the
-- ground and STEP_TIME in the air. Below 50% on the ground the two tripods stop
-- overlapping and there are moments with nothing holding the body up. 6.0 and
-- 0.22 against a walk of 18 puts it at about 60%.
local STEP_TRIGGER = 6.0
local STEP_TIME = 0.22
local STEP_LIFT = 2.2

local WALK_SPEED = 18

-- Deliberately slow. The feet sit ten studs out from the middle, so turning
-- drags them sideways at ten times this figure, far faster than the creature
-- ever walks. Anything quicker both looks wrong on something this wide and
-- leaves planted feet scuffing sideways while they wait their turn to step.
local TURN_RATE = 1.15
local ACCEL = 24

local SIGHT_RANGE = 185
local LOOM_RANGE = 19
local CALL_COOLDOWN = 11

-- How far a joint may travel from its rest angle before the leg has to re-plant,
-- regardless of how little the foot has drifted. The front and rear legs sit at
-- roughly 50 degrees of splay, so walking forwards compresses them instead of
-- just swinging them: the femur rears up and leans back over the carapace long
-- before the foot has strayed far enough to trip the distance test on its own.
local SWING_LIMIT = 0.78
local PITCH_HEADROOM = 0.34

local GROUND_PROBE = 44
local FEELER_SPREAD = 0.6

-- Range has to cover the turn. At WALK_SPEED with TURN_RATE it takes about a
-- second to swing ninety degrees, which is eighteen studs of travel, and the
-- body is another ten studs wide on top of that.
local FEELER_RANGE = 42

-- Radius of the sphere swept ahead of the creature. Well short of the full leg
-- span: matching it exactly makes the creature flinch away from doorways it
-- could comfortably walk through. Sits high enough that anything it could just
-- step over does not register.
local BODY_CLEARANCE = 7.5

-- Nothing on this creature collides, on purpose: six legs of loose parts sharing
-- an assembly is a good way to make the physics solver explode. That leaves
-- steering as the only thing keeping it out of walls, and steering alone loses
-- whenever it meets one at a shallow angle. This slides the body back out along
-- the surface once it is already too close for turning to save it.
local WALL_PUSH = 17

-- Once a side has been chosen to go round an obstacle, hold it for this long
-- after the way ahead looks clear. Releasing the moment the centre feeler slips
-- past a corner turns the creature straight back into the wall it was clearing,
-- and it oscillates there instead of getting past.
local DODGE_HOLD = 1.1

-- Twelve ground casts and three feelers go out every frame. One params object is
-- built here and reused for all of them; building a fresh one per cast is the
-- usual way a loop like this turns expensive for no reason.
local probe = RaycastParams.new()
probe.FilterType = Enum.RaycastFilterType.Exclude
probe.IgnoreWater = true
-- Without this a raycast hits anything queryable, so decoration that players
-- walk straight through still stops the creature dead and still holds its feet
-- up in the air. Only things you could actually collide with should count.
probe.RespectCanCollide = true

local rng = Random.new()

local function smoothstep(t: number): number
	return t * t * (3 - 2 * t)
end

-- Signed difference between two headings, always taking the short way round.
-- Plain subtraction makes the creature spin the long way whenever the target
-- crosses behind it and the angle wraps.
local function shortestTurn(from: number, to: number): number
	return (to - from + math.pi) % (math.pi * 2) - math.pi
end

local function groundUnder(anchor: Vector3): (Vector3, Vector3)
	local hit = workspace:Raycast(
		anchor + Vector3.new(0, GROUND_PROBE * 0.45, 0),
		Vector3.new(0, -GROUND_PROBE, 0),
		probe)

	if hit then
		return hit.Position, hit.Normal
	end

	-- Out over a ledge with nothing below. Leave the foot dangling at stance
	-- height rather than letting it chase a target down at the world origin.
	return anchor - Vector3.new(0, RIDE_HEIGHT, 0), Vector3.yAxis
end

-- Two bone inverse kinematics, solved in the vertical plane the leg swings
-- through. Given a foot target at (reach, -drop) measured from the hip, returns
-- the thigh pitch and the knee bend that put the foot there.
--
-- The hip angle comes from the law of cosines on the triangle hip-knee-foot,
-- whose three sides are the two bone lengths and the straight line span to the
-- target. The target is pulled onto the reachable ring before any of that runs:
-- solving an impossible triangle and clamping the angle afterwards makes the
-- knee snap inside out the moment the creature stretches for a ledge.
local function solveLeg(reach: number, drop: number, femur: number, shin: number): (number, number)
	local span = math.sqrt(reach * reach + drop * drop)
	local longest = femur + shin - 0.25
	local shortest = math.abs(femur - shin) + 0.25

	if span > longest then
		reach, drop, span = reach * longest / span, drop * longest / span, longest
	elseif span < shortest then
		reach, drop, span = reach * shortest / span, drop * shortest / span, shortest
	end

	local toTarget = math.atan2(-drop, reach)
	local spread = math.acos(math.clamp(
		(femur * femur + span * span - shin * shin) / (2 * femur * span), -1, 1))

	local pitch = toTarget + spread
	local kneeX = femur * math.cos(pitch)
	local kneeY = femur * math.sin(pitch)

	return pitch, math.atan2(-drop - kneeY, reach - kneeX) - pitch
end

-- One leg of the six. Holds the three Motor6D joints it drives, the bone
-- measurements taken off the rig, and the world position of its foot.

-- A foot is either planted or swinging. Planted means it is pinned to a fixed
-- world point and does not move at all while the body travels over it, which is
-- what stops the creature from ice skating. Swinging means it is travelling on
-- an arc towards a new plant. Only Strider decides when that changes.
local Leg = {}
Leg.__index = Leg

type LegFields = {
	yaw: Motor6D,
	pitch: Motor6D,
	bend: Motor6D,
	thud: Sound,
	restYaw: CFrame,
	restPitch: number,
	femurLength: number,
	shinLength: number,
	shinTilt: number,
	stanceSpan: number,
	stance: Vector3,
	group: number,
	swingTime: number,
	strain: number,
	foot: Vector3,
	rest: Vector3,
	ground: Vector3,
	normal: Vector3,
	swinging: boolean,
	swingFrom: Vector3,
	swingTo: Vector3,
	swingClock: number,
}

export type Leg = typeof(setmetatable({} :: LegFields, Leg))

function Leg.new(model: Model, tag: string, index: number, group: number): Leg
	local hull = model.PrimaryPart :: BasePart
	local coxa = model:FindFirstChild("Coxa" .. tag) :: BasePart
	local femur = model:FindFirstChild("Femur" .. tag) :: BasePart
	local tibia = model:FindFirstChild("Tibia" .. tag) :: BasePart
	local foot = model:FindFirstChild("Foot" .. tag) :: BasePart

	local yaw = coxa:FindFirstChildOfClass("Motor6D") :: Motor6D
	local pitch = femur:FindFirstChildOfClass("Motor6D") :: Motor6D
	local bend = tibia:FindFirstChildOfClass("Motor6D") :: Motor6D

	-- The foot hangs off a kinked tip welded past the end of the tibia, so the
	-- second IK bone runs knee to foot and matches neither the tibia's length nor
	-- its direction. C1 is where the knee joint sits inside the tibia, so
	-- subtracting it converts the foot into an offset from the knee.
	local kneeToFoot = tibia.CFrame:PointToObjectSpace(foot.Position) - bend.C1.Position

	return setmetatable({
		yaw = yaw,
		pitch = pitch,
		bend = bend,
		-- Lives on the foot, not the body, so a step on your left is heard on
		-- your left. Roblox positions the audio off the part it is parented to.
		thud = foot:FindFirstChild("Step") :: Sound,
		restYaw = yaw.C0,
		-- The builder bakes the rest pose into C0 as a pure X rotation, so the
		-- first Euler angle is exactly the angle this thigh was posed at.
		restPitch = pitch.C0:ToEulerAnglesXYZ(),
		femurLength = pitch.C1.Position.Z * 2,
		shinLength = kneeToFoot.Magnitude,
		shinTilt = math.atan2(kneeToFoot.Y, -kneeToFoot.Z),
		stanceSpan = (foot.Position - (hull.CFrame * yaw.C0).Position).Magnitude,
		stance = hull.CFrame:PointToObjectSpace(foot.Position),
		group = group,
		-- Six legs all swinging for exactly STEP_TIME reads as clockwork. A few
		-- percent of spread per leg is enough to kill that.
		swingTime = STEP_TIME * (0.91 + index * 0.019),
		strain = 0,
		foot = foot.Position,
		rest = foot.Position,
		ground = foot.Position,
		normal = Vector3.yAxis,
		swinging = false,
		swingFrom = foot.Position,
		swingTo = foot.Position,
		swingClock = 0,
	}, Leg)
end

-- How badly this leg wants to move, as one number for Strider to compare across
-- all six. Distance drift and joint strain are different units, so strain is
-- scaled onto the same range as the distance trigger and the worse of the two
-- wins. Either reason on its own is enough to justify a step.
function Leg.strayed(self: Leg): number
	return math.max((self.foot - self.rest).Magnitude, self.strain * STEP_TRIGGER)
end

function Leg.beginSwing(self: Leg)
	self.swinging = true
	self.swingClock = 0
	self.swingFrom = self.foot
	self.swingTo = self.ground
end

-- Two different body frames come in here and they are not interchangeable.

-- "plan" is the pose the creature intends to hold. Stance points are taken off
-- it so the gait never sees the physical body's wobble and cannot feed itself.

-- "body" is where the hull actually is. The leg parts are jointed to that hull,
-- so the IK has to solve against it. Solve against the plan instead and the foot
-- lands wherever the constraints happen to be lagging that frame, which shows up
-- as every planted foot creeping.
function Leg.update(self: Leg, dt: number, plan: CFrame, body: CFrame, lead: Vector3)
	local hip = body * self.restYaw
	local home = plan * self.stance

	-- Two probes, and the gap between them is the whole trick. The rest point is
	-- where this foot ought to be standing right now, and it is what decides when
	-- the foot has strayed far enough to move. The lead point is where it would
	-- land if it stepped this instant, one swing's worth of travel further on, so
	-- the foot arrives under the body rather than trailing behind it. Measure the
	-- stray against the lead point instead and the walker shuffles: the foot is
	-- born already out of position and re-steps immediately.
	self.rest, self.normal = groundUnder(home)
	self.ground = groundUnder(home + lead)

	if self.swinging then
		self.swingClock += dt / self.swingTime

		if self.swingClock >= 1 then
			self.swinging = false
			self.foot = self.swingTo

			-- One clip landing four or five times a second turns into a machine
			-- gun. Nudging the pitch on every landing is enough to hide that it
			-- is the same recording each time.
			self.thud.PlaybackSpeed = rng:NextNumber(0.88, 1.14)
			self.thud:Play()
		else
			-- Eased along the ground, but the lift runs off the raw clock so the
			-- arc peaks in the middle of the swing instead of leaning late.
			local arch = math.sin(self.swingClock * math.pi)
			local raw = self.swingFrom:Lerp(self.swingTo, smoothstep(self.swingClock))
				+ Vector3.new(0, arch * STEP_LIFT, 0)

			-- Straight up is the wrong way to lift this foot. It shortens the hip
			-- to foot distance, and with a femur less than half the length of the
			-- shin the only way the leg can absorb that is by folding the thigh up
			-- and back across the carapace. Easing the foot out onto the distance
			-- it holds while planted swings it on an arc about the hip instead,
			-- the way a real leg travels. The correction is zero at both ends of
			-- the swing, so the foot still lands exactly on target.
			self.foot = raw:Lerp(hip.Position + (raw - hip.Position).Unit * self.stanceSpan, arch)
		end
	end

	self:apply(hip)
end

-- Turns one world space foot position into three joint angles.
function Leg.apply(self: Leg, hip: CFrame)
	local target = hip:PointToObjectSpace(self.foot)
	local reach = math.sqrt(target.X * target.X + target.Z * target.Z)

	-- The hip joint's own -Z is "outward" for this leg, so this is the yaw that
	-- swings the whole leg into line with the foot. Once that is applied the rest
	-- is a flat two bone problem inside the plane the leg now lies in.
	local swing = math.atan2(-target.X, -target.Z)
	local pitch, bend = solveLeg(reach, -target.Y, self.femurLength, self.shinLength)

	-- 0 sitting at rest, 1 where the leg starts to look wrong. Reported to the
	-- gait rather than clamped here on purpose: clamping would hold the joint
	-- still while the foot carried on moving, and the foot would visibly slide.
	self.strain = math.max(
		math.abs(swing) / SWING_LIMIT,
		(pitch - self.restPitch) / PITCH_HEADROOM)

	self.yaw.C0 = self.restYaw * CFrame.Angles(0, swing, 0)
	self.pitch.C0 = CFrame.Angles(pitch, 0, 0)
	self.bend.C0 = CFrame.new(0, 0, -self.femurLength / 2)
		* CFrame.Angles(bend - self.shinTilt, 0, 0)
end

-- The creature itself: the body, the six legs, and the decisions.

-- Dormant  no player in range. Sways gently and drifts its heading.
-- Hunting  walking towards a player.
-- Looming  standing over one. Stops moving, turns slowly, calls out.
local Strider = {}
Strider.__index = Strider

type State = "Dormant" | "Hunting" | "Looming"

type StriderFields = {
	model: Model,
	hull: BasePart,
	gaze: SpotLight,
	breath: Sound,
	call: Sound,
	align: AlignPosition,
	orient: AlignOrientation,
	legs: { Leg },
	facing: number,
	speed: number,
	dodge: number,
	dodgeHold: number,
	shove: Vector3,
	travel: Vector3,
	pose: CFrame,
	state: State,
	clock: number,
	callClock: number,
	stride: RBXScriptConnection?,
}

export type Strider = typeof(setmetatable({} :: StriderFields, Strider))

-- Left front, left rear and right middle carry the weight while the other three
-- swing, then they trade. Kept as a list rather than a name keyed table so the
-- leg order, and with it the per leg swing timing, is the same on every run.
local TRIPOD: { { tag: string, group: number } } = {
	{ tag = "L1", group = 1 },
	{ tag = "L2", group = 2 },
	{ tag = "L3", group = 1 },
	{ tag = "R4", group = 2 },
	{ tag = "R5", group = 1 },
	{ tag = "R6", group = 2 },
}

function Strider.new(template: Model, spawnAt: CFrame): Strider
	local model = template:Clone()
	local hull = model.PrimaryPart :: BasePart
	model.Parent = workspace
	model:PivotTo(spawnAt)

	hull.Anchored = false
	hull:SetNetworkOwner(nil)

	local carry = Instance.new("Attachment")
	carry.Name = "Carry"
	carry.Parent = hull

	local align = Instance.new("AlignPosition")
	align.Attachment0 = carry
	align.Mode = Enum.PositionAlignmentMode.OneAttachment
	align.MaxForce = 4.5e6
	align.MaxVelocity = 240
	-- Stiff on purpose. Slack here shows up as the hull trailing the pose the
	-- legs were solved against, which reads as the whole creature swimming.
	align.Responsiveness = 65
	align.ApplyAtCenterOfMass = true
	align.Parent = hull

	local orient = Instance.new("AlignOrientation")
	orient.Attachment0 = carry
	orient.Mode = Enum.OrientationAlignmentMode.OneAttachment
	orient.MaxTorque = 3.6e6
	orient.MaxAngularVelocity = 24
	-- Softer than the position so the body rolls into slopes and turns instead of
	-- snapping onto every change of normal the feet report.
	orient.Responsiveness = 24
	orient.Parent = hull

	local skull = model:FindFirstChild("Skull") :: BasePart

	local self = setmetatable({
		model = model,
		hull = hull,
		gaze = skull:FindFirstChildOfClass("SpotLight") :: SpotLight,
		breath = hull:FindFirstChild("Breath") :: Sound,
		call = hull:FindFirstChild("Call") :: Sound,
		align = align,
		orient = orient,
		legs = {},
		facing = 0,
		speed = 0,
		dodge = 0,
		dodgeHold = 0,
		shove = Vector3.zero,
		travel = spawnAt.Position,
		pose = spawnAt,
		state = "Dormant" :: State,
		clock = 0,
		-- Backdated so the first sighting calls out immediately.
		callClock = -CALL_COOLDOWN,
		stride = nil :: RBXScriptConnection?,
	}, Strider)

	for index, entry in TRIPOD do
		self.legs[index] = Leg.new(model, entry.tag, index, entry.group)
	end

	self:refreshFilter()
	self.breath:Play()

	return self
end

-- Characters come and go, and a foot planted on a player's head looks terrible,
-- so the exclude list is rebuilt whenever one appears or leaves rather than
-- being set once at startup. The creature excludes itself for the same reason:
-- without it the ground casts land on its own legs.
function Strider.refreshFilter(self: Strider)
	local exclude: { Instance } = { self.model }

	for _, player in Players:GetPlayers() do
		local character = player.Character
		if character then
			table.insert(exclude, character)
		end
	end

	probe.FilterDescendantsInstances = exclude
end

function Strider.enter(self: Strider, state: State)
	if self.state == state then
		return
	end
	self.state = state

	if state == "Looming" and self.clock - self.callClock > CALL_COOLDOWN then
		self.callClock = self.clock
		self.call:Play()
	end
end

function Strider.nearestPlayer(self: Strider): BasePart?
	local here = self.hull.Position
	local best: BasePart? = nil
	local bestDistance = SIGHT_RANGE

	for _, player in Players:GetPlayers() do
		local character = player.Character
		if not character then
			continue
		end

		local root = character:FindFirstChild("HumanoidRootPart")
		local humanoid = character:FindFirstChildOfClass("Humanoid")
		if not root or not root:IsA("BasePart") or not humanoid or humanoid.Health <= 0 then
			continue
		end

		local distance = (root.Position - here).Magnitude
		if distance < bestDistance then
			best = root
			bestDistance = distance
		end
	end

	return best
end

-- Picks the state and returns the heading the creature would like to face.
function Strider.think(self: Strider): number
	local quarry = self:nearestPlayer()
	if not quarry then
		self:enter("Dormant")
		return self.facing + math.sin(self.clock * 0.23) * 0.015
	end

	-- Flattened, so looking up at a player on a ledge does not tip the body.
	local toQuarry = (quarry.Position - self.hull.Position) * Vector3.new(1, 0, 1)
	self:enter(if toQuarry.Magnitude < LOOM_RANGE then "Looming" else "Hunting")

	return math.atan2(-toQuarry.X, -toQuarry.Z)
end

-- Three forward feelers, one straight ahead and one to each side. The feet
-- already climb anything shorter than the knees by themselves because they
-- raycast for real ground, so these only have to catch what the creature
-- genuinely cannot straddle. The side casts are compared against each other to
-- work out which way is more open.

-- Returns a turn to add on top of the heading the creature wants, rather than a
-- heading of its own. Replacing the goal outright is what made it oscillate: the
-- two headings took turns winning and it never committed to either.
function Strider.avoid(self: Strider, dt: number): number
	local eye = self.hull.Position + Vector3.new(0, 2.5, 0)
	local forward = CFrame.Angles(0, self.facing, 0).LookVector

	-- Swept as a sphere, not a ray. A single line down the middle only knows what
	-- the head would hit, and this creature is twenty studs across, so it would
	-- happily clear a corner by its nose and drag both left legs through it.
	local blocked = workspace:Spherecast(eye, BODY_CLEARANCE, forward * FEELER_RANGE, probe)

	if not blocked then
		self.shove = Vector3.zero

		-- Path looks clear, but keep leaning away for a moment longer so the body
		-- and the trailing legs clear the corner before the goal heading wins.
		self.dodgeHold -= dt
		if self.dodgeHold <= 0 then
			self.dodge = 0
			return 0
		end
		return self.dodge * 0.5
	end

	-- Close enough that turning will not save it, so push straight back out along
	-- the face. A sphere that starts already overlapping reports no useful normal,
	-- so reversing is the fallback that always gets it clear.
	if blocked.Distance < BODY_CLEARANCE then
		local face = Vector3.new(blocked.Normal.X, 0, blocked.Normal.Z)
		local away = if face.Magnitude > 0.1 then face.Unit else -forward
		self.shove = away * (1 - blocked.Distance / BODY_CLEARANCE) * WALL_PUSH
	else
		self.shove = Vector3.zero
	end

	-- Only worked out once per obstacle. Re-deciding every frame is how it ends
	-- up grinding between two options instead of committing to one.
	if self.dodge == 0 then
		local bias = 0
		for _, offset in { -FEELER_SPREAD, FEELER_SPREAD } do
			local aim = CFrame.Angles(0, self.facing + offset, 0).LookVector * FEELER_RANGE
			local hit = workspace:Raycast(eye, aim, probe)
			bias -= math.sign(offset) * (if hit then 1 - hit.Distance / FEELER_RANGE else 0)
		end

		-- Boxed in equally both ways, so pick a side off the clock rather than
		-- standing still weighing up two identical options.
		self.dodge = if math.abs(bias) < 0.05
			then (if self.clock % 3 < 1.5 then 1 else -1)
			else math.sign(bias)
	end

	self.dodgeHold = DODGE_HOLD
	return self.dodge * (0.7 + (1 - blocked.Distance / FEELER_RANGE) * 1.1)
end

-- The referee. At most one leg may begin a swing per frame, and only if it
-- belongs to the tripod that is already moving. Drop the group test and all six
-- lift together the moment the creature accelerates, and the body drops.
function Strider.gait(self: Strider)
	local swingingGroup = 0
	for _, leg in self.legs do
		if leg.swinging then
			swingingGroup = leg.group
			break
		end
	end

	local pick: Leg? = nil
	local worst = STEP_TRIGGER

	for _, leg in self.legs do
		if leg.swinging or (swingingGroup ~= 0 and leg.group ~= swingingGroup) then
			continue
		end

		local stray = leg:strayed()
		if stray > worst then
			pick = leg
			worst = stray
		end
	end

	if pick then
		pick:beginSwing()
	end
end

function Strider.step(self: Strider, dt: number)
	self.clock += dt

	local wanted = self:think() + self:avoid(dt)
	local drifted = shortestTurn(self.facing, wanted)

	-- Full turn rate only while actually chasing. Stood over a player who keeps
	-- circling, the bearing swings faster than the body ever could, and at full
	-- rate the creature just spins on the spot.
	local rate = if self.state == "Hunting" then TURN_RATE else TURN_RATE * 0.4
	self.facing += math.clamp(drifted, -rate * dt, rate * dt)

	-- Slows down as the heading error grows, so a hard turn does not drag the
	-- whole stance sideways and cross the legs. It never reaches zero while
	-- hunting though, and that floor matters: getting round an obstacle needs
	-- forward travel, and stopping dead in front of one left the creature turning
	-- on the spot against a wall forever.
	local wantedSpeed = 0
	if self.state == "Hunting" then
		wantedSpeed = WALK_SPEED * math.max(0.25, 1 - math.abs(drifted) / 1.6)
	end
	self.speed += math.clamp(wantedSpeed - self.speed, -ACCEL * dt, ACCEL * dt)

	local heading = CFrame.Angles(0, self.facing, 0).LookVector
	self.travel += (heading * self.speed + self.shove) * dt

	local body = self.hull.CFrame
	local lead = heading * (self.speed * STEP_TIME)
	for _, leg in self.legs do
		leg:update(dt, self.pose, body, lead)
	end
	self:gait()

	local floor = 0
	local slope = Vector3.zero
	for _, leg in self.legs do
		floor += leg.rest.Y
		slope += leg.normal
	end

	-- Only lean part of the way into the slope. Matching it exactly makes the
	-- body look welded to the stairs rather than walking up them.
	local up = (slope / #self.legs).Unit:Lerp(Vector3.yAxis, 0.42).Unit
	local idle = if self.state == "Hunting" then 0 else math.sin(self.clock * 1.55) * 0.3
	local perch = Vector3.new(self.travel.X, floor / #self.legs + RIDE_HEIGHT + idle, self.travel.Z)

	-- Lean is taken from what the creature is trying to do, not from what it
	-- measurably did. Differentiating AssemblyLinearVelocity for this is far too
	-- noisy to drive a pose with and you get a body that shivers.
	local push = self.speed / WALK_SPEED
	local roll = math.clamp(drifted, -1, 1) * push * 0.22

	self.pose = CFrame.lookAt(perch, perch + heading, up) * CFrame.Angles(-push * 0.15, 0, roll)
	self.align.Position = self.pose.Position
	self.orient.CFrame = self.pose.Rotation

	local wantedGlow = if self.state == "Dormant" then 0.5 else 1.5
	self.gaze.Brightness += (wantedGlow - self.gaze.Brightness) * math.min(dt * 3, 1)
end

-- PreSimulation rather than Heartbeat: the joints have to be written before the
-- physics step that the AlignPosition acts on, otherwise the body is always
-- chasing a pose the legs solved for the previous frame.
function Strider.start(self: Strider)
	local pivot = self.model:GetPivot()
	self.facing = math.atan2(-pivot.LookVector.X, -pivot.LookVector.Z)
	self.travel = pivot.Position
	self.pose = pivot
	self.align.Position = pivot.Position
	self.orient.CFrame = pivot.Rotation

	self.stride = RunService.PreSimulation:Connect(function(dt)
		self:step(dt)
	end)
end

local strider = Strider.new(
	ServerStorage:WaitForChild("StriderRig") :: Model,
	CFrame.new(0, RIDE_HEIGHT, -30))

local function refresh()
	strider:refreshFilter()
end

local function watch(player: Player)
	player.CharacterAdded:Connect(refresh)
	player.CharacterRemoving:Connect(refresh)
end

for _, player in Players:GetPlayers() do
	watch(player)
end
Players.PlayerAdded:Connect(watch)
Players.PlayerRemoving:Connect(refresh)

strider:start()
