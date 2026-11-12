% A Prolog robot on the core binding supplied by the arena.
% Precise contacts lead shots; remembered passive cues guide the search.
% Clunkyard robot API 2.0.0: one core under every language, the same
% answers everywhere. Facts (a call shows its arguments):
% match: mode ruleset control robots is_3d tick_rate max_ticks
%   robot_id(slot)
% clock: tick seconds remaining_ticks
% arena: size gravity terrain_kind ground_height(x y)
% me: health position velocity heading pitch roll angular_velocity
%   attitude grounded altitude applied
% battery: charge capacity regen sustainable_drive compute
%   drive_cost(fraction)
% drive: hover_lift
% gun: aim_now cooldown_ticks spread max_power damage_scale
%   can_fire(power reserve) shot_speed(power) shot_cost(power)
%   reload_ticks(power) shot_damage(power) power_for_speed(speed)
%   power_for_flight_time(range seconds) lead(contact power)
%   lead_point(position velocity power)
% scanner: last proximity_range reach(arc) arc_for_reach(range)
% sensors: contacts nearest cues contact(slot) position_of(contact)
% tracker: cue nearest_bearing_rate velocity(slot) position(slot)
%   last_seen(slot)
% nav: zone zone.margin(point) zone.contains(point margin)
%   zone.distance_from(point) storm storm.margin(point)
%   storm.contains(point margin) storm.distance_from(point) hill
%   hill.margin(point) hill.contains(point margin)
%   hill.distance_from(point) pickups nearest_pickup(kind)
%   field_distance(a b)
% score: me best armour
% angles: normalize(degrees) angle_diff(target current)
%   bearing_to(from to) elevation_to(from to) direction(heading pitch)
%   distance(a b) clamp(value low high)
% raw: number(path) text(path) has(path) flag(path) count(path) line()
% Controls: drive.set(throttle turn pitch_turn)
%   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
%   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
%   tracker.configure(smoothing forget_ticks cue_ticks)
%   status.set(text value) raw.extra(json)
% Every control argument is optional; a second call keeps what the first
% set. Absent facts are your language's none. Names are snake_case here;
% your language cases them its own way. Angles are degrees from the +x
% axis; pitch is positive up.
% Manual: https://clunkyard.com/manual#sdk
:- use_module(library(clunkyard)).
main :- run(user).
on_start(R,Free) :- match_is_3d(R,Is3d),arena_gravity(R,Gravity),(Is3d==true,Gravity.z=:=0->Free=true;Free=false).
clamp(Value,Clamped) :- angles_clamp(Value,-1,1,Clamped).
steer(Towards,Current,Turn) :- angles_angle_diff(Towards,Current,Off),clamp(Off/45,Turn).
on_turn(R,Free,Free) :-
    me_position(R,Pos),me_heading(R,Heading),me_pitch(R,Pitch),drive_forces(R,[brake=0.2]),
    sensors_nearest(R,Target),tracker_cue(R,Cue),
    (Target\==null->track(R,Target,Heading,Pitch);Cue\==null->follow(R,Cue,Free,Pos,Heading,Pitch);search(R,Free,Pitch)),
    battery_charge(R,Charge),battery_capacity(R,Capacity),
    (Charge<Capacity/4->drive_get(R,throttle,Throttle),drive_set(R,[throttle=Throttle*0.5]);true),
    nav_storm(R,Storm),
    (Storm\==null,nav_storm_margin(R,Pos,Margin),Margin> -60->
        angles_bearing_to(Storm.center,Pos,Away),angles_angle_diff(Away,Heading,Off),clamp(Off/30,Turn),drive_set(R,[throttle=0.8,turn=Turn]);true),
    nav_zone(R,Zone),
    (Zone\==null,nav_zone_margin(R,Pos,Space),Space<60->
        (Zone.shape==sphere->angles_elevation_to(Pos,Zone.center,Climb);Climb=0),
        angles_bearing_to(Pos,Zone.center,Home),angles_angle_diff(Home,Heading,HomeOff),clamp(HomeOff/30,HomeTurn),clamp((Climb-Pitch)/30,HomePitch),
        drive_set(R,[throttle=0.8,turn=HomeTurn,pitch_turn=HomePitch]);true),
    true.
track(R,Target,Heading,Pitch) :-
    (Target.range>350->Throttle=0.7;Target.range<200->Throttle= -0.4;Throttle=0.2),
    steer(Target.bearing,Heading,Turn),steer(Target.elevation,Pitch,PitchTurn),
    drive_set(R,[throttle=Throttle,turn=Turn,pitch_turn=PitchTurn]),scanner_set(R,[heading=Target.bearing,arc=20,pitch=Target.elevation]),
    gun_power_for_flight_time(R,Target.range,1,Power),gun_lead(R,Target,Power,Aim),
    (Aim==null->H=Target.bearing,P=Target.elevation;H=Aim.heading,P=Aim.pitch),
    gun_aim(R,[heading=H,pitch=P]),gun_aim_now(R,Now),angles_angle_diff(H,Now.heading,Off),
    (abs(Off)<3,abs(P-Now.pitch)<3,gun_can_fire(R,Power,10,true)->gun_fire(R,[power=Power]);true),
    status_set(R,[text="TRACK",value=Target.range]).
follow(R,Cue,Free,Pos,Heading,Pitch) :-
    angles_bearing_to(Pos,Cue.position,Bearing),angles_distance(Pos,Cue.position,Distance),
    (Free==true->angles_elevation_to(Pos,Cue.position,Elevation);Elevation=0),clock_tick(R,Tick),
    Error is min(Cue.bearing_error,30),Across is (Tick mod 5-2)/2*Error,
    (Free==true->Up is ((Tick//5) mod 3-1)*Error/2;Up=0),
    steer(Bearing,Heading,Turn),steer(Elevation,Pitch,PitchTurn),drive_set(R,[throttle=0.7,turn=Turn,pitch_turn=PitchTurn]),
    scanner_arc_for_reach(R,Distance+100,Arc),scanner_set(R,[heading=Bearing+Across,arc=Arc,pitch=Elevation+Up]),status_set(R,[text="CUE",value=Distance]).
search(R,Free,Pitch) :-
    clock_tick(R,Tick),(Free==true->Up is ((Tick//10) mod 3-1)*35;Up=0),
    steer(0,Pitch,PitchTurn),drive_set(R,[throttle=0.5,turn=0.33,pitch_turn=PitchTurn]),
    scanner_last(R,Scan),angles_normalize(Scan.heading+36,Heading),scanner_set(R,[heading=Heading,arc=40,pitch=Up]),status_set(R,[text="SEARCH"]).
