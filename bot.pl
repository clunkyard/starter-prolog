% A Prolog robot on the core binding supplied by the arena.
% Precise contacts lead shots; remembered passive cues guide the search.
% Bearing is earned: a scan contact's bearing is off by up to its
% bearing_error (degrees, a fixed share of the arc), so a wide beam finds a
% robot and a narrow one aims at it. The robot narrows the beam, holds it
% on the target and aims along a line fitted through its recent sightings.
% Contacts within 120 units are exact.
% Clunkyard robot API 2.0.0: one core under every language, the same
% answers everywhere. Facts (a call shows its arguments):
% match: mode ruleset control robots is_3d tick_rate max_ticks
%   robot_id(slot)
% clock: tick seconds remaining_ticks
% arena: size gravity terrain_kind ground_height(x y)
% me: health position velocity heading pitch roll angular_velocity
%   attitude grounded altitude wall_contact applied hull_mounts
%   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
% battery: charge capacity regen sustainable_drive compute
%   drive_cost(fraction)
% drive: hover_lift
% gun: aim_now cooldown_ticks spread max_power damage_scale
%   can_fire(power reserve) shot_speed(power) shot_cost(power)
%   reload_ticks(power) shot_damage(power) power_for_speed(speed)
%   power_for_flight_time(range seconds) lead(contact power)
%   lead_point(position velocity power)
% scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
% sensors: contacts nearest cues contact(slot) position_of(contact)
% tracker: cue nearest_bearing_rate velocity(slot) position(slot)
%   history(slot) last_seen(slot)
% nav: zone zone.margin(point) zone.contains(point margin)
%   zone.distance_from(point) storm storm.margin(point)
%   storm.contains(point margin) storm.distance_from(point) hill
%   hill.margin(point) hill.contains(point margin)
%   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
%   field_distance(a b)
% score: me best armour
% angles: normalize(degrees) angle_diff(target current)
%   bearing_to(from to) elevation_to(from to) direction(heading pitch)
%   distance(a b) to_chassis(heading pitch attitude)
%   from_chassis(heading pitch attitude) clamp(value low high)
% raw: number(path) text(path) has(path) flag(path) count(path) line()
% Controls: drive.set(throttle turn pitch_turn)
%   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
%   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
%   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
%   status.set(text value) debug.log(line) raw.extra(json)
% Every control argument is optional; a second call keeps what the first
% set. Absent facts are your language's none. Names are snake_case here;
% your language cases them its own way. Angles are degrees from the +x
% axis; pitch is positive up.
% Manual: https://clunkyard.com/manual#sdk
:- use_module(library(clunkyard)).
main :- run(user).
search_arc(40).  % degrees: sweeps the circle in half a second
hold_width(60).  % units across at the target: it stays in the beam from tick to tick
min_arc(6).      % degrees: the narrowest the beam holds
window(12).      % ticks of sightings the aim averages
sightings(8).    % narrow sightings in the window before a shot
look_every(40).  % ticks between looks around the target, for a nearer robot
reacquire(10).   % ticks to look again where a lost target was
% The state each turn passes on: whether it flies free, its dodge(Taken,EndTick) and the last target's slot.
on_start(R,starter(Free,dodge(0,0),0)) :- match_is_3d(R,Is3d),arena_gravity(R,Gravity),(Is3d==true,Gravity.z=:=0->Free=true;Free=false).
clamp(Value,Clamped) :- angles_clamp(Value,-1,1,Clamped).
steer(Towards,Current,Turn) :- angles_angle_diff(Towards,Current,Off),clamp(Off/45,Turn).
on_turn(R,starter(Free,Dodge0,Slot0),starter(Free,Dodge,Slot)) :-
    me_position(R,Pos),me_heading(R,Heading),me_pitch(R,Pitch),drive_forces(R,[brake=0.2]),
    sensors_nearest(R,Target),tracker_cue(R,Cue),
    clock_tick(R,Tick),
    (Target\==null->Slot=Target.slot,track(R,Target,Heading,Pitch,Tick)
    ;lost(R,Slot0,Tick)->Slot=Slot0,reacquire(R,Slot,Free,Pos,Heading,Pitch)
    ;Cue\==null->Slot=Slot0,follow(R,Cue,Free,Pos,Heading,Pitch)
    ;Slot=Slot0,search(R,Free,Pitch)),
    dodge(R,Dodge0,Dodge),
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
track(R,Target,Heading,Pitch,Tick) :-
    (Target.range>350->Throttle=0.7;Target.range<200->Throttle= -0.4;Throttle=0.2),
    steer(Target.bearing,Heading,Turn),steer(Target.elevation,Pitch,PitchTurn),
    drive_set(R,[throttle=Throttle,turn=Turn,pitch_turn=PitchTurn]),
    % Halve the beam each tick until it is hold_width across at the target:
    % narrow enough for a precise bearing, wide enough to keep it in.
    hold_width(Width),min_arc(MinArc),search_arc(SearchArc),look_every(LookEvery),scanner_last(R,Last),
    Hold is max(MinArc,2*atan2(Width/2,Target.range)*180/pi),
    % In a crowd a narrow beam sees one robot. Every look_every ticks,
    % widen it for a tick: a nearer robot it finds becomes the target.
    (Tick mod LookEvery=:=0->Arc=SearchArc;Arc is max(Hold,min(SearchArc,Last.arc/2))),
    scanner_set(R,[heading=Target.bearing,arc=Arc,pitch=Target.elevation]),
    gun_power_for_flight_time(R,Target.range,1,Power),
    % An exact contact: gun_lead is the intercept if the target holds course.
    % A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
    (Target.bearing_error>0->
        sightings(Enough),
        (fit(R,Target.slot,Tick,line(Where,Velocity,Count))->gun_lead_point(R,Where,Velocity,Power,Aim),(Count>=Enough->Ready=true;Ready=false);Aim=null,Ready=false)
    ;gun_lead(R,Target,Power,Aim),Ready=true),
    (Aim==null->H=Target.bearing,P=Target.elevation;H=Aim.heading,P=Aim.pitch),
    gun_aim(R,[heading=H,pitch=P]),gun_aim_now(R,Now),angles_angle_diff(H,Now.heading,Off),
    (Ready==true,abs(Off)<3,abs(P-Now.pitch)<3,gun_can_fire(R,Power,10,true)->gun_fire(R,[power=Power]);true),
    (Ready==true->Text="TRACK";Text="NARROW"),
    status_set(R,[text=Text,value=Target.range]).
% Lost it: look again where it was last seen, widening the beam, before searching.
lost(R,Slot,Tick) :- Slot\==0,tracker_last_seen(R,Slot,Seen),Seen\==null,reacquire(Within),Tick-Seen=<Within.
reacquire(R,Slot,Free,Pos,Heading,Pitch) :-
    tracker_position(R,Slot,Where0),(Where0==null->Where=Pos;Where=Where0),
    angles_bearing_to(Pos,Where,Bearing),(Free==true->angles_elevation_to(Pos,Where,Elevation);Elevation=0),
    steer(Bearing,Heading,Turn),steer(Elevation,Pitch,PitchTurn),drive_set(R,[throttle=0.5,turn=Turn,pitch_turn=PitchTurn]),
    search_arc(SearchArc),scanner_last(R,Last),Arc is min(SearchArc,Last.arc*2),
    scanner_set(R,[heading=Bearing,arc=Arc,pitch=Elevation]),status_set(R,[text="REACQUIRE"]).
% A straight line through the narrowest of the last window ticks' sightings:
% where the robot is now and how fast it moves, as line(Position,Velocity,Count).
% Each sighting is off by its bearing_error; the line averages that out.
fit(R,Slot,Tick,line(Position,Velocity,Count)) :-
    tracker_history(R,Slot,History),window(Window),Since is Tick-Window,
    include(later(Since),History,Recent),Recent=[_,_|_],
    maplist(error_of,Recent,Errors),min_list(Errors,Best),Limit is 1.5*Best,
    include(narrow(Limit),Recent,Seen),Seen=[_,_|_],                % the narrow ones
    length(Seen,Count),maplist(tick_of,Seen,Ticks),sum_list(Ticks,TickSum),T0 is TickSum/Count,
    foldl(spread(T0),Ticks,0,Spread),match_tick_rate(R,Rate),
    axis(x,Seen,Ticks,T0,Spread,Count,Tick,Rate,X,VX),axis(y,Seen,Ticks,T0,Spread,Count,Tick,Rate,Y,VY),axis(z,Seen,Ticks,T0,Spread,Count,Tick,Rate,Z,VZ),
    Position=vec3{x:X,y:Y,z:Z},Velocity=vec3{x:VX,y:VY,z:VZ}.
% One axis: the line's position now and its speed.
axis(Axis,Seen,Ticks,T0,Spread,Count,Tick,Rate,Position,Velocity) :-
    maplist(along(Axis),Seen,Values),sum_list(Values,Sum),Mean is Sum/Count,
    foldl(cross(T0,Mean),Ticks,Values,0,Cross),Slope is Cross/Spread,
    Position is Mean+Slope*(Tick-T0),Velocity is Slope*Rate.
later(Since,Sighting) :- Sighting.tick>Since.
narrow(Limit,Sighting) :- Sighting.bearing_error=<Limit.
error_of(Sighting,Error) :- Error=Sighting.bearing_error.
tick_of(Sighting,Tick) :- Tick=Sighting.tick.
along(Axis,Sighting,Value) :- get_dict(Axis,Sighting.position,Value).
spread(T0,Tick,Sum0,Sum) :- Sum is Sum0+(Tick-T0)*(Tick-T0).
cross(T0,Mean,Tick,Value,Sum0,Sum) :- Sum is Sum0+(Tick-T0)*(Value-Mean).
follow(R,Cue,Free,Pos,Heading,Pitch);search(R,Free,Pitch)),
    dodge(R,Dodge0,Dodge),
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
    % gun_lead is the intercept if the target holds course.
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
    scanner_last(R,Scan),angles_normalize(Scan.heading+36,Heading),search_arc(Arc),scanner_set(R,[heading=Heading,arc=Arc,pitch=Up]),status_set(R,[text="SEARCH"]).
% A simple dodge: vary it, or predict the shooter.
dodge(R,dodge(N,End),Dodge) :-
    clock_tick(R,Tick),Tick>=End,sensors_cues(R,Cues),member(Cue,Cues),Cue.source==muzzle,!,
    N1 is N+1,End1 is Tick+10,Dodge=dodge(N1,End1),dodging(R,N1).
dodge(R,Dodge,Dodge) :- clock_tick(R,Tick),Dodge=dodge(N,End),(Tick<End->dodging(R,N);true).
% Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
dodging(R,N) :-
    (N mod 2=:=1->Side=left,Turn=1;Side=right,Turn= -1),match_control(R,Control),
    (Control==forces->Strafe is -Turn,drive_set(R,[throttle=0.5]),drive_forces(R,[strafe=Strafe]);drive_set(R,[throttle=1,turn=Turn])),
    format(string(Text),"dodge:~w",[Side]),status_set(R,[text=Text,value=N]).
