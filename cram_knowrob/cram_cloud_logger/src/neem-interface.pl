:- rdf_meta(mem_event_create(r,r,r)).
:- use_module(library('db/mongo/client')).

mem_episode_start(Action) :- tf_logger_disable, tripledb_drop(),forall(mng_collection(roslog,Coll),mng_drop(roslog,Coll)), tf_logger_enable,
    tripledb_load('package://knowrob/owl/knowrob.owl',[graph(tbox),namespace(knowrob)]),
    tripledb_load('package://iai_semantic_maps/owl/kitchen.owl'),
    tripledb_load('package://knowrob/owl/robots/PR2.owl'),
    urdf_load('http://knowrob.org/kb/PR2.owl#PR2_0', 'package://knowrob/urdf/pr2.urdf', [load_rdf]),
    urdf_load('http://knowrob.org/kb/IAI-kitchen.owl#iai_kitchen_room_link', 'package://iai_kitchen/urdf_obj/iai_kitchen_python.urdf', [load_rdf,prefix('iai_kitchen/')]),
    tell([is_episode(Episode), is_action(Action), has_type(Task,soma:'PhysicalTask'), 
            executes_task(Action,Task),is_setting_for(Episode,Action)]),notify_synchronize(event(Action)),!.

%is_recording_episode(Result) :- assertz(cramEpisodeName('None')), retract(cramEpisodeName('None')), (cramEpisodeName(Name) -> Result = Name ; Result = 'NoName').
%delete_episode_name(Name) :- retract(cramEpisodeName(Name)).
mem_episode_stop(NeemPath) :- get_time(CurrentTime), atom_concat(NeemPath,'/',X1), atom_concat(X1,CurrentTime,X2), memorize(X2),tripledb_drop(),forall(mng_collection(roslog,Coll),mng_drop(roslog,Coll)).
mem_event_set_failed(Action) :- tell(action_failed(Action)).
mem_event_set_succeeded(Action) :- tell(action_succeeded(Action)).
mem_event_add_diagnosis(Situation, Diagnosis) :- tell(satisfies(Situation, Diagnosis)).

add_subaction_with_task(Action,SubAction,TaskType) :- tell([is_action(SubAction), has_type(Task,TaskType), executes_task(SubAction,Task), has_subevent(Action,SubAction)]),notify_synchronize(event(Event)), !.

mem_event_end(Event) :- get_time(CurrentTime),ask(triple(Event,dul:'hasTimeInterval',TimeInterval)),tripledb_forget(TimeInterval, soma:'hasIntervalEnd', _),tell(holds(TimeInterval, soma:'hasIntervalEnd', CurrentTime)),!.

mem_event_begin(Event) :- get_time(CurrentTime),tell(occurs(Event) since CurrentTime),!.

%belief_perceived_at(ObjectType, Frame, Object) :- get_time(CurrentTime),tell([has_type(Object,ObjectType),is_at(Object,Frame) since CurrentTime]).
belief_perceived_at(ObjectType, Frame, Object) :- tell([has_type(Object,ObjectType)]).
add_participant_with_role(Action, ObjectId, RoleType) :- tell([has_participant(Action,ObjectId), has_type(Role, RoleType), has_role(ObjectId,Role) during [0.0,0.0]]).
add_parameter(Task,ParameterType,RegionType) :- tell([has_type(Parameter, ParameterType), has_type(Region,RegionType),has_assignment(Parameter,Region) during [0.0,0.1], has_parameter(Task, Parameter)]).

add_grasping_parameter(Action,GraspingOrientationType) :- ask(executes_task(Action, Task)), tell([has_type(GraspingOrientation,GraspingOrientationType), has_type(GraspingOrientationConcept,'http://www.ease-crc.org/ont/SOMA.owl#GraspingOrientation'), has_parameter(Task,GraspingOrientationConcept),holds(GraspingOrientationConcept, dul:classifies, GraspingOrientation),has_region(Action,GraspingOrientation)]),!.

add_comment(Entity,Comment) :- tell(triple(Entity, 'http://www.w3.org/2000/01/rdf-schema#comment', Comment)).
ros_logger_start :- process_create(path('rosrun'),['mongodb_log', 'mongodb_log.py','__name:=topic_logger', '--mongodb-name', 'roslog', '/tf_projection', '/tf'],[process(PID)]),asserta(ros_logger_pid(PID)).
ros_logger_stop :-     ros_logger_pid(PID),
    retractall(ros_logger_pid(PID)),
    process_create(path(rosnode), ['kill', '/topic_logger'], 
        [process(KillPID)]),process_wait(KillPID, _),
    process_wait(PID, _),
    process_create(path(rosnode),['cleanup'],
        [stdin(pipe(In)), detached(true), process(TLPID)]), 
    writeln(In,'y'),flush_output(In), process_wait(TLPID, _),
    print_message(informational,'Topic Logger stopped').



test_tf_query :- ask([triple('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_DCJBWPIE',dul:'hasTimeInterval',_O), triple(_O, soma:'hasIntervalBegin', _T2)]),time_scope(=<(_T2), >=(_T2), QScope),writeln(_T2),tf_get_pose('base_footprint', ['map',Position,Orientation], QScope, _),!.
