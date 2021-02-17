:- rdf_meta(mem_event_create(r,r,r)).
:- use_module(library('db/mongo/client')).
:- dynamic execution_agent/1.

mem_episode_start(Action,
     EnvOwl,
EnvOwlIndiName,
EnvUrdf,
EnvUrdfPrefix,
AgentOwl,
AgentOwlIndiName,
AgentUrdf) :- 
    retractall(execution_agent(_)),tf_logger_disable, tripledb_drop(),forall(mng_collection(roslog,Coll),mng_drop(roslog,Coll)), tf_logger_enable,tf_mem_clear,
    tripledb_load('package://knowrob/owl/knowrob.owl',[graph(tbox),namespace(knowrob)]),
    tripledb_load(EnvOwl),
    tripledb_load(AgentOwl),
    urdf_load(AgentOwlIndiName, AgentUrdf, [load_rdf]),
    urdf_load(EnvOwlIndiName, EnvUrdf, [load_rdf,prefix(EnvUrdfPrefix)]),
    assertz(execution_agent(AgentOwlIndiName)),
    execution_agent(Agent),
    tell([is_episode(Episode), is_action(Action), has_type(Task,soma:'PhysicalTask'), 
            executes_task(Action,Task),is_setting_for(Episode,Action), is_performed_by(Action,Agent), has_type(Role, soma:'AgentRole'), has_role(Agent,Role)]),notify_synchronize(event(Action)),!.
    %%executes_task(Action,Task),is_setting_for(Episode,Action),is_setting_for(Episode,Agent)]),notify_synchronize(event(Action)),!.

%is_recording_episode(Result) :- assertz(cramEpisodeName('None')), retract(cramEpisodeName('None')), (cramEpisodeName(Name) -> Result = Name ; Result = 'NoName').
%delete_episode_name(Name) :- retract(cramEpisodeName(Name)).
mem_episode_stop(NeemPath) :- get_time(CurrentTime), atom_concat(NeemPath,'/',X1), atom_concat(X1,CurrentTime,X2), memorize(X2),tripledb_drop(),forall(mng_collection(roslog,Coll),mng_drop(roslog,Coll)).
mem_event_set_failed(Action) :- tell(action_failed(Action)).
mem_event_set_succeeded(Action) :- tell(action_succeeded(Action)).
mem_event_add_diagnosis(Situation, Diagnosis) :- tell(satisfies(Situation, Diagnosis)).

%add_subaction_with_task(Action,SubAction,TaskType) :- execution_agent(Agent), tell([has_type(Role, soma:'AgentRole'), has_role(Agent,Role), is_action(SubAction), has_type(Task,TaskType), executes_task(SubAction,Task), has_subevent(Action,SubAction),is_performed_by(SubAction,Agent)]),notify_synchronize(event(Event)), !.

add_subaction_with_task(Action,SubAction,TaskType) :- execution_agent(Agent),tell([is_action(SubAction), has_type(Task,TaskType), executes_task(SubAction,Task), has_subevent(Action,SubAction), is_performed_by(SubAction,Agent)]),notify_synchronize(event(Event)), !.

mem_event_end(Event) :- execution_agent(Agent),get_time(CurrentTime), ask([triple(Event,dul:'hasTimeInterval',TimeInterval), triple(TimeInterval,soma:'hasIntervalBegin', Start), executes_task(Event,Task)]),tripledb_forget(TimeInterval, soma:'hasIntervalEnd', _),tell([holds(TimeInterval, soma:'hasIntervalEnd', CurrentTime),has_type(Role, soma:'AgentRole'), has_role(Agent,Role) during Event,task_role(Task, Role)]),!.

mem_event_begin(Event) :- get_time(CurrentTime),tell(occurs(Event) since CurrentTime),!.

%belief_perceived_at(ObjectType, Frame, Object) :- get_time(CurrentTime),execution_agent(Agent),tell([has_type(Object,ObjectType),is_at(Object,Frame) since CurrentTime]).

belief_perceived_at(ObjectType, Mesh, Rotation, Object) :- tell([has_type(Object,ObjectType),has_type(ShapeRegion, soma:'MeshShape'), has_type(Shape, soma:'Shape'), triple(Object, soma:'hasShape', Shape), triple(Shape, dul:'hasRegion', ShapeRegion), triple(ShapeRegion, soma:'hasFilePath', Mesh),has_type(Origin,soma:'Origin'),triple(ShapeRegion,'http://knowrob.org/kb/urdf.owl#hasOrigin',Origin),triple(Origin, 'http://www.ease-crc.org/ont/SOMA.owl#hasPositionVector', term([0.0,0.0,0.0])),triple(Origin, 'http://www.ease-crc.org/ont/SOMA.owl#hasOrientationVector',term(Rotation))]).

belief_perceived_at(ObjectType, Object) :- tell([has_type(Object,ObjectType)]).


add_participant_with_role(Action, ObjectId, RoleType) :- 
ask([executes_task(Action, Task),triple(Event,dul:'hasTimeInterval',TimeInterval),triple(TimeInterval,soma:'hasIntervalBegin',Start),triple(TimeInterval,soma:'hasIntervalEnd',End)]), 
tell([has_participant(Action,ObjectId), has_type(Role, RoleType), has_role(ObjectId,Role) during Action,task_role(Task, Role)]).

%add_participant_with_role(Action, ObjectId, RoleType) :- ask(executes_task(Action, Task)), tell([has_participant(Action,ObjectId), has_type(Role, RoleType), has_role(ObjectId,Role) during [0.0,0.0]]).

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
