:- rdf_meta(mem_event_create(r,r,r)).
:- use_module(library('db/mongo/client')).
:- dynamic execution_agent/1.

mem_clear_memory() :-
    drop_graph(user),
    tf_mem_clear,
    mng_drop(roslog, tf).

mem_episode_start(Action, EnvOwl, EnvOwlIndiName, EnvUrdf, EnvUrdfPrefix, AgentOwl, AgentOwlIndiName, AgentUrdf) :-
    get_time(StartTime),
    mem_episode_start(Action,  EnvOwl, EnvOwlIndiName, EnvUrdf, EnvUrdfPrefix, AgentOwl, AgentOwlIndiName, AgentUrdf, StartTime).

mem_episode_start(Action, EnvOwl, EnvOwlIndiName, EnvUrdf, EnvUrdfPrefix, AgentOwl, AgentOwlIndiName, AgentUrdf, StartTime) :-
    retractall(execution_agent(_)),
    tf_logger_disable,
    mem_clear_memory,
    tf_logger_enable,
    load_owl(EnvOwl),
    load_owl(AgentOwl),
    urdf_load(AgentOwlIndiName, AgentUrdf, [load_rdf]),
    urdf_load(EnvOwlIndiName, EnvUrdf, [load_rdf,prefix(EnvUrdfPrefix)]),
    assertz(execution_agent(AgentOwlIndiName)),
    execution_agent(Agent),
    kb_project([
        new_iri(Episode, soma:'Episode'), is_episode(Episode), % Using new_iri here and below is a hideous workaround for a KnowRob bug, see https://github.com/knowrob/knowrob/issues/299
        new_iri(Action, dul:'Action'), is_action(Action),
        new_iri(TimeInterval, dul:'TimeInterval'), holds(Action, dul:'hasTimeInterval', TimeInterval), holds(TimeInterval, soma:'hasIntervalBegin', StartTime),
        new_iri(Task, dul:'Task'), has_type(Task,soma:'PhysicalTask'), executes_task(Action,Task),
        is_setting_for(Episode,Action),
        is_performed_by(Action,Agent),
        new_iri(Role, soma:'AgentRole'), has_type(Role, soma:'AgentRole'), has_role(Agent,Role)
    ]),
    % notify_synchronize(event(Action)),
    !.

%is_recording_episode(Result) :- assertz(cramEpisodeName('None')), retract(cramEpisodeName('None')), (cramEpisodeName(Name) -> Result = Name ; Result = 'NoName').
%delete_episode_name(Name) :- retract(cramEpisodeName(Name)).

mem_episode_stop(NeemPath) :-
    get_time(EndTime),
    mem_episode_stop(NeemPath, EndTime).

mem_episode_stop(NeemPath, EndTime) :-
    kb_call([
        is_episode(Episode), is_action(Action), is_setting_for(Episode,Action),
        holds(Action, dul:'hasTimeInterval', TimeInterval)
    ]),
    ignore(kb_unproject(triple(TimeInterval, soma:'hasIntervalEnd', double('Infinity')))),
    kb_project([
        holds(TimeInterval, soma:'hasIntervalEnd', EndTime)
    ]),
    get_time(CurrentTime), atom_concat(NeemPath,'/',X1), atom_concat(X1,CurrentTime,X2), memorize(X2), mem_clear_memory,!.

mem_event_set_failed(Action) :- kb_project(action_failed(Action)).

mem_event_set_succeeded(Action) :- kb_project(action_succeeded(Action)).

mem_event_add_diagnosis(Situation, Diagnosis) :- kb_project(satisfies(Situation, Diagnosis)).

add_subaction_with_task(ParentAction,SubAction,TaskType) :-
    execution_agent(Agent),
    kb_project([
        new_iri(SubAction, dul:'Action'), has_type(SubAction,dul:'Action'),
        new_iri(Task, TaskType), has_type(Task,TaskType), executes_task(SubAction,Task),
        holds(ParentAction,dul:hasConstituent,SubAction), % replacement for has_subevent
        is_performed_by(SubAction,Agent)
    ]),!.

mem_event_end(Event) :- execution_agent(Agent),
    get_time(CurrentTime),
    kb_call([
        triple(Event,dul:'hasTimeInterval',TimeInterval),
        triple(TimeInterval,soma:'hasIntervalBegin', Start), executes_task(Event,Task)]),
    ignore(kb_unproject(triple(TimeInterval, soma:'hasIntervalEnd', double('Infinity')))),
    kb_project([holds(TimeInterval, soma:'hasIntervalEnd', CurrentTime),new_iri(Role, soma:'AgentRole'),has_type(Role, soma:'AgentRole')]),
    kb_project([has_role(Agent,Role) during Event, task_role(Task, Role)]),!.
    mem_event_begin(Event) :- get_time(CurrentTime),kb_project(occurs(Event) since CurrentTime),!.

%belief_perceived_at(ObjectType, Frame, Object) :- get_time(CurrentTime),execution_agent(Agent),kb_project([has_type(Object,ObjectType),is_at(Object,Frame) since CurrentTime]).

belief_perceived_at(ObjectType, Mesh, Rotation, Object) :- kb_project([
    has_type(Object,ObjectType),
    new_iri(ShapeRegion,soma:'MeshShape'), has_type(ShapeRegion, soma:'MeshShape'),
    new_iri(Shape, soma:'Shape'), has_type(Shape, soma:'Shape'),
    triple(Object, soma:'hasShape', Shape),
    triple(Shape, dul:'hasRegion', ShapeRegion),
    triple(ShapeRegion, soma:'hasFilePath', Mesh),
    new_iri(Origin, soma:'Origin'), has_type(Origin,soma:'Origin'),
    triple(ShapeRegion,'http://knowrob.org/kb/urdf.owl#hasOrigin',Origin),
    triple(Origin, 'http://www.ease-crc.org/ont/SOMA.owl#hasPositionVector', term([0.0,0.0,0.0])),
    triple(Origin, 'http://www.ease-crc.org/ont/SOMA.owl#hasOrientationVector', term(Rotation))]).

belief_perceived_at(ObjectType, Object) :- kb_project([has_type(Object,ObjectType)]).

mem_tf_set(Object, ReferenceFrame, Position, Rotation, Timestamp) :-
    time_scope(=(Timestamp), =<('Infinity'), FScope),
    tf_set_pose(Object, [ReferenceFrame, Position, Rotation], FScope).

mem_tf_get(Object, ReferenceFrame, Position, Rotation) :-
    current_scope(QScope),
    tf_get_pose(Object, [ReferenceFrame, Position, Rotation], QScope, _).

mem_tf_get(Object, ReferenceFrame, Position, Rotation, Timestamp) :-
    time_scope(=(Timestamp), =(Timestamp), QScope),
    tf_get_pose(Object, [ReferenceFrame, Position, Rotation], QScope, _).


add_participant_with_role(Action, ObjectId, RoleType) :-
    kb_call([executes_task(Action, Task),
            triple(Event,dul:'hasTimeInterval',TimeInterval),
            triple(TimeInterval,soma:'hasIntervalBegin',Start),
            triple(TimeInterval,soma:'hasIntervalEnd',End)]),
    kb_project([has_participant(Action,ObjectId),
                new_iri(Role, RoleType), has_type(Role, RoleType)]),
    kb_project(has_role(ObjectId,Role) during Action),!.

add_parameter(Task,ParameterType,RegionType) :- kb_project([new_iri(Parameter, ParameterType), has_type(Parameter, ParameterType),
                                                                new_iri(Region,RegionType),has_type(Region,RegionType),
                                                                has_assignment(Parameter,Region) during [0.0,0.1],
                                                                has_parameter(Task, Parameter)]).

add_grasping_parameter(Action,GraspingOrientationType) :- kb_call(executes_task(Action, Task)),
    kb_project([new_iri(GraspingOrientation,GraspingOrientationType), has_type(GraspingOrientation,GraspingOrientationType),
                new_iri(GraspingOrientationConcept,'http://www.ease-crc.org/ont/SOMA.owl#GraspingOrientation'),
                has_type(GraspingOrientationConcept,'http://www.ease-crc.org/ont/SOMA.owl#GraspingOrientation'),
                has_parameter(Task,GraspingOrientationConcept),
                holds(GraspingOrientationConcept, dul:classifies, GraspingOrientation),
                has_region(Action,GraspingOrientation)]),!.

add_comment(Entity,Comment) :- kb_project(triple(Entity, 'http://www.w3.org/2000/01/rdf-schema#comment', Comment)).
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
