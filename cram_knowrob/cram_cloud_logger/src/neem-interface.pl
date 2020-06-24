:- rdf_meta(mem_event_create(r,r,r)).
:- use_module(library('db/mongo/client')).

mem_episode_start(Action) :- tripledb_load('package://knowrob/owl/knowrob.owl',[graph(tbox),namespace(knowrob)]),tell([is_episode(Episode), is_action(Action), has_type(Task,ease_act:'PhysicalTask'), executes_task(Action,Task),is_setting_for(Episode,Action)]),!.
mem_episode_stop(NeemPath) :- get_time(CurrentTime), atom_concat(NeemPath,'/',X1), atom_concat(X1,CurrentTime,X2), memorize(X2), mng_export_collection('tf_projection',X2), mng_export_collection('tf',X2),tripledb_whipe(),forall(mng_collection(roslog,Coll),mng_drop(roslog,Coll)).
mem_event_set_failed(Action) :- tell(action_failed(Action)).
mem_event_set_succeeded(Action) :- tell(action_succeeded(Action)).
mem_event_add_diagnosis(Situation, Diagnosis) :- tell(satisfies(Situation, Diagnosis)).
add_subaction_with_task(Action,SubAction,TaskType) :- tell([is_action(SubAction), has_type(Task,TaskType), executes_task(SubAction,Task), has_subevent(Action,SubAction)]), !.
mem_event_end(Event) :- get_time(CurrentTime),tell([is_time_interval(TimeInterval),holds(Event,dul:hasTimeInterval,TimeInterval), occurs(Event) until CurrentTime]),!.
mem_event_begin(Event) :- get_time(CurrentTime),tell([is_time_interval(TimeInterval),holds(Event,dul:hasTimeInterval,TimeInterval), occurs(Event) since CurrentTime]),!.
%belief_perceived_at(ObjectType, Frame, Object) :- tell(has_type(Object,ObjectType),is_at(Object,Frame)).
belief_perceived_at(ObjectType, Frame, Object) :- tell(has_type(Object,ObjectType)).
add_participant_with_role(Action, ObjectId, RoleType) :- tell([has_participant(Action,ObjectId), has_type(Role, RoleType), has_role(ObjectId,Role) during [0.0,0.0]]).
add_parameter(Task,ParameterType,RegionType) :- tell([has_type(Parameter, ParameterType), has_type(Region,RegionType),has_assignment(Parameter,Region) during [0.0,0.1], has_parameter(Task, Parameter)]).
add_grasping_parameter(Action,GraspingOrientationType) :- ask(executes_task(Action, Task)), tell([has_type(GraspingOrientation,GraspingOrientationType), has_type(GraspingOrientationConcept,'http://www.ease-crc.org/ont/EASE.owl#GraspingOrientation'), has_parameter(Task,GraspingOrientationConcept),holds(GraspingOrientationConcept, dul:classifies, GraspingOrientation),has_region(Action,GraspingOrientation)]),!.
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

